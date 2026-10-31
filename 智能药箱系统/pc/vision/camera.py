#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 相机封装模块 (camera.py)
================================================================================
职责：把 cv2.VideoCapture 包一层，解决三个真实项目里一定会遇到的坑：

  坑1 自动曝光/白平衡/对焦
      OpenCV 打开相机后默认是全自动。现场灯一变，二维码亮度就跳，
      识别结果时好时坏，且极难复现。必须手动锁定全部自动参数。

  坑2 帧缓冲
      默认缓冲 3~5 帧。你以为读的是"现在"，实际是 200ms 以前。
      在机械臂刚到位就拍照的场景下，这会直接导致"扫的是旧画面"。
      必须 CAP_PROP_BUFFERSIZE = 1。

  坑3 相机被占用/断连
      USB 相机被拔了、被别的软件占了，read() 会一直返回 False。
      这里封装了自动重连，避免主线程卡死。

同时提供 MockCamera：用一张静态图片冒充相机，方便在没有硬件的电脑上
跑通整条流水线（本项目的仿真/自测就靠它）。
================================================================================
"""

from __future__ import annotations

import time
from typing import Optional, Tuple

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# 后端解析 + 亮度工具（模块级，供 main.py / 工具脚本复用）
# ---------------------------------------------------------------------------
def parse_backend(name) -> Optional[int]:
    """
    把配置里的采集后端名解析成 cv2 常量。

    name : None/'auto' -> None（由 Camera.open 自行按平台优先级试）
           'dshow'/'msmf'/'v4l2'/'any' -> 对应 cv2 常量；未知名字返回 None
    """
    if name is None:
        return None
    s = str(name).strip().lower()
    if s in ("", "auto", "none"):
        return None
    table = {
        "dshow": getattr(cv2, "CAP_DSHOW", 700),
        "msmf": getattr(cv2, "CAP_MSMF", 1400),
        "v4l2": getattr(cv2, "CAP_V4L2", 200),
        "any": getattr(cv2, "CAP_ANY", 0),
    }
    return table.get(s)


def brightness(frame) -> float:
    """画面平均亮度（0~255）。frame 为 None/异常时返回 0。"""
    if frame is None:
        return 0.0
    try:
        if frame.ndim == 2:
            return float(frame.mean())
        return float(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean())
    except Exception:
        return 0.0


class Camera:
    """
    USB 相机的稳健封装。

    用法
    ----
        cam = Camera(index=0, width=1280, height=720)
        if cam.open():
            ok, frame = cam.read()
            ...
        cam.release()

    也可以当上下文管理器用：
        with Camera(0) as cam:
            ok, frame = cam.read()
    """

    def __init__(self, index: int = 0, width: int = 1280, height: int = 720,
                 fps: int = 30, exposure: Optional[float] = -6.0,
                 backend: Optional[int] = None,
                 auto_exposure: bool = True,
                 ae_target_lo: int = 90, ae_target_hi: int = 160,
                 ae_min: float = -13.0, ae_max: float = 0.0,
                 ae_settle_frames: int = 5,
                 auto_wb: bool = True,
                 wb_blue_u=None, wb_red_u=None,
                 wb_settle_frames: int = 45):
        """
        参数
        ----
        index    : 相机序号，0 一般是内置/第一个外接相机
        width    : 请求宽度（相机会取最接近的 supported 值）
        height   : 请求高度
        fps      : 请求帧率
        exposure : 手动曝光值，None 表示不动（用相机默认）。
                   典型值 -4 ~ -8，越小越暗，需现场调。
                   ★ auto_exposure=True 时它只作为"起点"，不再决定最终值。
        backend  : cv2.CAP_DSHOW / CAP_MSMF / CAP_V4L2 / None=auto
                   auto 时 Windows 先试 MSMF（MJPG 路径帧率高），失败退 DSHOW。
        auto_exposure : 开机自适应曝光：搜索曝光使亮度落在 [ae_target_lo, hi]，
                   并取最短曝光（=最高帧率）。白天/晚上/换灯都不用手调。
        ae_*     : 自适应曝光的目标亮度区间、搜索范围、每档生效等待帧数
        auto_wb  : True=保持相机自动白平衡（默认，推荐）；
                   False=关自动白平衡并尝试写死 wb_blue_u/wb_red_u
                   （实测本机 USB 相机不支持写死，此时打印警告并保持自动）
        wb_settle_frames : 白平衡预热的上限帧数（偏色稳定即提前结束）
        """
        self.index = int(index)
        self.width, self.height, self.fps = int(width), int(height), int(fps)
        self.exposure = exposure
        self.backend = backend

        # ---- 自适应曝光参数 ----
        self.auto_exposure = bool(auto_exposure)
        self.ae_target_lo = float(ae_target_lo)
        self.ae_target_hi = float(ae_target_hi)
        self.ae_min = float(ae_min)
        self.ae_max = float(ae_max)
        self.ae_settle_frames = max(1, int(ae_settle_frames))

        # ---- 白平衡参数 ----
        self.auto_wb = bool(auto_wb)
        self.wb_blue_u = wb_blue_u
        self.wb_red_u = wb_red_u
        self.wb_settle_frames = max(1, int(wb_settle_frames))

        # ---- 运行时状态（main.py 日志会读）----
        self.backend_used: Optional[int] = None   # 实际打开用的后端
        self.backend_name: Optional[str] = None   # 'MSMF' / 'DSHOW' / 'V4L2' / None
        self.auto_exposure_on = self.auto_exposure
        self._fail_count = 0
        self._last_open_try = 0.0

    # ------------------------------------------------------------------
    def open(self) -> bool:
        """打开相机并锁定自动参数。backend=auto 时按平台优先级逐个试。成功返回 True"""
        candidates = []
        if self.backend is not None:
            candidates = [self.backend]
        else:
            # Windows：MSMF 与内置头走 MJPG 同路径，帧率明显高于 DSHOW(YUYV)。
            # 实测（1280x720, exposure=-1）：MSMF 高帧率 / DSHOW ~5fps。
            if hasattr(cv2, "CAP_MSMF"):
                candidates.append(cv2.CAP_MSMF)
            if hasattr(cv2, "CAP_DSHOW"):
                candidates.append(cv2.CAP_DSHOW)
            candidates.append(None)           # 最后兜底 CAP_ANY

        last_err = None
        for be in candidates:
            try:
                if be is not None:
                    self.cap = cv2.VideoCapture(self.index, be)
                else:
                    self.cap = cv2.VideoCapture(self.index)
            except Exception as e:
                self.cap = None
                last_err = e
                continue
            if self.cap is not None and self.cap.isOpened():
                self.backend_used = be
                self.backend_name = {getattr(cv2, "CAP_MSMF", 1400): "MSMF",
                                     getattr(cv2, "CAP_DSHOW", 700): "DSHOW",
                                     getattr(cv2, "CAP_V4L2", 200): "V4L2",
                                     }.get(be)
                break
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        else:
            if last_err:
                print("[Camera] 打开相机失败: %r" % (last_err,))
            return False

        # --- 关键 1：只留 1 帧缓冲，杜绝"读到旧画面" ---
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        # --- 关键 2：先设分辨率帧率 ---
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # --- 关键 3：曝光 / 对焦 / 白平衡 ---
        try:
            self._set_safe(cv2.CAP_PROP_AUTOFOCUS, 0)    # 关自动对焦（会拉风箱）
        except Exception:
            pass

        if self.auto_wb:
            # 保持自动白平衡（实测本机 USB 相机 AUTO_WB 可能是哑的，
            # 但强制关掉反而会打回出厂色温增益 -> 画面偏蓝，见 app.yaml 注释）
            self._set_safe(cv2.CAP_PROP_AUTO_WB, 1)
            # 读 wb_settle_frames 帧等它收敛（偏色稳定即提前结束）
            ref = None
            stable = 0
            for _ in range(self.wb_settle_frames):
                ok, f = self.cap.read()
                if not ok:
                    continue
                if ref is None:
                    ref = f
                    continue
                try:
                    diff = float(np.mean(cv2.absdiff(f, ref)))
                    if diff < 2.0:            # 相邻帧几乎不变 = 已稳定
                        stable += 1
                        if stable >= 3:
                            break
                except Exception:
                    break
                ref = f
        else:
            # 尝试写死白平衡（相机不支持时会打印警告，画面可能偏色）
            self._set_safe(cv2.CAP_PROP_AUTO_WB, 0)
            ok_u = self.wb_blue_u is not None and self._set_safe(
                cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, float(self.wb_blue_u))
            ok_r = self.wb_red_u is not None and self._set_safe(
                cv2.CAP_PROP_WHITE_BALANCE_RED_U, float(self.wb_red_u))
            if not (ok_u and ok_r):
                print("[Camera] 警告：该相机不支持写死白平衡，已保持自动白平衡")

        # --- 曝光：自适应 or 手动 ---
        if self.auto_exposure and self.auto_exposure_on:
            try:
                self.auto_expose()
            except Exception:
                # 自适应失败退回手动起点
                if self.exposure is not None:
                    self._set_manual_exposure(float(self.exposure))
        else:
            # 手动曝光：先切手动模式再写值（DShow 0.25 / V4L2 0，两个都试）
            self._set_manual_exposure(float(self.exposure)
                                      if self.exposure is not None else -6.0)

        # 预热几帧，让参数真正生效
        for _ in range(3):
            self.cap.read()
        return True

    # ------------------------------------------------------------------
    def _set_manual_exposure(self, value: float) -> None:
        """切手动曝光并写值（不同后端"手动模式"的枚举值不同，两个都试）"""
        try:
            self._set_safe(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)   # DShow: 0.25=手动
        except Exception:
            pass
        try:
            self._set_safe(cv2.CAP_PROP_AUTO_EXPOSURE, 0.0)    # V4L2: 0=手动
        except Exception:
            pass
        if value is not None:
            self._set_safe(cv2.CAP_PROP_EXPOSURE, float(value))
            self.exposure = float(value)

    def auto_expose(self, verbose: bool = True):
        """
        自适应曝光：搜索使亮度落在 [ae_target_lo, ae_target_hi] 的【最短曝光】
        （曝光越短帧率越高）。搜索方向：从最快（ae_min）到最慢（ae_max），
        第一个达标的就是"最短且够亮"的曝光 —— 既亮又不拖帧。

        返回 (曝光值, 亮度)；彻底失败返回 None（相机保持原状）。
        """
        if self.cap is None:
            return None
        # 切手动曝光模式（否则写 CAP_PROP_EXPOSURE 会被驱动忽略）
        try:
            self._set_safe(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            self._set_safe(cv2.CAP_PROP_AUTO_EXPOSURE, 0.0)
        except Exception:
            pass

        # 先看当前亮度，达标就不用搜（省 2~5 秒）
        ok, f = self.cap.read()
        b = brightness(f) if ok else 0.0
        if ok and self.ae_target_lo <= b <= self.ae_target_hi:
            exp = self._get_exposure()
            if exp is not None:
                self.exposure = exp
            if verbose:
                print("[Camera] 亮度 %.0f 已达标，曝光保持 %s"
                      % (b, self.exposure))
            return self.exposure, b

        # 一档一档试：从最短曝光开始（0.25ms 级），第一档达标的即最优
        lo, hi = self.ae_min, self.ae_max
        step = 1.0                                  # DShow 曝光≈2^n 秒，整档足够
        best = None
        v = lo
        while v <= hi + 1e-6:
            self._set_safe(cv2.CAP_PROP_EXPOSURE, float(v))
            self.exposure = float(v)
            # 丢几帧让参数生效，再测亮度
            b2 = 0.0
            for k in range(self.ae_settle_frames):
                ok, f = self.cap.read()
                if ok and k == self.ae_settle_frames - 1:
                    b2 = brightness(f)
            if verbose:
                print("[Camera] 曝光 %g -> 亮度 %.0f" % (v, b2))
            if b2 >= self.ae_target_lo:
                best = (float(v), b2)               # 第一档达标 = 最短曝光
                if b2 <= self.ae_target_hi:
                    break                           # 区间内，完美收工
                # 超过 hi（过曝）也停：再亮只会更过曝
                break
            v += step

        if best is None:
            if verbose:
                print("[Camera] 自适应曝光失败：全程偏暗（现场无光？）")
            return None
        self.exposure = best[0]
        self._set_safe(cv2.CAP_PROP_EXPOSURE, best[0])
        if verbose:
            print("[Camera] 自适应曝光 -> %g（亮度 %.0f）" % best)
        return best

    def _get_exposure(self) -> Optional[float]:
        try:
            v = self.cap.get(cv2.CAP_PROP_EXPOSURE)
            return float(v) if v else None
        except Exception:
            return None

    # ------------------------------------------------------------------
    def _set_safe(self, prop: int, value: float) -> bool:
        """设置相机属性，失败静默忽略（不同相机支持的参数差异很大）。返回是否成功"""
        try:
            if self.cap is not None:
                return bool(self.cap.set(prop, float(value)))
        except Exception:
            pass
        return False

    # ------------------------------------------------------------------
    def read(self) -> Tuple[bool, Optional["np.ndarray"]]:
        """
        读一帧。返回 (是否成功, 图像)。
        内部带"连续失败自动重连"，避免相机被拔掉后程序永久卡死。
        """
        if self.cap is None:
            return False, None
        try:
            ok, frame = self.cap.read()
        except Exception:
            ok, frame = False, None

        if ok and frame is not None:
            self._fail_count = 0
            return True, frame

        self._fail_count += 1
        # 连续 10 次失败且距上次重连超过 2 秒，尝试重开
        if self._fail_count >= 10 and (time.time() - self._last_open_try) > 2.0:
            self._last_open_try = time.time()
            self.release()
            time.sleep(0.3)
            if self.open():
                self._fail_count = 0
        return False, None

    # ------------------------------------------------------------------
    def release(self) -> None:
        """释放相机资源"""
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        self.cap = None

    # ------------------------------------------------------------------
    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *a):
        self.release()
        return False


class MockCamera:
    """
    假相机：始终返回同一张（或循环的几张）图片，接口与 Camera 完全一致。

    用途
    ----
    1) 没有摄像头时也能跑通/自测整条流水线
    2) 单元测试
    3) 答辩前预演（把现场拍的照片存下来，回家也能调算法）

    用法
    ----
        cam = MockCamera("tests/sample_box.jpg")      # 单图
        cam = MockCamera(["a.jpg", "b.jpg"])          # 多图循环
        cam = MockCamera(np_array)                    # 直接给数组
    """

    def __init__(self, source, loop: bool = True):
        """
        参数
        ----
        source : 图片路径 / 路径列表 / numpy 图像数组
        loop   : 多图时是否循环播放
        """
        self.frames = []
        if isinstance(source, np.ndarray):
            self.frames = [source]
        elif isinstance(source, (list, tuple)):
            for p in source:
                if isinstance(p, np.ndarray):
                    self.frames.append(p)          # 直接给数组（仿真/测试常用）
                else:
                    img = cv2.imread(str(p))
                    if img is not None:
                        self.frames.append(img)
        else:
            img = cv2.imread(str(source))
            if img is not None:
                self.frames = [img]

        if not self.frames:
            raise ValueError("MockCamera 未能加载任何图像: %r" % (source,))

        self.loop = loop
        self._i = 0
        self._exhausted = False
        self.expected_code: Optional[str] = None

    def set_expected(self, code) -> None:
        """
        告诉假相机"现在应该看到哪个码"，并重渲染场景图。

        ★ 这是 --mock 能跑通 auto 模式的关键钩子：
          state_machine 在列对位前调 set_expected("COL1")、在过检位前调
          set_expected("MED001")，本相机的下一帧就真的渲染出这个码 ——
          于是"列二维码校正 / 药品过检"在纯仿真下都能闭环。
          真实相机没有此方法（调用方用 getattr 探测，忽略即可）。
        """
        try:
            new_code = str(code).strip()
        except Exception:
            return
        if not new_code or new_code == self.expected_code:
            return
        self.expected_code = new_code
        self._rebuild_frames()

    def _rebuild_frames(self) -> None:
        """按 expected_code 重渲染"灰底 + 中央二维码"场景帧。"""
        try:
            from vision.qr_detector import make_qr_image
            img = np.full((420, 420, 3), 205, np.uint8)
            qr = make_qr_image(self.expected_code or "MED001", size=170)
            if qr is not None:
                q = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
                q = cv2.resize(q, (240, 240))
                img[90:330, 90:330] = q
            self.frames = [img]
            self._i = 0
        except Exception:
            # 渲染失败保持原帧，绝不影响主流程
            pass

    def read(self) -> Tuple[bool, Optional["np.ndarray"]]:
        """返回下一帧（与 Camera.read 接口一致）"""
        if self._i >= len(self.frames):
            if self.loop:
                self._i = 0
            else:
                self._exhausted = True
                return False, None
        f = self.frames[self._i]
        self._i += 1
        return True, f.copy()

    def release(self) -> None:
        """接口兼容，什么都不做"""
        pass

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False
