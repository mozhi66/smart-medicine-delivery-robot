#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
# =============================================================================
# ⚠ 本文件为片段留存，内容不完整（缺少后半部分）
#    可运行部分止于预览窗口主体逻辑，后续交互章节缺失。
# =============================================================================
#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 相机实时预览 + 码识别叠加 (camera_preview.py)
================================================================================
职责：把相机画面**实时显示在屏幕上**，并把识别到的二维码直接框出来、
把内容写在画面上。用来回答三个问题：

    · 摄像头到底拍到了什么？（取景范围对不对、有没有对焦、亮不亮）
    · 码在不在画面里？识别到了没有？
    · 现在这个参数下画面是什么样？

为什么要有这个工具
------------------
tools/camera_check.py 是"体检"——跑一遍、给结论、存一张快照，然后退出。
它解决不了"我想边看边调"。而这个工具就是那个"边看边调"的窗口。

关于一维条码
------------
本工具**只保证二维码**。一维条码（`--barcode`）是实验性开关，
本机 OpenCV 5.0 实测读不出（连标准 EAN-13 都读不到），默认关闭。
项目本身也只用二维码。

用法
----
    # 实时预览（最常用）
    python tools/camera_preview.py

    # 顺便画过检位 ROI 框（默认从 config/app.yaml 的 vision.inspect_roi 读）
    python tools/camera_preview.py --roi 340,120,600,480

    # 静态图片模式（没有摄像头也能用，用来确认"叠加层长什么样"）
    python tools/camera_preview.py --image tests/box.jpg
    # ↑ 静态模式默认**看整图**，不套用配置里的 inspect_roi
    #   （那个 ROI 是按 1280x720 过检位标的，套到任意图上会把码裁在框外）。
    #   确实想看局部再显式 --roi x,y,w,h。

    # 无图形界面（服务器/沙盒）：定时存带标注的图 + 终端打印识别结果
    python tools/camera_preview.py --no-gui --save-every 2 --frames 20

按键（窗口获得焦点时）
----------------------
    q / ESC : 退出
    s       : 存一张带标注的快照到 --out
    p / 空格: 暂停/继续（暂停时画面冻结，方便细看）
    d       : 开关码识别（关掉可以看纯画面，省 CPU）
    h       : 开关帮助条
    [ / ]   : **手动**把曝光调暗 / 调亮一档（覆盖自适应，现场微调用）
    a       : 重新跑一遍**自适应曝光**（手动调乱后想回到自动）
    r       : 重新打开相机（画面卡住/被别的软件抢走时用；也会重跑自适应曝光）

关于曝光
--------
    默认走**自适应曝光**（config: camera.auto_exposure=true）：开相机时自动
    在"够亮且曝光最短(=帧率最高)"之间找平衡，换环境/换灯不用改配置。
    [ ] 只是在此基础上手动覆盖；按 a 可随时回到自动。

注意
----
· 窗口里的中文靠 PIL + 系统 CJK 字体渲染（cv2.putText 画中文是乱码）。
  找不到字体时会自动退化成英文，不会崩。
· 无图形界面时**不会**尝试弹窗 —— 某些环境下 cv2.imshow 会直接 abort 进程。
================================================================================
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, "pc"))

# 引入 vision 包会顺带装好图片读写兼容补丁（中文路径 / cv2 文件 I/O 问题）
from vision.imgio import imread, imwrite_checked                    # noqa: E402
from vision.camera import Camera, color_cast, cast_verdict          # noqa: E402
from vision.gui import TextRenderer, gui_available, find_cjk_font   # noqa: E402

DEFAULT_CONFIG = os.path.join(_ROOT, "config", "app.yaml")
DEFAULT_OUT = os.path.join(_ROOT, "output", "camera_preview")

# 与 camera_check.py 保持一致，别处改这里也改
DARK_BLACK, DARK_LOW = 8, 45
BRIGHT_OK, BRIGHT_HOT = 205, 240
TARGET_LO, TARGET_HI = 90, 160

# 配色（BGR）
C_OK = (80, 220, 80)        # 绿：识别成功
C_WARN = (60, 200, 255)     # 橙：警告
C_BAD = (80, 80, 255)       # 红：异常
C_INFO = (240, 220, 120)    # 青：一般信息
C_ROI = (0, 220, 255)       # 黄：ROI 框
C_QR = (80, 255, 80)        # 亮绿：二维码框
C_BAR = (255, 160, 60)      # 蓝：条形码框


# ============================================================================
# 一、一维条码检测（可选，默认关闭）
# ============================================================================

class BarcodeDetector:
    """
    一维条码（EAN/UPC/Code128 等）检测封装 —— **实验性，默认关闭**。

    ⚠️ 未验证警告
    -------------
    本机 OpenCV 5.0.0 的 `cv2.barcode.BarcodeDetector` 实测**读不出任何条码**：
    用规范生成的 EAN-13（含标准测试码 5901234123457，校验位正确）在
    模块宽度 2/3/5/6/8 px、静区 12/20 模块、各种缩放与高斯模糊下，
    `detect()` 一律返回 False，`detectAndDecode()` 一律返回空串。
    换灰度输入、`setDetectorScales()` 也都无效。

    所以**不要指望它一定有效** —— 这是"顺手加上的能力"，不是承诺。
    默认不启用；想看就加 `--barcode`。

    项目本身只用二维码（见 `pc/vision/qr_detector.py`，
    `tools/make_qr_labels.py` 生成的也是 QR），条码不是必需品。
    `cv2.barcode` 属于 opencv-contrib，缺失时本类自动降级为空实现。
    """

    def __init__(self):
        self._det = None
        try:
            if hasattr(cv2, "barcode") and hasattr(cv2.barcode, "BarcodeDetector"):
                self._det = cv2.barcode.BarcodeDetector()
        except Exception:
            self._det = None

    @property
    def available(self) -> bool:
        return self._det is not None

    def detect(self, frame: "np.ndarray") -> List[Tuple[str, "np.ndarray"]]:
        """
        返回 [(文本, 四点坐标 ndarray(4,2)), ...]；没检测到返回空列表。

        兼容不同 OpenCV 版本的返回元组长度（4.x 带 retval，5.x 不带）。
        """
        if self._det is None or frame is None:
            return []
        try:
            r = self._det.detectAndDecode(frame)
        except Exception:
            return []
        if not r:
            return []

        # 4.x: (retval, decoded_info, decoded_type, points)
        # 5.x: (decoded_info, decoded_type, points)
        if len(r) >= 4:
            info, _types, pts = r[1], r[2], r[3]
        elif len(r) == 3:
            info, _types, pts = r[0], r[1], r[2]
        else:
            return []

        out: List[Tuple[str, "np.ndarray"]] = []
        if info is None or pts is None:
            return []
        try:
            for i, text in enumerate(info):
                if not text:
                    continue
                out.append((str(text), np.asarray(pts[i], dtype=np.float32)))
        except Exception:
            return out
        return out


# ============================================================================
# 二、叠加层绘制
# ============================================================================

def _brightness(frame: "np.ndarray") -> float:
    """灰度均值（0~255）"""
    if frame is None:
        return 0.0
    if frame.ndim == 2:
        return float(frame.mean())
    return float(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean())


def _brightness_tag(b: float) -> Tuple[str, Tuple[int, int, int]]:
    """亮度 → (中文判定, 颜色)"""
    if b < DARK_BLACK:
        return "全黑", C_BAD
    if b < DARK_LOW:
        return "偏暗", C_WARN
    if b > BRIGHT_HOT:
        return "过曝", C_BAD
    if b > BRIGHT_OK:
        return "偏亮", C_WARN
    if TARGET_LO <= b <= TARGET_HI:
        return "正常", C_OK
    return "可用", C_INFO


def draw_roi(frame: "np.ndarray", roi: Optional[Sequence[int]]) -> None:
    """画过检位 ROI 框（黄色虚线 + 角标）"""
    if not roi:
        return
    x, y, w, h = [int(v) for v in roi]
    H, W = frame.shape[:2]
    x2, y2 = min(x + w, W - 1), min(y + h, H - 1)
    x, y = max(0, x), max(0, y)

    # 虚线：每 14 像素画 8 像素
    step, dash = 14, 8
    for px in range(x, x2, step):
        cv2.line(frame, (px, y), (min(px + dash, x2), y), C_ROI, 2)
        cv2.line(frame, (px, y2), (min(px + dash, x2), y2), C_ROI, 2)
    for py in range(y, y2, step):
        cv2.line(frame, (x, py), (x, min(py + dash, y2)), C_ROI, 2)
        cv2.line(frame, (x2, py), (x2, min(py + dash, y2)), C_ROI, 2)


def draw_qr_results(frame: "np.ndarray", results: List, offset: Tuple[int, int],
                    tr: TextRenderer) -> None:
    """
    把 QRResult 列表画到 frame 上。

    注意：`QRResult.cx/cy/polygon` 是**相对传入图像**的坐标（即 ROI 内坐标），
    传了 roi 时必须加上偏移才能画在整帧的正确位置 —— 这是很容易踩的坑。
    """
    ox, oy = offset
    for r in results:
        poly = getattr(r, "polygon", None)
        if poly is not None and len(poly) >= 4:
            pts = np.asarray(poly, dtype=np.float32).reshape(-1, 2) + np.array([ox, oy])
            cv2.polylines(frame, [pts.astype(np.int32)], True, C_QR, 3)
            anchor_x = int(pts[:, 0].min())
            anchor_y = int(pts[:, 1].min())
        else:
            half = float(getattr(r, "width", 60)) / 2.0
            cx, cy = r.cx + ox, r.cy + oy
            anchor_x, anchor_y = int(cx - half), int(cy - half)
            cv2.rectangle(frame, (anchor_x, anchor_y),
                          (int(cx + half), int(cy + half)), C_QR, 3)

        label = "%s  [%s x%d]" % (r.code, r.backend, getattr(r, "votes", 1))
        ty = max(0, anchor_y - tr.size - 12)
        tr.draw(frame, [label], (anchor_x, ty), color=C_QR, bg=True)


def draw_bar_results(frame: "np.ndarray", bars: List[Tuple[str, "np.ndarray"]],
                     tr: TextRenderer) -> None:
    """把一维条码结果画到 frame 上"""
    for text, pts in bars:
        p = np.asarray(pts, dtype=np.float32).reshape(-1, 2)
        cv2.polylines(frame, [p.astype(np.int32)], True, C_BAR, 3)
        ax, ay = int(p[:, 0].min()), int(p[:, 1].min())
        tr.draw(frame, ["BAR: " + text], (ax, max(0, ay - tr.size - 12)),
                color=C_BAR, bg=True)


def draw_hud(frame: "np.ndarray", tr: TextRenderer, lines: List[str],
             colors: Optional[List[Tuple[int, int, int]]] = None) -> None:
    """左上角信息面板：逐行画，每行可带自己的颜色"""
    x, y = 12, 12
    for i, text in enumerate(lines):
        col = (colors[i] if colors and i < len(colors) else C_INFO)
        y = tr.draw(frame, [text], (x, y), color=col, bg=True, alpha=0.95)
        y += 4


def draw_hint_bar(frame: "np.ndarray", tr: TextRenderer, text: str) -> None:
    """底部按键提示条（半透明黑底）"""
    h, w = frame.shape[:2]
    tw, th = tr.measure(text)
    y0 = h - th - 14
    overlay = frame[y0 - 6: h, 0: min(w, tw + 28)].copy()
    overlay[:] = (18, 18, 18)
    cv2.addWeighted(overlay, 0.72, frame[y0 - 6: h, 0: min(w, tw + 28)], 0.28, 0,
                    frame[y0 - 6: h, 0: min(w, tw + 28)])
    tr.draw(frame, [text], (14, y0), color=(215, 215, 215))


# ============================================================================
# 三、识别
# ============================================================================

def make_detector(prefer: str = "auto"):
    """构造项目的 QRDetector（失败返回 None，预览不因识别器缺失而挂掉）"""
    try:
        from vision.qr_detector import QRDetector
        return QRDetector(prefer=prefer)
    except Exception as e:
        print("[预览] 二维码识别器不可用: %s" % e)
        return None


def detect_all(frame: "np.ndarray", qr_det, bar_det: BarcodeDetector,
               roi: Optional[Sequence[int]],
               enable_barcode: bool = False) -> Tuple[List, List, Tuple[int, int]]:
    """
    在帧（或 ROI 内）识别二维码与条形码。

    返回 (qr结果列表, 条码结果列表, 坐标偏移)

    enable_barcode 默认 False —— 一维条码检测在本机实测无效，见 BarcodeDetector 的说明。
    """
    off = (0, 0)
    target = frame
    if roi:
        x, y, w, h = [int(v) for v in roi]
        H, W = frame.shape[:2]
        x, y = max(0, x), max(0, y)
        x2, y2 = min(x + w, W), min(y + h, H)
        if x2 > x and y2 > y:
            target = frame[y:y2, x:x2]
            off = (x, y)

    qrs = []
    if qr_det is not None:
        try:
            qrs = qr_det.detect(target)
        except Exception:
            qrs = []

    bars = bar_det.detect(target) if (enable_barcode and bar_det.available) else []
    return qrs, bars, off


# ============================================================================
# 四、运行模式
# ============================================================================

def _build_camera(cfg_cam: dict, args) -> Camera:
    from vision.camera import parse_backend
    idx = args.camera if args.camera is not None else int(cfg_cam.get("index", 0))
    return Camera(index=idx,
                  width=args.width or int(cfg_cam.get("width", 1280)),
                  height=args.height or int(cfg_cam.get("height", 720)),
                  fps=args.fps or int(cfg_cam.get("fps", 30)),
                  exposure=(args.exposure if args.exposure is not None
                            else cfg_cam.get("exposure", -1)),
                  backend=parse_backend(cfg_cam.get("backend")),
                  auto_wb=cfg_cam.get("auto_wb", True),
                  wb_blue_u=cfg_cam.get("wb_blue_u"),
                  wb_red_u=cfg_cam.get("wb_red_u"),
                  wb_settle_frames=cfg_cam.get("wb_settle_frames", 45),
                  # 自适应曝光：配置开启后，开相机时自动选"够亮且帧率最高"的曝光。
                  # 显式传 --exposure 时以手动值为准（关掉自适应）。
                  auto_exposure=(cfg_cam.get("auto_exposure", False)
                                 and args.exposure is None),
                  ae_target_lo=cfg_cam.get("ae_target_lo", 90),
                  ae_target_hi=cfg_cam.get("ae_target_hi", 160),
                  ae_min=cfg_cam.get("ae_min", -13),
                  ae_max=cfg_cam.get("ae_max", 0),
                  ae_settle_frames=cfg_cam.get("ae_settle_frames", 3))


def _set_exposure(cam: Camera, value: float) -> None:
    """运行中改曝光（不重开相机，改完丢几帧让参数生效）"""
    try:
        cam.cap.set(cv2.CAP_PROP_EXPOSURE, float(value))
        cam.exposure = float(value)
        for _ in range(3):
            cam.read()
    except Exception:
        pass


def _hud_lines(tr: TextRenderer, frame, fps: float, cam: Camera, det_on: bool,
               n_qr: int, n_bar: int, paused: bool, extra: str = "") -> Tuple[List[str], List[Tuple[int, int, int]]]:
    """组装左上角 HUD 的文字与配色"""
    b = _brightness(frame)
    btag, bcol = _brightness_tag(b)
    c = color_cast(frame)
    ctag, clevel = cast_verdict(c[3])
    ccol = {"ok": C_OK, "warn": C_WARN, "bad": C_BAD}.get(clevel, C_INFO)

    exp = "自动" if cam.exposure is None else "%g" % cam.exposure
    lines = [
        "分辨率 %dx%d   帧率 %.1f fps" % (frame.shape[1], frame.shape[0], fps),
        "亮度 %.1f / 255  [%s]" % (b, btag),
        "偏色 R-B %+.1f  [%s]" % (c[3], "正常" if clevel == "ok" else ctag[:4]),
        "曝光 %s   后端 %s" % (exp, "DSHOW" if cam.backend_used else "auto"),
    ]
    colors = [C_INFO, bcol, ccol, C_INFO]

    if det_on:
        if n_qr or n_bar:
            lines.append("识别: 二维码 %d 个%s"
                         % (n_qr, (" · 一维条码 %d 个" % n_bar) if n_bar else ""))
            colors.append(C_OK)
        else:
            lines.append("识别: 未找到码 —— 把码放进取景框内")
            colors.append(C_WARN)
    else:
        lines.append("识别: 已关闭（按 d 打开）")
        colors.append(C_INFO)

    if paused:
        lines.append("⏸ 已暂停（按空格继续）")
        colors.append(C_WARN)
    if extra:
        lines.append(extra)
        colors.append(C_WARN)
    return lines, colors


def _save(frame, out_dir: str, tag: str = "preview") -> Optional[str]:
    try:
        os.makedirs(out_dir, exist_ok=True)
        fp = os.path.join(out_dir, "%s_%s.jpg" % (tag, time.strftime("%H%M%S")))
        imwrite_checked(fp, frame)
        return fp
    except Exception as e:
        print("[预览] 存图失败: %s" % e)
        return None


# ---------------------------------------------------------------------------

def run_gui(cam: Camera, qr_det, bar_det: BarcodeDetector,
           aim_roi, det_roi, args) -> int:
    """实时窗口模式（交互）"""
    win = "智能药箱 - 相机预览"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    if args.scale and args.scale != 1.0:
        cv2.resizeWindow(win, int(cam.width * args.scale), int(cam.height * args.scale))

    tr = TextRenderer(size=args.font_size)
    print()
    print("=" * 74)
    print(" 相机预览已启动。窗口标题栏需要获得焦点，按键才生效。")
    print("   q/ESC 退出 · s 存图 · p 暂停 · d 开关识别 · [ ] 手动调曝光 · a 自动曝光 · r 重开相机 · h 帮助")
    print("   说明：本工具只做『实时预览 + 识别显示』，不控制机械臂、也不语音播报。")
    print("         要听到取药语音播报请运行 pc/main.py（PyCharm 配置 ⑧ 图片回放 / ⑨ 真摄像头，加 --real-tts）。")
    if not tr.has_cjk:
        print(" ⚠ 没找到中文字体，窗口里的中文会显示成方块/英文。")
    print("=" * 74)
    print()

    det_on = True
    paused = False
    show_help = True
    frozen = None
    t_prev = time.time()
    fps = 0.0
    n_qr = n_bar = 0
    last_qr_text = ""
    note = ""

    while True:
        if paused and frozen is not None:
            frame = frozen.copy()
        else:
            ok, frame = cam.read()
            if not ok or frame is None:
                # 读不到帧：等一会儿再试，别把窗口卡死
                time.sleep(0.1)
                continue
            frozen = frame.copy()
            now = time.time()
            dt = now - t_prev
            t_prev = now
            if dt > 0:
                # 指数平滑，避免数字乱跳
                inst = 1.0 / dt
                fps = inst if fps <= 0 else (fps * 0.7 + inst * 0.3)

            if det_on:
                qrs, bars, off = detect_all(frame, qr_det, bar_det, det_roi,
                                            enable_barcode=args.barcode)
                n_qr, n_bar = len(qrs), len(bars)
                draw_qr_results(frame, qrs, off, tr)
                draw_bar_results(frame, bars, tr)
                if qrs:
                    top = qrs[0]
                    if top.code != last_qr_text:
                        last_qr_text = top.code
                        print("[识别] 二维码: %s  (backend=%s, votes=%d)"
                              % (top.code, top.backend, getattr(top, "votes", 1)))
                for text, _p in bars:
                    print("[识别] 条形码: %s" % text)
            else:
                n_qr = n_bar = 0

        draw_roi(frame, aim_roi)
        lines, colors = _hud_lines(tr, frame, fps, cam, det_on, n_qr, n_bar,
                                   paused, note)
        draw_hud(frame, tr, lines, colors)
        if show_help:
            draw_hint_bar(frame, tr,
                          "q 退出 | s 存图 | p 暂停 | d 识别开关 | [ 调暗 ] 调亮(手动) | a 自动曝光 | r 重开 | h 帮助")
        if not tr.has_cjk:
            cv2.putText(frame, "no CJK font", (12, frame.shape[0] - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_WARN, 2)

        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):                      # q / ESC
            break
        elif key == ord("s"):
            fp = _save(frame, args.out, "snapshot")
            note = ("已存图: %s" % os.path.basename(fp)) if fp else "存图失败"
            print("[预览] %s" % note)
        elif key in (ord("p"), ord(" ")):
            paused = not paused
            note = ""
        elif key == ord("d"):
            det_on = not det_on
            note = ""
        elif key == ord("h"):
            show_help = not show_help
        elif key == ord("["):
            if cam.exposure is not None:
                _set_exposure(cam, float(cam.exposure) - 1)
                note = "曝光 -> %g（更暗）" % cam.exposure
                print("[预览] %s" % note)
        elif key == ord("]"):
            if cam.exposure is not None:
                _set_exposure(cam, float(cam.exposure) + 1)
                note = "曝光 -> %g（更亮）" % cam.exposure
                print("[预览] %s" % note)
        elif key == ord("a"):
            # 重新跑一遍自适应曝光（[ ] 手动调过之后想回到自动，按这个）
            if cam.cap is not None:
                res = cam.auto_expose()
                if res is not None:
                    note = "已自适应曝光 -> %g（亮度 %.0f）" % (cam.exposure, res[1])
                else:
                    note = "自适应曝光失败：读不到画面"
                print("[预览] %s" % note)
        elif key == ord("r"):
            cam.release()
            if cam.open():
                note = "相机已重开"
            else:
                note = "重开失败，检查 USB/占用"
            print("[预览] %s" % note)

    cv2.destroyAllWindows()
    return 0


# ---------------------------------------------------------------------------
