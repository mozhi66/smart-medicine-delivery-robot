#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 二维码识别模块 (qr_detector.py)
================================================================================
职责：在图像中找出二维码并返回"内容 + 位置"。

设计要点
--------
1) 三级解码后端，按能力自动降级：
     wechat  : cv2.wechat_qrcode（opencv-contrib，需额外 4 个模型文件）
               小图/模糊/倾斜/光照变化下鲁棒性最高，首选
     aruco   : cv2.QRCodeDetectorAruco（OpenCV >=4.3 主库自带）
               基于 ArUco 定位，比内置好，次选
     builtin : cv2.QRCodeDetector（一定有）
               人人都有但失败率最高，兜底
   逐级降级 = 换电脑/换 OpenCV 版本不会直接跑不起来。

2) 提供 robust_scan()：连续采多帧投票，票数不足返回 None。
   单帧识别有偶发失败，投票是低成本高收益的稳定性手段，强烈建议用。

3) 支持 ROI：只在给定矩形内解码，速度更快、误检更少。
================================================================================
"""

from __future__ import annotations

import os
import time
from collections import Counter
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class QRResult:
    """一次二维码识别的结果"""
    code: str                      # 二维码文本内容，如 'MED003'
    cx: float                      # 二维码中心 x（相对传入图像的坐标，未加 ROI 偏移）
    cy: float                      # 二维码中心 y
    width: float                   # 二维码外接宽度（像素，可用于粗估距离）
    polygon: Optional[np.ndarray]  # 四个角点 (4,2)，失败时为 None
    backend: str                   # 读出它的后端名，融合时为 'aruco+builtin' 形式
    votes: int = 1                 # 有多少个后端读出了它（>=2 说明多后端一致，非常可信）

    def __repr__(self) -> str:
        return "QRResult(%s, at=(%.0f,%.0f), w=%.0f, via=%s, votes=%d)" % (
            self.code, self.cx, self.cy, self.width, self.backend, self.votes)


class QRDetector:
    """
    多级二维码解码器。

    典型用法
    --------
        det = QRDetector()                     # 自动挑选最佳后端
        print(det.backend_name)                # 看看实际用了哪个
        res = det.detect(frame)                # 单帧，返回 List[QRResult]
        res = det.detect(frame, roi=(x,y,w,h)) # 只在 ROI 内找
        one = det.robust_scan(camera, n=5)     # 多帧投票，返回最优 QRResult 或 None
    """

    # WeChat QR 需要的四个模型文件名（放在 models/wechat_qr/ 下）
    WECHAT_FILES = ("detect.prototxt", "detect.caffemodel",
                    "sr.prototxt", "sr.caffemodel")

    def __init__(self, prefer: str = "auto", model_dir: Optional[str] = None):
        """
        参数
        ----
        prefer    : 'auto' 自动挑最强 | 'wechat' | 'aruco' | 'builtin' 强制指定
        model_dir : WeChat QR 模型目录，默认取本文件同级 ../models/wechat_qr/
        """
        if model_dir is None:
            model_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                "models", "wechat_qr")
        self.model_dir = model_dir
        self.prefer = prefer

        # ---- 初始化【所有】可用后端，而不只是挑一个 ----
        # 原因（由 tools/vision_sim.py 的实测数据得出，不是猜测）：
        #   aruco  后端对模糊极其敏感：高斯模糊 k=3 时识别率从 8/8 掉到 2/8
        #   builtin 后端抗模糊好得多：k=3 仍有 8/8，k=7 还有 7/8
        #   但在复杂背景/严重透视下，aruco 又常常比 builtin 强
        # 没有任何单一后端在所有条件下都最优 —— 所以让它们全部上，投票决定。
        # 代价只是多几次解码调用（毫秒级），而本项目的解码频率只有 5Hz，完全可接受。
        self._backends = []          # [(name, detector, is_wechat), ...]
        for name in ("wechat", "aruco", "builtin"):
            det, is_wechat = self._make_backend(name)
            if det is not None:
                self._backends.append((name, det, is_wechat))

        if not self._backends:
            raise RuntimeError("无法初始化任何二维码解码器，请检查 OpenCV 安装")

        if prefer in ("wechat", "aruco", "builtin"):
            # 强制只用指定后端（调试/对比用）
            self._backends = [b for b in self._backends if b[0] == prefer] or self._backends

        self.backend_name = "fusion:" + "+".join(b[0] for b in self._backends)
        self._det = self._backends[0][1]        # 兼容旧的直接访问
        self._wechat = self._backends[0][2]

    # ------------------------------------------------------------------
    def _make_backend(self, name: str):
        """
        创建单个解码后端。

        返回
        ----
        (detector, is_wechat) 或 (None, False) 表示该后端不可用
        """
        try:
            if name == "wechat":
                if not hasattr(cv2, "wechat_qrcode_WeChatQRCode"):
                    return None, False
                paths = [os.path.join(self.model_dir, f) for f in self.WECHAT_FILES]
                if not all(os.path.isfile(p) for p in paths):
                    return None, False
                return cv2.wechat_qrcode_WeChatQRCode(*paths), True

            if name == "aruco":
                if hasattr(cv2, "QRCodeDetectorAruco"):
                    return cv2.QRCodeDetectorAruco(), False
                return None, False

            if name == "builtin":
                return cv2.QRCodeDetector(), False
        except Exception:
            return None, False
        return None, False

    # ------------------------------------------------------------------
    def _decode_with(self, det, is_wechat: bool,
                     img: "np.ndarray") -> List[Tuple[str, Optional["np.ndarray"], str]]:
        """
        用某一个后端解码，统一三种 API 的差异。

        三种后端的返回签名完全不同，这里全部归一化成 (text, polygon, backend_name)：
          wechat  : det.detectAndDecode(img) -> (list[str], list[np.ndarray])
          aruco   : det.detectAndDecode(img) -> (str, ndarray|None, ndarray|None)
          builtin : 同上；后两者都支持 detectAndDecodeMulti（OpenCV>=4.x）
        """
        out: List[Tuple[str, Optional["np.ndarray"], str]] = []

        if is_wechat:
            try:
                texts, points = det.detectAndDecode(img)
                for t, p in zip(texts or [], points or []):
                    if t:
                        out.append((t, np.asarray(p).reshape(-1, 2), "wechat"))
                return out
            except Exception:
                return out

        name = "builtin"
        try:
            if det.__class__.__name__ == "QRCodeDetectorAruco":
                name = "aruco"
        except Exception:
            pass

        # 先试多码版本
        try:
            ok, infos, points, _ = det.detectAndDecodeMulti(img)
            if ok:
                for t, p in zip(infos, points):
                    if t:
                        out.append((t, np.asarray(p).reshape(-1, 2), name))
                if out:
                    return out
        except Exception:
            pass

        # 回退单码版本
        try:
            text, pts, _ = det.detectAndDecode(img)
            if text:
                out.append((text, np.asarray(pts).reshape(-1, 2)
                            if pts is not None else None, name))
        except Exception:
            pass
        return out

    # ------------------------------------------------------------------
    def _decode_fusion(self, img: "np.ndarray") -> List[Tuple[str, Optional["np.ndarray"], int, str]]:
        """
        用所有可用后端解码并投票融合。

        返回
        ----
        [(text, polygon, votes, backends_str), ...]
        votes = 有多少个后端读出了这个 text（去重后）
        """
        found: Dict[str, List] = {}
        order: List[str] = []
        for name, det, is_wechat in self._backends:
            try:
                for text, poly, bn in self._decode_with(det, is_wechat, img):
                    if text not in found:
                        found[text] = []
                        order.append(text)
                    found[text].append((poly, bn))
            except Exception:
                continue

        out = []
        for text in order:
            items = found[text]
            # 选该 text 下"最宽"的多边形作为代表
            best_poly = None
            best_w = -1.0
            for poly, _bn in items:
                if poly is not None and len(poly) >= 2:
                    w = float(np.max(poly[:, 0]) - np.min(poly[:, 0]))
                    if w > best_w:
                        best_w, best_poly = w, poly
            backends_str = "+".join(sorted(set(bn for _p, bn in items)))
            out.append((text, best_poly, len(items), backends_str))
        return out

    # ------------------------------------------------------------------
    def detect(self, frame: "np.ndarray",
               roi: Optional[Sequence[int]] = None) -> List[QRResult]:
        """
        在图像（或 ROI 子图）中识别所有二维码。

        参数
        ----
        frame : BGR 或灰度图
        roi   : (x, y, w, h) 感兴趣区域；给出时只在该区域内解码，
                返回的坐标已换算回原图坐标系

        返回
        ----
        List[QRResult]，按二维码在图中的宽度从大到小排序（大的更近更可信）
        """
        if frame is None or frame.size == 0:
            return []

        ox = oy = 0
        if roi is not None:
            x, y, w, h = [int(v) for v in roi]
            H, W = frame.shape[:2]
            x = max(0, min(x, W - 1)); y = max(0, min(y, H - 1))
            w = max(1, min(w, W - x));  h = max(1, min(h, H - y))
            frame = frame[y:y + h, x:x + w]
            ox, oy = x, y

        # 灰度更快；二维码本身只吃亮度信息
        gray = frame if (frame.ndim == 2) else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results: List[QRResult] = []
        for text, poly, votes, bns in self._decode_fusion(gray):
            if poly is not None and len(poly) >= 3:
                cx = float(np.mean(poly[:, 0])) + ox
                cy = float(np.mean(poly[:, 1])) + oy
                xs = poly[:, 0]
                width = float(np.max(xs) - np.min(xs))
            else:
                cx = cy = -1.0
                width = 0.0
            results.append(QRResult(code=str(text), cx=cx, cy=cy,
                                    width=width,
                                    polygon=(poly + np.array([ox, oy]) if poly is not None else None),
                                    backend=bns, votes=int(votes)))
        # 排序：先按后端投票数（多后端一致 = 更可信），再按宽度（大 = 近 = 清晰）
        results.sort(key=lambda r: (-r.votes, -r.width))
        return results

    # ------------------------------------------------------------------
    def detect_best(self, frame: "np.ndarray",
                    roi: Optional[Sequence[int]] = None) -> Optional[QRResult]:
        """同 detect()，但只返回最大的那一个（没有则返回 None）"""
        rs = self.detect(frame, roi)
        return rs[0] if rs else None

    # ------------------------------------------------------------------
    def robust_scan(self, camera, n: int = 5, interval: float = 0.06,
                    min_votes: int = 2,
                    roi: Optional[Sequence[int]] = None,
                    on_frame=None,
                    timeout_s: float = 0.0,
                    accept_votes: Optional[int] = None) -> Optional[QRResult]:
        """
        连续采帧做投票识别，抗单帧偶发失败。

        参数
        ----
        camera    : 任何提供 read() -> (ok, frame) 的对象（Camera / cv2.VideoCapture）
        n         : 采样帧数上限。timeout_s>0 时只是安全上限，真正结束看时间/票数
        interval  : 帧间隔秒数（给相机自动曝光/机械稳定留时间）
        min_votes : 最少多少票才采信（默认 2，设 1 等于关闭投票）
        roi       : 只在 ROI 内识别；None = 整帧（与 tools/camera_preview.py 一致，手举码更稳）
        on_frame  : 可选回调 on_frame(frame, result, idx, total)，每读到一帧调用一次。
                    给"边扫边显示窗口"用；result 为 None 表示这一帧没读到码。
                    **回调内部抛异常会被吞掉**，绝不影响扫码主流程。
        timeout_s : >0 时按时间扫（给人举码留窗口），同一码集满 accept_votes 票提前结束。
        accept_votes : 提前采信所需票数；默认等于 min_votes。

        返回
        ----
        得票最高且 >= min_votes 的 QRResult；否则 None
        """
        votes: Counter = Counter()
        best_result: Optional[QRResult] = None
        if accept_votes is None:
            accept_votes = min_votes
        accept_votes = max(1, int(accept_votes))

        use_timeout = float(timeout_s or 0.0) > 0
        if use_timeout:
            deadline = time.time() + float(timeout_s)
            # 时间模式：n 只是安全帽，按 interval 估算显示用的 total
            total = max(int(n), int(float(timeout_s) / max(float(interval), 0.01)) + 20)
        else:
            deadline = None
            total = max(1, int(n))

        i = 0
        while i < total:
            if deadline is not None and time.time() >= deadline:
                break
            t_frame = time.time()
            ok, frame = camera.read()
            if not ok or frame is None:
                time.sleep(max(0.0, float(interval or 0.0)))
                continue
            r = self.detect_best(frame, roi)
            if r is not None:
                votes[r.code] += 1
                # 保存该 code 里"最宽"的一次结果，宽度大 = 离得近 = 更可信；
                # 若另一码票数反超，则改跟踪新领先码。
                if best_result is None or r.code == best_result.code:
                    if best_result is None or r.width > best_result.width:
                        best_result = r
                elif votes[r.code] > votes.get(best_result.code, 0):
                    best_result = r
                # 同一码已够票：立刻采信，不必扫满（给人的体感是"对上就走"）
                if votes[r.code] >= accept_votes:
                    out = r if (best_result is None or best_result.code != r.code) else best_result
                    if on_frame is not None:
                        try:
                            on_frame(frame, r, i, total)
                        except Exception:
                            pass
                    return out
            if on_frame is not None:
                try:
                    on_frame(frame, r, i, total)
                except Exception:
                    pass          # 显示失败绝不能拖垮识别
            i += 1
            # ★ 只补足 interval 的"剩余时间"，不要在识别/显示耗时之外再睡满
            #   interval —— 否则 50ms 固定 sleep 会把预览压到 ~10fps。
            if interval:
                remain = float(interval) - (time.time() - t_frame)
                if remain > 0:
                    time.sleep(remain)

        if not votes:
            return None
        top, cnt = votes.most_common(1)[0]
        if cnt < min_votes:
            return None
        if best_result is not None and best_result.code == top:
            return best_result
        return QRResult(code=top, cx=-1, cy=-1, width=0,
                        polygon=None, backend=self.backend_name)


# ============================================================================
# 便捷函数：生成二维码图片（做标签用，也用于仿真自测）
# ============================================================================
def make_qr_image(text: str, size: int = 160, quiet_zone_ratio: float = 0.18,
                  ecc_level: int = 3) -> Optional["np.ndarray"]:
    """
    用 OpenCV 内置编码器生成一张二维码图片（白底黑码）。

    参数
    ----
    text            : 编码内容，建议短一些（如 MED003），版本低、好识别
    size            : 二维码主体边长（像素，不含静区）
    quiet_zone_ratio: 静区（四周白边）相对 size 的比例，建议 >=0.15
    ecc_level       : 纠错等级 0=L(7%) 1=M(15%) 2=Q(25%) 3=H(30%)
                      药盒标签强烈建议用 3(H)，抗污抗遮挡

    返回
    ----
    灰度图 uint8（255=白），失败返回 None
    """
    if not hasattr(cv2, "QRCodeEncoder"):
        return None
    try:
        enc = cv2.QRCodeEncoder_create()
        # OpenCV 4.13 的 setECCLevel 存在；老版本可能没有，忽略即可
        try:
            enc.setECCLevel(int(ecc_level))
        except Exception:
            pass
        core = enc.encode(str(text))
        if core is None or core.size == 0:
            return None
        core = cv2.resize(core, (size, size), interpolation=cv2.INTER_NEAREST)

        pad = int(size * quiet_zone_ratio)
        canvas = np.full((size + 2 * pad, size + 2 * pad), 255, np.uint8)
        canvas[pad:pad + size, pad:pad + size] = core
        return canvas
    except Exception:
        return None
