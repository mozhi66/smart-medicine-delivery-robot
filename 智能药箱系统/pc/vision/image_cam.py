#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 图片回放相机 (image_cam.py)
================================================================================
职责：把一张或多张图片伪装成相机 —— read() 轮流返回这些图片。

用途：把"识别链路"钉死在已知输入上，排除光照/手抖/帧率干扰。
    python pc/main.py --cam-image output/qr_labels/label_MED003.png
    python pc/main.py --cam-image MED003        # 自动去 output/qr_labels/ 找

为什么要有它：真机联调时"扫不到码"可能是相机/光照问题，也可能是识别链
本身退化。用图片回放可以把后者单独拎出来验证 —— 同一张图，识别结果
必须 100% 稳定；不稳定就是识别链的问题，与硬件无关。

接口与 vision.camera.Camera 对齐：read() / open() / close() / describe()。
state_machine 用 type(camera).__name__ == "ImageCamera" 识别回放模式，
扫码前不再等"举标签倒计时"（画面已钉死，等了也没意义）。
================================================================================
"""

from __future__ import annotations

import os
from typing import List, Optional, Tuple

import numpy as np

try:
    import cv2
    _HAS_CV2 = True
except Exception:                                     # pragma: no cover
    _HAS_CV2 = False


class ImageCamera:
    """图片回放"相机"。read() 按帧序循环返回预置图片。"""

    def __init__(self, images: List[np.ndarray], loop: bool = True,
                 fps: float = 15.0):
        """
        参数
        ----
        images : 预置画面列表（BGR ndarray）；空列表会在 read() 里给黑帧
        loop   : True = 播完从头再来（多图轮播）
        fps    : 名义帧率（仅用于 describe 展示，实际 read 不节流）
        """
        self.images: List[np.ndarray] = [im for im in (images or [])
                                         if im is not None and getattr(im, "size", 0) > 0]
        self.loop = bool(loop)
        self.fps = float(fps)
        self._idx = 0
        self._opened = False

    # ------------------------------------------------------------------
    @classmethod
    def from_paths(cls, paths: List[str], loop: bool = True) -> "ImageCamera":
        """从文件路径构造。中文路径安全（imdecode），坏图跳过并提示。"""
        imgs: List[np.ndarray] = []
        for p in paths or []:
            im = _imread_unicode(str(p))
            if im is None:
                print("[ImageCamera] 读不了图片（跳过）: %s" % p)
                continue
            imgs.append(im)
        return cls(imgs, loop=loop)

    # ------------------------------------------------------------------
    def open(self) -> bool:
        self._opened = True
        return True

    def close(self) -> None:
        self._opened = False

    def release(self) -> None:
        self.close()

    def is_opened(self) -> bool:
        return self._opened

    def describe(self) -> str:
        return "ImageCamera(%d 张, loop=%s)" % (len(self.images), self.loop)

    # ------------------------------------------------------------------
    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """返回下一张预置图；单图恒返回同一张。空列表给 640x480 黑帧。"""
        if not self.images:
            black = np.zeros((480, 640, 3), np.uint8)
            return True, black
        im = self.images[self._idx]
        if self.loop or len(self.images) == 1:
            self._idx = (self._idx + 1) % len(self.images)
        else:
            self._idx = min(self._idx + 1, len(self.images) - 1)
        return True, im.copy()

    # 兼容部分代码用 cap.read() 的习惯
    def __getattr__(self, name):
        if name == "cap":
            return None
        raise AttributeError(name)


def _imread_unicode(path: str) -> Optional[np.ndarray]:
    """
    中文路径安全的 imread（Windows 下 cv2.imread 走 fopen 常静默失败）。
    优先用 imgio 接管的 imdecode；没有就用 imencode + fromfile 的经典解法。
    """
    if not _HAS_CV2 or not os.path.isfile(path):
        return None
    try:
        buf = np.fromfile(path, dtype=np.uint8)
        if buf.size == 0:
            return None
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return img
    except Exception:
        try:
            with open(path, "rb") as f:
                data = f.read()
            return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        except Exception:
            return None
