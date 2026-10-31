#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 GUI 基础设施 (gui.py)
================================================================================
职责：给扫码预览窗等 OpenCV 窗口提供两样东西：

  1) gui_available()  —— 当前环境能不能开窗口（有桌面 / cv2.imshow 可用）
  2) TextRenderer     —— 在 BGR 图上画【中文】文字（OpenCV 的 putText
                         不支持中文，必须借 PIL + TrueType 字体渲染后贴回）

TextRenderer 接口（scan_view.py 的用法约定）：
    tr = TextRenderer(size=26)
    tr.size        字号 px（int）
    tr.line_gap    行距 px
    w, h = tr.measure("文本")                 # 测量
    y = tr.draw(img, ["第一行", "第二行"], (x, y),
                color=(b,g,r), bg=True, bg_color=(25,25,30))  # 返回画完后的 y

设计要点：
· 字体按 Windows 常见字体逐个尝试（微软雅黑/黑体/宋体），全没有就退
  PIL 默认字体（中文会显示方块，但不崩）。
· 同字号渲染结果缓存（字面 → RGBA 位图），逐帧 HUD 不至于把 CPU 吃满。
· 所有 PIL 调用包 try —— 没装 pillow 只是中文变方块，主流程照跑。
================================================================================
"""

from __future__ import annotations

import os
from typing import List, Optional, Sequence, Tuple

import numpy as np

try:
    import cv2
    _HAS_CV2 = True
except Exception:                                     # pragma: no cover
    _HAS_CV2 = False

try:
    from PIL import Image, ImageDraw, ImageFont
    _HAS_PIL = True
except Exception:                                     # pragma: no cover
    _HAS_PIL = False


def gui_available() -> bool:
    """
    当前环境是否可以开 OpenCV 窗口。
    判据：cv2 可导入 + 有 DISPLAY（Windows 恒真）/ imshow 试探不炸。
    容器 / CI / 无桌面环境返回 False，调用方自动退化为无窗口模式。
    """
    if not _HAS_CV2:
        return False
    try:
        if os.name == "nt":
            return True
        if os.environ.get("DISPLAY"):
            return True
        return False
    except Exception:
        return False


# ---------------------------------------------------------------------------
# 字体定位
# ---------------------------------------------------------------------------
_FONT_CANDIDATES = [
    # Windows
    r"C:\Windows\Fonts\msyh.ttc",        # 微软雅黑
    r"C:\Windows\Fonts\msyhbd.ttc",      # 微软雅黑 粗体
    r"C:\Windows\Fonts\simhei.ttf",      # 黑体
    r"C:\Windows\Fonts\simsun.ttc",      # 宋体
    # Linux
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
]


def _find_font_path() -> Optional[str]:
    for p in _FONT_CANDIDATES:
        try:
            if os.path.isfile(p):
                return p
        except Exception:
            continue
    return None


class TextRenderer:
    """在 BGR ndarray 上渲染多行中文文本（PIL 画 RGBA，再 cv2 贴回）。"""

    _font_cache = {}      # (path, size) -> ImageFont
    _glyph_cache = {}     # (text, size, color) -> ndarray(RGBA) 渲染缓存

    def __init__(self, size: int = 26):
        self.size = int(size)
        self.line_gap = max(4, int(self.size * 0.30))
        self._font_path = _find_font_path() if _HAS_PIL else None
        self._font = self._get_font(self._font_path, self.size)

    # ------------------------------------------------------------------
    @classmethod
    def _get_font(cls, path: Optional[str], size: int):
        key = (path, int(size))
        if key in cls._font_cache:
            return cls._font_cache[key]
        font = None
        if _HAS_PIL:
            try:
                if path:
                    font = ImageFont.truetype(path, int(size))
                else:
                    font = ImageFont.load_default()
            except Exception:
                try:
                    font = ImageFont.load_default()
                except Exception:
                    font = None
        cls._font_cache[key] = font
        return font

    # ------------------------------------------------------------------
    def measure(self, text: str) -> Tuple[int, int]:
        """测量单行文本的 (宽, 高) 像素。"""
        if not _HAS_PIL or self._font is None:
            # 粗略估计：中文 ≈ size，ASCII ≈ size/2
            w = sum(self.size if ord(c) > 0x2E80 else self.size // 2
                    for c in str(text))
            return w, self.size
        try:
            try:
                bbox = self._font.getbbox(str(text))
                return int(bbox[2] - bbox[0]), int(bbox[3] - bbox[1])
            except Exception:
                w, h = self._font.getsize(str(text))
                return int(w), int(h)
        except Exception:
            return len(str(text)) * self.size // 2, self.size

    # ------------------------------------------------------------------
    def _render_line(self, text: str, color: Tuple[int, int, int]) -> Optional[np.ndarray]:
        """渲染单行文本为 RGBA ndarray（透明底）。带缓存。"""
        if not _HAS_PIL:
            return None
        key = (str(text), self.size, tuple(color))
        cached = TextRenderer._glyph_cache.get(key)
        if cached is not None:
            return cached
        try:
            font = self._font
            if font is None:
                return None
            try:
                bbox = font.getbbox(str(text))
                w = max(1, int(bbox[2] - bbox[0]))
                asc, desc = font.getmetrics()
                h = max(1, int(asc + desc))
                ox, oy = -bbox[0], 0
            except Exception:
                w, h = font.getsize(str(text))
                ox = oy = 0
            w += 4
            h += 4
            img = Image.new("RGBA", (w, h), (0, 0, 0, 0))
            d = ImageDraw.Draw(img)
            # BGR 入参 → PIL RGB
            r, g, b = int(color[2]), int(color[1]), int(color[0])
            d.text((ox + 2, oy + 2), str(text), font=font, fill=(r, g, b, 255))
            arr = np.asarray(img)                    # H,W,4 RGBA
            if TextRenderer.__name__:
                # 缓存上限保护：2048 条足够一屏 HUD + 结论，超出就清
                if len(TextRenderer._glyph_cache) > 2048:
                    TextRenderer._glyph_cache.clear()
                TextRenderer._glyph_cache[key] = arr
            return arr
        except Exception:
            return None

    # ------------------------------------------------------------------
    def draw(self, img: np.ndarray, lines: Sequence[str],
             pos: Tuple[int, int],
             color: Tuple[int, int, int] = (245, 245, 245),
             bg: bool = False,
             bg_color: Tuple[int, int, int] = (25, 25, 30)) -> int:
        """
        从 (x, y) 开始画多行文本，返回下一行的 y 坐标。

        bg=True 时给整块文本区垫半透明底条，保证在任何背景上可读。
        渲染失败（无 PIL）时退化成 cv2.putText 英文灰条，绝不抛异常。
        """
        x, y = int(pos[0]), int(pos[1])
        lines = [str(t) for t in lines if t]
        if not lines:
            return y
        H, W = img.shape[:2]

        if bg:
            # 垫底条
            try:
                widths = [self.measure(t)[0] for t in lines]
                tw = max(widths) if widths else 0
                th = len(lines) * (self.size + self.line_gap) + 8
                x0 = max(0, x - 6)
                y0 = max(0, y - 4)
                x1 = min(W - 1, x + tw + 10)
                y1 = min(H - 1, y + th)
                if _HAS_CV2 and x1 > x0 and y1 > y0:
                    band = img[y0:y1, x0:x1].copy()
                    bc = bg_color
                    overlay = np.empty_like(band)
                    overlay[:] = bc
                    img[y0:y1, x0:x1] = cv2.addWeighted(
                        overlay, 0.65, band, 0.35, 0)
            except Exception:
                pass

        cy = y
        for t in lines:
            rgba = self._render_line(t, color)
            if rgba is not None and _HAS_CV2:
                try:
                    gh, gw = rgba.shape[:2]
                    gx, gy = x, cy
                    gx = max(0, min(gx, W - 1))
                    gy = max(0, min(gy, H - 1))
                    gh2 = min(gh, H - gy)
                    gw2 = min(gw, W - gx)
                    if gh2 <= 0 or gw2 <= 0:
                        cy += self.size + self.line_gap
                        continue
                    region = img[gy:gy + gh2, gx:gx + gw2]
                    alpha = rgba[:gh2, :gw2, 3:4].astype(np.float32) / 255.0
                    fg = rgba[:gh2, :gw2, :3].astype(np.float32)
                    # RGBA 里 PIL 输出的是 RGB，直接混到 BGR 图上（红蓝互换影响小，
                    # 但为准确起见做一次通道翻转）
                    fg = fg[:, :, ::-1]
                    region[:] = (fg * alpha + region.astype(np.float32) * (1 - alpha)).astype(np.uint8)
                except Exception:
                    self._draw_fallback(img, t, x, cy)
            else:
                self._draw_fallback(img, t, x, cy)
            cy += self.size + self.line_gap
        return cy

    # ------------------------------------------------------------------
    def _draw_fallback(self, img: np.ndarray, text: str,
                       x: int, y: int) -> None:
        """无 PIL 时的兜底：cv2.putText（中文会变 ??，至少不崩）。"""
        try:
            if not _HAS_CV2:
                return
            cv2.putText(img, text.encode("ascii", "replace").decode(),
                        (int(x), int(y + self.size * 0.8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        max(0.4, self.size / 32.0),
                        (200, 200, 200), 1, cv2.LINE_AA)
        except Exception:
            pass
