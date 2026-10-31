#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 药品二维码标签生成工具 (make_qr_labels.py)
================================================================================
职责：读药品清单 -> 给每个药生成一张"可打印的标签图"（二维码 + 药名 + 分类 +
     槽位 + 风险标识），再把所有标签排版到 A4 上，输出到 output/qr_labels/。

为什么标签要这么设计
--------------------
1) 二维码内容是药品 code（如 MED003），不是药名。
   药名可能改（"感冒灵颗粒" -> "感冒灵胶囊"），code 是稳定主键；
   视觉模块认 code，再去知识库里查药名/槽位，两者解耦。
2) 纠错等级用 H（30%）。药瓶标签会被手摸脏、被胶带压住、被光照反射，
   H 级纠错是现场能不能扫出来的关键，比省那点码面积值太多。
3) 中文必须用 PIL 画。OpenCV 的 putText 只支持 ASCII 字形，中文全是 '???'，
   这是新手最容易踩的坑。这里统一走 PIL，并在找不到中文字体时降级+警告。
4) 高风险药（high_risk=true）加红框 + ⚠ 标记。
   取药时机械臂和人都会看到这张标签，"这个药别搞错"必须视觉上第一眼就能看出来。

用法
----
    # 用项目里的 config/medicines.json（不存在则用内置样例清单）
    python3.11 tools/make_qr_labels.py

    # 指定清单和输出目录，并同时保留每张单独的小图（方便一张张打印/贴瓶）
    python3.11 tools/make_qr_labels.py --medicines config/medicines.json \\
                                       --out output/qr_labels --individual

    # 只挑某几个槽位出标签（补打用）
    python3.11 tools/make_qr_labels.py --slots 1,3,5
================================================================================
"""

from __future__ import annotations

import json
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

# 让脚本能直接 import pc/ 下的模块（与 tools/vision_sim.py 保持一致）
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_ROOT, "pc"))

from vision.qr_detector import make_qr_image            # noqa: E402

# ---- 可选依赖：PIL 画中文（必需，缺失时给出安装提示并明确降级）----
try:
    from PIL import Image, ImageDraw, ImageFont
except Exception:                                        # pragma: no cover
    Image = ImageDraw = ImageFont = None

DEFAULT_MEDICINES = os.path.join("config", "medicines.json")
DEFAULT_OUT = os.path.join("output", "qr_labels")

# 内置样例清单：config/medicines.json 不存在时用它，保证工具开箱即用
SAMPLE_MEDICINES: List[dict] = [
    {"code": "MED001", "name": "感冒灵颗粒", "category": "感冒药", "slot": 1,
     "high_risk": False, "stock": 12, "note": "饭后服用"},
    {"code": "MED002", "name": "布洛芬缓释胶囊", "category": "退烧止痛", "slot": 2,
     "high_risk": False, "stock": 20, "note": "间隔6小时以上"},
    {"code": "MED003", "name": "阿莫西林胶囊", "category": "抗生素", "slot": 3,
     "high_risk": True, "stock": 8, "note": "需遵医嘱，勿随意停药"},
    {"code": "MED004", "name": "硝苯地平片", "category": "降压药", "slot": 4,
     "high_risk": True, "stock": 30, "note": "每日一次，勿漏服"},
    {"code": "MED005", "name": "铝碳酸镁咀嚼片", "category": "胃药", "slot": 5,
     "high_risk": False, "stock": 15, "note": "嚼碎后服用"},
    {"code": "MED006", "name": "氯雷他定片", "category": "抗过敏", "slot": 6,
     "high_risk": False, "stock": 10, "note": "睡前服用"},
    {"code": "MED007", "name": "复方甘草片", "category": "止咳", "slot": 7,
     "high_risk": False, "stock": 6, "note": "含服，勿超量"},
    {"code": "MED008", "name": "云南白药气雾剂", "category": "外用", "slot": 8,
     "high_risk": False, "stock": 4, "note": "仅外用"},
]

# 中文（CJK）字体候选路径，按平台排列
_FONT_CANDIDATES = (
    "C:/Windows/Fonts/msyh.ttc",        # Windows 微软雅黑
    "C:/Windows/Fonts/msyhbd.ttc",
    "C:/Windows/Fonts/simhei.ttf",      # 黑体
    "C:/Windows/Fonts/simsun.ttc",      # 宋体
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
    "/System/Library/Fonts/PingFang.ttc",
    "/Library/Fonts/Arial Unicode.ttf",
)

# A4 @ 300dpi（纸张 210x297mm）
A4_W, A4_H = 2480, 3508


@dataclass
class Medicine:
    """一条药品记录（字段与 config/medicines.json 对齐）"""
    code: str                                  # 二维码内容 / 主键，如 MED003
    name: str = ""                             # 药品名（中文）
    category: str = ""                         # 分类，如 感冒药
    slot: int = 0                              # 所在槽位号
    high_risk: bool = False                    # 是否高风险（加红框 + ⚠）
    stock: int = 0                             # 库存（仅打印到标签上参考）
    note: str = ""                             # 用药提示

    @staticmethod
    def from_dict(d: dict) -> "Medicine":
        """
        从 JSON 字典构造 Medicine，缺字段自动补默认值。

        参数
        ----
        d : medicines.json 里的一条记录

        返回
        ----
        Medicine 实例
        """
        return Medicine(
            code=str(d.get("code", "")).strip(),
            name=str(d.get("name", "")).strip(),
            category=str(d.get("category", "")).strip(),
            slot=int(d.get("slot", 0) or 0),
            high_risk=bool(d.get("high_risk", False)),
            stock=int(d.get("stock", 0) or 0),
            note=str(d.get("note", "")).strip(),
        )


# ===========================================================================
# 字体
# ===========================================================================
def find_cjk_font() -> Optional[str]:
    """
    找一个能显示中文的字体文件。

    返回
    ----
    字体路径；找不到返回 None（调用方会退化为 PIL 默认字体并警告）
    """
    for p in _FONT_CANDIDATES:
        if os.path.isfile(p):
            return p
    if sys.platform.startswith("linux"):
        for root in ("/usr/share/fonts", os.path.expanduser("~/.fonts")):
            if not os.path.isdir(root):
                continue
            for dirpath, _dirs, files in os.walk(root):
                for f in files:
                    low = f.lower()
                    if low.endswith((".ttf", ".ttc", ".otf")) and \
                            ("cjk" in low or "wqy" in low or "droidsansfallback" in low):
                        return os.path.join(dirpath, f)
    return None


_FONT_CACHE: Dict[int, object] = {}


def get_font(size: int):
    """
    按字号获取 PIL 字体对象（带缓存）。

    参数
    ----
    size : 字号（像素）

    返回
    ----
    PIL 字体对象；PIL 不可用或没有中文字体时返回 None（用默认字体，中文会变方块）
    """
    if ImageFont is None:
        return None
    key = int(size)
    if key in _FONT_CACHE:
        return _FONT_CACHE[key]

    font = None
    path = find_cjk_font()
    if path:
        try:
            font = ImageFont.truetype(path, key)
        except Exception:
            font = None
    if font is not None and not _FONT_CACHE:
        print("[字体] 使用中文字体：%s" % path)
    _FONT_CACHE[key] = font
    return font


def _default_font_warning() -> None:
    """中文字体缺失时打印一次性的降级警告与解决办法"""
    if _FONT_CACHE.get(0):
        return
    _FONT_CACHE[0] = True                    # 用 key 0 做"已警告过"的标记
    print("-" * 72)
    print("[WARN] 找不到中文字体，将退化为 PIL 内置默认字体（中文会显示为方块）。")
    print("       解决办法：")
    print("         Windows: 确认存在 C:/Windows/Fonts/msyh.ttc（微软雅黑）")
    print("         Linux  : sudo apt install fonts-noto-cjk")
    print("         macOS  : 一般自带 /System/Library/Fonts/PingFang.ttc")
    print("-" * 72)


# ===========================================================================
# 单张标签绘制
# ===========================================================================
def _qr_to_bgr(code: str, size: int) -> Optional["np.ndarray"]:
    """
    生成二维码并转成 PIL 可用的 RGB 图。

    参数
    ----
    code : 二维码内容（药 code）
    size : 二维码边长（像素，含静区）

    返回
    ----
    RGB uint8 图像；生成失败返回 None
    """
    qr = make_qr_image(code, size=size, quiet_zone_ratio=0.18, ecc_level=3)
    if qr is None:
        return None
    if qr.ndim == 2:
        qr = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
    rgb = cv2.cvtColor(qr, cv2.COLOR_BGR2RGB)
    return rgb


def make_label(med: Medicine, width: int = 620, qr_size: int = 300,
               font_dir: Optional[str] = None) -> Optional["np.ndarray"]:
    """
    生成单张药品标签（白底 + 二维码 + 中文药名/分类/槽位 + 风险标识）。

    布局（自上而下）
        [二维码]
        药名（大号加粗）
        分类 | 槽位 N | 库存 M
        用药提示（灰色小字）
        高风险药：整张标签外框变红，右上角打 ⚠

    参数
    ----
    med      : 药品记录
    width    : 标签宽度（像素）
    qr_size  : 二维码边长（像素）
    font_dir : 预留参数，指定字体文件（None 表示自动查找）

    返回
    ----
    BGR uint8 图像（供 cv2.imwrite 保存）；PIL 不可用或二维码生成失败返回 None
    """
    if Image is None or ImageDraw is None:
        print("[错误] 未安装 Pillow，无法绘制中文标签：pip install pillow")
        return None

    if not med.code:
        print("[跳过] 药品缺少 code 字段，无法生成二维码")
        return None

    font_name = None
    if font_dir and os.path.isfile(font_dir) and ImageFont is not None:
        try:
            font_name = ImageFont.truetype(font_dir, 52)
        except Exception:
            font_name = None
    if font_name is None:
        font_name = get_font(52)
    font_meta = get_font(30)
    font_note = get_font(26)
    if font_name is None:
        _default_font_warning()

    # ---- 高度按内容估算，避免底部留一大片空白 ----
    pad = 24
    has_note = bool(med.note)
    height = int(pad + qr_size + 20 + 78 + 46 + (48 if has_note else 0) + pad)

    canvas = Image.new("RGB", (int(width), height), (255, 255, 255))
    d = ImageDraw.Draw(canvas)

    def _text(x, y, s, font=None, fill=(20, 20, 20)):
        """统一的文字绘制（兼容新旧 PIL：优先 font，其次默认字体）"""
        try:
            d.text((x, y), s, font=font, fill=fill)
        except Exception:
            d.text((x, y), s, fill=fill)

    def _tw(s, font):
        """量文字宽度"""
        try:
            b = d.textbbox((0, 0), s, font=font)
            return b[2] - b[0], b[3] - b[1]
        except Exception:
            return len(s) * 16, 16

    # ---- 二维码（居中）----
    qr_rgb = _qr_to_bgr(med.code, qr_size)
    if qr_rgb is None:
        print("[错误] 生成二维码失败（code=%s），请确认 OpenCV >= 4.5 且带 QRCodeEncoder"
              % med.code)
        return None
    qx = (width - qr_rgb.shape[1]) // 2
    canvas.paste(Image.fromarray(qr_rgb), (qx, pad))
    y = pad + qr_rgb.shape[0] + 20

    # ---- 药名（居中，大号）----
    name = med.name or med.code
    tw, _th = _tw(name, font_name)
    _text((width - tw) // 2, y, name, font=font_name, fill=(15, 15, 15))
    y += 78

    # ---- 元信息行：分类 | 槽位 | 库存 ----
    meta_bits = []
    if med.category:
        meta_bits.append(med.category)
    if med.slot:
        meta_bits.append("槽位 %d" % med.slot)
    if med.stock:
        meta_bits.append("库存 %d" % med.stock)
    meta = "  |  ".join(meta_bits)
    if meta:
        tw, _th = _tw(meta, font_meta)
        _text((width - tw) // 2, y, meta, font=font_meta, fill=(90, 90, 90))
    y += 46

    # ---- 用药提示 ----
    if has_note:
        tw, _th = _tw(med.note, font_note)
        _text((width - tw) // 2, y, med.note, font=font_note, fill=(130, 130, 130))

    # ---- 风险标识：高风险药加红框 + ⚠ ----
    if med.high_risk:
        d.rectangle([2, 2, width - 3, height - 3], outline=(200, 20, 20), width=8)
        _text(pad, pad + 4, "\u26a0", font=font_name, fill=(200, 20, 20))
        tw, th = _tw("高风险", font_note)
        _text(width - tw - pad, height - th - pad, "高风险", font=font_note,
              fill=(200, 20, 20))

    # 裁剪线（虚线不太必要，画一圈浅灰细边方便剪）
    d.rectangle([0, 0, width - 1, height - 1], outline=(205, 205, 205), width=1)

    return cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)


# ===========================================================================
# A4 排版
# ===========================================================================
def compose_a4(labels: Sequence["np.ndarray"], page_w: int = A4_W,
               page_h: int = A4_H, margin: int = 90, gap: int = 40) -> "np.ndarray":
    """
    把若干张标签排版到一张 A4 大图上（自动分多页，返回第一页）。

    参数
    ----
    labels   : 标签图像列表（BGR）
    page_w   : 页宽（像素），默认 A4@300dpi
    page_h   : 页高（像素）
    margin   : 页边距
    gap      : 标签间距

    返回
    ----
    A4 大小的 BGR 图像（白底）
    """
    page = np.full((page_h, page_w, 3), 255, np.uint8)
    if not labels:
        return page

    cell_w = max(l.shape[1] for l in labels)
    cell_h = max(l.shape[0] for l in labels)
    cols = max(1, (page_w - 2 * margin + gap) // (cell_w + gap))
    rows = max(1, (page_h - 2 * margin + gap) // (cell_h + gap))

    for i, lab in enumerate(labels):
        r, c = divmod(i, cols)
        if r >= rows:                       # 装不下就不贴（调用方应分页）
            break
        x = margin + c * (cell_w + gap)
        y = margin + r * (cell_h + gap)
        h, w = lab.shape[:2]
        page[y:y + h, x:x + w] = lab
    return page


# ===========================================================================
# 清单读取
# ===========================================================================
def load_medicines(path: Optional[str]) -> List[Medicine]:
    """
    读取药品清单。

    参数
    ----
    path : medicines.json 路径；文件不存在/解析失败时使用内置样例清单

    返回
    ----
    List[Medicine]（可能为空列表，若清单里一条都没有）
    """
    data = None
    if path and os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            print("[清单] 已读取 %s" % path)
        except Exception as e:
            print("[警告] 解析 %s 失败：%r" % (path, e))
            data = None
    elif path:
        print("[提示] 找不到 %s，改用内置样例清单（8 种药）" % path)

    if data is None:
        raw = SAMPLE_MEDICINES
        print("[清单] 使用内置样例清单：%d 条" % len(raw))
    else:
        raw = data.get("medicines", data) if isinstance(data, dict) else data
        if not isinstance(raw, list):
            print("[警告] 清单格式不对（期望 {'medicines': [...]}），改用内置样例")
            raw = SAMPLE_MEDICINES

    meds = [Medicine.from_dict(d) for d in raw if isinstance(d, dict)]
    meds = [m for m in meds if m.code]
    # 按槽位排序，打印出来顺手贴瓶
    meds.sort(key=lambda m: (m.slot if m.slot else 999, m.code))
    return meds


# ===========================================================================
# 出口
# ===========================================================================
def parse_args(argv=None):
    """
    解析命令行参数。

    参数
    ----
    argv : 参数列表，默认 sys.argv[1:]

    返回
    ----
    argparse.Namespace
    """
    import argparse
    p = argparse.ArgumentParser(
        description="智能药箱 - 生成可打印的药品二维码标签（A4 排版 + 逐张小图）")
    p.add_argument("--medicines", default=DEFAULT_MEDICINES,
                   help="药品清单 JSON，默认 %s（不存在则用内置样例）" % DEFAULT_MEDICINES)
    p.add_argument("--out", default=DEFAULT_OUT,
                   help="输出目录，默认 %s" % DEFAULT_OUT)
    p.add_argument("--slots", default=None,
                   help="只生成这些槽位的标签，逗号分隔，如 1,3,5（默认全部）")
    p.add_argument("--individual", action="store_true",
                   help="除 A4 大图外，再保存每张单独的小图（方便逐张打印/贴瓶）")
    p.add_argument("--no-a4", action="store_true", help="只出单独小图，不出 A4 大图")
    p.add_argument("--qr-size", type=int, default=300, help="二维码边长像素，默认 300")
    p.add_argument("--label-width", type=int, default=620, help="标签宽度像素，默认 620")
    p.add_argument("--font", default=None, help="手动指定中文字体文件路径")
    return p.parse_args(argv)


def main(argv=None) -> int:
    """
    程序入口：读清单 -> 逐张画标签 -> 输出 A4 大图（分页）和/或单独小图。

    参数
    ----
    argv : 命令行参数列表

    返回
    ----
    退出码：0 成功，非 0 失败
    """
    args = parse_args(argv)

    meds = load_medicines(args.medicines)
    if args.slots:
        want = set()
        for s in str(args.slots).split(","):
            s = s.strip()
            if s.isdigit():
                want.add(int(s))
        meds = [m for m in meds if m.slot in want]
        print("[清单] 按 --slots 过滤后剩 %d 条" % len(meds))
    if not meds:
        print("[错误] 没有可生成的药品（清单为空，或 --slots 过滤太严）")
        return 1

    os.makedirs(args.out, exist_ok=True)
    print("[输出] 目录：%s" % os.path.abspath(args.out))

    labels: List["np.ndarray"] = []
    for m in meds:
        lab = make_label(m, width=args.label_width, qr_size=args.qr_size,
                         font_dir=args.font)
        if lab is None:
            print("[失败] %s %s 标签生成失败" % (m.code, m.name))
            continue
        labels.append(lab)
        tag = " [高风险]" if m.high_risk else ""
        print("    + %s  %s  槽位%s%s -> %dx%d" %
              (m.code, m.name or "(无药名)", m.slot, tag,
               lab.shape[1], lab.shape[0]))

    if not labels:
        print("[错误] 一张标签都没生成成功")
        return 1

    saved: List[str] = []

    # ---- 单独小图 ----
    if args.individual or args.no_a4:
        for m, lab in zip(meds, labels):
            fn = "label_%s.png" % (m.code or "unknown")
            fp = os.path.join(args.out, fn)
            cv2.imwrite(fp, lab)
            saved.append(fp)
        print("[输出] 已保存 %d 张单独标签" % len(labels))

    # ---- A4 大图（超过一页自动分页）----
    if not args.no_a4:
        cell_w = max(l.shape[1] for l in labels)
        cell_h = max(l.shape[0] for l in labels)
        margin, gap = 90, 40
        cols = max(1, (A4_W - 2 * margin + gap) // (cell_w + gap))
        rows = max(1, (A4_H - 2 * margin + gap) // (cell_h + gap))
        per_page = max(1, cols * rows)

        for page_i, start in enumerate(range(0, len(labels), per_page), start=1):
            chunk = labels[start:start + per_page]
            page = compose_a4(chunk, margin=margin, gap=gap)
            fn = "labels_A4.png" if page_i == 1 else "labels_A4_%d.png" % page_i
            fp = os.path.join(args.out, fn)
            cv2.imwrite(fp, page)
            saved.append(fp)
            print("[输出] A4 第 %d 页：%s（本页 %d 张，每页最多 %d 张 = %d行x%d列）"
                  % (page_i, fp, len(chunk), per_page, rows, cols))
        print("[提示] A4 图按 300dpi 生成，打印时选\"实际大小/100%%\"，否则二维码会被缩放")

    print("-" * 70)
    print("[完成] 共 %d 个药品，输出 %d 个文件到 %s" %
          (len(labels), len(saved), os.path.abspath(args.out)))
    for fp in saved:
        print("    %s" % fp)
    return 0


if __name__ == "__main__":
    sys.exit(main())
