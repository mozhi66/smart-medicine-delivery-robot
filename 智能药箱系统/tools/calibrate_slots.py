#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 槽位相机 ROI 标定工具 (calibrate_slots.py)
================================================================================
职责：让用户在相机画面（或一张照片）上依次框出 N 个药槽的位置，
     把每个槽位的 (x, y, w, h) 存成 config/slots.yaml，供视觉模块使用。

为什么必须有这个工具
--------------------
相机一旦被挪动/重新对焦，所有槽位坐标就全废了。靠人眼去 yaml 里手改数字
既慢又容易错，必须"看着画面点几下"完成标定，并且能立刻看到框得对不对。

操作方式
--------
    按住鼠标左键拖拽        -> 画框（松手后框变黄，处于"待确认"状态）
    Enter                   -> 确认当前框，加入序列（按顺序编号 1,2,3...）
    u                       -> 撤销上一个（待确认的框优先撤销）
    q / ESC                 -> 完成，保存到 --out
    （窗口关闭也算结束）

输出格式（严格按主控程序读取的格式手写，不用 yaml.dump 以免风格变化）
--------
    slots:
      - id: 1
        roi: [x, y, w, h]
    slot_monitor:
      mode: dot

两种画面来源
------------
    实时相机：python3.11 tools/calibrate_slots.py --camera 0 --n 8
    静态照片：python3.11 tools/calibrate_slots.py --image tests/box.jpg --n 8
              （没有摄像头 / 想在家里对着现场照片预标定时用）
    无界面环境（沙盒/服务器）：加 --auto-grid，直接按画面自动切出 N 个网格并保存，
              交互式标定在此模式下不可用（OpenCV 需要图形界面）。
================================================================================
"""

from __future__ import annotations

import math
import os
import sys
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

# 让脚本能直接 import pc/ 下的模块（与 tools/vision_sim.py 保持一致）
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_ROOT, "pc"))

from vision.camera import Camera, MockCamera            # noqa: E402

# ---- 可选依赖：PyYAML 只用于"保存后回读校验"，没有也能正常保存 ----
try:
    import yaml                                          # pip install pyyaml
except Exception:                                        # pragma: no cover
    yaml = None

# ---- 可选依赖：PIL 用于在画面上画中文（cv2.putText 画中文会乱码）----
try:
    from PIL import Image, ImageDraw, ImageFont
except Exception:                                        # pragma: no cover
    Image = ImageDraw = ImageFont = None

DEFAULT_OUT = os.path.join("config", "slots.yaml")

# 中文（CJK）字体候选路径，按平台排列，第一个存在的就用
_FONT_CANDIDATES = (
    # Windows
    "C:/Windows/Fonts/msyh.ttc",        # 微软雅黑
    "C:/Windows/Fonts/msyhbd.ttc",      # 微软雅黑粗体
    "C:/Windows/Fonts/simhei.ttf",      # 黑体
    "C:/Windows/Fonts/simsun.ttc",      # 宋体
    # Linux
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
    # macOS
    "/System/Library/Fonts/PingFang.ttc",
    "/Library/Fonts/Arial Unicode.ttf",
)


# ===========================================================================
# 字体与中文绘制
# ===========================================================================
def find_cjk_font() -> Optional[str]:
    """
    在系统里找一个能显示中文的字体文件。

    返回
    ----
    字体文件路径；找不到返回 None（调用方会退化为 PIL 默认字体并警告）
    """
    for p in _FONT_CANDIDATES:
        if os.path.isfile(p):
            return p
    # 兜底：在 Linux 字体目录里扫一遍常见中文字体（有上限，避免扫太久）
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


_FONT_CACHE = {}


def _get_font(size: int):
    """
    按字号取字体对象（带缓存，避免每帧重复加载 ttf）。

    参数
    ----
    size : 字号（像素）

    返回
    ----
    PIL 字体对象；PIL 不可用或找不到中文字体时返回 None（调用方用默认字体）
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
    if font is None and not _FONT_CACHE:      # 只在第一次时警告，避免刷屏
        print("[WARN] 找不到中文字体，界面提示文字可能显示为方块/乱码")
        print("       Windows: 确认存在 C:/Windows/Fonts/msyh.ttc")
        print("       Linux  : sudo apt install fonts-noto-cjk")
    _FONT_CACHE[key] = font                   # None 也会被缓存
    return font


def draw_text_block(img: "np.ndarray", lines: Sequence[str], org: Tuple[int, int] = (10, 10),
                    size: int = 20, text_color: Tuple[int, int, int] = (255, 255, 255),
                    bg_color: Tuple[int, int, int] = (0, 0, 0), alpha: float = 0.55,
                    pad: int = 8, gap: int = 6) -> "np.ndarray":
    """
    在图像上画一块【多行中文】文字（带半透明底衬，保证任何背景下都看得清）。

    为什么不用 cv2.putText：OpenCV 内置字体不含中文，画出来全是 '???'。
    这里统一走 PIL 渲染（BGR <-> RGB 互转），中文正常显示。

    参数
    ----
    img        : BGR 图像，会被就地修改
    lines      : 多行文本
    org        : 文字块左上角 (x, y)
    size       : 字号
    text_color : 文字颜色（BGR）
    bg_color   : 底衬颜色（BGR）
    alpha      : 底衬不透明度
    pad        : 内边距
    gap        : 行间距

    返回
    ----
    画好文字的 BGR 图像（就是传入的那个对象）
    """
    if not lines or Image is None or ImageDraw is None:
        return img

    font = _get_font(size)
    # 先量出每行的尺寸，才能画合适大小的底衬
    tmp = Image.fromarray(np.zeros((10, 10, 3), np.uint8))
    tmp_draw = ImageDraw.Draw(tmp)
    sizes = []
    for ln in lines:
        try:
            b = tmp_draw.textbbox((0, 0), ln, font=font) if font else tmp_draw.textbbox((0, 0), ln)
        except Exception:
            b = (0, 0, len(ln) * size, size)
        sizes.append((b[2] - b[0], b[3] - b[1]))

    line_h = max(h for _w, h in sizes) if sizes else size
    box_w = max(w for w, _h in sizes) + 2 * pad
    box_h = len(lines) * line_h + (len(lines) - 1) * gap + 2 * pad

    x0, y0 = int(org[0]), int(org[1])
    x1 = min(img.shape[1], x0 + box_w)
    y1 = min(img.shape[0], y0 + box_h)
    if x1 <= x0 or y1 <= y0:
        return img

    # 半透明底衬
    overlay = img.copy()
    cv2.rectangle(overlay, (x0, y0), (x1, y1), bg_color, -1)
    cv2.addWeighted(overlay, alpha, img, 1.0 - alpha, 0, img)

    # 中文文字
    pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    d = ImageDraw.Draw(pil)
    fill = (int(text_color[2]), int(text_color[1]), int(text_color[0]))   # BGR -> RGB
    yy = y0 + pad
    for ln, (_w, h) in zip(lines, sizes):
        d.text((x0 + pad, yy), ln, font=font, fill=fill)
        yy += line_h + gap
    img[:] = cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)
    return img


# ===========================================================================
# 标定器：鼠标拖拽 + 键盘确认
# ===========================================================================
class SlotCalibrator:
    """
    交互式槽位框选器：管理"已确认的框 / 待确认的框 / 正在拖的框"三种状态，
    并负责把状态画到画面上。

    状态机
    ------
        拖拽松手 -> pending（黄框）
        Enter     -> pending 转正，进入 rois（绿框，编号 +1）
        u         -> 先撤 pending，没有 pending 再撤最后一个 rois
    """

    MIN_SIZE = 5           # 小于这个像素的框视为误点，忽略

    def __init__(self, n: int = 8):
        """
        参数
        ----
        n : 计划标定的槽位数量（只用于进度显示和上限提醒，不强制）
        """
        self.n = int(n)
        self.rois: List[Tuple[int, int, int, int]] = []     # 已确认 [(x,y,w,h)]
        self.pending: Optional[Tuple[int, int, int, int]] = None
        self.drawing = False
        self.p0: Optional[Tuple[int, int]] = None
        self.p1: Optional[Tuple[int, int]] = None

    # ------------------------------------------------------------------
    def on_mouse(self, event: int, x: int, y: int, flags: int, param) -> None:
        """
        OpenCV 鼠标回调函数（配合 cv2.setMouseCallback 使用）。

        参数
        ----
        event : cv2.EVENT_* 事件类型
        x, y  : 鼠标在图像坐标系中的位置
        flags : 按键/修饰键状态（未使用）
        param : setMouseCallback 传入的用户数据（未使用）
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.p0 = (x, y)
            self.p1 = (x, y)
            self.pending = None
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            self.p1 = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.drawing:
            self.drawing = False
            self.p1 = (x, y)
            r = self._norm_rect(self.p0, self.p1)
            if r[2] < self.MIN_SIZE or r[3] < self.MIN_SIZE:
                print("[提示] 框太小（<%dpx），已忽略，请按住左键拖拽出一个框" % self.MIN_SIZE)
                self.p0 = self.p1 = None
                return
            self.pending = r
            print("[提示] 已画框 %s（编号将是 %d），按 Enter 确认 / 重新拖拽可覆盖"
                  % (list(r), len(self.rois) + 1))

    # ------------------------------------------------------------------
    @staticmethod
    def _norm_rect(p0, p1) -> Tuple[int, int, int, int]:
        """把两个对角点整理成 (x, y, w, h)，允许从任意方向拖拽"""
        if p0 is None or p1 is None:
            return (0, 0, 0, 0)
        x0, y0 = int(p0[0]), int(p0[1])
        x1, y1 = int(p1[0]), int(p1[1])
        x, w = (x0, x1 - x0) if x1 >= x0 else (x1, x0 - x1)
        y, h = (y0, y1 - y0) if y1 >= y0 else (y1, y0 - y1)
        return (x, y, w, h)

    # ------------------------------------------------------------------
    def confirm_pending(self) -> bool:
        """
        确认当前待确认的框（Enter 键）。

        返回
        ----
        True 成功加入；False 没有待确认的框或已达计划数量
        """
        if self.pending is None:
            print("[提示] 还没有待确认的框：请先按住左键拖出一个框")
            return False
        if len(self.rois) >= self.n:
            print("[提示] 已达到计划数量 %d 个；按 u 撤销，或按 q 直接保存" % self.n)
            return False
        self.rois.append(self.pending)
        print("[确认] 槽位 %d -> roi=%s" % (len(self.rois), list(self.pending)))
        self.pending = None
        self.p0 = self.p1 = None
        return True

    # ------------------------------------------------------------------
    def undo(self) -> bool:
        """
        撤销（u 键）：优先撤销待确认的框，其次撤销最后一个已确认的框。

        返回
        ----
        True 撤销成功；False 没有可撤销的内容
        """
        if self.pending is not None:
            self.pending = None
            self.p0 = self.p1 = None
            print("[撤销] 已丢弃待确认的框")
            return True
        if self.rois:
            r = self.rois.pop()
            print("[撤销] 已删除槽位 %d（roi=%s），当前还剩 %d 个"
                  % (len(self.rois) + 1, list(r), len(self.rois)))
            return True
        print("[撤销] 没有可撤销的框")
        return False

    # ------------------------------------------------------------------
    def draw(self, frame: "np.ndarray") -> "np.ndarray":
        """
        把当前状态画到画面上（已确认绿框+编号 / 待确认黄框 / 正在拖的青框 / 提示文字）。

        参数
        ----
        frame : 原始相机画面（BGR）

        返回
        ----
        画好标注的可视化图（BGR）
        """
        vis = frame.copy()

        # 已确认：绿色 + 编号
        for i, (x, y, w, h) in enumerate(self.rois, start=1):
            cv2.rectangle(vis, (x, y), (x + w, y + h), (60, 200, 60), 2)
            cv2.rectangle(vis, (x, y), (x + 26, y + 22), (60, 200, 60), -1)
            draw_text_block(vis, [str(i)], org=(x + 4, y + 1), size=18,
                            text_color=(255, 255, 255), bg_color=(40, 150, 40),
                            alpha=0.85, pad=3)

        # 待确认：黄色（Enter 后转正）
        if self.pending is not None:
            x, y, w, h = self.pending
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 220, 255), 2)

        # 正在拖拽：青色虚线感（用细线画十字对角辅助）
        if self.drawing and self.p0 is not None and self.p1 is not None:
            x, y, w, h = self._norm_rect(self.p0, self.p1)
            cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 220, 0), 1)

        # 顶部操作提示（中文）
        lines = [
            "已标定 %d / %d 个槽位" % (len(self.rois), self.n),
            "按住左键拖拽画框 -> Enter 确认 | u 撤销上一个 | q 完成并保存",
        ]
        if self.pending is not None:
            lines.append("当前黄框待确认：Enter=确认（编号 %d），重新拖拽=重画"
                         % (len(self.rois) + 1))
        else:
            lines.append("提示：按 1、2、3…… 的顺序框选，编号就是槽位 id")
        draw_text_block(vis, lines, org=(10, 10), size=21)
        return vis


# ===========================================================================
# 保存 YAML
# ===========================================================================
def save_slots_yaml(path: str, rois: Sequence[Tuple[int, int, int, int]],
                    mode: str = "dot") -> bool:
    """
    把槽位 ROI 写成 YAML（格式固定，手写而非 yaml.dump，避免风格漂移）。

    参数
    ----
    path : 输出文件路径，如 config/slots.yaml
    rois : [(x, y, w, h), ...]，顺序即槽位 id（1, 2, 3...）
    mode : slot_monitor 的判据模式，默认 'dot'（槽底圆点）

    返回
    ----
    True 保存成功（并且回读校验通过或跳过校验）
    """
    if not rois:
        print("[错误] 没有槽位可保存")
        return False

    lines = ["slots:"]
    for i, (x, y, w, h) in enumerate(rois, start=1):
        lines.append("  - id: %d" % i)
        lines.append("    roi: [%d, %d, %d, %d]" % (int(x), int(y), int(w), int(h)))
    lines.append("slot_monitor:")
    lines.append("  mode: %s" % mode)
    text = "\n".join(lines) + "\n"

    out_dir = os.path.dirname(os.path.abspath(path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    print("[保存] 已写入 %s（%d 个槽位）" % (path, len(rois)))

    # 回读校验：确认主控程序用 yaml.safe_load 能原样读出来
    if yaml is None:
        print("[提示] 未安装 PyYAML，跳过回读校验（pip install pyyaml）")
        return True
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        n = len(data.get("slots", []))
        ok = (n == len(rois)) and data.get("slot_monitor", {}).get("mode") == mode
        print("[校验] 回读成功：%d 个槽位，mode=%s -> %s"
              % (n, data.get("slot_monitor", {}).get("mode"),
                 "OK" if ok else "不一致，请检查"))
        return bool(ok)
    except Exception as e:
        print("[校验] 回读失败（文件已写出，请人工确认）：%r" % (e,))
        return True


# ===========================================================================
# 无界面环境：自动网格
# ===========================================================================
def auto_grid(shape, n: int, margin_ratio: float = 0.06,
              fill_ratio: float = 0.82) -> List[Tuple[int, int, int, int]]:
    """
    在画面上自动切出 n 个等分网格，作为槽位 ROI 的初始值。

    用途：无摄像头/无图形界面时先生成一份能用的配置，之后有界面再微调；
         也适合药槽本身就是规则阵列排列的情况。

    参数
    ----
    shape        : 图像 shape（H, W）或 (H, W, C)
    n            : 槽位数量
    margin_ratio : 四周留白比例
    fill_ratio   : 每个网格里 ROI 占的比例（留出格子间隙）

    返回
    ----
    [(x, y, w, h), ...]，共 n 个
    """
    h, w = int(shape[0]), int(shape[1])
    cols = max(1, int(math.ceil(math.sqrt(n))))
    rows = max(1, int(math.ceil(n / float(cols))))
    mx, my = int(w * margin_ratio), int(h * margin_ratio)
    cw = max(1, (w - 2 * mx) // cols)
    ch = max(1, (h - 2 * my) // rows)

    rois = []
    for i in range(int(n)):
        r, c = divmod(i, cols)
        x = mx + c * cw
        y = my + r * ch
        rois.append((int(x), int(y), int(cw * fill_ratio), int(ch * fill_ratio)))
    return rois


def _display_reachable(disp: str) -> bool:
    """
    探测 X11/Wayland 显示服务是不是真的连得上。

    为什么不能只看 DISPLAY 环境变量：
        容器/沙盒里经常出现 DISPLAY=:0 但根本没有 X server 的情况。
        此时 cv2.imshow() 不是抛异常，而是 Qt 内部 qFatal 直接 abort 进程
        （退出码 134），try/except 完全拦不住——所以必须在调用 imshow 之前
        用 socket 探测一次。

    参数
    ----
    disp : DISPLAY 字符串，形如 ':0'、'localhost:10.0'

    返回
    ----
    True 显示服务可连接；False 连不上（应跳过所有 imshow）
    """
    import re
    import socket

    # Wayland：socket 文件存在于 XDG_RUNTIME_DIR 下即认为可用
    wl = os.environ.get("WAYLAND_DISPLAY")
    if wl:
        rt = os.environ.get("XDG_RUNTIME_DIR") or ("/run/user/%d" % os.getuid())
        return os.path.exists(os.path.join(rt, wl))

    if not disp:
        return False
    try:
        host, _, tail = disp.rpartition(":")
        disp_no = tail.split(".")[0]
        if not host or host == "unix":                     # 本机：走 unix socket
            path = "/tmp/.X11-unix/X%s" % disp_no
            if not os.path.exists(path):
                return False
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(1.0)
            try:
                s.connect(path)
                return True
            finally:
                s.close()
        host2 = socket.gethostbyname(host)                 # 远程：TCP 6000+display
        s = socket.create_connection((host2, 6000 + int(disp_no)), timeout=1.0)
        s.close()
        return True
    except Exception:
        return False


def gui_available() -> bool:
    """
    判断当前环境能不能弹 OpenCV 窗口。

    返回
    ----
    True 可以显示窗口；False 无图形界面（应跳过 imshow，改用 --auto-grid 等无界面路径）
    """
    if sys.platform.startswith("win") or sys.platform == "darwin":
        return True
    return _display_reachable(os.environ.get("DISPLAY", ""))


# ===========================================================================
# 主流程
# ===========================================================================
def run_interactive(cam, n: int, out_path: str) -> int:
    """
    打开窗口，进入"拖拽画框"的交互循环，直到用户按 q 退出。

    参数
    ----
    cam      : 提供 read() -> (ok, frame) 的对象（Camera 或 MockCamera）
    n        : 计划标定的槽位数量
    out_path : 退出时保存的 YAML 路径

    返回
    ----
    0 正常结束并保存；1 未保存（没有框任何槽位）
    """
    win = "calibrate_slots - drag / Enter / u / q"
    cal = SlotCalibrator(n)
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, cal.on_mouse)
    print("-" * 70)
    print("操作：按住左键拖拽画框 -> Enter 确认 | u 撤销 | q 完成并保存")
    print("目标：框出 %d 个槽位，编号顺序即槽位 id（1,2,3...）" % n)
    print("-" * 70)

    try:
        while True:
            ok, frame = cam.read()
            if not ok or frame is None:
                print("[错误] 读取画面失败")
                break
            cv2.imshow(win, cal.draw(frame))
            key = cv2.waitKey(30) & 0xFF
            if key in (13, 10):              # Enter
                cal.confirm_pending()
            elif key == 117:                 # u
                cal.undo()
            elif key in (113, 27):           # q / ESC
                break
    finally:
        cv2.destroyAllWindows()
        try:
            cam.release()
        except Exception:
            pass

    if not cal.rois:
        print("[结束] 没有标定任何槽位，未保存文件")
        return 1
    save_slots_yaml(out_path, cal.rois)
    return 0


def parse_args(argv=None):
    """
    解析命令行参数。

    参数
    ----
    argv : 参数列表，默认取 sys.argv[1:]

    返回
    ----
    argparse.Namespace
    """
    import argparse
    p = argparse.ArgumentParser(
        description="智能药箱 - 槽位 ROI 标定工具（拖拽画框 -> config/slots.yaml）")
    p.add_argument("--camera", type=int, default=0,
                   help="相机序号，默认 0")
    p.add_argument("--image", default=None,
                   help="用一张静态照片代替摄像头（无摄像头环境/预标定时用）")
    p.add_argument("--n", type=int, default=8, help="要标定的槽位数量，默认 8")
    p.add_argument("--out", default=DEFAULT_OUT,
                   help="输出 YAML 路径，默认 %s" % DEFAULT_OUT)
    p.add_argument("--width", type=int, default=1280, help="相机请求宽度（实时相机模式）")
    p.add_argument("--height", type=int, default=720, help="相机请求高度（实时相机模式）")
    p.add_argument("--auto-grid", action="store_true",
                   help="不进交互界面，直接按画面自动切 N 个网格并保存（无界面环境用）")
    p.add_argument("--preview", default=None,
                   help="仅 --auto-grid 时有效：把网格结果画成图片存到该路径，方便检查")
    return p.parse_args(argv)


def main(argv=None) -> int:
    """
    程序入口：取一帧画面 ->（auto-grid 直接保存 | 进入交互标定）。

    参数
    ----
    argv : 命令行参数列表

    返回
    ----
    退出码：0 成功，非 0 失败
    """
    args = parse_args(argv)

    # ---- 画面来源：静态照片 或 实时相机 ----
    if args.image:
        if not os.path.isfile(args.image):
            print("[错误] 图片不存在：%s" % args.image)
            return 2
        try:
            cam = MockCamera(args.image)
        except Exception as e:
            print("[错误] 无法读取图片 %s：%r" % (args.image, e))
            return 2
        print("[画面] 静态照片模式：%s" % args.image)
    else:
        cam = Camera(index=args.camera, width=args.width, height=args.height)
        if not cam.open():
            print("[错误] 打不开相机 %d。请检查：1) 相机是否被其它软件占用 "
                  "2) 序号对不对（试试 --camera 1）" % args.camera)
            print("       没有摄像头？用 --image 照片 来标定。")
            return 2
        print("[画面] 实时相机模式：index=%d" % args.camera)

    ok, frame = cam.read()
    if not ok or frame is None:
        print("[错误] 读不到画面")
        cam.release()
        return 2
    print("[画面] 分辨率：%dx%d" % (frame.shape[1], frame.shape[0]))

    # ---- 无界面/自动网格模式 ----
    if args.auto_grid:
        rois = auto_grid(frame.shape, args.n)
        print("[auto-grid] 已按 %d 个槽位自动切分画面：" % args.n)
        for i, r in enumerate(rois, start=1):
            print("    id=%d roi=%s" % (i, list(r)))
        if args.preview:
            vis = frame.copy()
            for i, (x, y, w, h) in enumerate(rois, start=1):
                cv2.rectangle(vis, (x, y), (x + w, y + h), (60, 200, 60), 2)
                draw_text_block(vis, [str(i)], org=(x + 4, y + 2), size=18,
                                bg_color=(40, 150, 40), alpha=0.85, pad=3)
            os.makedirs(os.path.dirname(os.path.abspath(args.preview)) or ".",
                        exist_ok=True)
            cv2.imwrite(args.preview, vis)
            print("[auto-grid] 预览图已保存：%s" % args.preview)
        cam.release()
        return 0 if save_slots_yaml(args.out, rois) else 1

    # ---- 交互模式需要图形界面 ----
    if not gui_available():
        print("[跳过] 当前环境没有图形界面（无 DISPLAY），交互式标定不可用。")
        print("       解决办法：")
        print("         1) 加 --auto-grid 参数，直接按网格自动生成并保存；")
        print("         2) 在有桌面的本机/Windows 上运行本脚本。")
        cam.release()
        return 0

    return run_interactive(cam, args.n, args.out)


if __name__ == "__main__":
    sys.exit(main())
