#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 HSV 颜色阈值调试工具 (color_tuner.py)
================================================================================
职责：现场调 HSV 上下限，并把结果直接抄进配置。用于槽底圆点识别、药瓶色块识别
     （对应 pc/vision/slot_monitor.py 的 dot_hsv_lower/upper、bottle_hsv_*）。

为什么必须有个工具来调
--------------------
HSV 阈值是整条视觉链路上最"靠现场"的参数：
    灯一换、相机白平衡一变，同一瓶药在 HSV 里的数值能差出一大截。
在代码里猜数字 -> 跑一遍 -> 再改，效率极低；必须在同一画面里"边拖边看"。

核心设计：把 ROI 抠出来单独显示
----------------------------
判断"阈值调得对不对"，在整帧小窗口里肉眼根本看不清一块几十像素的圆点。
所以本工具会把 ROI 区域放大好几倍单独显示（--roi x,y,w,h 指定），
这样滑动条动一下，mask 的变化立刻看得见。

三个窗口
--------
    1) original+mask : 原图与二值 mask 左右拼接
    2) result        : 把 mask 覆盖回原图（看"选中的是哪块"）
    3) 滑动条        : H/S/V 上下限 + 形态学开闭 + ROI 放大倍数

按键
----
    s : 在终端打印（并尽力复制到系统剪贴板）当前 HSV 范围，可直接粘进配置
    e : 导出当前结果图（result 和 mask）到文件
    r : 重置为初始阈值
    q / ESC : 退出

用法
----
    # 实时相机
    python3.11 tools/color_tuner.py --camera 0 --roi 120,90,60,60

    # 静态照片（没有摄像头时，用现场拍的照片慢慢调）
    python3.11 tools/color_tuner.py --image tests/box.jpg

    # 无图形界面（服务器/沙盒）：直接给出某张图里颜色的统计区间，不用滑动条
    python3.11 tools/color_tuner.py --image tests/box.jpg --roi 120,90,60,60 --analyze

注意：cv2.putText 画中文会乱码，所以窗口里的标签全部用英文，中文只出现在注释和终端输出里。
================================================================================
"""

from __future__ import annotations

import os
import sys
import time
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

# 让脚本能直接 import pc/ 下的模块（与 tools/vision_sim.py 保持一致）
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_ROOT, "pc"))

from vision.camera import Camera, MockCamera            # noqa: E402

WIN_TRACK = "color_tuner - trackbars (s=print  e=export  r=reset  q=quit)"
WIN_MAIN = "color_tuner - original | mask"
WIN_RESULT = "color_tuner - result (mask overlay)"

# 默认阈值：绿色（槽底圆点常用色），对应 slot_monitor 的默认值附近
DEFAULT_LOWER = (35, 60, 60)
DEFAULT_UPPER = (85, 255, 255)

# 形态学核大小的可调范围（0 表示不开运算）
MAX_KERNEL = 15

# 窗口显示时的最大宽度（超过就缩小，避免窗口比屏幕还大）
MAX_VIEW_W = 1100


# ===========================================================================
# ROI 解析
# ===========================================================================
def parse_roi(text: Optional[str]) -> Optional[Tuple[int, int, int, int]]:
    """
    解析 --roi 参数。

    参数
    ----
    text : "x,y,w,h" 形式的字符串；None 或空表示不指定

    返回
    ----
    (x, y, w, h)；解析失败返回 None（并打印提示）
    """
    if not text:
        return None
    parts = [p.strip() for p in str(text).replace(";", ",").split(",") if p.strip()]
    if len(parts) != 4:
        print("[警告] --roi 需要 4 个数：x,y,w,h（例如 120,90,60,60），已忽略")
        return None
    try:
        x, y, w, h = [int(float(p)) for p in parts]
    except Exception:
        print("[警告] --roi 解析失败：%r，已忽略" % text)
        return None
    if w <= 0 or h <= 0:
        print("[警告] --roi 的宽高必须为正，已忽略")
        return None
    return (x, y, w, h)


def crop_roi(frame: "np.ndarray",
             roi: Optional[Sequence[int]]) -> Tuple["np.ndarray", Tuple[int, int]]:
    """
    按 ROI 裁剪画面（越界自动截断，不会抛异常）。

    参数
    ----
    frame : BGR 原图
    roi   : (x, y, w, h) 或 None（表示整幅图）

    返回
    ----
    (裁剪后的图, ROI 左上角坐标)。ROI 为空时返回 (原图, (0,0))
    """
    if frame is None or frame.size == 0:
        return frame, (0, 0)
    if not roi:
        return frame, (0, 0)
    H, W = frame.shape[:2]
    x, y, w, h = [int(v) for v in roi]
    x = max(0, min(x, W - 1))
    y = max(0, min(y, H - 1))
    x2 = max(x + 1, min(x + w, W))
    y2 = max(y + 1, min(y + h, H))
    return frame[y:y2, x:x2], (x, y)


# ===========================================================================
# 遮罩生成
# ===========================================================================
def build_mask(bgr: "np.ndarray", lower: Sequence[int], upper: Sequence[int],
               open_k: int = 0, close_k: int = 0) -> "np.ndarray":
    """
    由 BGR 图生成 HSV 阈值 mask，可选形态学开/闭运算。

    参数
    ----
    bgr      : BGR 图
    lower    : HSV 下限 [H, S, V]
    upper    : HSV 上限 [H, S, V]
    open_k   : 开运算核大小（奇数，0=不做）——去噪点
    close_k  : 闭运算核大小（奇数，0=不做）——补空洞

    返回
    ----
    单通道 uint8 mask（白=命中）
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lo = np.array([int(lower[0]), int(lower[1]), int(lower[2])], dtype=np.uint8)
    hi = np.array([int(upper[0]), int(upper[1]), int(upper[2])], dtype=np.uint8)
    mask = cv2.inRange(hsv, lo, hi)

    for kind, k in ((cv2.MORPH_OPEN, open_k), (cv2.MORPH_CLOSE, close_k)):
        k = int(k)
        if k >= 3:
            k = k if k % 2 == 1 else k + 1          # 核必须是奇数
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            mask = cv2.morphologyEx(mask, kind, kernel)
    return mask


def overlay_mask(bgr: "np.ndarray", mask: "np.ndarray",
                 color: Tuple[int, int, int] = (0, 255, 0)) -> "np.ndarray":
    """
    把 mask 以彩色叠加到原图上，直观展示"命中的是画面的哪一块"。

    参数
    ----
    bgr   : BGR 原图
    mask  : 单通道 mask
    color : 叠加颜色（BGR），默认绿色

    返回
    ----
    叠加后的 BGR 图
    """
    vis = bgr.copy()
    layer = np.zeros_like(bgr)
    layer[mask > 0] = color
    return cv2.addWeighted(vis, 0.65, layer, 0.35, 0)


# ===========================================================================
# 视图拼接与缩放
# ===========================================================================
def _fit_width(img: "np.ndarray", max_w: int = MAX_VIEW_W) -> "np.ndarray":
    """
    宽度超过上限就等比缩小，保证窗口放得下。

    参数
    ----
    img   : 待显示图
    max_w : 最大宽度

    返回
    ----
    缩放后的图（原本就不超限则原样返回）
    """
    h, w = img.shape[:2]
    if w <= max_w:
        return img
    s = max_w / float(w)
    return cv2.resize(img, (int(w * s), max(1, int(h * s))), interpolation=cv2.INTER_AREA)


def _to_view(img: "np.ndarray") -> "np.ndarray":
    """
    把图转成窗口能显示的 BGR 三通道图（灰度图会自动转彩色）。

    参数
    ----
    img : 任意通道数的图

    返回
    ----
    BGR 三通道图
    """
    if img is None:
        return np.zeros((10, 10, 3), np.uint8)
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if img.shape[2] == 4:
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img


def _hstack(a: "np.ndarray", b: "np.ndarray", gap: int = 10) -> "np.ndarray":
    """
    水平拼接两张图（高度不同时自动缩放到同一高度）。

    参数
    ----
    a, b : 待拼接的 BGR 图
    gap  : 中间间隔像素

    返回
    ----
    拼接后的 BGR 图
    """
    ha, hb = a.shape[0], b.shape[0]
    h = max(ha, hb)
    if ha != h:
        s = h / float(ha)
        a = cv2.resize(a, (max(1, int(a.shape[1] * s)), h))
    if hb != h:
        s = h / float(hb)
        b = cv2.resize(b, (max(1, int(b.shape[1] * s)), h))
    sep = np.full((h, gap, 3), 40, np.uint8)
    return np.hstack([a, sep, b])


def _annotate(img: "np.ndarray", lines: Sequence[str],
              color: Tuple[int, int, int] = (255, 255, 255)) -> "np.ndarray":
    """
    在图上画左上角信息文字（英文，避免 cv2.putText 中文乱码）。

    参数
    ----
    img   : BGR 图（就地修改）
    lines : 文本行
    color : 文字颜色（BGR）

    返回
    ----
    画好文字的图
    """
    for i, ln in enumerate(lines):
        cv2.putText(img, ln, (10, 26 + i * 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.62, (0, 0, 0), 4, cv2.LINE_AA)      # 黑描边，任何底色都看得清
        cv2.putText(img, ln, (10, 26 + i * 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.62, color, 1, cv2.LINE_AA)
    return img


# ===========================================================================
# 滑动条
# ===========================================================================
class TrackbarUI:
    """
    管理一组滑动条，并对外提供"当前实际取值"。

    滑动条的可调范围：H 0~179，S/V 0~255（OpenCV 的 HSV 就这个范围）。
    """

    def __init__(self, win: str, lower: Sequence[int] = DEFAULT_LOWER,
                 upper: Sequence[int] = DEFAULT_UPPER):
        """
        参数
        ----
        win   : 承载滑动条的窗口名
        lower : 初始 HSV 下限
        upper : 初始 HSV 上限
        """
        self.win = win
        self.init_lower = tuple(int(v) for v in lower)
        self.init_upper = tuple(int(v) for v in upper)

        for name, val, hi in (("H min", self.init_lower[0], 179),
                              ("S min", self.init_lower[1], 255),
                              ("V min", self.init_lower[2], 255),
                              ("H max", self.init_upper[0], 179),
                              ("S max", self.init_upper[1], 255),
                              ("V max", self.init_upper[2], 255)):
            cv2.createTrackbar(name, win, int(val), int(hi), lambda _v: None)
        cv2.createTrackbar("open k", win, 0, MAX_KERNEL, lambda _v: None)
        cv2.createTrackbar("close k", win, 0, MAX_KERNEL, lambda _v: None)
        cv2.createTrackbar("zoom", win, 4, 12, lambda _v: None)

    def get(self) -> Tuple[Tuple[int, int, int], Tuple[int, int, int], int, int, int]:
        """
        读取当前所有滑动条的值。

        返回
        ----
        (lower, upper, open_k, close_k, zoom)
        lower/upper 为 (H, S, V)
        """
        g = lambda n: int(cv2.getTrackbarPos(n, self.win))       # noqa: E731
        lower = (g("H min"), g("S min"), g("V min"))
        upper = (g("H max"), g("S max"), g("V max"))
        return lower, upper, g("open k"), g("close k"), g("zoom")

    def reset(self) -> None:
        """把所有滑动条恢复到初始值（r 键）"""
        for name, val in (("H min", self.init_lower[0]), ("S min", self.init_lower[1]),
                          ("V min", self.init_lower[2]), ("H max", self.init_upper[0]),
                          ("S max", self.init_upper[1]), ("V max", self.init_upper[2])):
            cv2.setTrackbarPos(name, self.win, int(val))
        cv2.setTrackbarPos("open k", self.win, 0)
        cv2.setTrackbarPos("close k", self.win, 0)
        cv2.setTrackbarPos("zoom", self.win, 4)
        print("[重置] 阈值已恢复为初始值")


def _sanity_warn(lower, upper) -> None:
    """
    阈值明显不合理时给出提醒（只在终端打印，不打断操作）。

    参数
    ----
    lower, upper : 当前 HSV 上下限
    """
    if any(u < l for l, u in zip(lower, upper)):
        print("[提醒] 存在 max < min 的通道，mask 会是全黑：请把 max 调大或 min 调小")


def format_result(lower, upper, open_k: int = 0, close_k: int = 0) -> str:
    """
    把当前阈值格式化成可直接粘进配置的文本。

    参数
    ----
    lower, upper  : HSV 上下限
    open_k, close_k : 形态学核大小

    返回
    ----
    多行字符串
    """
    lines = [
        "--------------- copy into config/slots.yaml ---------------",
        "slot_monitor:",
        "  dot_hsv_lower: [%d, %d, %d]" % tuple(lower),
        "  dot_hsv_upper: [%d, %d, %d]" % tuple(upper),
    ]
    if open_k >= 3 or close_k >= 3:
        lines.append("  # 形态学: open=%d close=%d（酌情加进视觉模块）"
                     % (open_k, close_k))
    lines.append("# 一行版：dot_hsv_lower=(%d,%d,%d)  dot_hsv_upper=(%d,%d,%d)"
                 % (tuple(lower) + tuple(upper)))
    lines.append("----------------------------------------------------------")
    return "\n".join(lines)


def copy_to_clipboard(text: str) -> bool:
    """
    尽力把文本复制到系统剪贴板（失败不影响主流程）。

    Windows 用自带的 clip；macOS 用 pbcopy；Linux 依次尝试 xclip/xsel/wl-copy。

    参数
    ----
    text : 要复制的文本

    返回
    ----
    True 已调用某个剪贴板工具；False 全部不可用（终端里手动复制即可）
    """
    import subprocess
    try:
        if sys.platform.startswith("win"):
            subprocess.run("clip", input=text.encode("utf-8"), shell=True,
                           timeout=3, check=False)
            return True
        if sys.platform == "darwin":
            subprocess.run(["pbcopy"], input=text.encode("utf-8"),
                           timeout=3, check=False)
            return True
        for cmd in (["xclip", "-selection", "clipboard"], ["xsel", "--clipboard"],
                    ["wl-copy"]):
            try:
                p = subprocess.run(cmd, input=text.encode("utf-8"),
                                   timeout=3, check=False)
                if p.returncode == 0:
                    return True
            except FileNotFoundError:
                continue
    except Exception:
        pass
    return False


# ===========================================================================
# 无界面：颜色统计
# ===========================================================================
def analyze_region(bgr: "np.ndarray", roi: Optional[Sequence[int]] = None,
                   low_pct: float = 2.0, high_pct: float = 98.0) -> Optional[dict]:
    """
    统计 ROI 内 HSV 各通道的分位数，给出"颜色大概落在什么范围"的建议值。

    这是给无图形界面环境（服务器/沙盒）用的：滑动条用不了，但可以直接
    拿到一个靠谱的初始阈值，到了有界面的机器上再微调。

    参数
    ----
    bgr      : BGR 图
    roi      : (x, y, w, h)，None 表示整幅图
    low_pct  : 下分位（默认 2%，过滤离群点）
    high_pct : 上分位（默认 98%）

    ！注意：这个统计是"整块区域的主色"，如果 ROI 里既有目标色又有背景色，
    结果会被背景带偏。所以请把 ROI 框在目标颜色那块上。

    返回
    ----
    dict，包含 lower/upper/median/count 等；输入无效返回 None
    """
    if bgr is None or bgr.size == 0:
        return None
    sub, _org = crop_roi(bgr, roi)
    if sub.size == 0:
        return None

    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    # 排除过于暗的像素（黑背景没有可靠色相，会把 H 的统计拉偏）
    v = hsv[:, 2]
    keep = hsv[v > max(30, int(np.percentile(v, 20)))]
    if keep.size == 0:
        keep = hsv

    lower = tuple(int(np.percentile(keep[:, i], low_pct)) for i in range(3))
    upper = tuple(int(np.percentile(keep[:, i], high_pct)) for i in range(3))
    median = tuple(int(np.median(keep[:, i])) for i in range(3))
    return {"lower": lower, "upper": upper, "median": median,
            "count": int(keep.shape[0]), "total": int(hsv.shape[0])}


# ===========================================================================
# 交互主循环
# ===========================================================================
def run_interactive(cam, roi: Optional[Sequence[int]], out_dir: str) -> int:
    """
    打开三个窗口，进入"拖滑动条看效果"的交互循环。

    参数
    ----
    cam     : 提供 read() -> (ok, frame) 的对象
    roi     : (x, y, w, h) 或 None
    out_dir : e 键导出图片的目录

    返回
    ----
    0 正常退出
    """
    cv2.namedWindow(WIN_TRACK, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_MAIN, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_RESULT, cv2.WINDOW_NORMAL)
    ui = TrackbarUI(WIN_TRACK)

    print("-" * 70)
    print("操作：拖滑动条调阈值 | s=打印并复制当前范围 | e=导出结果图 | r=重置 | q=退出")
    print("提示：优先调 H min/H max 圈定色相，再用 S/V 排除灰白或太暗的干扰")
    if roi:
        print("当前 ROI=%s（只处理这块区域，并放大显示）" % (list(roi),))
    else:
        print("当前为整幅画面；建议加 --roi x,y,w,h 只看目标区域，看得更清楚")
    print("-" * 70)

    try:
        while True:
            ok, frame = cam.read()
            if not ok or frame is None:
                print("[错误] 读取画面失败")
                break

            lower, upper, open_k, close_k, zoom = ui.get()
            _sanity_warn(lower, upper)

            sub, org = crop_roi(frame, roi)

            # ROI 放大：小圆点在小窗口里根本看不清，这是本工具的关键一步
            if roi and zoom > 1:
                z = max(1, int(zoom))
                sub_big = cv2.resize(sub, (sub.shape[1] * z, sub.shape[0] * z),
                                     interpolation=cv2.INTER_NEAREST)
            else:
                sub_big = sub

            mask = build_mask(sub_big, lower, upper, open_k, close_k)
            result = overlay_mask(sub_big, mask)

            # 在 mask 上统计命中比例，便于判断"是不是太多/太少"
            ratio = cv2.countNonZero(mask) / float(mask.size if mask.size else 1)

            left = sub_big.copy()
            if roi:                                   # 放大图只到 ROI，看不到范围，标注一下
                _annotate(left, ["ROI %s  zoom x%d" % (list(org), max(1, int(zoom)))])
            _annotate(mask, ["mask  hit=%.1f%%" % (ratio * 100.0)])
            view = _hstack(_fit_width(_to_view(left)), _fit_width(_to_view(mask)))
            view = _annotate(view, ["H %d-%d  S %d-%d  V %d-%d  open=%d close=%d"
                                    % (lower[0], upper[0], lower[1], upper[1],
                                       lower[2], upper[2], open_k, close_k)])
            cv2.imshow(WIN_MAIN, view)
            cv2.imshow(WIN_RESULT, _fit_width(_to_view(result)))

            key = cv2.waitKey(30) & 0xFF
            if key in (113, 27):                     # q / ESC
                break
            if key == 115:                           # s：打印 + 复制
                text = format_result(lower, upper, open_k, close_k)
                print("\n[当前阈值]")
                print(text)
                if copy_to_clipboard(text):
                    print("[剪贴板] 已复制，可直接 Ctrl+V 粘进配置")
                else:
                    print("[剪贴板] 系统剪贴板不可用，请手动复制上面的文本")
            elif key == 101:                         # e：导出
                os.makedirs(out_dir, exist_ok=True)
                ts = time.strftime("%H%M%S")
                p1 = os.path.join(out_dir, "hsv_result_%s.png" % ts)
                p2 = os.path.join(out_dir, "hsv_mask_%s.png" % ts)
                cv2.imwrite(p1, result)
                cv2.imwrite(p2, mask)
                print("[导出] %s" % p1)
                print("[导出] %s" % p2)
            elif key == 114:                         # r：重置
                ui.reset()
    finally:
        cv2.destroyAllWindows()
        try:
            cam.release()
        except Exception:
            pass
    return 0


# ===========================================================================
# 无界面环境探测
# ===========================================================================
def _display_reachable(disp: str) -> bool:
    """
    探测 X11/Wayland 显示服务是否真的可连接。

    原因：容器里常出现 DISPLAY=:0 但没有 X server 的情况，此时 cv2.imshow()
    会让 Qt 直接 abort 进程（退出码 134），try/except 拦不住。必须先探测。

    参数
    ----
    disp : DISPLAY 环境变量值

    返回
    ----
    True 可用图形界面
    """
    import socket
    wl = os.environ.get("WAYLAND_DISPLAY")
    if wl:
        rt = os.environ.get("XDG_RUNTIME_DIR") or ("/run/user/%d" % os.getuid())
        return os.path.exists(os.path.join(rt, wl))
    if not disp:
        return False
    try:
        host, _, tail = disp.rpartition(":")
        no = tail.split(".")[0]
        if not host or host == "unix":
            path = "/tmp/.X11-unix/X%s" % no
            if not os.path.exists(path):
                return False
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(1.0)
            try:
                s.connect(path)
                return True
            finally:
                s.close()
        s = socket.create_connection((socket.gethostbyname(host), 6000 + int(no)),
                                     timeout=1.0)
        s.close()
        return True
    except Exception:
        return False


def gui_available() -> bool:
    """
    当前环境能否弹 OpenCV 窗口。

    返回
    ----
    True 可以显示；False 无图形界面
    """
    if sys.platform.startswith("win") or sys.platform == "darwin":
        return True
    return _display_reachable(os.environ.get("DISPLAY", ""))


# ===========================================================================
# 命令行
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
        description="智能药箱 - HSV 颜色阈值调试（滑动条实时看 mask，s 键导出数值）")
    p.add_argument("--camera", type=int, default=0, help="相机序号，默认 0")
    p.add_argument("--image", default=None,
                   help="用静态照片代替摄像头（没有摄像头时用）")
    p.add_argument("--roi", default=None,
                   help="只调这块区域，格式 x,y,w,h，如 120,90,60,60")
    p.add_argument("--lower", default=None,
                   help="初始 HSV 下限，格式 H,S,V，如 35,60,60")
    p.add_argument("--upper", default=None,
                   help="初始 HSV 上限，格式 H,S,V，如 85,255,255")
    p.add_argument("--width", type=int, default=1280, help="相机请求宽度")
    p.add_argument("--height", type=int, default=720, help="相机请求高度")
    p.add_argument("--analyze", action="store_true",
                   help="不进界面：统计 ROI 内 HSV 分位数并打印建议阈值（无界面环境用）")
    p.add_argument("--out", default=os.path.join("output", "color_tuner"),
                   help="e 键/分析结果的输出目录")
    return p.parse_args(argv)


def _parse_triple(text: Optional[str], default: Sequence[int]) -> Tuple[int, int, int]:
    """
    解析 "H,S,V" 形式的参数。

    参数
    ----
    text    : 输入字符串，None/空则用默认值
    default : 默认三元组

    返回
    ----
    (h, s, v) 整数三元组
    """
    if not text:
        return tuple(int(v) for v in default)          # type: ignore[return-value]
    parts = [p.strip() for p in str(text).split(",") if p.strip()]
    try:
        return tuple(int(float(p)) for p in parts[:3])  # type: ignore[return-value]
    except Exception:
        print("[警告] 解析 %r 失败，使用默认值 %s" % (text, list(default)))
        return tuple(int(v) for v in default)          # type: ignore[return-value]


def main(argv=None) -> int:
    """
    程序入口：取图 ->（--analyze 打印统计 | 进入交互调参）。

    参数
    ----
    argv : 命令行参数列表

    返回
    ----
    退出码：0 成功，非 0 失败
    """
    args = parse_args(argv)
    roi = parse_roi(args.roi)
    lower = _parse_triple(args.lower, DEFAULT_LOWER)
    upper = _parse_triple(args.upper, DEFAULT_UPPER)

    # ---- 取一帧画面：静态图 或 相机 ----
    cam = None
    frame = None
    if args.image:
        if not os.path.isfile(args.image):
            print("[错误] 图片不存在：%s" % args.image)
            return 2
        try:
            cam = MockCamera(args.image)
        except Exception as e:
            print("[错误] 无法读取图片 %s：%r" % (args.image, e))
            return 2
        ok, frame = cam.read()
        print("[画面] 静态照片模式：%s" % args.image)
    else:
        cam = Camera(index=args.camera, width=args.width, height=args.height)
        if not cam.open():
            print("[错误] 打不开相机 %d。检查：1) 是否被占用 2) 序号对不对 "
                  "3) 没摄像头就用 --image 照片" % args.camera)
            return 2
        ok, frame = cam.read()
        print("[画面] 实时相机模式：index=%d" % args.camera)

    if not ok or frame is None:
        print("[错误] 读不到画面")
        cam.release()
        return 2
    print("[画面] 分辨率：%dx%d%s" % (frame.shape[1], frame.shape[0],
                                      ("，ROI=%s" % (list(roi),)) if roi else ""))

    # ---- 无界面：统计模式 ----
    if args.analyze or not gui_available():
        if not args.analyze:
            print("[提示] 当前环境没有图形界面（无 X server），改用统计模式。")
            print("       要在有桌面的机器上拖滑动条，请在有屏幕的环境运行本脚本。")
        stat = analyze_region(frame, roi, 2.0, 98.0)
        cam.release() if not args.image else None
        if stat is None:
            print("[错误] 统计失败（ROI 是否越界？）")
            return 1

        print("\n[统计] ROI 内有效像素 %d / %d" % (stat["count"], stat["total"]))
        print("[统计] HSV 分位数(2%~98%) -> 建议初始阈值：")
        print(format_result(stat["lower"], stat["upper"]))
        print("[统计] 中位数 HSV = %s（用它判断主色相在哪个通道）" % (stat["median"],))
        print("[提醒] 这是【整块 ROI 的主色】统计：如果 ROI 里混了背景色，结果会被带偏，")
        print("       请把 --roi 框在目标颜色那一块上，再到有界面的机器上微调。")

        # 顺手导出可视化，方便判断 ROI 选得对不对
        os.makedirs(args.out, exist_ok=True)
        sub, _org = crop_roi(frame, roi)
        mask = build_mask(sub, stat["lower"], stat["upper"])
        cv2.imwrite(os.path.join(args.out, "analyze_result.png"),
                    overlay_mask(sub, mask))
        cv2.imwrite(os.path.join(args.out, "analyze_mask.png"), mask)
        print("[导出] %s" % os.path.join(args.out, "analyze_result.png"))
        return 0

    return run_interactive(cam, roi, args.out)


if __name__ == "__main__":
    sys.exit(main())
