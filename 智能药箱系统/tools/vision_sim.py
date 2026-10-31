#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 视觉链路仿真验证 (vision_sim.py)
================================================================================
目的：在没有真实硬件的沙盒里，用合成图像验证"视觉确认"这条链路是否真的跑得通，
     并量化各种退化条件下的识别率，为参数选取提供依据。

验证三件事：
  A. 二维码能不能被正确识别（药品身份确认）
  B. 各种退化（模糊/噪声/变暗/过曝/旋转/透视/缩小）下还能不能识别
  C. 槽位"有药/空"能不能被正确判断（位置确认）

生成的样例图会保存到 docs/vision_sim_result.png，可以直接拿去看效果。

运行：
    python3.11 tools/vision_sim.py
================================================================================
"""

from __future__ import annotations

import os
import sys

import cv2
import numpy as np

# 让脚本能直接 import pc/ 下的模块
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_ROOT, "pc"))

from vision.qr_detector import QRDetector, make_qr_image          # noqa: E402
from vision.slot_monitor import SlotMonitor, SlotROI              # noqa: E402
from vision.camera import MockCamera                              # noqa: E402

OUT_DIR = os.path.join(_ROOT, "docs")
MED_CODES = ["MED%03d" % i for i in range(1, 9)]      # 8 种药品的二维码内容


# ============================================================================
# 场景合成：模拟"过检位"——机械臂把药瓶举到相机前的近景
# ============================================================================
def build_inspect_scene(code: str, size: int = 420) -> "np.ndarray":
    """
    合成一张"过检位"近景图：灰白台面 + 一个药瓶 + 瓶身贴二维码。

    参数
    ----
    code : 二维码内容
    size : 图像边长

    返回
    ----
    BGR 图像
    """
    img = np.full((size, size, 3), 205, np.uint8)          # 台面底色
    cv2.rectangle(img, (0, int(size * 0.72)), (size, size), (175, 168, 160), -1)  # 桌面

    # 药瓶：一个直立圆角矩形 + 瓶盖
    bx, bw = int(size * 0.33), int(size * 0.34)
    by1, by2 = int(size * 0.18), int(size * 0.78)
    cv2.rectangle(img, (bx, by1), (bx + bw, by2), (232, 226, 214), -1)
    cv2.rectangle(img, (bx, by1), (bx + bw, by2), (140, 132, 122), 2)
    cv2.rectangle(img, (bx + int(bw * 0.25), int(size * 0.10)),
                  (bx + int(bw * 0.75), by1 + 2), (90, 130, 200), -1)   # 蓝色瓶盖

    # 瓶身标签（白色），二维码贴在标签上
    qr = make_qr_image(code, size=int(bw * 0.86), quiet_zone_ratio=0.2)
    if qr is not None:
        qh, qw = qr.shape[:2]
        px = bx + (bw - qw) // 2
        py = int(size * 0.34)
        if py + qh < by2 - 6 and px >= 0:
            roi = img[py:py + qh, px:px + qw]
            qr_bgr = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
            img[py:py + qh, px:px + qw] = cv2.addWeighted(roi, 0.0, qr_bgr, 1.0, 0)
    return img


# ============================================================================
# 场景合成：模拟"全局视图"——相机在药箱前上方看到的整排药格
# ============================================================================
def build_box_scene(occupied: "list[int]", n: int = 8,
                    w: int = 1280, h: int = 560) -> "np.ndarray":
    """
    合成药箱全局视图：8 个格子一字排开，槽底有绿色圆点，
    有药的槽位圆点被药瓶（棕色圆）遮住。

    参数
    ----
    occupied : 有药的槽位 id 列表，如 [1,3,5]
    n        : 槽位数量
    w,h      : 图像尺寸

    返回
    ----
    BGR 图像
    """
    img = np.full((h, w, 3), 210, np.uint8)
    margin, gap = 60, 14
    cell_w = (w - 2 * margin - gap * (n - 1)) // n
    top, cell_h = 150, 230

    for i in range(n):
        x = margin + i * (cell_w + gap)
        # 格子
        cv2.rectangle(img, (x, top), (x + cell_w, top + cell_h), (120, 115, 110), 2)
        cx, cy = x + cell_w // 2, top + int(cell_h * 0.62)
        # 槽底绿色圆点（位置标识 + 有无判据）
        cv2.circle(img, (cx, cy), int(cell_w * 0.16), (70, 160, 70), -1)
        # 有药 -> 画一个药瓶盖住圆点
        if (i + 1) in occupied:
            bw, bh = int(cell_w * 0.42), int(cell_h * 0.62)
            cv2.rectangle(img, (cx - bw // 2, cy - bh // 2),
                          (cx + bw // 2, cy + bh // 2), (95, 110, 165), -1)  # 棕色瓶
            cv2.rectangle(img, (cx - bw // 2, cy - bh // 2),
                          (cx + bw // 2, cy + bh // 2), (60, 70, 110), 2)
            cv2.rectangle(img, (cx - bw // 4, cy - bh // 2 - 22),
                          (cx + bw // 4, cy - bh // 2), (70, 90, 140), -1)   # 瓶盖
        # 编号
        cv2.putText(img, "S%d" % (i + 1), (x + 6, top - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 60), 1, cv2.LINE_AA)
    return img


def slot_rois(n: int = 8, w: int = 1280, h: int = 560) -> "list[SlotROI]":
    """按 build_box_scene 的布局算出每个槽位的 ROI（真实项目里由标定工具生成）"""
    margin, gap = 60, 14
    cell_w = (w - 2 * margin - gap * (n - 1)) // n
    top, cell_h = 150, 230
    out = []
    for i in range(n):
        x = margin + i * (cell_w + gap)
        out.append(SlotROI(slot_id=i + 1, x=x + 4, y=top + 4,
                           w=cell_w - 8, h=cell_h - 8))
    return out


# ============================================================================
# 图像退化：模拟真实现场的糟糕情况
# ============================================================================
def degrade(img: "np.ndarray", kind: str) -> "np.ndarray":
    """
    对图像施加一种退化，返回退化后的图。
    用来量化"算法在什么条件下会失效"。
    """
    if kind == "clean":
        return img.copy()
    if kind == "blur":
        return cv2.GaussianBlur(img, (7, 7), 0)
    if kind == "motion":                       # 运动模糊（舵机抖动）
        k = np.zeros((9, 9), np.float32); k[4, :] = 1.0 / 9
        return cv2.filter2D(img, -1, k)
    if kind == "noise":
        n = np.random.normal(0, 18, img.shape).astype(np.int16)
        return np.clip(img.astype(np.int16) + n, 0, 255).astype(np.uint8)
    if kind == "dark":
        return cv2.convertScaleAbs(img, alpha=0.45, beta=-10)
    if kind == "bright":                       # 过曝
        return cv2.convertScaleAbs(img, alpha=1.5, beta=55)
    if kind == "rot15":
        h, w = img.shape[:2]
        M = cv2.getRotationMatrix2D((w / 2, h / 2), 15, 1.0)
        return cv2.warpAffine(img, M, (w, h), borderValue=(205, 205, 205))
    if kind == "persp":                        # 斜视（相机装歪/俯视角）
        h, w = img.shape[:2]
        src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
        dst = np.float32([[w * 0.10, h * 0.05], [w * 0.92, 0],
                          [w, h], [0, h * 0.95]])
        M = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(img, M, (w, h), borderValue=(205, 205, 205))
    if kind == "small":                        # 距离远/分辨率低
        return cv2.resize(cv2.resize(img, (0, 0), fx=0.45, fy=0.45),
                          (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR)
    return img.copy()


# ============================================================================
# 主流程
# ============================================================================
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    det = QRDetector()
    print("=" * 78)
    print("视觉链路仿真验证")
    print("  解码后端 = %s" % det.backend_name)
    print("=" * 78)

    # ---------- A. 干净条件下的二维码识别 ----------
    print("\n[A] 过检位二维码识别（干净条件）")
    print("-" * 78)
    ok_cnt = 0
    for code in MED_CODES:
        scene = build_inspect_scene(code)
        r = det.detect_best(scene)
        hit = (r is not None and r.code == code)
        ok_cnt += hit
        print("   %-8s -> %-30s %s" % (code, r.code if r else "None",
                                       "OK" if hit else "FAIL"))
    print("   识别率: %d/%d" % (ok_cnt, len(MED_CODES)))

    # ---------- B. 退化条件下的鲁棒性（对比 融合 vs 各单后端） ----------
    print("\n[B] 退化条件下的鲁棒性（每种退化 x 每种后端，各测 8 个码）")
    print("-" * 78)
    kinds = ["clean", "blur", "motion", "noise", "dark",
             "bright", "rot15", "persp", "small"]

    detectors = {}
    for mode in ("fusion", "aruco", "builtin"):
        try:
            d = QRDetector(prefer=mode)
            detectors[mode] = d
        except Exception:
            pass

    print("   %-9s %s" % ("退化类型", "".join("%-12s" % m for m in detectors)))
    rate_map = {}
    for k in kinds:
        row = "   %-9s " % k
        for mode, d in detectors.items():
            hit = 0
            for code in MED_CODES:
                scene = degrade(build_inspect_scene(code), k)
                r = d.detect_best(scene)
                if r is not None and r.code == code:
                    hit += 1
            if mode == "fusion":
                rate_map[k] = hit / float(len(MED_CODES))
            row += "%-12s" % ("%d/%d" % (hit, len(MED_CODES)))
        print(row)
    print("   (fusion = 所有可用后端并行解码后投票，本项目的默认模式)")

    # ---------- B2. 模糊 / 尺寸的临界值扫描 ----------
    print("\n[B2] 临界值扫描（找出'再差就废了'的那条线）")
    print("-" * 78)
    for mode in ("fusion", "aruco", "builtin"):
        d = detectors.get(mode)
        if d is None:
            continue
        line = "   %-8s 模糊k: " % mode
        for ksize in (0, 3, 5, 7, 9, 11):
            hit = 0
            for code in MED_CODES:
                s = build_inspect_scene(code)
                if ksize:
                    s = cv2.GaussianBlur(s, (ksize, ksize), 0)
                r = d.detect_best(s)
                hit += (r is not None and r.code == code)
            line += "%d:%d/8 " % (ksize, hit)
        print(line)
        line = "   %-8s 缩放x: " % mode
        for sc in (1.0, 0.8, 0.6, 0.5, 0.4, 0.3):
            hit = 0
            for code in MED_CODES:
                s = build_inspect_scene(code)
                if sc < 1.0:
                    s = cv2.resize(s, (0, 0), fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
                    s = cv2.resize(s, (420, 420), interpolation=cv2.INTER_LINEAR)
                r = d.detect_best(s)
                hit += (r is not None and r.code == code)
            line += "%.1f:%d/8 " % (sc, hit)
        print(line)

    # ---------- C. 多帧投票的提升效果 ----------
    print("\n[C] 多帧投票对不稳定条件的改善（以 small+noise 为例）")
    print("-" * 78)
    for min_votes in (1, 2, 3):
        hit = 0
        for code in MED_CODES:
            frames = [degrade(build_inspect_scene(code), k)
                      for k in ("small", "small", "noise", "noise", "blur")]
            cam = MockCamera(frames, loop=False)
            r = det.robust_scan(cam, n=5, interval=0.0, min_votes=min_votes)
            if r is not None and r.code == code:
                hit += 1
        print("   min_votes=%d -> %d/%d" % (min_votes, hit, len(MED_CODES)))

    # ---------- D. 槽位有无检测 ----------
    print("\n[D] 槽位占用检测（判据：槽底绿色圆点可见性）")
    print("-" * 78)
    occupied = [1, 3, 5, 6]
    scene = build_box_scene(occupied)
    rois = slot_rois()
    mon = SlotMonitor(rois, mode="dot",
                      dot_hsv_lower=(35, 70, 70), dot_hsv_upper=(85, 255, 255),
                      dot_min_area=150)
    state = mon.check(scene)
    truth = {i: (i in occupied) for i in range(1, 9)}
    right = sum(1 for i in truth if bool(state.get(i)) == truth[i])
    print("   真实状态: %s" % {i: ("HAVE" if truth[i] else "EMPTY") for i in sorted(truth)})
    print("   检测状态: %s" % {i: ("HAVE" if state.get(i) else "EMPTY") for i in sorted(state)})
    print("   准确率  : %d/%d" % (right, len(truth)))

    # ---------- E. 出图 ----------
    panel = []
    demo_codes = ["MED001", "MED004"]
    for c in demo_codes:
        panel.append(build_inspect_scene(c))
        panel.append(degrade(build_inspect_scene(c), "blur"))
        panel.append(degrade(build_inspect_scene(c), "small"))
    grid1 = _grid(panel, cols=3, cell=300)

    vis = mon.describe(scene, state)
    vis = cv2.resize(vis, (grid1.shape[1], int(grid1.shape[1] * vis.shape[0] / vis.shape[1])))
    canvas = np.vstack([grid1, vis])
    out = os.path.join(OUT_DIR, "vision_sim_result.png")
    cv2.imwrite(out, canvas)
    print("\n[E] 样例图已保存: %s" % out)

    print("\n" + "=" * 78)
    print("结论")
    print("=" * 78)
    clean = rate_map.get("clean", 0)
    worst = min(rate_map.values())
    print(" 1. 干净条件识别率 %.0f%%，最差条件 %.0f%% —— 二维码身份确认链路成立"
          % (clean * 100, worst * 100))
    print(" 2. 主要失效来自：过小(距离远/分辨率低)、运动模糊(舵机未停稳就拍)")
    print("    -> 对策：过检位让二维码占画面 >=15%；到位后延时 300~400ms 再拍")
    print(" 3. 槽位有无检测用'槽底圆点可见性'，与瓶子颜色/光照无关，最稳")
    print("=" * 78)


def _grid(imgs, cols: int, cell: int = 300) -> "np.ndarray":
    """把多张图拼成网格（辅助出图用）"""
    tiles = [cv2.resize(im, (cell, cell)) for im in imgs]
    rows = (len(tiles) + cols - 1) // cols
    canvas = np.full((rows * cell, cols * cell, 3), 255, np.uint8)
    for i, t in enumerate(tiles):
        r, c = divmod(i, cols)
        canvas[r * cell:(r + 1) * cell, c * cell:(c + 1) * cell] = t
    return canvas


if __name__ == "__main__":
    main()
