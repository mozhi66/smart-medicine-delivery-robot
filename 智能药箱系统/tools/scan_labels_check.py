#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 扫码标签核对 (scan_labels_check.py)
================================================================================
职责：用摄像头逐个扫描 output/qr_labels/ 里的药品标签，验证「打印出来的码
真能被相机认出来」，并把核对结果拼成一张总览图存到
output/scan_labels_check/labels_check.png。

为什么需要：二维码打印糊了/尺寸太小/反光，都会导致过检扫码失败——那一步
在取药流程里是「不出库不放行」的硬关卡，必须提前用本工具验证。

判定策略（防抖）：
  · 每个标签扫描 hold-s 秒，期间取 frames 帧；
  · 命中次数 >= min-votes 才判「识别通过」；
  · 认出的内容必须与标签文件名里的药品码一致（MED001...MED009）。

用法：
  python tools/scan_labels_check.py                     # 默认摄像头 0
  python tools/scan_labels_check.py --camera 1          # 指定摄像头 index
  python tools/scan_labels_check.py --hold-s 1.5 --frames 30 --min-votes 3
  python tools/scan_labels_check.py --mock              # 无相机：用标签图自验解码

退出码：0 = 全部通过；1 = 有失败或没法验。
================================================================================
"""
from __future__ import annotations

import argparse
import os
import re
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, os.path.join(_ROOT, "pc")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

LABELS_DIR = os.path.join(_ROOT, "output", "qr_labels")
OUT_DIR = os.path.join(_ROOT, "output", "scan_labels_check")


def _codes_from_labels() -> list:
    """从标签文件名提取药品码（label_MED001.png -> MED001）。"""
    if not os.path.isdir(LABELS_DIR):
        return []
    out = []
    for fn in sorted(os.listdir(LABELS_DIR)):
        m = re.match(r"label_(MED\d+)\.png$", fn)
        if m:
            out.append((m.group(1), os.path.join(LABELS_DIR, fn)))
    return out


def _mock_check(codes) -> list:
    """无相机模式：直接把标签图喂给解码器自验。"""
    print("[模式] --mock：用标签图片本身验证可解码性（不经过相机）")
    try:
        import cv2
        from vision import imgio  # noqa: F401  接管中文路径 imread/imwrite
        from vision.qr_detector import QRDetector
    except Exception as e:
        print("[FAIL] 缺 cv2 或 qr_detector：%s" % e)
        return [(c, False, "依赖缺失") for c, _ in codes]
    det = QRDetector()
    results = []
    for code, path in codes:
        img = cv2.imread(path)
        if img is None:
            results.append((code, False, "图读不出"))
            continue
        hits = det.detect(img)
        texts = [h.code for h in hits if getattr(h, "code", None)]
        ok = any(code in t for t in texts)
        results.append((code, ok, ("认出: %s" % texts[:2]) if ok else "未认出"))
    return results


def _camera_check(codes, cam_idx, hold_s, frames, min_votes) -> list:
    """相机模式：每个标签扫 hold_s 秒，按票数判定。"""
    try:
        import cv2
        from vision.qr_detector import QRDetector
    except Exception as e:
        print("[FAIL] 缺 cv2 或 qr_detector：%s" % e)
        return [(c, False, "依赖缺失") for c, _ in codes]

    cap = cv2.VideoCapture(cam_idx)
    if not cap.isOpened():
        print("[FAIL] 打不开摄像头 index %d（换 --camera 或插好 USB）" % cam_idx)
        return [(c, False, "无相机") for c, _ in codes]
    det = QRDetector()
    results = []
    for code, _path in codes:
        print("-" * 70)
        print("[扫描] %s —— 请在 %.1f 秒内把该标签对准相机" % (code, hold_s))
        votes, seen = 0, []
        t_end = time.time() + hold_s
        n = 0
        while time.time() < t_end and n < frames:
            n += 1
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            hits = det.detect(frame)
            texts = [h.code for h in hits if getattr(h, "code", None)]
            if any(code in t for t in texts):
                votes += 1
                if texts[0] not in seen:
                    seen.append(texts[0])
            time.sleep(0.03)
        ok = votes >= min_votes
        detail = ("%d/%d 票，认出 %s" % (votes, n, seen[:2])) if ok else \
                 ("仅 %d 票（需 >= %d）" % (votes, min_votes))
        results.append((code, ok, detail))
        print("   %s %s：%s" % ("[ OK ]" if ok else "[FAIL]", code, detail))
    cap.release()
    return results


def _save_overview(results, codes) -> str:
    """把核对结果拼成总览图（每格：标签缩略图 + 药品码 + 判定）。"""
    try:
        import cv2
        import numpy as np
        from vision import imgio  # noqa: F401  接管中文路径 imread/imwrite
    except Exception:
        print("[WARN] 无 cv2/numpy，跳过总览图生成")
        return ""
    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, "labels_check.png")
    cell_w, cell_h, cols = 420, 300, 3
    rows = (len(codes) + cols - 1) // cols
    canvas = np.full((rows * cell_h + 60, cols * cell_w + 40, 3), 245, dtype="uint8")
    by_code = {c: (ok, detail) for c, ok, detail in results}
    for i, (code, path) in enumerate(codes):
        r, c = divmod(i, cols)
        x, y = 20 + c * cell_w, 50 + r * cell_h
        thumb = cv2.imread(path)
        if thumb is not None:
            th, tw = thumb.shape[:2]
            scale = min((cell_w - 30) / tw, (cell_h - 90) / th)
            thumb = cv2.resize(thumb, (max(1, int(tw * scale)), max(1, int(th * scale))))
            canvas[y + 40:y + 40 + thumb.shape[0], x:x + thumb.shape[1]] = thumb
        ok, _d = by_code.get(code, (False, ""))
        color = (60, 160, 60) if ok else (60, 60, 200)
        cv2.putText(canvas, "%s %s" % (code, "OK" if ok else "FAIL"),
                    (x, y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.putText(canvas, "scan_labels_check overview", (20, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (40, 40, 40), 2)
    cv2.imencode(".png", canvas)[1].tofile(out)  # 中文路径安全
    print("[输出] 总览图: %s" % out)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="扫码标签核对")
    ap.add_argument("--camera", type=int, default=0, help="摄像头 index")
    ap.add_argument("--hold-s", type=float, default=1.2, dest="hold_s",
                    help="每个标签扫描时长（秒）")
    ap.add_argument("--frames", type=int, default=40, help="最多取多少帧")
    ap.add_argument("--min-votes", type=int, default=3, dest="min_votes",
                    help="判定通过所需命中票数")
    ap.add_argument("--mock", action="store_true", help="无相机，用标签图自验")
    a = ap.parse_args(argv)

    print("=" * 70)
    print("扫码标签核对（scan_labels_check）")
    print("=" * 70)

    codes = _codes_from_labels()
    if not codes:
        print("[FAIL] %s 里没有 label_MED*.png" % LABELS_DIR)
        print("       先跑：python tools/make_qr_labels.py --individual")
        return 1
    print("[信息] 待核对标签 %d 个: %s" % (len(codes), ", ".join(c for c, _ in codes)))

    if a.mock:
        results = _mock_check(codes)
    else:
        results = _camera_check(codes, a.camera, a.hold_s, a.frames, a.min_votes)

    _save_overview(results, codes)

    bad = [c for c, ok, _ in results if not ok]
    print("=" * 70)
    if bad:
        print("[结论] %d/%d 未通过: %s" % (len(bad), len(results), ", ".join(bad)))
        print("       排查：打印尺寸是否太小 / 是否糊码 / 相机对焦与光照")
        return 1
    print("[结论] 全部 %d 个标签均可被识别——过检扫码关卡稳了" % len(results))
    return 0


if __name__ == "__main__":
    sys.exit(main())
