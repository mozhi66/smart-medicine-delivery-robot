#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 槽位状态监测模块 (slot_monitor.py)
================================================================================
职责：判断"某个槽位里现在有没有药"。

为什么不用"直接识别二维码"来判断有无？
    因为药瓶放在槽里时会把贴在槽底的码挡住——看见码反而说明没药。
    这条路逻辑能走通但反直觉、且受遮挡影响大。
    更稳的做法是下面【三级判据】，任一可用即可，按可靠性排序：

  判据1（最稳，强烈推荐）槽底彩色圆点可见性
      每个槽底贴一个固定颜色的圆点。有药瓶压住 -> 看不见；
      空槽 -> 看得见。本质是"有没有东西盖住这个位置"，
      与瓶子颜色/形状/光照无关，极其稳定。

  判据2（次选）背景帧差
      开局拍一张"空箱背景"，之后每帧与它做差分。
      适合已经做好硬件、不想再贴圆点的场合，对光照变化敏感。

  判据3（兜底）药瓶色块面积
      在槽内统计特定 HSV 范围内的像素占比。
      只在药瓶颜色统一且与环境对比明显时才好用。

SlotMonitor 把三种判据统一成一个 check() 接口，配置里切 mode 即可。
================================================================================
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class SlotROI:
    """一个槽位在图像中的矩形区域"""
    slot_id: int
    x: int
    y: int
    w: int
    h: int

    @property
    def rect(self) -> Tuple[int, int, int, int]:
        """返回 (x, y, w, h)，方便直接切片"""
        return (self.x, self.y, self.w, self.h)


class SlotMonitor:
    """
    槽位占用状态监测器。

    用法（判据1：槽底圆点）
    --------
        mon = SlotMonitor(slots, mode="dot", dot_hsv_lower=(40,80,80),
                                              dot_hsv_upper=(80,255,255),
                                              dot_min_area=150)
        state = mon.check(frame)          # {1: True, 2: False, ...} True=有药
        print(mon.describe(frame, state)) # 画在图上看看

    用法（判据2：背景帧差）
    --------
        mon = SlotMonitor(slots, mode="diff")
        mon.set_background(empty_frame)   # 空箱时拍一张存下来
        state = mon.check(frame)
    """

    def __init__(self, slots: Sequence[SlotROI], mode: str = "dot",
                 diff_threshold: int = 30,
                 occupy_ratio: float = 0.08,
                 dot_hsv_lower: Sequence[int] = (35, 60, 60),
                 dot_hsv_upper: Sequence[int] = (85, 255, 255),
                 dot_min_area: int = 120,
                 bottle_hsv_lower: Sequence[int] = (0, 0, 0),
                 bottle_hsv_upper: Sequence[int] = (179, 255, 255),
                 bottle_min_ratio: float = 0.15):
        """
        参数
        ----
        slots           : 槽位 ROI 列表
        mode            : 'dot'(槽底圆点) | 'diff'(背景帧差) | 'blob'(药瓶色块)
        diff_threshold  : 判据2 的差分二值化阈值(0-255)
        occupy_ratio    : 判据2 判定"有药"的非零像素占比阈值
        dot_hsv_lower/upper : 判据1 槽底圆点的 HSV 范围
        dot_min_area    : 判据1 圆点最小面积（小于它认为被挡住了）
        bottle_hsv_*    : 判据3 药瓶颜色的 HSV 范围
        bottle_min_ratio: 判据3 判定"有药"的色块占比阈值
        """
        self.slots: List[SlotROI] = list(slots)
        self.mode = mode
        self.diff_threshold = int(diff_threshold)
        self.occupy_ratio = float(occupy_ratio)
        self.dot_lower = np.array(dot_hsv_lower, dtype=np.uint8)
        self.dot_upper = np.array(dot_hsv_upper, dtype=np.uint8)
        self.dot_min_area = int(dot_min_area)
        self.bottle_lower = np.array(bottle_hsv_lower, dtype=np.uint8)
        self.bottle_upper = np.array(bottle_hsv_upper, dtype=np.uint8)
        self.bottle_min_ratio = float(bottle_min_ratio)

        self._bg: Dict[int, "np.ndarray"] = {}      # 各槽位的空箱背景（灰度+模糊）
        self._kernel = np.ones((5, 5), np.uint8)

    # ------------------------------------------------------------------
    def set_background(self, frame: "np.ndarray") -> None:
        """
        采集"空箱背景"——必须在所有槽位都没有药的时候调用。

        参数
        ----
        frame : 空箱状态下的相机画面
        """
        self._bg = {}
        for s in self.slots:
            crop = self._crop(frame, s)
            if crop is not None:
                g = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                self._bg[s.slot_id] = cv2.GaussianBlur(g, (5, 5), 0)

    # ------------------------------------------------------------------
    def _crop(self, frame: "np.ndarray", s: SlotROI) -> Optional["np.ndarray"]:
        """从整帧里安全裁出某个槽位（越界自动裁剪，不会抛异常）"""
        if frame is None:
            return None
        H, W = frame.shape[:2]
        x1 = max(0, min(s.x, W - 1)); y1 = max(0, min(s.y, H - 1))
        x2 = max(x1 + 1, min(s.x + s.w, W)); y2 = max(y1 + 1, min(s.y + s.h, H))
        return frame[y1:y2, x1:x2]

    # ------------------------------------------------------------------
    def _judge_dot(self, crop: "np.ndarray") -> bool:
        """
        判据1：槽底圆点可见性。
        看得见圆点 -> 空槽(False)；看不见 -> 有药(True)
        """
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.dot_lower, self.dot_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        area = int(cv2.countNonZero(mask))
        return area < self.dot_min_area        # 圆点被遮住 => 有药

    # ------------------------------------------------------------------
    def _judge_diff(self, crop: "np.ndarray", slot_id: int) -> Optional[bool]:
        """
        判据2：与空箱背景做帧差。
        差异像素占比 > occupy_ratio -> 有药。没设背景返回 None。
        """
        bg = self._bg.get(slot_id)
        if bg is None:
            return None
        g = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        g = cv2.GaussianBlur(g, (5, 5), 0)
        if g.shape != bg.shape:                # 尺寸不一致就缩放到背景尺寸
            g = cv2.resize(g, (bg.shape[1], bg.shape[0]))
        diff = cv2.absdiff(g, bg)
        _, mask = cv2.threshold(diff, self.diff_threshold, 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        ratio = cv2.countNonZero(mask) / float(mask.size)
        return ratio > self.occupy_ratio

    # ------------------------------------------------------------------
    def _judge_blob(self, crop: "np.ndarray") -> bool:
        """
        判据3：药瓶颜色色块占比。
        指定 HSV 范围内像素占比 > bottle_min_ratio -> 有药。
        """
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.bottle_lower, self.bottle_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        ratio = cv2.countNonZero(mask) / float(mask.size)
        return ratio > self.bottle_min_ratio

    # ------------------------------------------------------------------
    def check(self, frame: "np.ndarray") -> Dict[int, bool]:
        """
        检测所有槽位的占用状态。

        参数
        ----
        frame : 当前相机画面

        返回
        ----
        {slot_id: bool}，True 表示该槽位有药。
        判据2 若未设置背景会自动退化为判据1。
        """
        state: Dict[int, bool] = {}
        for s in self.slots:
            crop = self._crop(frame, s)
            if crop is None or crop.size == 0:
                state[s.slot_id] = False
                continue
            if self.mode == "diff":
                r = self._judge_diff(crop, s.slot_id)
                if r is None:
                    r = self._judge_dot(crop)
                state[s.slot_id] = bool(r)
            elif self.mode == "blob":
                state[s.slot_id] = bool(self._judge_blob(crop))
            else:
                state[s.slot_id] = bool(self._judge_dot(crop))
        return state

    # ------------------------------------------------------------------
    def describe(self, frame: "np.ndarray",
                 state: Optional[Dict[int, bool]] = None) -> "np.ndarray":
        """
        把槽位框和状态画到图上，返回一张可视化图（绿=有药，灰=空）。
        调试和答辩演示都靠它。
        """
        vis = frame.copy()
        if state is None:
            state = self.check(frame)
        for s in self.slots:
            occupied = state.get(s.slot_id, False)
            color = (60, 200, 60) if occupied else (140, 140, 140)
            cv2.rectangle(vis, (s.x, s.y), (s.x + s.w, s.y + s.h), color, 2)
            label = "S%d:%s" % (s.slot_id, "HAVE" if occupied else "EMPTY")
            cv2.putText(vis, label, (s.x + 3, s.y + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        return vis


# ============================================================================
# 便捷函数：用配置文件里的槽位定义构造 SlotMonitor
# ============================================================================
def build_from_config(cfg: dict) -> SlotMonitor:
    """
    从配置字典构造 SlotMonitor。

    cfg 期望结构（对应 config/slots.yaml）：
        slot_monitor:
          mode: dot
          dot_hsv_lower: [35, 60, 60]
          dot_hsv_upper: [85, 255, 255]
          dot_min_area: 120
          diff_threshold: 30
          occupy_ratio: 0.08
        slots:
          - {id: 1, roi: [x, y, w, h]}
          ...
    """
    sm = cfg.get("slot_monitor", {}) or {}
    roi_list = []
    for item in cfg.get("slots", []) or []:
        x, y, w, h = item["roi"]
        roi_list.append(SlotROI(slot_id=int(item["id"]),
                                x=int(x), y=int(y), w=int(w), h=int(h)))
    return SlotMonitor(
        roi_list,
        mode=sm.get("mode", "dot"),
        diff_threshold=sm.get("diff_threshold", 30),
        occupy_ratio=sm.get("occupy_ratio", 0.08),
        dot_hsv_lower=sm.get("dot_hsv_lower", (35, 60, 60)),
        dot_hsv_upper=sm.get("dot_hsv_upper", (85, 255, 255)),
        dot_min_area=sm.get("dot_min_area", 120),
    )
