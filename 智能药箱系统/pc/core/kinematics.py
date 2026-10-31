#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
r"""
================================================================================
 机械臂运动学 (kinematics.py) —— 平面 3R 串联臂
================================================================================
职责：提供 4 舵机机械臂的正解(FK)与逆解(IK)。

机械模型（★ 按实机结构：3 个俯仰关节串联，末端限制在单一竖直平面内）
----------------------------------------------------------------------
        Z(上)
        |
        |                     ◇ TCP（夹爪中心）
        |                    /
        |                L3 /  第三段连杆（小臂 → 夹爪）
        |                  /
        |             ● 关节3（腕）
        |                |
        |            L2  |  第二段连杆（大臂）
        |                |
        |           ● 关节2（肘）
        |              |
        |          L1  |  第一段连杆（底座段）
        |              |
        |         ● 关节1（底座俯仰关节）
        |            |
        |         H0 |  基座立柱
        |            |
        O------------+------------------> X (水平前方)
        基座（固定在小车上）

自由度说明（这是全项目最重要的一个认知）
----------------------------------------
  4 个舵机 = 3 个俯仰关节 + 1 个夹爪开合。
  前 3 个关节**全部是俯仰摆动**（绕同一方向的水平轴），
  所以末端被限制在**单一竖直平面**内，工作空间是**平面扇形**。

  ★ 与"底座旋转(yaw) + 3D 空间"方案的本质区别：
      - 没有方位角自由度：末端只能在这一个竖直平面里前后/上下移动，
        药箱的所有槽位必须排在**同一个竖直平面**内；
      - 3 个俯仰关节 = 平面内 3 个自由度，而平面内
        "位置(2) + 末端姿态(1)" 也正好是 3 维，
        所以【位置和末端姿态可以同时指定】。
      - 这点很关键：抓竖直药瓶时可以要求夹爪保持竖直（phi = -90°），
        旧的三维方案做不到（末端姿态由位置决定，改不了）。

  第 4 个舵机是夹爪开合，不参与定位。

坐标约定
--------
  (x, z) 平面坐标，单位 mm：
      x = 水平前方距离（基座正前方为 +）
      z = 离地/台面高度
  所有槽位都取 y = 0（末端不能离开这个平面）。
================================================================================
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


# ============================================================================
# 舵机软限位 —— ★ 全项目唯一权威来源，必须与固件 `0stm/app/main.h` 一致 ★
# ============================================================================
# 顺序：s0 底座俯仰 / s1 第二段(大臂) / s2 第三段(小臂) / s3 夹爪
# 固件侧 app/servo.c 判定时会放宽 SERVO_LIMIT_EPS_DEG（0.1°）容差，
# 超过就回 `$ERR,03,LIM`（PROTO_ERR_LIMIT）并拒绝执行。
# 逆解必须用同一套限位，否则会出现"上位机认为可行、固件却拒绝"的假可行。
SERVO_LIMITS: Tuple[Tuple[float, float], ...] = (
    (0.0, 180.0),      # s0  SERVO0_LIMIT_MIN_DEG / SERVO0_LIMIT_MAX_DEG
    (15.0, 165.0),     # s1  SERVO1_LIMIT_MIN_DEG / SERVO1_LIMIT_MAX_DEG
    (0.0, 150.0),      # s2  SERVO2_LIMIT_MIN_DEG / SERVO2_LIMIT_MAX_DEG
    (140.0, 180.0),    # s3  夹爪（180=复位/张开 140=抓药夹紧；135 实测堵转卡死，禁用）
)


@dataclass
class ArmGeometry:
    """
    机械臂的几何与舵机安装参数（改机械臂后只需改这里）。

    舵机中位的含义（★ 标定后必须修正）：
        s0_home : 舵机 s0 取该角度时，**第一段连杆水平**（phi1 = 0）
        s1_home : 舵机 s1 取该角度时，**第二段与第一段共线**（关节2 转角 = 0）
        s2_home : 舵机 s2 取该角度时，**第三段与第二段共线**（关节3 转角 = 0）
    即：几何角 = 舵机角 - home 角。装配方向不同时改这三个值即可。

    ★ 为什么 s2_home 默认是 180 而不是 90 ★
    实机装配时，夹爪舵机打到中位（90°）夹爪就已经朝下了 —— 也就是
    第三段相对第二段已经转了 -90°。要让"第三段与第二段共线"成立，
    就得把这个 -90° 补回来：180° - 90° = 90°，所以 s2_home = 180。
    如果这里错填 90，逆解算出的 s2 会普遍偏小甚至变负（超固件限位），
    工作空间也会被压窄一半以上（实测 49mm → 21mm）。标定时重点核对这一项。
    """
    h0: float = 60.0        # 基座台面 到 关节1(底座俯仰关节)转轴 的高度 mm
    l1: float = 100.0       # 第一段连杆 关节1 -> 关节2
    l2: float = 95.0        # 第二段连杆 关节2 -> 关节3
    l3: float = 55.0        # 第三段连杆 关节3 -> 夹爪中心(TCP)

    s0_home: float = 90.0   # 舵机 s0 中位（第一段水平）
    s1_home: float = 90.0   # 舵机 s1 中位（第二段与第一段共线）
    s2_home: float = 180.0  # 舵机 s2 中位（第三段与第二段共线，见上方说明）

    # 三个定位舵机的软限位，默认取固件值（见模块级 SERVO_LIMITS）
    servo_limits: Tuple[Tuple[float, float], ...] = SERVO_LIMITS

    @property
    def total_reach(self) -> float:
        """完全伸直时能到的最大半径（从关节1 算起）"""
        return self.l1 + self.l2 + self.l3

    def in_limits(self, s0: float, s1: float, s2: float) -> bool:
        """三个定位舵机角是否都在软限位内"""
        return all(lo <= v <= hi for v, (lo, hi) in
                   zip((s0, s1, s2), self.servo_limits[:3]))


def fk(s0: float, s1: float, s2: float, g: ArmGeometry = None) -> tuple:
    """
    正解：三个俯仰舵机角 -> 末端位置与各段方向角。

    参数
    ----
    s0, s1, s2 : 三个定位舵机的角度（度）
    g          : 机械参数，默认用 ArmGeometry()

    返回
    ----
    ((x, z), (phi1_deg, phi2_deg, phi3_deg))
      x, z   末端 TCP 的平面坐标 mm
      phi1   第一段连杆的绝对方向角（相对水平面，逆时针为正）
      phi2   第二段连杆的绝对方向角
      phi3   第三段连杆的绝对方向角 = **末端姿态角**（夹爪指向）
             phi3 = -90° 表示夹爪竖直朝下（抓竖直药瓶的标准姿态）
    """
    g = g or ArmGeometry()
    p1 = np.radians(s0 - g.s0_home)
    p2 = p1 + np.radians(s1 - g.s1_home)
    p3 = p2 + np.radians(s2 - g.s2_home)

    x = g.l1 * np.cos(p1) + g.l2 * np.cos(p2) + g.l3 * np.cos(p3)
    z = g.h0 + g.l1 * np.sin(p1) + g.l2 * np.sin(p2) + g.l3 * np.sin(p3)
    return ((float(x), float(z)),
            (float(np.degrees(p1)), float(np.degrees(p2)), float(np.degrees(p3))))


def ik(x: float, z: float, phi_deg: float = -90.0, elbow: str = "up",
       g: ArmGeometry = None) -> Optional[dict]:
    """
    逆解：末端目标位置 + 末端姿态角 -> 三个舵机角度。

    数学步骤
    --------
    1) 由末端姿态角 phi3 反推**腕点**（关节3）位置：
           W = TCP - l3 * (cos phi3, sin phi3)
       这一步把 3R 问题降成"基座 -> 腕点"的平面 2R 问题。
    2) 从关节1（坐标为 (0, h0)）到 W 做标准 2R 余弦定理逆解，
       得到第一段方向角 phi1 与第二段方向角 phi2。
    3) phi3 就是给定的末端姿态角。
    4) 几何角 -> 舵机角（fk 的逆映射）。

    参数
    ----
    x, z    : 目标 TCP 平面坐标 mm（x 为水平前方距离）
    phi_deg : 期望的末端姿态角（度）。默认 -90° = 夹爪竖直朝下，
              这是抓取竖直药瓶的标准姿态。
    elbow   : 'up' 肘上构型（关节2 在基座-腕点连线上方，推荐）/'down' 肘下
    g       : 机械参数

    返回
    ----
    dict 或 None（不可达）
      s0, s1, s2      三个定位舵机角（度）
      phi1/2/3_deg    三段连杆的绝对方向角
      elbow_x/elbow_z 关节2（肘）坐标，用于碰撞检查
      wrist_x/wrist_z 关节3（腕）坐标
      reach           基座关节1 到腕点的距离
      in_limits       三个舵机角是否都在安全区间内
    """
    g = g or ArmGeometry()
    p3 = np.radians(float(phi_deg))

    # 1) 腕点（关节3）位置
    wx = float(x) - g.l3 * np.cos(p3)
    wz = float(z) - g.l3 * np.sin(p3)

    # 2) 关节1 -> 腕点 的平面 2R 逆解
    dx, dz = wx, wz - g.h0
    D = float(np.hypot(dx, dz))
    if D > (g.l1 + g.l2) or D < abs(g.l1 - g.l2):
        return None                                  # 工作空间之外

    c_beta = np.clip((D * D + g.l1 * g.l1 - g.l2 * g.l2) / (2 * D * g.l1), -1.0, 1.0)
    beta = float(np.arccos(c_beta))
    alpha = float(np.arctan2(dz, dx))
    p1 = alpha + beta if elbow == "up" else alpha - beta

    # 3) 由关节2 位置反推第二段方向角
    jx = g.l1 * np.cos(p1)
    jz = g.h0 + g.l1 * np.sin(p1)
    p2 = float(np.arctan2(wz - jz, wx - jx))

    # 4) 几何角 -> 舵机角
    s0 = g.s0_home + float(np.degrees(p1))
    s1 = g.s1_home + float(np.degrees(p2 - p1))
    s2 = g.s2_home + float(np.degrees(p3 - p2))

    in_limits = g.in_limits(s0, s1, s2)
    return dict(
        s0=s0, s1=s1, s2=s2,
        phi1_deg=float(np.degrees(p1)),
        phi2_deg=float(np.degrees(p2)),
        phi3_deg=float(np.degrees(p3)),
        elbow_x=float(jx), elbow_z=float(jz),
        wrist_x=float(wx), wrist_z=float(wz),
        reach=D, in_limits=in_limits,
    )


def reachable_x_range(z: float, phi_deg: float = -90.0,
                      g: ArmGeometry = None, n: int = 3601) -> Optional[Tuple[float, float]]:
    """
    给定抓取高度 z 和末端姿态角，算出该高度下可达的**水平距离区间**。

    做法：扫描第一段方向角 phi1，由高度方程反解 phi2，再算出对应的 x，
    收集所有解取 min/max。比二维网格扫描快得多。

    用途
    ----
    设计药箱布局时先跑这个，就知道"这个高度上药箱能摆多宽"，
    避免把槽位放到够不着的位置。

    返回
    ----
    (x_min, x_max) 或 None（该高度完全不可达）
    """
    g = g or ArmGeometry()
    p3 = np.radians(float(phi_deg))
    # 高度方程：h0 + l1 sin(p1) + l2 sin(p2) + l3 sin(p3) = z
    base = float(z) - g.h0 - g.l3 * np.sin(p3)

    xs = []
    for p1 in np.linspace(-np.pi, np.pi, n):
        k = (base - g.l1 * np.sin(p1)) / g.l2
        if abs(k) > 1.0:
            continue
        a = np.arcsin(k)
        for p2 in (a, np.pi - a):
            s0 = g.s0_home + np.degrees(p1)
            s1 = g.s1_home + np.degrees(p2 - p1)
            s2 = g.s2_home + np.degrees(p3 - p2)
            if not g.in_limits(s0, s1, s2):
                continue                              # 舵机超限的解不要
            xs.append(g.l1 * np.cos(p1) + g.l2 * np.cos(p2) + g.l3 * np.cos(p3))
    if not xs:
        return None
    return (float(min(xs)), float(max(xs)))
