#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 预设点位表 (pose_table.py)
================================================================================
职责：管理"槽位 <-> 四舵机角度"的对应关系，也就是"预设点位"。

为什么用预设点位而不是每次算逆解？
    1) 舵机有回差（齿轮间隙），重复定位误差 1~3 度，逆解算得再准也没用
    2) 人工示教的点位天然避开磕碰、姿态也更适合夹取
    3) 本机械臂是**平面 3R 串联臂**（3 个俯仰关节），平面内自由度 = 3，
       正好等于"位置(2) + 末端姿态(1)"，所以逆解能直接给出"夹爪竖直朝下"的解；
       但可达范围受舵机限位约束、区间较窄，最终仍建议用示教点位微调。
    本模块两者都支持：优先读示教点位，没有时用逆解自动生成。

★ 坐标是【平面坐标】(x, z)，没有 y ★
    本机械臂没有底座旋转自由度，末端只能在一个竖直平面内运动，
    所以槽位都用 (x = 水平前方距离, z = 高度) 描述，y 恒为 0。

每个槽位存三个相位（标准抓取动作序列）
----------------------------------------
    approach  接近位：在药瓶正上方，夹爪张开，还没下降
    grasp     抓取位：下降到夹取高度，夹爪张开，准备夹紧
    lift      抬起位：夹紧后抬起来（第 4 个角度 = 夹紧角）

动作流程：home -> approach -> grasp -> [夹紧] -> lift -> ...（搬运）... -> home

poses.json 结构
---------------
{
  "home":     [s0, s1, s2, s3],
  "inspect":  [s0, s1, s2, s3],     过检位：把药举到相机前
  "deliver":  [s0, s1, s2, s3],     交付位：把药递给用户
  "slots": {
      "1": {"approach": [...], "grasp": [...], "lift": [...]},
      ...
  },
  "gripper": {"open": 70, "close": 15}
}
================================================================================
"""

from __future__ import annotations

import json
import math
import os
from typing import Dict, List, Optional, Sequence

from .kinematics import ArmGeometry, ik, SERVO_LIMITS

DEFAULT_FILENAME = "poses.json"

# 舵机软限位 SERVO_LIMITS 的**唯一权威定义在 kinematics.py**
# （与固件 `0stm/app/main.h` 的 SERVOx_LIMIT_MIN/MAX_DEG 一一对应），
# 这里直接复用，避免"逆解认为可行、固件却回 $ERR,03,LIM"的假可行。
#
# 历史坑：poses.json 里夹爪写 110°，而固件 SERVO3_LIMIT_MAX_DEG = 90°
# （注释原话"夹爪超过 90 度容易憋舵机"），真机取药到"夹紧抬起"那一步
# 必然被拒 → 整轮失败。所以 validate() 会**提前校验**，
# 让配置错误在启动/标定时就暴露，而不是等真机跑到一半才报错。
SERVO_NAMES = ("s0 底座俯仰", "s1 大臂", "s2 小臂", "s3 夹爪")

# ============================================================================
# 夹爪三个关键角 —— 同样必须与固件 `0stm/app/main.h` 一致
# ============================================================================
# ★★ 语义（2026-10-22 实机复测确认）：180°=复位/完全张开，140°=抓药夹紧 ★★
#    （135° 实测堵转卡死，禁止使用。数字变小 = 夹紧。）
#
# 固件侧的三条指令/常量（见 0stm/app/main.h）：
#     $GC,open  -> GRIPPER_OPEN_DEG  = 180.0  复位/完全张开
#     $GC,close -> GRIPPER_CLOSE_DEG = 140.0  抓药夹紧（135 堵转，勿再调小）
#     SERVO3_HOME_DEG                = 180.0  上电/复位全开
#     SERVO3_LIMIT_MIN/MAX_DEG       = 140/180 机械有效行程
#     SERVO3_LIMIT_MAX_DEG           = 180.0  夹爪机械有效行程上限
#
# 为什么必须一致：状态机抓取序列是
#     grip(True)[$GC,open→180°] -> $SA,...,approach[3] -> $SA,...,grasp[3]
#     -> grip(False)[$GC,close→140°] -> $SA,...,lift[3]
# 最后一步 `lift[3]` 会**再次**改写夹爪角。lift/inspect/deliver 必须用
# **夹紧角**，approach/grasp 必须用**张开角**。
SERVO3_HOME_DEG = 180.0     # 固件 SERVO3_HOME_DEG（复位全开）
GRIPPER_OPEN_DEG = 180.0    # 固件 GRIPPER_OPEN_DEG（$GC,open；复位张开）
GRIPPER_CLOSE_DEG = 140.0   # 固件 GRIPPER_CLOSE_DEG（$GC,close；135 堵转已复测确认）

# 接近位 / 抬起位"至少要抬高多少 mm"。低于这个值就认为抬不起来 ——
# 宁可报警也不要静默退化成"原地不动"（真机表现是贴着药瓶平移，会刮擦/撞翻）。
_MIN_LIFT_MM = 5.0


class PoseTable:
    """
    预设点位表的加载、查询与生成。

    用法
    ----
        pt = PoseTable.load("config/poses.json")
        pose = pt.get(3, "approach")     # 3 号槽位的接近位
        print(pt.home)                   # 初始位
    """

    def __init__(self, data: dict):
        """
        参数
        ----
        data : poses 字典（结构见文件头注释）
        """
        self.data = data or {}
        self.home: List[float] = list(self.data.get("home", [146.1, 41.1, 82.8, GRIPPER_CLOSE_DEG]))
        self.inspect: List[float] = list(self.data.get("inspect", [138.5, 50.6, 80.9, GRIPPER_CLOSE_DEG]))
        self.deliver: List[float] = list(self.data.get("deliver", [120.6, 67.9, 81.5, GRIPPER_CLOSE_DEG]))
        self.slots: Dict[int, dict] = {
            int(k): v for k, v in (self.data.get("slots") or {}).items()
        }
        g = self.data.get("gripper") or {}
        self.gripper_open: float = float(g.get("open", GRIPPER_OPEN_DEG))
        self.gripper_close: float = float(g.get("close", GRIPPER_CLOSE_DEG))

    # ------------------------------------------------------------------
    @classmethod
    def load(cls, path: str) -> "PoseTable":
        """
        从 JSON 文件加载点位表。文件不存在时返回一张空表（不抛异常，
        由调用方决定是报错还是自动生成）。
        """
        if not os.path.isfile(path):
            return cls({})
        with open(path, "r", encoding="utf-8") as f:
            return cls(json.load(f))

    def save(self, path: str) -> None:
        """把点位表写回 JSON（标定完要保存）"""
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.data, f, ensure_ascii=False, indent=2)

    # ------------------------------------------------------------------
    def has(self, slot_id: int) -> bool:
        """该槽位是否已标定了点位"""
        return int(slot_id) in self.slots

    def get(self, slot_id: int, phase: str) -> Optional[List[float]]:
        """
        取某个槽位某个相位的四舵机角度。

        参数
        ----
        slot_id : 槽位号（从 1 开始）
        phase   : 'approach' | 'grasp' | 'lift'

        返回
        ----
        [s0, s1, s2, s3] 或 None（未标定）
        """
        item = self.slots.get(int(slot_id))
        if not item:
            return None
        v = item.get(phase)
        return list(v) if v else None

    def set_slot(self, slot_id: int, phase: str, angles: Sequence[float]) -> None:
        """写入/覆盖某个槽位某个相位的角度"""
        sid = str(int(slot_id))
        self.slots.setdefault(int(slot_id), {})
        self.data.setdefault("slots", {}).setdefault(sid, {})
        self.data["slots"][sid][phase] = [float(a) for a in angles]
        self.slots[int(slot_id)] = self.data["slots"][sid]

    def slot_ids(self) -> List[int]:
        """所有已标定槽位的编号（升序）"""
        return sorted(self.slots.keys())

    # ------------------------------------------------------------------
    def validate(self) -> List[str]:
        """
        检查所有点位是否都在固件舵机限位内（见模块级 SERVO_LIMITS）。

        返回
        ----
        问题描述列表；**空列表 = 全部合法**。

        说明
        ----
        只做检查、不改数据。超限时固件会回 `$ERR,03,LIM` 拒绝执行，
        所以最好在启动/标定时就把问题暴露出来，而不是等真机跑到一半。
        """
        problems: List[str] = []

        def _chk(tag: str, angles: Sequence[float]) -> None:
            if not angles or len(angles) < 4:
                problems.append("%s：角度不足 4 个（%s）" % (tag, list(angles)))
                return
            for i, (lo, hi) in enumerate(SERVO_LIMITS):
                v = float(angles[i])
                if v < lo or v > hi:
                    problems.append("%s 的 %s = %.1f°，超出固件限位 %.0f~%.0f°"
                                    % (tag, SERVO_NAMES[i], v, lo, hi))

        _chk("home", self.home)
        _chk("inspect", self.inspect)
        _chk("deliver", self.deliver)
        for sid in self.slot_ids():
            for phase in ("approach", "grasp", "lift"):
                v = self.get(sid, phase)
                if v:
                    _chk("槽位 %d 的 %s" % (sid, phase), v)
        # 夹爪开合角单独校验（它们不在上面的四元组里）
        for tag, val in (("gripper.open", self.gripper_open),
                         ("gripper.close", self.gripper_close)):
            lo, hi = SERVO_LIMITS[3]
            if val < lo or val > hi:
                problems.append("%s = %.1f°，超出固件限位 %.0f~%.0f°"
                                % (tag, val, lo, hi))

        # --- 夹爪语义方向 ---
        # 固件把 $GC,close 硬映射到 GRIPPER_CLOSE_DEG(45°)、$GC,open 映射到
        # GRIPPER_OPEN_DEG(90°)，即"角度小 = 夹紧"。本表若写成 close > open，
        # 方向就是反的：夹紧动作会变成张开。
        if self.gripper_close > self.gripper_open:
            problems.append(
                "gripper.close(%.1f°) > gripper.open(%.1f°) —— 与固件语义相反"
                "（固件 close=%.0f < open=%.0f，角度小才是夹紧）"
                % (self.gripper_close, self.gripper_open,
                   GRIPPER_CLOSE_DEG, GRIPPER_OPEN_DEG))

        # --- 动作语义：抓取序列里 lift 会再次改写夹爪角 ---
        # 顺序：$GC,open -> $SA(approach) -> $SA(grasp) -> $GC,close -> $SA(lift)
        # 所以 approach/grasp 必须是"张开"，lift 必须是"夹紧"，
        # 否则夹爪会在抬起瞬间松开，药瓶掉落。
        _tol = 5.0
        for sid in self.slot_ids():
            for phase, want, desc in (("approach", self.gripper_open, "张开"),
                                      ("grasp", self.gripper_open, "张开"),
                                      ("lift", self.gripper_close, "夹紧")):
                v = self.get(sid, phase)
                if v and len(v) >= 4 and abs(float(v[3]) - want) > _tol:
                    problems.append(
                        "槽位 %d 的 %s 夹爪角 %.1f° 与期望的%s角 %.1f° 差得较多"
                        "（该步会改写夹爪，可能导致掉瓶）"
                        % (sid, phase, float(v[3]), desc, want))
        for tag, pose in (("inspect", self.inspect), ("deliver", self.deliver)):
            if pose and len(pose) >= 4 and abs(float(pose[3]) - self.gripper_close) > _tol:
                problems.append(
                    "%s 夹爪角 %.1f° 与夹紧角 %.1f° 差得较多（这一步应保持夹住药瓶）"
                    % (tag, float(pose[3]), self.gripper_close))

        return problems


# ============================================================================
# 槽位坐标生成（★ 单一来源：tools/gen_poses.py 与 pc/main.py 都调它）
# ============================================================================
def slots_from_layout(layout: dict, count: Optional[int] = None) -> Dict[int, tuple]:
    """
    按 `box_layout` 配置生成槽位的**平面坐标** `(x, z)`。

    ★ 本机械臂是平面 3R 臂（3 个俯仰关节串联），末端只能在一个竖直平面内
      前后/上下运动，**没有方位角自由度**。所以所有槽位必须排在同一个竖直
      平面里；旧的"弧形布局（等半径、靠底座转角区分槽位）"在本结构下不成立。

    参数
    ----
    layout : `app.yaml` 的 `box_layout` 段
    count  : 槽位数量；None 时取 `layout["count"]`（默认 8）

    返回
    ----
    {slot_id: (x, z)}   x = 水平前方距离 mm，z = 高度 mm

    布局形状
    --------
    plane_grid : 竖直平面内网格，`cols` 列 × `rows` 层。
                 编号**从下往上、从左往右**（1 号在左下角）。
    plane_line : 同一高度沿 x 等间距排开。
    """
    layout = layout or {}
    shape = str(layout.get("shape", "plane_grid"))
    n = int(count if count is not None else layout.get("count", 8))
    n = max(1, n)

    x0 = float(layout.get("x_start", 150.0))
    px = float(layout.get("pitch_x", 11.0))

    if shape == "plane_line":
        z = float(layout.get("grasp_z", 65.0))
        return {i + 1: (x0 + i * px, z) for i in range(n)}

    # plane_grid
    cols = max(1, int(layout.get("cols", 4)))
    rows = max(1, int(layout.get("rows", math.ceil(n / cols))))
    z0 = float(layout.get("z_start", layout.get("grasp_z", 45.0)))
    pz = float(layout.get("pitch_z", 26.0))
    layers = _layer_params(layout)
    out: Dict[int, tuple] = {}
    sid = 1
    for r in range(rows):
        for c in range(cols):
            if sid > n:
                return out
            if r < len(layers):
                lp = layers[r]
                # ★ 每层有各自的水平基准：货架是竖直对齐的九宫格，但本臂在
                #   不同高度的可用 x 完全不同，靠底盘在层间切换偏移来补偿。
                #   layers[r].x_start 就是"第 r 层用的机械臂 x 基准"。
                xr = float(lp.get("x_start", x0))
                zr = float(lp["z"])
            else:
                xr, zr = x0, z0 + r * pz
            out[sid] = (xr + c * px, zr)
            sid += 1
    return out


def _layer_params(layout: dict) -> List[dict]:
    """
    把 `box_layout.layers` 归一化成"从下往上"的层参数列表。

    为什么需要它
    ------------
    本臂是平面 3R 臂，同一个目标点逆解有两个分支（肘上/肘下），
    解出来的舵机角能差几十度。实测的三个演示姿态恰好落在不同分支上：
        下层 D 用 elbow=down   中层 C 用 elbow=up   上层 B 用 elbow=down
    所以"层"不能只有高度 z，必须连 (phi, elbow) 一起带下来，
    否则生成的点位必定有一层跟真机对不上。

    配置里的 `layers` 允许两种写法：
      1) 带 name/z/phi/elbow 的完整字典（推荐，app.yaml 用的就是这个）
      2) 纯数字高度列表（老配置兼容）

    返回: [{'z':float,'phi':float,'elbow':str,'name':str}, ...] 按 z 升序
    """
    raw = layout.get("layers")
    default_phi = float(layout.get("grasp_phi", -90.0))
    default_elbow = str(layout.get("elbow", "up"))

    if not isinstance(raw, (list, tuple)) or not raw:
        return []

    out: List[dict] = []
    for item in raw:
        if isinstance(item, dict):
            out.append({
                "name": str(item.get("name", "")),
                "z": float(item.get("z", 0.0)),
                "phi": float(item.get("phi", default_phi)),
                "elbow": str(item.get("elbow", default_elbow)),
                "x_start": item.get("x_start"),
                "x_lo": item.get("x_lo"),
                "x_hi": item.get("x_hi"),
            })
        else:
            try:
                out.append({"name": "", "z": float(item), "phi": default_phi,
                            "elbow": default_elbow, "x_start": None,
                            "x_lo": None, "x_hi": None})
            except (TypeError, ValueError):
                continue

    out.sort(key=lambda d: d["z"])
    return out


def per_slot_from_layout(layout: dict, count: Optional[int] = None) -> Dict[int, dict]:
    """
    从 `box_layout.layers` 推导出"逐槽位的逆解参数"，喂给 `generate_poses(per_slot=...)`。

    为什么必须逐层指定，而不能全局一套参数
    --------------------------------------
    同一个 (x, z) 目标点，逆解有肘上 / 肘下两个分支，解出的舵机角差几十度。
    真机实测的三个演示姿态分别落在不同分支：
        下层 z=148.2  phi=-20  elbow=down
        中层 z=193.6  phi=-30  elbow=up
        上层 z=214.8  phi=-30  elbow=down
    用一套参数全局生成，必然有一层解出来跟真机姿态对不上（机械臂会撞箱体）。

    返回
    ----
    {slot_id: {'phi': float, 'elbow': str, 'layer': int}}；没有 layers 配置时返回 {}
    """
    lay = layout or {}
    layers = _layer_params(lay)
    if not layers:
        return {}

    n = int(count if count is not None else lay.get("count", 8))
    cols = max(1, int(lay.get("cols", 4)))

    out: Dict[int, dict] = {}
    for sid in range(1, n + 1):
        r = (sid - 1) // cols               # 第几层（0 起，从下往上）
        if r >= len(layers):
            continue
        lp = layers[r]
        out[sid] = {"phi": float(lp["phi"]), "elbow": str(lp["elbow"]),
                    "layer": r + 1}
    return out


# ============================================================================
# 用运动学逆解自动生成点位表（没做人工示教时的兜底/初值）
# ============================================================================
def generate_poses(slot_positions: Dict[int, Sequence[float]],
                   g: Optional[ArmGeometry] = None,
                   grasp_phi: float = -90.0,
                   elbow: str = "up",
                   approach_dz: float = 40.0,
                   lift_dz: float = 40.0,
                   gripper_open: float = GRIPPER_OPEN_DEG,
                   gripper_close: float = GRIPPER_CLOSE_DEG,
                   home: Optional[Sequence[float]] = None,
                   inspect: Optional[Sequence[float]] = None,
                   deliver: Optional[Sequence[float]] = None,
                   per_slot: Optional[Dict[int, dict]] = None) -> dict:
    """
    根据槽位的平面坐标，用逆解自动生成一整张点位表。

    参数
    ----
    slot_positions : {slot_id: (x, z)} 槽位抓取点的**平面坐标** mm
                     x = 水平前方距离，z = 高度。
                     本机械臂末端只能在一个竖直平面内运动，所以没有 y。
    g              : 机械参数
    grasp_phi      : 抓取时的末端姿态角（度）。默认 -90° = 夹爪竖直朝下，
                     这是抓竖直药瓶的标准姿态。
    elbow          : 逆解分支 'up'（肘上）/ 'down'（肘下）。**不是无所谓的开关**：
                     同一个点两个分支解出的舵机角能差几十度，必须与真机示教
                     姿态落在同一分支上。逐槽位的分支可用 per_slot 覆盖。
    approach_dz    : 接近位比抓取位高多少 mm（下降前的安全高度）
    lift_dz        : 抬起位比抓取位高多少 mm
    gripper_open   : 夹爪张开角度
    gripper_close  : 夹爪夹紧角度
    home/inspect/deliver : 三个特殊位（四元组 [s0,s1,s2,夹爪角]）；
                     不给的话自动取"举臂待命 / 举给相机 / 前伸交付"三个安全点
    per_slot       : {slot_id: {'phi': float, 'elbow': str, 'dz': float}}
                     逐槽位覆盖姿态/分支/抬升量。九宫格三层各自的实测姿态
                     落在不同分支（下层 down / 中层 up / 上层 down），
                     必须靠这个参数逐层指定，否则必有一层跟真机对不上。

    返回
    ----
    poses 字典，可直接 PoseTable(data) 或存成 JSON

    说明
    ----
    自动生成的是"数学上正确"的初值，实际使用前建议手动微调，
    因为舵机有回差（重复定位误差 1~3 度）。
    """
    g = g or ArmGeometry()
    slots_out: Dict[str, dict] = {}
    per_slot = per_slot or {}

    def _solve(x: float, z: float, grip: float,
               phi: Optional[float] = None,
               eb: Optional[str] = None):
        """求一个点的四元组 [s0, s1, s2, 夹爪角]；不可达返回 None"""
        sol = ik(x, z, phi_deg=(grasp_phi if phi is None else phi),
                 elbow=(elbow if eb is None else eb), g=g)
        if sol is None:
            return None
        return [round(sol["s0"], 1), round(sol["s1"], 1),
                round(sol["s2"], 1), round(grip, 1)]

    def _solve_lifted(x: float, z: float, dz: float, grip: float,
                      phi: Optional[float] = None,
                      eb: Optional[str] = None):
        """
        求"抬高 dz"的点位。抬高后不可达就逐步减半，返回 (pose, 实际抬高量)。

        为什么要逐步减半而不是直接放弃：
        本臂工作空间很窄（实测 41×43mm），靠边的槽位抬高 40mm 就出界了。
        直接退回抓取位会让"接近/抬起"动作**完全不动**，真机上表现为
        机械臂贴着药瓶平移 → 刮擦甚至撞翻。宁可少抬一点，也要真的抬起来。
        返回 (None, 0.0) 表示连最小抬高量都做不到，调用方必须报警。
        """
        d = float(dz)
        # dz == 0：不是"抬不起来"，而是设计上就不抬高（靠底盘退出把药拔出来）。
        # 直接复用抓取点，且**不算作告警**。见 app.yaml 的 lift_dz 注释。
        if d <= 0.0:
            sol = _solve(x, z, grip, phi, eb)
            return (sol, 0.0) if sol is not None else (None, 0.0)
        while d >= _MIN_LIFT_MM:
            sol = _solve(x, z + d, grip, phi, eb)
            if sol is not None:
                return sol, d
            d *= 0.5
        return None, 0.0

    for sid, p in sorted(slot_positions.items()):
        x, z = float(p[0]), float(p[1])
        ov = per_slot.get(int(sid)) or {}
        phi = ov.get("phi")
        eb = ov.get("elbow")
        a_dz_cfg = float(ov.get("approach_dz", approach_dz))
        l_dz_cfg = float(ov.get("lift_dz", lift_dz))

        grasp = _solve(x, z, gripper_open, phi, eb)
        if grasp is None:
            print("[警告] 槽位 %s 目标 (%s) 不可达，跳过" % (sid, p))
            continue

        approach, a_dz = _solve_lifted(x, z, a_dz_cfg, gripper_open, phi, eb)
        if approach is None:
            print("[警告] 槽位 %s @(%.0f,%.0f)：抬高 %.0fmm 后不可达，接近位"
                  "退化为抓取位 —— 真机会贴着药瓶平移！请调小 approach_dz、"
                  "或把该槽位往工作空间中心挪（跑 tools/kinematics_check.py 看范围）"
                  % (sid, x, z, a_dz_cfg))
            approach = list(grasp)
        elif a_dz < a_dz_cfg:
            print("[提示] 槽位 %s 的接近位只能抬高 %.0fmm（原设 %.0fmm，已自动收窄）"
                  % (sid, a_dz, a_dz_cfg))

        lift, l_dz = _solve_lifted(x, z, l_dz_cfg, gripper_close, phi, eb)
        if lift is None:
            # l_dz_cfg == 0 是【正常配置】而不是故障：本项目的取药策略是
            # "夹住后不抬高，让底盘把小车开走"（见 app.yaml 的 lift_dz 注释）。
            # 此时 lift 位本就该等于 grasp 位，不该报"抬不高"的警告。
            if l_dz_cfg > 0.0:
                print("[警告] 槽位 %s @(%.0f,%.0f)：抬高 %.0fmm 后不可达，抬起位"
                      "退化为抓取位 —— 抓起后抬不高，可能与药箱边缘干涉！"
                      % (sid, x, z, l_dz_cfg))
            lift = list(grasp)
        elif l_dz < l_dz_cfg:
            print("[提示] 槽位 %s 的抬起位只能抬高 %.0fmm（原设 %.0fmm，已自动收窄）"
                  % (sid, l_dz, l_dz_cfg))
        lift[3] = round(gripper_close, 1)      # 抬起必须夹紧，不能沿用张开的兜底值

        slots_out[str(int(sid))] = {
            "approach": approach,
            "grasp": grasp,
            "lift": lift,
        }

    # ---- 三个特殊位 ----
    # 没显式给就用兜底点。注意兜底用的是 phi=-90 / elbow=up，
    # 与本臂实际的抓取姿态（phi=-20~-30）不同 —— 这三个位是"待命/展示/交付"，
    # 姿态不要求与抓取一致，只要可达且在限位内即可。
    # 真机上更推荐按实测姿态手填（gen_poses.py 会把 pose_demo.yaml 的 A 姿态
    # 当 home 传进来）。
    home_pose = list(home) if home else (
        _solve(150.0, 100.0, gripper_close, -90.0, "up")
        or [110.0, 90.0, 90.0, gripper_close])
    inspect_pose = list(inspect) if inspect else (
        _solve(160.0, 95.0, gripper_close, -90.0, "up") or list(home_pose))
    deliver_pose = list(deliver) if deliver else (
        _solve(180.0, 70.0, gripper_close, -90.0, "up") or list(home_pose))

    return {
        "home": home_pose,
        "inspect": inspect_pose,
        "deliver": deliver_pose,
        "slots": slots_out,
        "gripper": {"open": gripper_open, "close": gripper_close},
    }
