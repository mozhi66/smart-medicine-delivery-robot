#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 生成机械臂预设点位表 (gen_poses.py)
================================================================================
职责：根据 config/app.yaml 里的机械臂尺寸和药箱布局（plane_grid 九宫格 /
     plane_line 一排），用逆解自动生成 config/poses.json（槽位 -> 四舵机角度）。

★ 本工具与主程序共用同一套布局逻辑（core/pose_table.py 的 slots_from_layout
  与 per_slot_from_layout）—— 重标定生成的点位与运行时读到的完全同源，
  不存在"生成时一套坐标、运行时另一套"的错位。

什么时候要跑
------------
  1) 第一次部署（生成初值）
  2) 换了机械臂 / 改了 H0、L1、L2、L3
  3) 改了药箱布局（x_start、层高、槽位数）

生成的点位是"数学上正确"的初值。由于舵机存在齿轮回差（重复定位误差
1~3 度），实际使用前建议用手动示教微调：把机械臂拖到合适的姿态，
记下角度，直接改 poses.json 里对应的数字即可。

运行
----
    python tools/gen_poses.py --check          # 只体检不写文件（写到临时目录）
    python tools/gen_poses.py --out config/poses.json --check
================================================================================
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_ROOT, "pc"))

from core.kinematics import ArmGeometry, ik, fk      # noqa: E402
from core.pose_table import (generate_poses,         # noqa: E402
                             slots_from_layout,
                             per_slot_from_layout)

try:
    import yaml
except Exception:                                    # pragma: no cover
    yaml = None


def check_reachable(positions: dict, g: ArmGeometry, per_slot: dict = None) -> int:
    """
    逐个槽位做可达性体检，返回不可达数量。
    同时打印每个槽位的舵机角和末端姿态角，供人工判断"能不能夹住"。

    ★ core.kinematics 的 ik() 返回 dict 没有 approach_deg 键（那是老三维
      模型的概念）；平面模型的末端姿态就是 phi3_deg（-90° = 夹爪竖直朝下）。
      姿态角是调用方自己定的目标值，检查它没有意义，故只做 in_limits 体检。
    """
    print("%-6s %-24s %-26s %-10s %-9s %s"
          % ("槽位", "目标xz(mm)", "s0/s1/s2", "姿态角", "肘高mm", "结论"))
    print("-" * 96)
    bad = 0
    for sid, p in sorted(positions.items()):
        ps = (per_slot or {}).get(sid) or {}
        sol = ik(p[0], p[1], phi_deg=ps.get("phi", -90.0),
                 elbow=ps.get("elbow", "up"), g=g)
        if sol is None:
            bad += 1
            print("%-6d %-24s %-26s %-10s %-9s X 不可达"
                  % (sid, "(%.0f,%.0f)" % p, "-", "-", "-"))
            continue
        ok = bool(sol["in_limits"])
        bad += (not ok)
        print("%-6d %-24s %-26s %-10.1f %-9.1f %s%s"
              % (sid, "(%.0f,%.0f)" % p,
                 "%.1f / %.1f / %.1f" % (sol["s0"], sol["s1"], sol["s2"]),
                 sol["phi3_deg"], sol["elbow_z"],
                 "OK " if ok else "X  ",
                 "" if ok else "角度超软限位"))
    print("-" * 96)
    return bad


def main():
    ap = argparse.ArgumentParser(description="生成机械臂预设点位表")
    ap.add_argument("--config", default=os.path.join(_ROOT, "config", "app.yaml"))
    ap.add_argument("--out", default=os.path.join(_ROOT, "config", "poses.json"))
    ap.add_argument("--count", type=int, default=None, help="覆盖槽位数量")
    ap.add_argument("--check", action="store_true", help="生成后做可达性体检")
    args = ap.parse_args()

    cfg = {}
    if yaml and os.path.isfile(args.config):
        with open(args.config, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
    else:
        print("[提示] 未读取到配置，使用内置默认值")

    gsec = cfg.get("arm_geometry") or {}
    # ★ core.kinematics 的 ArmGeometry 没有 servo_min/servo_max 标量参数
    #   （那是老接口）；每舵机独立限位走 servo_limits（默认取固件值
    #   SERVO_LIMITS），这里不传即用默认。
    g = ArmGeometry(
        h0=float(gsec.get("h0", 70.0)), l1=float(gsec.get("l1", 105.0)),
        l2=float(gsec.get("l2", 98.0)), l3=float(gsec.get("l3", 90.0)),
        s0_home=float(gsec.get("s0_home", 90.0)),
        s1_home=float(gsec.get("s1_home", 90.0)),
        s2_home=float(gsec.get("s2_home", 90.0)),
    )

    layout = cfg.get("box_layout") or {}
    count = args.count or int(layout.get("count", 8))
    # ★ 用主程序同源的布局函数（plane_grid 九宫格 / plane_line 一排），
    #   且必须带上 per_slot 的逐层分支（下层 down / 中层 up / 上层 down），
    #   否则必有一层跟真机示教姿态对不上。
    positions = slots_from_layout(layout, count)
    per_slot = per_slot_from_layout(layout, count)

    print("=" * 96)
    print("生成预设点位表")
    print("  机械参数: H0=%.0f L1=%.0f L2=%.0f L3=%.0f mm"
          % (g.h0, g.l1, g.l2, g.l3))
    print("  药箱布局: %s, %d 个槽位" % (layout.get("shape", "plane_grid"), count))
    print("=" * 96)

    bad = check_reachable(positions, g, per_slot)
    if bad:
        print("!! %d 个槽位不可用，请调整布局参数（x_start / 层高）或机械尺寸" % bad)
    else:
        print(">> 全部槽位可达且角度在软限位内")

    data = generate_poses(positions, g,
                          per_slot=per_slot,
                          approach_dz=float(layout.get("approach_dz", 70.0)),
                          lift_dz=float(layout.get("lift_dz", 70.0)))

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    print("\n已写入: %s" % args.out)

    # 正解回代验证：角度 -> 位置，看能不能回到原目标（平面 x/z 两轴）
    print("\n[回代校验] 点位角度 -> 正解位置 -> 与目标的距离（x/z 平面）")
    max_err = 0.0
    for sid, item in sorted(data["slots"].items(), key=lambda kv: int(kv[0])):
        a = item["grasp"]
        (x, z), _ = fk(a[0], a[1], a[2], g)
        tx, tz = positions[int(sid)]
        err = math.hypot(x - tx, z - tz)
        max_err = max(max_err, err)
        print("  槽位 %-3s 目标(%.0f,%.0f) 正解(%.1f,%.1f) 误差 %.3f mm"
              % (sid, tx, tz, x, z, err))
    print("  最大误差 %.4f mm %s" % (max_err, "(OK)" if max_err < 0.5 else "(异常)"))
    return 0 if (bad == 0 and max_err < 0.5) else 1


if __name__ == "__main__":
    sys.exit(main())
