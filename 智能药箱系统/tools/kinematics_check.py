#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 机械臂运动学可行性验证工具  (kinematics_check.py)
================================================================================
用途：在写任何嵌入式/视觉代码之前，先用数值仿真回答三个"理论能否成立"的问题：

  Q1. 药箱里每一个槽位，机械臂够得到吗？（可达性）
  Q2. 够得到的时候，4 个舵机的角度都落在 0~180 度内吗？（舵机限位）
  Q3. 末端夹爪的姿态能不能夹住竖直放置的药瓶？（末端姿态约束）
      —— 这是 4 舵机方案最容易翻车的地方，详见 README

机械结构模型（典型 4 舵机桌面教育机械臂，如 LeArm / DOFBOT 一类）：

        Z(上)
        |         肩关节 ●  ...................... 高度 H0
        |        / 大臂 L1
        |      /
        |    ●  肘关节
        |    \
        |     \ 小臂 L2
        |      \
        |       ●  腕关节
        |        \
        |         \ 夹爪 L3（沿小臂方向延伸）
        |          ◇ 夹爪中心 = 末端 TCP
        |
        O------------------> r (水平半径, r=sqrt(x^2+y^2))
        底座旋转轴 (yaw)

 4 个舵机的分工：
   servo0 = 底座 yaw 旋转      → 决定末端在哪个方位角
   servo1 = 大臂俯仰 (肩)      → 决定远近/高度
   servo2 = 小臂俯仰 (肘)      → 决定远近/高度
   servo3 = 夹爪开合           → 不参与定位

 关键结论（务必理解）：
   定位可用自由度只有 3 个 (yaw, 肩, 肘)，而末端位置也是 3 维 (x,y,z)，
   所以 3 = 3 —— 位置可达，但【末端姿态完全由位置决定，不可自由指定】。
   这就是"为什么不能要求夹爪在任意点都保持竖直"的原因，也是 Q3 要验证的。

运行：
   python3.11 tools/kinematics_check.py
   python3.11 tools/kinematics_check.py --png docs/kinematics.png
================================================================================
"""

import argparse
import numpy as np

# ============================================================================
# 一、机械尺寸（单位：mm）—— 按你自己的机械臂实测修改这里
# ============================================================================
H0 = 70.0      # 底座旋转轴所在平面 到 肩关节 的垂直高度
L1 = 105.0     # 大臂：肩关节 -> 肘关节
L2 = 98.0      # 小臂：肘关节 -> 腕关节
L3 = 90.0      # 夹爪：腕关节 -> 夹爪中心(TCP)，沿小臂方向
B = L2 + L3    # 有效第二连杆（小臂 + 夹爪共线）

# 舵机安全角度区间（留 5 度余量，避免顶到机械限位导致堵转）
SERVO_MIN, SERVO_MAX = 10.0, 170.0

# 舵机中位与安装偏置（示教标定后需要修正，见 tools/calibrate_slots.py）
SERVO0_HOME = 90.0   # 底座舵机 90 度时臂指向 +X（药箱正前方）
SERVO1_HOME = 90.0   # 大臂舵机 90 度时大臂水平
SERVO2_HOME = 90.0   # 小臂舵机 90 度时小臂与大臂垂直（即相对偏角 90 度）

# ============================================================================
# 二、药箱布局（单位：mm，机械臂基座坐标系）
#     X 向前（药箱方向），Y 向左，Z 向上，原点 = 底座旋转轴与台面交点
#
#  【重要结论 —— 由本脚本的数值仿真得出，不是拍脑袋】
#  机械臂在抓取高度处的最大水平半径约 293mm（= L1 + B），
#  因此"矩形网格药箱"只有靠近基座的一小块区域可用：
#     矩形 3x4 布局实测 12 格中只有 6 格可达（见 --layout grid）。
#  正确做法是把药箱排成【以底座为圆心的扇形/弧形】：
#     所有槽位半径 R 相同，只靠底座 yaw 区分。
#  好处有三个：
#     1) 全部槽位半径相同 -> 全部可达（不再有够不着的角）
#     2) 全部槽位的大臂角 s1、小臂角 s2 完全相同，只有 s0 不同
#        -> 示教标定只需调一次高度，工作量从 N 次降到 1 次
#     3) 抓取动作统一，重复精度最高，演示最稳
#  所以默认推荐 --layout arc（弧形单排）。
# ============================================================================
LAYOUT = "arc"       # 'arc' 弧形(推荐) | 'grid' 矩形(对比用)

# --- 弧形布局参数 ---
ARC_R = 250.0        # 槽位所在圆弧半径（推荐 230~265，中间值最省舵机行程）
ARC_COUNT = 8        # 槽位数量
ARC_SPAN_DEG = 105.0  # 弧的总张角，槽位均匀分布在 [-span/2, +span/2]

# --- 矩形布局参数（仅对比用，实测只能放下很小一片）---
SLOT_ROWS = 3
SLOT_COLS = 3
SLOT_PITCH_X = 42.0
SLOT_PITCH_Y = 58.0
BOX_X0 = 192.0
BOX_Y_CENTER = 0.0

GRASP_Z = 65.0       # 抓取点高度（药瓶重心，约瓶高的 60%）

BOX_TOP_Z = 55.0     # 药箱隔板/格口顶面高度（用于碰撞检查）
BOTTLE_H = 110.0     # 药瓶高度

# 可接受的末端接近角区间（小臂方向角，负值表示向下）
#   -15 度 : 太平，夹爪接近水平插入，容易把瓶子推倒
#   -75 度 : 太陡，夹爪几乎垂直，大臂会压得很低压到药箱
APPROACH_MIN, APPROACH_MAX = -75.0, -15.0


# ============================================================================
# 三、运动学：正解 Forward Kinematics
# ============================================================================
def fk(s0, s1, s2, h0=H0, l1=L1, b=B):
    """
    正解：舵机角度(度) -> 末端 TCP 的世界坐标 (x, y, z)，单位 mm

    参数
    ----
    s0 : float  底座舵机角(度)，90 度时臂指向 +X
    s1 : float  大臂舵机角(度)，90 度时大臂水平
    s2 : float  小臂舵机角(度)，90 度时小臂相对大臂偏 90 度
    h0, l1, b : 机械尺寸，默认用模块常量

    返回
    ----
    (x, y, z) : 末端 TCP 世界坐标 mm
    (yaw_deg, theta1_deg, theta2_deg) : 中间量，便于调试
        theta1 = 大臂方向角（相对水平面）
        theta2 = 小臂方向角（绝对，相对水平面）—— 它就是"末端接近角"
    """
    # 舵机角 -> 几何角
    yaw = np.radians(SERVO0_HOME - s0)       # 底座：舵机增大，臂向右(-Y)转
    t1 = np.radians(s1 - SERVO1_HOME)        # 大臂：舵机增大，大臂抬起
    t2 = t1 - np.radians(180.0 - s2)         # 小臂：绝对方向角

    r = l1 * np.cos(t1) + b * np.cos(t2)     # 水平半径
    z = h0 + l1 * np.sin(t1) + b * np.sin(t2)  # 高度

    x = r * np.cos(yaw)
    y = r * np.sin(yaw)
    return (x, y, z), (np.degrees(yaw), np.degrees(t1), np.degrees(t2))


# ============================================================================
# 四、运动学：逆解 Inverse Kinematics（平面 2R 解析解）
# ============================================================================
def ik(x, y, z, elbow="up", h0=H0, l1=L1, b=B):
    """
    逆解：末端目标世界坐标 -> 三个舵机角度

    数学原理
    --------
    1) 底座 yaw 只由 (x, y) 决定：yaw = atan2(y, x)
    2) 剩余问题退化到"通过底座轴的竖直平面"内的平面 2R 臂：
       目标点在该平面内的坐标是 (r, z)，其中 r = hypot(x, y)
       肩关节在 (0, h0)，所以相对坐标 (dr, dz) = (r, z - h0)
    3) 平面 2R 标准余弦定理：
       D     = hypot(dr, dz)                      末端到肩的距离
       beta  = acos((D^2 + l1^2 - b^2)/(2*D*l1))  dr 向量与大臂的夹角
       elbow = acos((l1^2 + b^2 - D^2)/(2*l1*b))  两连杆之间的夹角
       theta1 = atan2(dz, dr) ± beta              '+' 肘上 / '-' 肘下
       theta2 = theta1 - (pi - elbow)             小臂绝对方向角
    4) 几何角 -> 舵机角（上面 fk 的逆映射）

    参数
    ----
    x, y, z : 目标 TCP 世界坐标 mm
    elbow   : 'up' 肘上构型（大臂在上，常用）/ 'down' 肘下构型
    h0,l1,b : 机械尺寸

    返回
    ----
    dict or None
        None 表示不可达（超出工作空间）。否则包含：
        s0,s1,s2      三个舵机角(度)
        approach_deg  末端接近角(度) = 小臂绝对方向角
        elbow_z       肘关节高度，用于碰撞检查
    """
    r = float(np.hypot(x, y))
    yaw = float(np.arctan2(y, x))

    dr, dz = r, z - h0
    D = float(np.hypot(dr, dz))

    # --- 可达性判据：|l1-b| <= D <= l1+b ---
    if D > (l1 + b) or D < abs(l1 - b):
        return None

    # 余弦定理（clip 防止浮点误差导致 acos 定义域越界）
    c_beta = np.clip((D * D + l1 * l1 - b * b) / (2 * D * l1), -1.0, 1.0)
    c_elb = np.clip((l1 * l1 + b * b - D * D) / (2 * l1 * b), -1.0, 1.0)
    beta = float(np.arccos(c_beta))
    elbow_ang = float(np.arccos(c_elb))

    alpha = float(np.arctan2(dz, dr))
    if elbow == "up":
        t1 = alpha + beta
    else:
        t1 = alpha - beta
    t2 = t1 - (np.pi - elbow_ang)

    # 几何角 -> 舵机角（fk 的逆映射）
    s0 = SERVO0_HOME - np.degrees(yaw)
    s1 = SERVO1_HOME + np.degrees(t1)
    s2 = 180.0 - np.degrees(t1 - t2)

    # 肘关节世界高度（碰撞检查用）
    elbow_z = h0 + l1 * np.sin(t1)

    return dict(s0=s0, s1=s1, s2=s2,
                approach_deg=np.degrees(t2),
                elbow_z=elbow_z,
                reach=D, yaw_deg=np.degrees(yaw))


# ============================================================================
# 五、单次求解的"体检"：角度限位 + 姿态 + 碰撞
# ============================================================================
def check_solution(sol, verbose_slot=""):
    """
    对一个逆解结果做三项体检，返回 (是否通过, 不通过原因列表)
      1. 三个舵机角是否在 [SERVO_MIN, SERVO_MAX]
      2. 末端接近角是否在 [APPROACH_MIN, APPROACH_MAX]（能否夹住竖直药瓶）
      3. 肘关节是否高于药箱顶面（避免大臂压到隔板）
    """
    if sol is None:
        return False, ["不可达：目标点在工作空间之外"]

    reasons = []
    for name, v in (("s0", sol["s0"]), ("s1", sol["s1"]), ("s2", sol["s2"])):
        if not (SERVO_MIN <= v <= SERVO_MAX):
            reasons.append("%s=%.1f 超出舵机限位[%.0f,%.0f]" % (name, v, SERVO_MIN, SERVO_MAX))

    ap = sol["approach_deg"]
    if not (APPROACH_MIN <= ap <= APPROACH_MAX):
        reasons.append("接近角 %.1f° 不在可夹持区间[%.0f°,%.0f°]" % (ap, APPROACH_MIN, APPROACH_MAX))

    if sol["elbow_z"] < BOX_TOP_Z + BOTTLE_H * 0.5:
        reasons.append("肘高 %.1fmm 过低，大臂可能压到药瓶/隔板" % sol["elbow_z"])

    return (len(reasons) == 0), reasons


# ============================================================================
# 六、生成药箱槽位表
# ============================================================================
def build_slots(layout=None):
    """
    生成半结构化药箱的槽位中心坐标。

    参数
    ----
    layout : 'arc'  以底座为圆心的单排弧形（推荐）
                    slot 1..N 沿圆弧从 -yaw 到 +yaw 均匀排布
             'grid' 矩形网格（对比用，用来证明弧形布局为什么更优）

    返回
    ----
    [(slot_id, x, y, z), ...]
    """
    layout = layout or LAYOUT
    slots = []
    if layout == "arc":
        half = ARC_SPAN_DEG / 2.0
        if ARC_COUNT == 1:
            angs = [0.0]
        else:
            step = ARC_SPAN_DEG / (ARC_COUNT - 1)
            angs = [-half + i * step for i in range(ARC_COUNT)]
        for i, a_deg in enumerate(angs, start=1):
            a = np.radians(a_deg)
            slots.append((i, ARC_R * np.cos(a), ARC_R * np.sin(a), GRASP_Z))
    else:
        y_list = [(i - (SLOT_ROWS - 1) / 2.0) * SLOT_PITCH_Y + BOX_Y_CENTER
                  for i in range(SLOT_ROWS)]
        x_list = [BOX_X0 + j * SLOT_PITCH_X for j in range(SLOT_COLS)]
        sid = 1
        for y in y_list:
            for x in x_list:
                slots.append((sid, x, y, GRASP_Z))
                sid += 1
    return slots


# ============================================================================
# 七、主流程：遍历全部槽位，逐个体检
# ============================================================================
def run(png_path=None, layout=None):
    layout = layout or LAYOUT
    slots = build_slots(layout)
    print("=" * 96)
    print("机械臂参数: H0=%.0f  L1=%.0f  L2=%.0f  L3=%.0f  (有效第二连杆 B=%.0f) mm"
          % (H0, L1, L2, L3, B))
    if layout == "arc":
        print("药箱布局  : 弧形  R=%.0fmm, %d 槽位, 张角 %.0f 度, 抓取高度 %.0f mm"
              % (ARC_R, ARC_COUNT, ARC_SPAN_DEG, GRASP_Z))
    else:
        print("药箱布局  : 矩形 %d x %d = %d 槽位, 间距 %.0f x %.0f mm, 抓取高度 %.0f mm"
              % (SLOT_ROWS, SLOT_COLS, len(slots), SLOT_PITCH_X, SLOT_PITCH_Y, GRASP_Z))
    print("舵机安全区: [%.0f, %.0f] 度 ; 可接受接近角: [%.0f, %.0f] 度"
          % (SERVO_MIN, SERVO_MAX, APPROACH_MIN, APPROACH_MAX))
    print("=" * 96)
    print("%-6s %-20s %-24s %-9s %-9s %s"
          % ("槽位", "目标(mm)", "舵机角 s0/s1/s2", "接近角", "肘高mm", "结论"))
    print("-" * 96)

    ok_up = ok_down = 0
    results = []
    for sid, x, y, z in slots:
        best = None
        for cfg in ("up", "down"):
            sol = ik(x, y, z, elbow=cfg)
            passed, reasons = check_solution(sol)
            if passed:
                best = (cfg, sol, reasons)
                if cfg == "up":
                    ok_up += 1
                else:
                    ok_down += 1
                break
            if best is None:
                best = (cfg, sol, reasons)   # 记下第一个构型用于报错展示

        cfg, sol, reasons = best
        results.append((sid, x, y, z, cfg, sol, reasons))
        if sol is None:
            print("%-6d (%.0f,%.0f,%.0f)      %-24s %-9s %-9s X %s"
                  % (sid, x, y, z, "----", "--", "--", reasons[0]))
        else:
            flag = "OK " if not reasons else "X  "
            print("%-6d (%.0f,%.0f,%.0f)      %-24s %-9.1f %-9.1f %s%s"
                  % (sid, x, y, z,
                     "%6.1f/%6.1f/%6.1f" % (sol["s0"], sol["s1"], sol["s2"]),
                     sol["approach_deg"], sol["elbow_z"], flag,
                     "" if not reasons else "; ".join(reasons)))

    print("-" * 96)
    total = len(slots)
    n_ok = ok_up + ok_down
    print("结果: %d/%d 槽位可用（肘上构型 %d 个，肘下构型 %d 个）"
          % (n_ok, total, ok_up, ok_down))
    if n_ok < total:
        print("!! 有 %d 个槽位不可用 —— 请调整药箱距离 BOX_X0 / 抓取高度 GRASP_Z / 机械臂尺寸"
              % (total - n_ok))
    else:
        print(">> 全部槽位可达且姿态可夹持，运动学层面方案成立。")
    print("=" * 96)

    # 正解回代校验：把求出的舵机角代回正解，误差应 < 0.5mm
    print("\n[正解回代校验] 逆解 -> 正解 的位置误差")
    max_err = 0.0
    for sid, x, y, z, cfg, sol, reasons in results:
        if sol is None or reasons:
            continue
        (fx, fy, fz), _ = fk(sol["s0"], sol["s1"], sol["s2"])
        err = float(np.hypot(fx - x, fy - y) + abs(fz - z))
        max_err = max(max_err, err)
    print("  最大回代误差 = %.4f mm  %s" % (max_err, "(<0.5mm，逆解正确)" if max_err < 0.5 else "(异常!)"))

    if png_path:
        _plot(png_path, slots, results)
        print("\n[图像] 工作空间侧视图已保存: %s" % png_path)

    return n_ok, total


def _plot(path, slots, results):
    """画出工作空间边界 + 各槽位可达情况（需要 matplotlib，失败不影响主流程）"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print("[跳过绘图] matplotlib 不可用: %s" % e)
        return

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.2))

    # --- 左图：r-z 平面工作空间（Y=0 剖面） ---
    ax = axes[0]
    th1s = np.linspace(-np.pi / 2, np.pi / 2, 160)
    th2s = np.linspace(-np.pi, 0.0, 200)
    pts = []
    for t1 in th1s:
        for t2 in th2s:
            r = L1 * np.cos(t1) + B * np.cos(t2)
            z = H0 + L1 * np.sin(t1) + B * np.sin(t2)
            pts.append((r, z))
    pts = np.array(pts)
    ax.scatter(pts[:, 0], pts[:, 1], s=1, c="#cfe3ff", label="reachable workspace")

    for sid, x, y, z, cfg, sol, reasons in results:
        r = np.hypot(x, y)
        ok = (sol is not None and not reasons)
        ax.scatter([r], [z], c="g" if ok else "r", s=70,
                   marker="o" if ok else "x", zorder=5)
        ax.annotate(str(sid), (r, z), fontsize=7, xytext=(3, 3), textcoords="offset points")

    ax.axhline(BOX_TOP_Z + BOTTLE_H, color="orange", ls="--", lw=1, label="bottle top")
    ax.axhline(BOX_TOP_Z, color="brown", ls=":", lw=1, label="box top")
    ax.set_xlabel("horizontal radius r (mm)")
    ax.set_ylabel("height z (mm)")
    ax.set_title("Workspace (r-z section) and slot reachability")
    ax.legend(fontsize=7)
    ax.grid(alpha=.3)
    ax.set_aspect("equal", adjustable="box")

    # --- 右图：药箱俯视 + 底座转角 ---
    ax = axes[1]
    for sid, x, y, z, cfg, sol, reasons in results:
        ok = (sol is not None and not reasons)
        ax.scatter([x], [y], c="g" if ok else "r", s=120,
                   marker="s" if ok else "x", zorder=5)
        ax.annotate(str(sid), (x, y), color="w" if ok else "r",
                    fontsize=7, ha="center", va="center", weight="bold")
    ax.scatter([0], [0], c="k", marker="^", s=140, label="arm base")
    th = np.linspace(-np.pi / 2, np.pi / 2, 100)
    ax.plot(280 * np.cos(th), 280 * np.sin(th), "b--", lw=.8, label="yaw limit (+/-90deg)")
    ax.set_xlabel("X (mm, forward)")
    ax.set_ylabel("Y (mm, left)")
    ax.set_title("Medicine box layout (top view)")
    ax.legend(fontsize=7)
    ax.grid(alpha=.3)
    ax.set_aspect("equal", adjustable="box")

    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="4-DOF arm kinematics feasibility check")
    ap.add_argument("--png", default=None, help="输出工作空间图路径")
    ap.add_argument("--layout", default=None, choices=["arc", "grid"], help="药箱布局")
    a = ap.parse_args()
    run(a.png, a.layout)
