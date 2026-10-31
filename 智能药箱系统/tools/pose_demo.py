

#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
r"""
================================================================================
 四姿态复现 Demo —— pose_demo.py
================================================================================
目的
----
把「照片里的 4 个姿态」变成 4 条可以直接下发的舵机指令，在真机上验证：
    这条机械臂到底能不能摆出这 4 个位置。

和 config/poses.json 的关系
---------------------------
poses.json 里存的是**药箱槽位**的点位（每个槽位的 approach / grasp / lift），
那是给主程序取药用的一套完整点位。

本脚本这 4 个姿态是**另一回事**：它们是"机械臂去够药箱上/中/下三排药"时应该
摆成的样子，外加一个复位位。之所以单独放一张表，是因为：
    - 槽位点位是"每个格子一套 approach/grasp/lift"，改起来会牵动取药流程；
      这 4 个姿态只是"臂该摆成什么样"，两边的关注点不同，混在一起必然互相干扰。
    - 真机微调完按 `w` 就能存回 `config/pose_demo.yaml`，下次直接跑。

四个姿态（角度顺序：s0 底座俯仰 / s1 大臂 / s2 小臂 / 夹爪）
------------------------------------------------------------
  编号   名称           s0/s1/s2/夹爪                  末端 (x, z) mm       对应照片
  A(1)  复位立正       180.0 / 120.0 / 120.0 / 20.0  ( -20.0, 289.9)     图片一
  B(2)  取【上层】药   180.0 / 120.0 /  30.0 / 60.0  (   0.1, 214.8)     图片二
  C(3)  取【中层】药   180.0 /  40.0 / 110.0 / 60.0  ( 120.4, 193.6)     图片四
  D(4)  取【下层】药   110.0 / 120.0 / 110.0 / 60.0  ( 206.7, 148.2)     图片三

  编号 A~D 是工具内部用的，数字 1~4 是你那份 demo_poses.py 的写法，**两边都认**
  （`--only 2` / `--pose B` / 交互模式按 `b` 是同一个姿态）。
  顺序按**末端高度从上往下**排：290 → 215 → 194 → 148 mm，
  所以跑一遍就是「先复位，再从最上面那层药一路取到最下面那层」。

★ 这套数字是从真机微调结果抄回来的（2026-09-16），不是算出来的 ★
  你那 4 组角度已经上过真机、看着对，所以这里直接采用。
  表里的末端坐标 / 各段绝对角是**按这套数字反算**出来的，用来交叉核对：
  如果算出来的位置跟你眼睛看到的差很远，说明机械参数（app.yaml 里的
  h0 / l1 / l2 / l3）该重新量了 —— 那是几何问题，不是角度问题。

★ 一个必须知道的硬约束：s2 到不了 180 ★
  s2_home = 180，几何上「第三段与第二段共线」要求 s2 = 180；
  但固件里 SERVO2_LIMIT_MAX_DEG = 150，所以**第三段永远比第二段低至少 30°**。
  好消息：这 4 个姿态里 s2 最大只用到 120，**没有顶到限位**，正常不会堵转嗡鸣。
  想彻底解决共线问题，只有两条路（都要动硬件，本脚本不改）：
      1) 固件把 SERVO2_LIMIT_MAX_DEG 放到 180；
      2) 重新标定 s2_home = 150（即把「共线」定义在 150° 上）。

★ 微调必读：每个舵机往哪个方向转 ★
------------------------------------
三个定位舵机（s0/s1/s2）是**同一套方向约定**：

    角度增大 → 那一段连杆绕自己的关节【逆时针】转（在 (x,z) 平面里看）
    角度减小 → 顺时针转

    直观说法：以该段当前朝前/朝上为参考，
        角度增大 → 该段末端往【上 / 后】走
        角度减小 → 该段末端往【下 / 前】走

| 舵机 | 作用 | 增大角度 | 减小角度 | 固件限位 |
|------|------|----------|----------|----------|
| s0 | 底座俯仰（带动整条臂） | 整条臂逆时针，末端往上/后 | 整条臂顺时针，末端往下/前 | 0 ~ 180 |
| s1 | 大臂（肘） | 大臂带小臂逆时针，末端往上/后 | 顺时针，末端往下/前 | 15 ~ 165 |
| s2 | 小臂（腕） | 小臂带夹爪逆时针，末端往上/后 | 顺时针，末端往下/前 | 0 ~ 150 |
| s3 | 夹爪 | **张开**（★ 反直觉） | **夹紧** | 0 ~ 90 |

★ 只有夹爪 s3 是"角度大 = 张开"，跟三个定位舵机相反。

★ 最好记的一个参照点：s0 = s1 = s2 = 90 ★
  这时三段的绝对角是 0° / 0° / -90°：
      第一段水平前伸 → 第二段继续水平前伸 → 第三段竖直朝下。
  先跑到这个姿态看清"水平"长什么样，再以它为基准加减，比空想快得多。

★ 两个很容易踩的误解（我一开始也写错过）★
  1. "三个 90 = 整条臂水平前伸" —— **只对前两段**。第三段是竖直朝下的，
     因为 s2 = 90 意味着第三段比第二段低了 90°。
  2. "s2 往大调就能把三节拉成一条直线（最远）" —— 三节共线要 s2 = 180，
     固件上限 150，**物理上做不到**。s2 顶到 150 时第三段仍比第二段低 30°。

★ 末端 TCP 最终往哪走，取决于那一刻三段各自朝哪，没有一句话的公式。
  **最省事的办法：只改 1~2°，看屏幕上打印的"末端 (x,z)"怎么变**，
  比背规则快得多。交互模式里 `+ 1 2` / `- 2 2` 就是干这个的。

★ 推荐微调流程（交互模式）
  1. 按 a/b/c/d 走到要调的姿态，先看它跟照片差在哪
  2. `+ 1 2` 或 `- 1 2` 一小步一小步逼近（`step 5` 可以把步长改成 5°）
  3. 每一步屏幕都会打印新的 末端(x,z) 和 各段绝对角，对着照片判断
  4. `delta` 看自己一共调了多少（和表格原值的差）
  5. 调好了按 `w` 存回 config/pose_demo.yaml；或者按 `p` 打印一段可直接粘贴的文本
  6. 四个姿态都调完后，把 `config/pose_demo.yaml` 里的数字抄到正式配置里

怎么跑（PyCharm）
-----------------
  直接右键 Run 本文件即可。命令行等价写法（建议按 0 → 4 的顺序走）：

    # 0) 离线核对：不接硬件、不空跑，只打印姿态表 + 限位体检
    #    （改完 YAML 里的数字先跑这个，最快也最安全）
    python tools/pose_demo.py --show

    # 1) 先看电脑上有哪些串口（蓝牙虚拟口不是你的板子）
    python tools/pose_demo.py --list-ports

    # 2) 空跑：不接硬件，只把要发的帧打出来看
    python tools/pose_demo.py --mock

    # 3) 自检：用内置的假 STM32 走一遍完整串口链路（不接板子也能验证协议对不对）
    python tools/pose_demo.py --selftest

    # 4) 真机交互模式（默认）：进去后按 a/b/c/d 逐个姿态试，用 + / - 微调
    python tools/pose_demo.py --port COM3

    # 5) 真机自动循环：走一遍，每个停 5 秒，重复 3 轮
    python tools/pose_demo.py --port COM3 --cycle --loop 3 --hold 5

    # 6) 只跑某几个姿态（编号认 A~D 也认 1~4），调角度时最省时间
    python tools/pose_demo.py --port COM3 --only 2 --hold 60
    python tools/pose_demo.py --port COM3 --only 2 4 --hold 20

    # 7) 不改文件，直接试一组角度（微调时最方便）
    python tools/pose_demo.py --port COM3 --angles "180,40,110,60"
    python tools/pose_demo.py --port COM3 --angles "180,40,110,60" --angles "180,45,100,60"

  ★ 自动模式默认每个姿态都「复位 → 姿态 → 复位」：
    去和回走同一条路径，去的时候没撞，回来也不会撞。
    想快一倍用 --no-home-each（但末端会在扇形里大范围扫掠，周围要清空）。

姿态表文件（手工微调就改它）
----------------------------
    config/pose_demo.yaml     ← 默认读写这个，带注释，直接改数字即可
    config/pose_demo.json     ← 也支持（--poses-file 指过去就行），但没注释

  ★ YAML 里每项的含义都写在文件里的注释里，不用回来翻代码。
  ★ 交互模式按 `w` 会把当前角度写回该文件，**注释会保留**（按模板重新生成）。
    如果你自己在 YAML 里加了模板之外的注释，按 `w` 会丢 —— 想保平安就用 `p`
    打印片段，手工粘贴。

接线（TTL 转串口）
------------------
    USB-TTL 模块            STM32 板
    --------------------    ------------------
    TX                      RX   (PA10 / USART1_RX)
    RX                      TX   (PA9  / USART1_TX)
    GND                     GND  ← ★ 必须共地，不共地必然乱码
    (VCC 不要接，板子自己供电)

    ★ 舵机电源和 STM32 逻辑电源分开供，舵机地要和逻辑地连一起。
      4 个舵机一起动的瞬间电流能到 2~3A，USB 那点电根本不够，
      电压一掉 STM32 就复位（表现为机械臂动一下就"重启"）。

安全须知（跑之前先看一眼）
--------------------------
  1. 第一次跑请把机械臂**空载**放在桌子中间，周围不要放东西，手别伸进臂的扇形范围。
  2. 手放在电源开关上。听到舵机持续嗡鸣（顶到限位堵转）立刻断电。
  3. 默认运动时长 1200ms，比主程序慢，就是为了让你看清每个姿态。
  4. Ctrl-C 会发急停并关串口。
================================================================================
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Dict, List, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if os.path.join(_ROOT, "pc") not in sys.path:
    sys.path.insert(0, os.path.join(_ROOT, "pc"))

from core.kinematics import ArmGeometry, fk, SERVO_LIMITS      # noqa: E402
from comm.arm_controller import ArmController                 # noqa: E402

try:
    import yaml
except Exception:                                             # pragma: no cover
    yaml = None


# ============================================================================
# 姿态表
# ============================================================================
# 每项：angles = [s0, s1, s2, 夹爪]（度）
#      photo  = 对应照片文件名，方便回头核对
#      desc   = 一句话说明这个姿态在干嘛
#
# ★ 姿态参考照片与下表的文件名一一对应（照片未随仓库分发） ★
#
# 照片 → 姿态的对应关系（用户确认过，别再自己猜）：
#   图片一 18eee6f1…  = 复位位            → A
#   图片二 77d0f915…  = 抓取最上方药品    → B
#   图片四 fab05272…  = 抓取中间药品      → C
#   图片三 576a8acb…  = 抓取最下面药品    → D
#
# ★ 这里只是"出厂默认值"，改它不会影响 config/pose_demo.yaml ★
#   真正生效的是 config/pose_demo.yaml（不存在时才用下面这份并生成文件）。
#   日常微调请直接改 YAML，不要改这里。
DEFAULT_POSES: Dict[str, dict] = {
    "A": {
        "name": "复位立正",
        "desc": "整机立起来收拢，末端在基座正上方略偏后（高 290mm），给下面的底盘让空间。照片一。",
        # 三段绝对角 90 / 120 / 60，末端 (-20.0, 289.9)
        "angles": [180.0, 120.0, 120.0, 0.0],
        "photo": "18eee6f1d07453b3a38574aeae978328.jpg",
    },
    "B": {
        "name": "取【上层】药",
        "desc": "底座立起、大臂抬到最高，小臂回折收回来，手停在最上面那层药前（高 215mm）。照片二。",
        # 三段绝对角 90 / 120 / -30，末端 (0.1, 214.8)
        # 这个姿态手几乎在基座正上方 —— 药箱上层要排在靠基座的位置
        "angles": [180.0, 120.0, 30.0, 90.0],
        "photo": "77d0f915ec0d2fd339f79aac8860cf00.jpg",
    },
    "C": {
        "name": "取【中层】药",
        "desc": "底座立起、大臂放低，小臂往前伸出去，手伸到中间那层药前（高 194mm）。照片四。",
        # 三段绝对角 90 / 40 / -30，末端 (120.4, 193.6)
        "angles": [180.0, 40.0, 110.0, 90.0],
        "photo": "fab052726cd4148dca8d9119fc44aab9.jpg",
    },
    "D": {
        "name": "取【下层】药",
        "desc": "底座放平、三段连杆摊开，手伸到最远最低（高 148mm）。照片三。",
        # 三段绝对角 20 / 50 / -20，末端 (206.7, 148.2)
        # 注意 s0=110 而不是 180：底座段已经放倒到 20° 仰角，手才能伸这么远
        "angles": [110.0, 120.0, 110.0, 90.0],
        "photo": "576a8acb27ddb761c825024ea4c66942.jpg",
    },
}

# 编号别名：你那份 demo_poses.py 用的是 1~4，这里一并认，省得两套编号打架。
# 命令 `--pose 2` / `--only 2 4` / 交互模式 `b`（等价 `2`）都能用。
POSE_ALIASES = {"1": "A", "2": "B", "3": "C", "4": "D"}

# 姿态顺序：按末端高度从上往下（290 → 215 → 194 → 148 mm）
# 所以 demo 跑完 A→B→C→D 就是「复位 → 抓最上 → 抓中间 → 抓最下」，
# 最后停在抓取最下面药品的姿态上。
# ★ 高度数字以 DEFAULT_POSES 的实测值为准（见各姿态的 desc 与 fk() 反算）。
# ★ 可以在 config/pose_demo.yaml 里写 `order: [A, B, C, D]` 覆盖它。
POSE_ORDER = ["A", "B", "C", "D"]

GRIPPER_OPEN = 180.0   # 夹爪全开（180°SG90 逆时针到顶）
GRIPPER_CLOSE = 0.0    # 夹爪夹紧（可微调 10~20）

# ============================================================================
# 四个舵机的"人话说明" —— 微调时全靠这张表，别删
# ============================================================================
# 字段：(代号, 名字, 增大角度的效果, 减小角度的效果)
# 限位不写在这里，统一取 core.kinematics.SERVO_LIMITS（唯一权威来源）
SERVO_ROLE = (
    ("s0", "底座俯仰", "整条臂逆时针，末端往上/后", "整条臂顺时针，末端往下/前"),
    ("s1", "大臂(肘)", "大臂带小臂逆时针，末端往上/后", "顺时针，末端往下/前"),
    ("s2", "小臂(腕)", "小臂带夹爪逆时针，末端往上/后", "顺时针，末端往下/前"),
    ("s3", "夹爪", "张开（★反直觉）", "夹紧"),
)


def _pad(s: str, width: int) -> str:
    """
    按**显示宽度**左对齐补空格。

    为什么不用 '%-16s'：Python 的 % 格式化按**字符数**补，而中文在终端里占 2 列，
    所以 "复位位"(3 字) 和 "抓取最上方药品"(7 字) 用同一个 %-16s 补出来，
    后面的列会差好几个字符对不齐。这里把中文按 2 列算。
    """
    w = sum(2 if ord(ch) > 0x2E80 else 1 for ch in str(s))
    return str(s) + " " * max(0, width - w)


def print_servo_guide() -> None:
    """打印"每个舵机往哪转"速查表（交互模式进来先看这个）"""
    print("-" * 92)
    print("舵机速查：三个定位舵机都是【角度增大 = 该段逆时针 = 末端往上/后】")
    print("-" * 92)
    print("%s %s %s %s %s" % (_pad("舵机", 6), _pad("作用", 12),
                              _pad("增大角度", 30), _pad("减小角度", 26), "固件限位"))
    for i, (code, name, up, down) in enumerate(SERVO_ROLE):
        lo, hi = SERVO_LIMITS[i]
        print("%s %s %s %s %s" % (_pad(code, 6), _pad(name, 12),
                                  _pad(up, 30), _pad(down, 26),
                                  "%.0f ~ %.0f" % (lo, hi)))
    print("-" * 92)
    print("★ 只有夹爪 s3 反过来：角度大 = 张开，角度小 = 夹紧")
    print("★ 末端最终往哪走取决于三段各自朝向 —— 改 1~2° 看打印的 (x,z) 最靠谱")
    print("-" * 92)


# ============================================================================
# 配置与姿态表读写
# ============================================================================
# 生成 YAML 注释时要反算末端位置，需要一份"当前机械参数"。
# make_geometry() 会把它更新成 config/app.yaml 里的真实值；
# 没读过配置时（比如单独 import 本模块）就用这里的默认值，跟 app.yaml 一致。
_GEO_FOR_COMMENT = ArmGeometry(h0=60.0, l1=100.0, l2=95.0, l3=55.0,
                               s0_home=90.0, s1_home=90.0, s2_home=180.0)


def load_config(path: str) -> dict:
    """读 config/app.yaml；读不到就用内置默认值（不报错，方便脱离项目单独跑）"""
    if yaml and os.path.isfile(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            print("[提示] 读取 %s 失败(%s)，改用内置默认值" % (path, e))
    return {}


def make_geometry(cfg: dict) -> ArmGeometry:
    """从 app.yaml 的 arm_geometry 段构造机械参数（改机械臂只需要改 yaml）"""
    global _GEO_FOR_COMMENT
    gsec = cfg.get("arm_geometry") or {}
    g = ArmGeometry(
        h0=float(gsec.get("h0", 60.0)),
        l1=float(gsec.get("l1", 100.0)),
        l2=float(gsec.get("l2", 95.0)),
        l3=float(gsec.get("l3", 55.0)),
        s0_home=float(gsec.get("s0_home", 90.0)),
        s1_home=float(gsec.get("s1_home", 90.0)),
        s2_home=float(gsec.get("s2_home", 180.0)),
    )
    _GEO_FOR_COMMENT = g      # 让 YAML 注释里的反算位置用真实机械参数
    return g


def _read_pose_file(path: str) -> dict:
    """
    读姿态表文件，返回 dict。按扩展名决定用 YAML 还是 JSON。

    YAML 用 PyYAML 的 safe_load —— 它**不保留注释**，但我们只在读的时候用，
    写的时候走 _dump_yaml_text() 自己拼带注释的文本，所以注释不会丢。
    """
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    ext = os.path.splitext(path)[1].lower()
    if ext in (".yaml", ".yml"):
        if yaml is None:
            raise RuntimeError("这个文件是 YAML，但没装 PyYAML（pip install pyyaml）")
        return yaml.safe_load(text) or {}
    return json.loads(text or "{}")


def load_poses(path: str) -> Dict[str, dict]:
    """
    读姿态表。文件不存在就用 DEFAULT_POSES 并**生成一份带注释的模板**，方便直接改。

    文件里可以给：
        poses: {A: {name, desc, photo, angles: [s0,s1,s2,夹爪]}}
        order: [A, B, C, D]        # 可选，覆盖演示顺序
        gripper: {open: 70, close: 15}
    其中 name/desc/photo 可省略，会从 DEFAULT_POSES 补全。
    """
    global POSE_ORDER
    poses = {k: dict(v) for k, v in DEFAULT_POSES.items()}
    if os.path.isfile(path):
        try:
            data = _read_pose_file(path)
            for k, v in (data.get("poses") or {}).items():
                k = str(k).upper()
                base = poses.setdefault(k, {"name": k, "desc": "", "photo": ""})
                if isinstance(v, list):                       # 允许 A: [180,101,150,70]
                    base["angles"] = [float(x) for x in v[:4]]
                else:
                    for fld in ("name", "desc", "photo"):
                        if fld in v:
                            base[fld] = v[fld]
                    if "angles" in v:
                        base["angles"] = [float(x) for x in v["angles"][:4]]
            if data.get("order"):
                POSE_ORDER = [str(x).upper() for x in data["order"]]
            if data.get("gripper"):
                global GRIPPER_OPEN, GRIPPER_CLOSE
                GRIPPER_OPEN = float(data["gripper"].get("open", GRIPPER_OPEN))
                GRIPPER_CLOSE = float(data["gripper"].get("close", GRIPPER_CLOSE))
        except Exception as e:
            print("[提示] 读取 %s 失败(%s)，改用内置姿态表" % (path, e))
    else:
        save_poses(path, poses)
        print("[提示] 已生成姿态表模板: %s（可以直接改里面的数字）" % path)
    return poses


def _dump_yaml_text(poses: Dict[str, dict]) -> str:
    """
    把姿态表渲染成**带完整注释**的 YAML 文本。

    为什么不用 yaml.safe_dump：它会把注释全丢掉，而这份文件的主要用途就是
    让人手工改数字，注释比什么都重要。所以这里自己拼文本 —— 结构是固定的，
    拼起来不难，而且注释能跟代码一起演进。
    """
    L: List[str] = []
    A = L.append
    A("# ============================================================================")
    A("# 四姿态复现 Demo 的姿态表 —— 手工微调就改这个文件")
    A("# ============================================================================")
    A("# 改完直接跑 tools/pose_demo.py 就生效，不用改任何代码。")
    A("#")
    A("# angles 四个数的顺序和含义（单位：度）：")
    A("#     [s0 底座俯仰, s1 大臂, s2 小臂, 夹爪]")
    A("#")
    A("# ★ 方向约定（三个定位舵机一样）★")
    A("#     角度增大 → 那一段绕自己的关节【逆时针】转 → 末端往【上/后】走")
    A("#     角度减小 → 顺时针转 → 末端往【下/前】走")
    A("#     只有夹爪 s3 反过来：角度大 = 张开，角度小 = 夹紧")
    A("#     拿不准就改 1~2°，看脚本打印出来的 末端(x,z) 怎么变，比背规则快。")
    A("#")
    A("# ★ 最好记的参照点：s0 = s1 = s2 = 90 ★")
    A("#     这时三段绝对角是 0° / 0° / -90°：前两段水平前伸，第三段竖直朝下。")
    A("#     先跑到这里看清\"水平\"长什么样，再以它为基准加减。")
    A("#")
    A("# ★ 两个常见误解 ★")
    A("#     1) \"三个 90 = 整条臂水平前伸\" —— 只对前两段，第三段是竖直朝下的。")
    A("#     2) \"s2 调大就能把三节拉直\" —— 共线要 s2 = 180，固件上限 150，做不到。")
    A("#")
    A("# ★ 固件限位（超了会被下位机回 $ERR,03,LIM 拒绝执行）★")
    for i, (code, name, _up, _dn) in enumerate(SERVO_ROLE):
        lo, hi = SERVO_LIMITS[i]
        # 用 _pad 而不是 %-10s：中文占 2 列，用 %-10s 会参差不齐
        A("#     %s %s %.0f ~ %.0f" % (_pad(code, 5), _pad(name, 12), lo, hi))
    A("#")
    A("# ★ 已知的物理缺口 ★")
    A("#     s2_home = 180，几何上要 s2 = 180 才能让第三段与第二段共线，")
    A("#     但固件把 s2 卡在 150，所以第三段永远比第二段低至少 30°。")
    A("#     想彻底解决只能改固件限位或重标定 s2_home。")
    A("#     （好消息：下面这 4 个姿态 s2 最大只用到 120，没顶到限位，不会堵转）")
    A("#")
    A("# ★ 手工编辑小抄 ★")
    A("#     想让某一段更竖直   → 调 s0/s1/s2 让对应的角度接近 90°")
    A("#     想让末端更高       → 一般先把 s0/s1 往大调（先改 1~2° 试）")
    A("#     想让末端伸更远     → 一般先把 s0 往小调（放倒底座段）")
    A("#     改完不确定能不能动 → 先跑 python tools/pose_demo.py --show 做限位体检")
    A("#     想真机慢慢比划     → python tools/pose_demo.py --port COM3 --only 2 --hold 60")
    A("# ============================================================================")
    A("")
    A("# 演示顺序。想改跑的姿态顺序就改这里。")
    A("# 编号 A~D 与数字 1~4 等价（1=A 复位立正 / 2=B 取上层 / 3=C 取中层 / 4=D 取下层）。")
    A("order: [%s]" % ", ".join(POSE_ORDER))
    A("")
    A("# 夹爪两个关键角。$GC,open / $GC,close 用这两个值。")
    A("gripper:")
    A("  open: %.1f      # 张开（角度大 = 张开）" % GRIPPER_OPEN)
    A("  close: %.1f     # 夹紧" % GRIPPER_CLOSE)
    A("")
    A("poses:")
    for key in list(POSE_ORDER) + [k for k in poses if k not in POSE_ORDER]:
        p = poses.get(key)
        if not p:
            continue
        a = [round(float(x), 1) for x in p["angles"]]
        # 数字别名（1=A …），方便你那份 demo_poses.py 的编号习惯
        alias = next((n for n, k in POSE_ALIASES.items() if k == key), None)
        head = "%s  %s" % (key, p.get("name", ""))
        if alias:
            head = "%s(%s)  %s" % (key, alias, p.get("name", ""))
        A("  # --------------------------------------------------------------------------")
        A("  # %s" % head)
        if p.get("desc"):
            A("  #   %s" % p["desc"])
        if p.get("photo"):
            A("  #   参考照片: %s" % p["photo"])
        # 顺手把反算出来的末端位置也写进注释，方便你核对
        if len(a) >= 3:
            (fx, fz), (q1, q2, q3) = fk(a[0], a[1], a[2], _GEO_FOR_COMMENT)
            A("  #   反算末端 (%.1f, %.1f) mm   各段绝对角 %.0f/%.0f/%.0f"
              % (fx, fz, q1, q2, q3))
        A("  %s:" % key)
        A("    name: %s" % _yaml_str(p.get("name", key)))
        A("    desc: %s" % _yaml_str(p.get("desc", "")))
        A("    photo: %s" % _yaml_str(p.get("photo", "")))
        A("    #          s0      s1      s2     夹爪")
        A("    angles: [%.1f, %.1f, %.1f, %.1f]" % tuple(a))
    A("")
    return "\n".join(L)


def _yaml_str(s: str) -> str:
    """把字符串安全地放进 YAML（简单转义，够用）"""
    s = str(s).replace("\\", "\\\\").replace('"', '\\"')
    return '"%s"' % s


def save_poses(path: str, poses: Dict[str, dict]) -> bool:
    """
    把姿态表写回文件（交互模式按 w 会调它）。

    .yaml/.yml → 走 _dump_yaml_text()，**注释完整保留**
    其它        → 写 JSON（没注释，兼容老文件）

    注意：YAML 是整份重新生成的，所以你自己额外加的注释会丢。
          想保平安就用交互模式的 `p` 命令打印片段、手工粘贴。
    """
    try:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        ext = os.path.splitext(path)[1].lower()
        if ext in (".yaml", ".yml"):
            text = _dump_yaml_text(poses)
        else:
            payload = {
                "_comment": "四姿态复现 Demo 的姿态表。angles = [s0 底座俯仰, s1 大臂, "
                            "s2 小臂, 夹爪]。夹爪：角度小=夹紧，角度大=张开。"
                            "改完直接跑 tools/pose_demo.py 即可生效。",
                "order": list(POSE_ORDER),
                "gripper": {"open": GRIPPER_OPEN, "close": GRIPPER_CLOSE},
                "poses": {k: {"name": v.get("name", k),
                              "desc": v.get("desc", ""),
                              "photo": v.get("photo", ""),
                              "angles": [round(float(x), 1) for x in v["angles"]]}
                          for k, v in poses.items()},
            }
            text = json.dumps(payload, ensure_ascii=False, indent=2)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        return True
    except Exception as e:
        print("[错误] 写入 %s 失败: %s" % (path, e))
        return False


# ============================================================================
# 姿态体检 / 打印
# ============================================================================
def _norm_key(key: str) -> str:
    """
    把用户写的姿态编号统一成大写字母。

    认两种写法：字母 A/B/C/D（工具内部用的），数字 1/2/3/4（你那份 demo_poses.py 用的）。
    1=A 复位立正、2=B 取上层、3=C 取中层、4=D 取下层。
    """
    k = str(key).strip().upper()
    return POSE_ALIASES.get(k, k)


def check_angles(angles: List[float]) -> List[str]:
    """
    校验一组角度能不能安全下发，返回问题列表（空列表 = 没问题）。

    为什么要在 PC 侧先查一遍：固件收到越限角度会回 $ERR,03,LIM 直接拒绝执行，
    但那时机械臂已经收到一半指令了。在 PC 侧先拦住，能给出更清楚的中文原因。
    """
    problems: List[str] = []
    names = ("s0 底座俯仰", "s1 大臂", "s2 小臂", "夹爪")
    for i, (v, (lo, hi)) in enumerate(zip(angles, SERVO_LIMITS)):
        if not (lo <= v <= hi):
            problems.append("%s=%.1f 超出固件限位 [%.0f, %.0f]" % (names[i], v, lo, hi))
    return problems


def pose_fk_text(angles: List[float], g: ArmGeometry) -> str:
    """算这个姿态的末端位置与各段方向角，返回一行人话"""
    (x, z), (p1, p2, p3) = fk(angles[0], angles[1], angles[2], g)
    return "末端(%7.1f,%7.1f)mm   各段绝对角 %5.1f/%5.1f/%5.1f   末端姿态 %6.1f°" % (
        x, z, p1, p2, p3, p3)


def print_pose_table(poses: Dict[str, dict], g: ArmGeometry) -> None:
    """打印完整姿态表（含末端位置、限位结论）"""
    print("=" * 112)
    print("四姿态表   机械参数: H0=%.0f  L1=%.0f L2=%.0f L3=%.0f  总臂展 %.0f mm"
          % (g.h0, g.l1, g.l2, g.l3, g.total_reach))
    print("  编号按末端高度从上往下排；演示顺序 A→B→C→D（数字 1~4 等价）")
    print("      = 复位立正 → 取上层 → 取中层 → 取下层")
    print("=" * 112)
    print("%s %s %s %s %s"
          % (_pad("编号", 8), _pad("名称", 16), _pad("s0/s1/s2/夹爪", 24),
             _pad("末端位置 / 各段方向角 / 末端姿态", 50), "限位"))
    print("-" * 112)
    for i, key in enumerate(POSE_ORDER):
        if key not in poses:
            continue
        p = poses[key]
        a = p["angles"]
        bad = check_angles(a)
        print("%s %s %s %s %s"
              % (_pad("%s(%d)" % (key, i + 1), 8), _pad(p.get("name", ""), 16),
                 _pad("%.1f/%.1f/%.1f/%.1f" % tuple(a), 24),
                 _pad(pose_fk_text(a, g), 50),
                 "OK" if not bad else "X " + "; ".join(bad)))
    print("-" * 112)
    print("夹爪：%.0f = 全开，%.0f = 夹紧（角度大=张开，180°SG90）" % (GRIPPER_OPEN, GRIPPER_CLOSE))
    print("★ 这四个姿态都没顶到 s2=150 的固件上限，正常不会堵转嗡鸣。")
    print("=" * 112)


# ============================================================================
# Demo 主逻辑
# ============================================================================
class PoseDemo:
    """
    四姿态演示器 + 微调工作台。

    演示：把姿态表里的角度 → 变成 $SA 帧 → 等 STM32 说做完。
    微调：`+`/`-`/`=` 一点点挪角度，每步都打印新的 末端(x,z)，调好按 `w` 存回文件。

    串口协议本身完全复用 pc/comm/arm_controller.py，不在这里重写一遍，
    免得两处协议实现慢慢跑偏。
    """

    def __init__(self, arm: ArmController, poses: Dict[str, dict],
                 g: ArmGeometry, poses_file: str, tt: int = 1200,
                 readback: bool = True):
        self.arm = arm
        self.poses = poses
        self.g = g
        self.poses_file = poses_file
        self.tt = int(tt)
        self.readback = bool(readback)

        self.cur: List[float] = [0.0, 0.0, 0.0, 0.0]   # 当前（最后一次下发的）角度
        self.last_key: Optional[str] = None            # 最后一次选中的姿态编号
        self.base: Optional[List[float]] = None        # 该姿态在文件里的原值（算差值用）
        self.step: float = 2.0                         # `+`/`-` 不写增量时用这个步长
        self._undo: Optional[List[float]] = None       # 上一步的角度（`u` 回退用）
        self._tuned: Dict[str, List[float]] = {}       # 本会话里被改过的姿态
        self._orig: Dict[str, List[float]] = {}        # 本会话首次看到该姿态时的「文件原值」
                                                       # （保存后 self.poses 会被改掉，
                                                       #   所以要单独留一份才能算出总改动）

    # ------------------------------------------------------------------
    # 基础动作
    # ------------------------------------------------------------------
    def _readback(self) -> Optional[List[float]]:
        """
        读回四个舵机的实际角度；空跑模式或读回关闭时返回 None。

        ★ 为什么要挡掉空跑模式：MockArmController.query_pose() 返回的是写死的
          占位值。如果拿它去更新 self.cur，交互模式按 w 就会把垃圾角度存回
          JSON，把好好的姿态表写坏（这个坑踩过一次）。
        """
        if not self.readback or getattr(self.arm, "mock", False):
            return None
        actual = self.arm.query_pose()
        return actual if actual and len(actual) >= 4 else None

    def _report(self, title: str, angles: List[float], ok: bool,
                actual: Optional[List[float]] = None) -> None:
        """统一的结果输出：目标角度 / 预测位置 / 实际角度 / 误差"""
        print("  [%s] %s" % ("OK " if ok else "失败", title))
        print("       指令角度 s0/s1/s2/夹爪 = %.1f/%.1f/%.1f/%.1f   (时长 %dms)"
              % (angles[0], angles[1], angles[2], angles[3], self.tt))
        print("       预测 %s" % pose_fk_text(angles, self.g))
        if getattr(self.arm, "mock", False):
            # 空跑模式下 MockArmController 返回的是写死的占位值，
            # 拿它算偏差只会误导人，直接说明白。
            print("       （空跑模式：没有真实舵机，不读回角度）")
        elif actual is not None and len(actual) >= 3:
            d = [actual[i] - angles[i] for i in range(min(4, len(actual)))]
            print("       实测 s0/s1/s2/夹爪 = %.1f/%.1f/%.1f/%.1f   偏差 %+.1f/%+.1f/%+.1f/%+.1f"
                  % (actual[0], actual[1], actual[2],
                     actual[3] if len(actual) > 3 else 0.0, d[0], d[1], d[2],
                     d[3] if len(d) > 3 else 0.0))
            worst = max(abs(x) for x in d[:3])
            if worst > 5.0:
                print("       !! 定位偏差 %.1f° 偏大：检查舵机供电是否够、"
                      "连杆有没有松动、或把 --tt 调大再试" % worst)

    def goto(self, key: str, tt: Optional[int] = None) -> bool:
        """走到某个姿态：下发角度 → 等 DONE → 读回实际角度对比（编号认 A~D 和 1~4）"""
        key = _norm_key(key)
        if key not in self.poses:
            print("  [错误] 没有姿态 %s，可用: %s（或数字 1~4）"
                  % (key, "/".join(POSE_ORDER)))
            return False

        p = self.poses[key]
        a = [float(x) for x in p["angles"]]
        bad = check_angles(a)
        if bad:
            print("  [拒绝下发] %s 的角度超出固件限位：" % key)
            for b in bad:
                print("             - %s" % b)
            return False

        print("\n>>> 姿态 %s  %s" % (key, p.get("name", "")))
        if p.get("desc"):
            print("    %s" % p["desc"])
        use_tt = int(tt if tt is not None else self.tt)

        ok = self.arm.move_all(a[0], a[1], a[2], a[3], tt=use_tt)
        self.cur = list(a)
        self.last_key = key
        # 记下"文件里的原值"，之后 `delta` 才能算出你一共调了多少
        self.base = list(a)
        # 本会话第一次看到这个姿态时留一份原值（保存后 poses 会被改掉，要留着对比）
        self._orig.setdefault(key, list(a))
        self._undo = None
        # 如果这个姿态之前被微调过，跳回来时用微调后的值，不要被文件原值冲掉
        if key in self._tuned:
            self.cur = list(self._tuned[key])

        actual = None
        if ok:
            time.sleep(0.15)
            actual = self._readback()
        self._report("姿态 %s %s" % (key, p.get("name", "")), a, ok, actual)
        return ok

    def home(self, tt: int = 1500) -> bool:
        """回零位（每次上电/开始演示前先做，保证起点一致）"""
        print("\n>>> 回零位 $HOM")
        ok = self.arm.home(timeout=tt / 1000.0 + 4.0)
        if ok:
            time.sleep(0.15)
            actual = self._readback()
            if actual:
                self.cur = list(actual)
        print("  [%s] 回零位" % ("OK " if ok else "失败"))
        return ok

    def grip(self, open_it: bool) -> bool:
        """夹爪开合（走 $GC，和 $SA 里的夹爪角度是两条独立通道）"""
        ang = GRIPPER_OPEN if open_it else GRIPPER_CLOSE
        print("\n>>> 夹爪 %s ($GC,%s)" % ("张开" if open_it else "夹紧",
                                          "open" if open_it else "close"))
        ok = self.arm.grip(open_it)
        if ok:
            self.cur[3] = ang
        print("  [%s] 夹爪 %s" % ("OK " if ok else "失败", "张开" if open_it else "夹紧"))
        return ok

    def servo(self, idx: int, ang: float) -> bool:
        """单舵机微调（绝对角度）—— 交互模式的 `=` 命令走这里"""
        return self._apply(idx, ang, why="")

    # ------------------------------------------------------------------
    # 微调工作台（对着照片一点点逼近时用这些）
    # ------------------------------------------------------------------
    def _status(self) -> None:
        """
        打印一行"当前状态"：四个角度 + 预测末端位置。

        每次微调后都会调它 —— 这是微调时最重要的反馈：
        改了 1~2° 到底让末端往哪走了，看这行比背方向规则快。
        """
        a = self.cur
        bad = check_angles(a)
        print("       当前 s0/s1/s2/夹爪 = %.1f / %.1f / %.1f / %.1f"
              % (a[0], a[1], a[2], a[3]))
        print("       %s" % pose_fk_text(a, self.g))
        if bad:
            print("       !! %s" % "; ".join(bad))

    def _apply(self, idx: int, ang: float, why: str = "") -> bool:
        """
        把一个舵机挪到指定角度：查限位 → 下发 $SS → 更新 self.cur。

        参数
        ----
        idx : 0 底座 / 1 大臂 / 2 小臂 / 3 夹爪
        ang : 目标绝对角度
        why : 打印在日志里的说明（比如 "+2.0°"），纯给人看
        """
        if not (0 <= idx <= 3):
            print("  [错误] 舵机号只能是 0~3（0 底座 / 1 大臂 / 2 小臂 / 3 夹爪）")
            return False
        lo, hi = SERVO_LIMITS[idx]
        if not (lo <= ang <= hi):
            print("  [拒绝] %s %s 想设成 %.1f，超出固件限位 [%.0f, %.0f]"
                  % (SERVO_ROLE[idx][0], SERVO_ROLE[idx][1], ang, lo, hi))
            print("         下位机收到越限角度会回 $ERR,03,LIM 拒绝执行，所以在 PC 侧先拦住")
            return False

        if self.last_key is None:
            # 不影响动作，但得提醒一句：没选姿态的话 `w` 没地方存
            print("  [提示] 还没选姿态，先按 a/b/c/d 选一个（不然调好了按 w 存不了）")

        before = list(self.cur)
        print("\n>>> %s%s -> %.1f   ($SS,%d,%.1f)"
              % (why, SERVO_ROLE[idx][1], ang, idx, ang))
        ok = self.arm.move_servo(idx, ang, tt=min(self.tt, 800))
        if ok:
            self._undo = before
            self.cur[idx] = float(ang)
            time.sleep(0.1)
            actual = self._readback()
            if actual:
                self.cur[idx] = float(actual[idx])
            if self.last_key:
                self._tuned[self.last_key] = list(self.cur)
        print("  [%s] %s = %.1f" % ("OK " if ok else "失败",
                                     SERVO_ROLE[idx][0], self.cur[idx]))
        self._status()
        return ok

    def nudge(self, idx: int, delta: float = None) -> bool:
        """
        相对微调：在当前角度上 +delta 度（delta 为负就是减）。

        这是对着照片逼近时最常用的命令 —— 一次只挪 1~2°，
        看末端 (x,z) 往哪走，再决定下一步加还是减。
        """
        if delta is None:
            delta = self.step
        if not (0 <= idx <= 3):
            print("  [错误] 舵机号只能是 0~3（0 底座 / 1 大臂 / 2 小臂 / 3 夹爪）")
            return False
        return self._apply(idx, self.cur[idx] + float(delta),
                           why="%+.1f° " % float(delta))

    def set_step(self, v: float = None) -> None:
        """查看/设置 `+`/`-` 不写增量时的默认步长"""
        if v is None:
            print("  当前微调步长 = %.1f°（命令：step 5 改成 5°）" % self.step)
            return
        if not (0.1 <= float(v) <= 30.0):
            print("  [拒绝] 步长请给 0.1 ~ 30 之间")
            return
        self.step = float(v)
        print("  微调步长已设为 %.1f°" % self.step)

    def set_all(self, angles: List[float]) -> bool:
        """四个舵机一起设到指定角度（走 $SA，四轴同步运动，比一个个挪平滑）"""
        if len(angles) < 3:
            print("  [错误] 至少要给 s0 s1 s2 三个角度，例：set 180 15 150 70")
            return False
        a = list(angles[:4])
        while len(a) < 4:
            a.append(self.cur[3])
        bad = check_angles(a)
        if bad:
            print("  [拒绝下发] 这组角度超出固件限位：")
            for b in bad:
                print("             - %s" % b)
            return False

        before = list(self.cur)
        print("\n>>> 四轴一起设 $SA,%.1f,%.1f,%.1f,%.1f,%d"
              % (a[0], a[1], a[2], a[3], self.tt))
        ok = self.arm.move_all(a[0], a[1], a[2], a[3], tt=self.tt)
        if ok:
            self._undo = before
            self.cur = list(a)
            time.sleep(0.15)
            actual = self._readback()
            if actual:
                self.cur = list(actual)
            if self.last_key:
                self._tuned[self.last_key] = list(self.cur)
        print("  [%s] 四轴到位" % ("OK " if ok else "失败"))
        self._status()
        return ok

    def undo_last(self) -> bool:
        """回退到上一次微调之前的角度（挪过头了用）"""
        if self._undo is None:
            print("  [提示] 没有可回退的步骤（只记一步）")
            return False
        a = list(self._undo)
        self._undo = None
        print("\n>>> 回退到 %.1f / %.1f / %.1f / %.1f" % tuple(a))
        ok = self.arm.move_all(a[0], a[1], a[2], a[3], tt=min(self.tt, 800))
        if ok:
            self.cur = list(a)
            if self.last_key:
                self._tuned[self.last_key] = list(self.cur)
        print("  [%s] 已回退" % ("OK " if ok else "失败"))
        self._status()
        return ok

    def show_delta(self) -> None:
        """
        显示"当前角度"与"文件原值"的差 —— 看自己一共调了多少。

        顺便给出末端位置的位移，方便判断这轮微调是不是白调了。
        """
        if not self.last_key or self.base is None:
            print("  [提示] 先按 a/b/c/d 选一个姿态，再打 delta 才有的比")
            return
        p = self.poses.get(self.last_key, {})
        cur = list(self.cur)
        base = list(self.base)
        d = [cur[i] - base[i] for i in range(4)]
        (bx, bz), _ = fk(base[0], base[1], base[2], self.g)
        (cx, cz), _ = fk(cur[0], cur[1], cur[2], self.g)
        print("-" * 84)
        print("姿态 %s  %s   （与 config 文件里的原值对比）" % (self.last_key, p.get("name", "")))
        print("  文件原值   s0/s1/s2/夹爪 = %.1f / %.1f / %.1f / %.1f" % tuple(base))
        print("  当前值     s0/s1/s2/夹爪 = %.1f / %.1f / %.1f / %.1f" % tuple(cur))
        print("  差值             %+.1f    %+.1f    %+.1f    %+.1f" % tuple(d))
        print("  末端位置   原值 (%7.1f, %7.1f)  ->  当前 (%7.1f, %7.1f)   位移 (%+.1f, %+.1f) mm"
              % (bx, bz, cx, cz, cx - bx, cz - bz))
        if all(abs(x) < 0.05 for x in d):
            print("  （还没调过，或者已经调回原值了）")
        print("-" * 84)

    def print_snippet(self) -> None:
        """
        打印一段可以直接粘贴回 YAML 的文本。

        用途：手工加过注释、不想让 `w` 覆盖掉整个文件时，用这个复制粘贴。
        """
        if not self.last_key:
            print("  [提示] 先按 a/b/c/d 选一个姿态")
            return
        key = self.last_key
        a = [round(float(x), 1) for x in self.cur]
        print("-" * 84)
        print("把下面两行贴到 %s 里 %s 的 angles 上：" % (self.poses_file, key))
        print("")
        print("    #          s0      s1      s2     夹爪")
        print("    angles: [%.1f, %.1f, %.1f, %.1f]" % tuple(a))
        print("")
        print("（当前 %s 的 angles 原文是 %s）"
              % (key, "[" + ", ".join("%.1f" % x for x in self.base) + "]"
                 if self.base else "未记录"))
        print("-" * 84)

    def save_current(self) -> bool:
        """把当前角度存回 last_key 对应的姿态（对着照片调好了就按这个）"""
        if not self.last_key:
            print("  [提示] 还没选过姿态，先按 a/b/c/d 选一个再调，然后才能存")
            return False
        key = self.last_key
        self.poses[key]["angles"] = [round(float(x), 1) for x in self.cur]
        if save_poses(self.poses_file, self.poses):
            self.base = list(self.poses[key]["angles"])
            print("  [已保存] 姿态 %s 的角度更新为 %.1f/%.1f/%.1f/%.1f -> %s"
                  % (key, self.cur[0], self.cur[1], self.cur[2], self.cur[3],
                     self.poses_file))
            print("          （%s 里的注释是按模板重新生成的，保留完整）"
                  % os.path.basename(self.poses_file))
            return True
        return False

    def save_report(self) -> None:
        """
        把本会话改过的姿态汇总打印一遍，方便一次性抄走。

        「原值」取 self._orig（本会话第一次看到该姿态时的文件值），
        而不是 self.poses —— 因为按过 w 之后 self.poses 已经被更新了，
        拿它当"原值"会看到「原值 == 现在」这种没意义的输出。
        """
        if not self._tuned:
            print("  [提示] 本次会话还没有改过任何姿态")
            return
        print("=" * 84)
        print("本次会话改过的姿态（可直接抄进配置文件）")
        print("=" * 84)
        for key in POSE_ORDER:
            if key not in self._tuned:
                continue
            base = list(self._orig.get(key) or [])
            cur = list(self._tuned[key])
            print("  %s  %s" % (key, self.poses.get(key, {}).get("name", "")))
            if len(base) >= 3:
                (bx, bz), _ = fk(base[0], base[1], base[2], self.g)
                print("      原值 %s   末端 (%.1f, %.1f)"
                      % ("[" + ", ".join("%.1f" % x for x in base) + "]", bx, bz))
            else:
                print("      原值 %s" % ("[" + ", ".join("%.1f" % x for x in base) + "]"))
            if len(cur) >= 3:
                (cx, cz), _ = fk(cur[0], cur[1], cur[2], self.g)
                print("      现在 %s   末端 (%.1f, %.1f)"
                      % ("[" + ", ".join("%.1f" % x for x in cur) + "]", cx, cz))
            else:
                print("      现在 %s" % ("[" + ", ".join("%.1f" % x for x in cur) + "]"))
        print("=" * 84)

    # ------------------------------------------------------------------
    # 三种运行模式
    # ------------------------------------------------------------------
    def run_sequence(self, keys: List[str], repeat: int, dwell: float,
                     home_first: bool = True, home_each: bool = True) -> int:
        """
        自动依次走完给定姿态，每个停 dwell 秒。返回成功次数。

        home_each=True（默认）时，每个姿态都 **从复位位出发、回复位位结束**：
            复位 → 姿态1 → 复位 → 姿态2 → 复位 → …
        为什么默认开：去和回走的是同一条路径，去的时候没撞，回来也不会撞。
        姿态之间直接切换会让末端在扇形里大范围扫掠，容易带倒药瓶或扫到人手。
        代价是慢一倍（每个姿态多两次 $SA）。想省时间用 --no-home-each。

        注意：这里的"复位位"用的是姿态表里的第一个姿态（A / 1），
        不是固件的 $HOM —— $HOM 是舵机回中位，跟"立正收拢"不是一回事。
        """
        if home_first:
            self.home()
            time.sleep(0.3)
        n_ok = 0
        home_key = POSE_ORDER[0] if POSE_ORDER else "A"
        for r in range(1, max(1, repeat) + 1):
            print("\n" + "#" * 72)
            print("# 第 %d/%d 轮" % (r, max(1, repeat)))
            print("#" * 72)
            for k in keys:
                if self.goto(k):
                    n_ok += 1
                time.sleep(max(0.0, dwell))
                if home_each and _norm_key(k) != home_key:
                    # 回"复位位"（姿态表里第一个），为下一个姿态准备同一条来路
                    self.goto(home_key)
                    time.sleep(0.3)
        print("\n>>> 序列结束：%d 次成功 / %d 次下发" % (n_ok, len(keys) * max(1, repeat)))
        return n_ok

    def run_interactive(self) -> None:
        """
        交互模式：命令行一行一个命令。

        为什么不用键盘单键（msvcrt）：PyCharm 的 Run 窗口不是真终端，
        单键读取拿不到，反而会吞输入。按行读 input() 在哪都能用。
        """
        print("""
================================================================================
 交互模式 —— 一行一个命令，回车执行
--------------------------------------------------------------------------------
 【走姿态】
   a / b / c / d     走到对应姿态（也可直接打数字 1 / 2 / 3 / 4）
                     A(1) 复位立正 · B(2) 取上层 · C(3) 取中层 · D(4) 取下层
   n  或 直接回车     走到下一个姿态（按 A→B→C→D 的顺序循环）
   h                 回零位（$HOM，舵机回中位；跟"复位立正"不是一回事）
   o / f             夹爪张开 / 夹紧

 【微调】★ 对着照片一点点逼近，主要就用这几条 ★
   + <舵机> [步长]    某舵机加一点。例：+ 1 2  → s1 加 2°；不写步长用默认(见 step)
   - <舵机> [步长]    某舵机减一点。例：- 2 3  → s2 减 3°
   = <舵机> <角度>    直接设成绝对角度。例：= 2 150
   set <s0> <s1> <s2> [夹爪]   四个一起设，走 $SA 同步运动（比一个个挪平滑）
   step [度数]        查看 / 设置 +/- 的默认步长（默认 2°）
   u                 回退一步（挪过头了用；只记一步）
   delta             看当前角度和表格原值差多少（含末端位移，判断这轮有没有白调）
                     ★ 注意：`d` 是「走到姿态 D」，看差值要打 delta

 【查看 / 保存】
   l                 打印四姿态表
   g                 查询当前四个舵机的实际角度（真机才有意义）
   guide 或 ?         再看一遍「每个舵机往哪转」的速查表
   p                 打印一段可直接粘贴进 YAML 的 angles（不改文件）
   w                 把当前角度存回姿态表（按模板重生成，注释保留）
   e                 汇总本次会话改过的所有姿态

   t <毫秒>           设置运动时长，例：t 2000（越大越慢越稳）
   q                 退出（会先回零位）

 舵机号：0 底座俯仰 / 1 大臂(肘) / 2 小臂(腕) / 3 夹爪
 微调心法：一次只挪 1~2°，看屏幕上打印的「末端 (x,z)」往哪走，再决定下一步。
           下面这张速查表就是干这个用的 ↓
================================================================================""")
        print_servo_guide()
        idx = -1
        while True:
            try:
                line = input("\n[pose_demo] 命令 > ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n收到中断，退出。")
                return

            if line == "":
                line = "n"
            parts = line.split()
            cmd = parts[0].lower()

            try:
                if cmd in ("q", "quit", "exit"):
                    print("退出前回零位…")
                    self.home()
                    return

                elif cmd in ("a", "b", "c", "d", "1", "2", "3", "4"):
                    self.goto(cmd)

                elif cmd == "n":
                    idx = (idx + 1) % len(POSE_ORDER)
                    self.goto(POSE_ORDER[idx])

                elif cmd == "h":
                    self.home()

                elif cmd in ("o", "open"):
                    self.grip(True)

                elif cmd in ("f", "close"):
                    self.grip(False)

                # ---------------- 微调：+ / - 相对挪 ----------------
                # 三种写法都认： "+ 1 2" / "+1 2" / "- 2"
                elif cmd in ("+", "-") or (len(cmd) > 1 and cmd[0] in "+-" and cmd[1:].isdigit()):
                    if len(cmd) > 1:                    # "+1 2" 这种连着写的
                        sidx, rest = int(cmd[1:]), parts[1:]
                    elif len(parts) >= 2:               # "+ 1 2" 这种分开写的
                        sidx, rest = int(parts[1]), parts[2:]
                    else:
                        print("  用法：+ <舵机号> [步长]，例：+ 1 2（s1 加 2°）")
                        continue
                    sign = 1.0 if cmd[0] == "+" else -1.0
                    mag = abs(float(rest[0])) if rest else self.step
                    self.nudge(sidx, sign * mag)

                # ---------------- 微调：= / s 设绝对角 ----------------
                elif cmd in ("=", "s") and len(parts) >= 3:
                    self._apply(int(parts[1]), float(parts[2]), why="设为 ")

                # ---------------- 微调：四个一起设（$SA 同步运动）----------------
                elif cmd == "set":
                    if len(parts) < 4:
                        print("  用法：set <s0> <s1> <s2> [夹爪]，例：set 180 15 150 70")
                        continue
                    self.set_all([float(x) for x in parts[1:5]])

                elif cmd == "step":
                    self.set_step(float(parts[1]) if len(parts) >= 2 else None)

                elif cmd in ("u", "undo"):
                    self.undo_last()

                elif cmd in ("delta", "dv", "df"):
                    self.show_delta()

                elif cmd in ("p", "snippet"):
                    self.print_snippet()

                elif cmd in ("w", "save"):
                    self.save_current()

                elif cmd in ("e", "report"):
                    self.save_report()

                # ---------------- 查看 ----------------
                elif cmd == "g":
                    actual = self._readback()
                    if actual:
                        print("  实测 s0/s1/s2/夹爪 = %.1f/%.1f/%.1f/%.1f" % tuple(actual[:4]))
                    elif getattr(self.arm, "mock", False):
                        print("  空跑模式，没有真实角度可读。")
                    else:
                        print("  查询失败（超时，或下位机固件不支持 $QRY,POSE）")

                elif cmd == "l":
                    print_pose_table(self.poses, self.g)

                elif cmd in ("guide", "?", "help"):
                    print_servo_guide()

                elif cmd == "t" and len(parts) >= 2:
                    self.tt = int(parts[1])
                    print("  运动时长已设为 %d ms" % self.tt)

                else:
                    print("  看不懂这个命令。输入 guide 看舵机速查表，输入 l 看四姿态表。")
            except ValueError:
                print("  参数不是数字，例：s 1 95 / t 2000")
            except Exception as e:
                print("  执行出错: %s" % e)


# ============================================================================
# 自检：内置假 STM32，不接板子也能验证整条串口链路
# ============================================================================
class _FakeSerial:
    """
    内存版串口：把 PC 写出去的帧收下来，按固件的行为回 ACK / DONE / POSE。

    用途：没有板子在手上时，验证「帧格式、校验和、收发时序」这套东西是通的。
    只实现 arm_controller 用到的几个方法，不追求完整。
    """

    def __init__(self, *a, **kw):
        self._rxbuf = bytearray()    # STM32 -> PC（我们往里塞回包）
        self._txbuf = bytearray()    # PC -> STM32（PC 写进来的字节）
        self.is_open = True
        self.dtr = False
        self.rts = False
        self.seen: List[str] = []
        # 模拟固件的"当前舵机角"状态：$SA/$SS/$GC 会改它，$QRY 会把它回给 PC。
        # 这样读回校验才有意义（真板子就是这么干的）。
        self.last_angles: List[float] = [90.0, 90.0, 150.0, 70.0]
        self._rxbuf.extend(_frame("READY"))   # 上电就绪帧

    @property
    def in_waiting(self) -> int:
        self._pump()
        return len(self._rxbuf)

    def read(self, n: int = 1) -> bytes:
        self._pump()
        if not self._rxbuf:
            time.sleep(0.001)
            return b""
        data = bytes(self._rxbuf[:n])
        del self._rxbuf[:n]
        return data

    def write(self, b: bytes) -> int:
        self._txbuf.extend(b)
        self._pump()
        return len(b)

    def _pump(self) -> None:
        """把 PC 写来的完整行解析出来，模拟固件回包"""
        while True:
            i = self._txbuf.find(b"\r\n")
            if i < 0:
                return
            raw = bytes(self._txbuf[:i])
            del self._txbuf[:i + 2]
            body = _verify(raw.decode("ascii", "ignore"))
            if body is None:
                self._rxbuf.extend(_frame("ERR,01,CS"))
                continue
            parts = body.split(",")
            cmd = parts[0].strip().upper()
            self.seen.append(body)

            # --- 先按固件的行为更新"当前角度" ---
            try:
                if cmd == "SA" and len(parts) >= 5:
                    self.last_angles = [float(x) for x in parts[1:5]]
                elif cmd == "SS" and len(parts) >= 3:
                    i2 = int(parts[1])
                    if 0 <= i2 <= 3:
                        self.last_angles[i2] = float(parts[2])
                elif cmd == "GC" and len(parts) >= 2:
                    self.last_angles[3] = 70.0 if parts[1].lower().startswith("open") else 15.0
                elif cmd == "HOM":
                    self.last_angles = [146.1, 41.1, 82.8, 15.0]
            except ValueError:
                pass

            self._rxbuf.extend(_frame("ACK,%s" % cmd))
            if cmd in ("SA", "SS", "HOM", "GC", "RUN"):
                self._rxbuf.extend(_frame("DONE,%s" % cmd))
            elif cmd == "QRY":
                self._rxbuf.extend(_frame("POSE," + ",".join(
                    "%.1f" % v for v in self.last_angles)))


class _FakeSerialModule:
    Serial = _FakeSerial


def _frame(body: str) -> bytes:
    from comm.serial_protocol import build_frame
    return build_frame(body)


def _verify(line: str) -> Optional[str]:
    from comm.serial_protocol import verify_line
    return verify_line(line)


def run_selftest(poses: Dict[str, dict], g: ArmGeometry, poses_file: str) -> int:
    """
    用内置假 STM32 把四个姿态完整走一遍，验证：
      1) 帧能正常打包（含 XOR 校验）
      2) 收得到 ACK / DONE，握手不超时
      3) 四个姿态的角度都下发出去了
      4) $QRY,POSE 能读回角度
    返回 0 = 通过，1 = 失败。
    """
    import comm.arm_controller as ac

    print("=" * 108)
    print("自检模式：使用内置假 STM32（不碰真实串口）")
    print("=" * 108)

    real_serial, real_has = ac.serial, ac._HAS_SERIAL
    fake = _FakeSerial()
    ac.serial = _FakeSerialModule()
    ac._HAS_SERIAL = True
    # 让每次新建 Serial 都返回同一个 fake，方便最后取 seen
    _FakeSerialModule.Serial = lambda *a, **kw: fake

    rc = 0
    try:
        arm = ArmController(port="FAKE", baud=115200, timeout=2.0, verbose=False)
        if not arm.connect():
            print("X 连接失败")
            return 1
        if not arm.wait_ready(timeout=2.0):
            print("X 没等到 $READY")
            rc = 1
        else:
            print("OK  等到 $READY（握手正常）")

        demo = PoseDemo(arm, poses, g, poses_file, tt=200, readback=True)
        demo.home()                      # 顺带验证 $HOM
        n_ok = 0
        for k in POSE_ORDER:
            if demo.goto(k):
                n_ok += 1
        print("\n>>> 姿态下发成功 %d/%d" % (n_ok, len(POSE_ORDER)))
        if n_ok != len(POSE_ORDER):
            rc = 1

        # 逐条核对：每个姿态的 $SA 帧都在假 STM32 的记录里
        seen = fake.seen
        for k in POSE_ORDER:
            a = poses[k]["angles"]
            want = "SA,%.1f,%.1f,%.1f,%.1f" % (a[0], a[1], a[2], a[3])
            hit = any(s.startswith(want) for s in seen)
            print("  %s 姿态 %s 的 $%s… %s" % ("OK " if hit else "X ",
                                              k, want, "已下发" if hit else "没找到！"))
            if not hit:
                rc = 1

        if any(s.startswith("QRY") for s in seen):
            print("OK  读回指令 $QRY,POSE 已下发，且收到 POSE 帧")
        else:
            print("X  没发出读回指令")
            rc = 1

        print("\n共发出 %d 帧: %s" % (len(seen), ", ".join(seen)))
        arm.close()
    finally:
        ac.serial, ac._HAS_SERIAL = real_serial, real_has

    print("=" * 108)
    print("自检结果: %s" % ("通过 —— 串口协议链路没问题，可以接板子跑了"
                            if rc == 0 else "失败 —— 见上面 X 的条目"))
    print("=" * 108)
    return rc


# ============================================================================
# 命令行入口
# ============================================================================
def main() -> int:
    ap = argparse.ArgumentParser(
        description="四姿态复现 Demo：把照片里的 4 个姿态发给 STM32 机械臂",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="例: python tools/pose_demo.py --show                 # 先离线核对角度\n"
               "    python tools/pose_demo.py --mock                 # 再空跑看帧\n"
               "    python tools/pose_demo.py --selftest             # 再假 STM32 自检\n"
               "    python tools/pose_demo.py --port COM3            # 最后真机交互\n"
               "    python tools/pose_demo.py --port COM3 --cycle --loop 3 --hold 5\n"
               "    python tools/pose_demo.py --port COM3 --only 2 --hold 60   # 只调第2个\n"
               "    python tools/pose_demo.py --port COM3 --angles \"180,15,150,70\"")
    ap.add_argument("--config", default=os.path.join(_ROOT, "config", "app.yaml"),
                    help="主配置文件（取串口号与机械尺寸）")
    ap.add_argument("--poses-file", default=os.path.join(_ROOT, "config", "pose_demo.yaml"),
                    help="四姿态表（.yaml 带注释，默认；也支持 .json）。"
                         "交互模式按 w 会存回这里")
    ap.add_argument("--port", default=None, help="串口号，如 COM3，覆盖配置文件")
    ap.add_argument("--baud", type=int, default=None, help="波特率，默认取配置文件（115200）")
    ap.add_argument("--show", "--check", dest="show", action="store_true",
                    help="离线核对：不连硬件也不空跑，只打印姿态表 + 限位体检（改完数字先跑这个）")
    ap.add_argument("--mock", "--dry-run", dest="mock", action="store_true",
                    help="空跑：不碰串口，只打印要发的帧（第一次务必先跑这个）")
    ap.add_argument("--list-ports", dest="list_ports", action="store_true",
                    help="列出本机串口并分类（蓝牙虚拟口不是你的板子）")
    ap.add_argument("--pose", "--only", dest="pose", nargs="+", default=None,
                    metavar="编号",
                    help="只跑指定的几个姿态，可写多个。编号认 A/B/C/D 也认 1/2/3/4，"
                         "如 --only 2 / --pose B C / --pose all")
    ap.add_argument("--angles", action="append", default=None, metavar='"s0,s1,s2[,夹爪]"',
                    help='不改文件、直接下发一组角度，可重复多次（微调时最方便）。'
                         '例：--angles "180,15,150,70"')
    ap.add_argument("--cycle", action="store_true", help="自动循环走一遍所有姿态")
    ap.add_argument("--repeat", "--loop", dest="repeat", type=int, default=1,
                    help="循环轮数，默认 1")
    ap.add_argument("--dwell", "--hold", dest="dwell", type=float, default=2.0,
                    help="每个姿态停留秒数，默认 2（慢慢看就 --hold 20）")
    ap.add_argument("--tt", "--move-ms", dest="tt", type=int, default=1200,
                    help="运动时长 ms，默认 1200（慢=安全；想更稳用 --move-ms 2500）")
    ap.add_argument("--home-each", dest="home_each", action="store_true", default=True,
                    help="每个姿态都「复位→姿态→复位」（默认开，安全）")
    ap.add_argument("--no-home-each", dest="home_each", action="store_false",
                    help="姿态之间直接切换，不回复位位（快一倍，但末端会大范围扫掠）")
    ap.add_argument("--no-readback", dest="readback", action="store_false", default=True,
                    help="不读回实际角度（下位机不支持 $QRY 时用）")
    ap.add_argument("--yes", action="store_true", help="跳过开始前的安全确认")
    ap.add_argument("--selftest", action="store_true",
                    help="用内置假 STM32 自检串口链路，不接板子")
    args = ap.parse_args()

    # ---- 只列串口：连配置都不用读，直接给结果 ----
    if args.list_ports:
        print(ArmController.format_ports())
        return 0

    cfg = load_config(_abs(args.config))
    g = make_geometry(cfg)
    poses = load_poses(_abs(args.poses_file))

    # ---- 自检：不碰真实串口 ----
    if args.selftest:
        return run_selftest(poses, g, _abs(args.poses_file))

    # ---- 离线核对：连配置里的串口都不读，只看角度对不对 ----
    # 改完 YAML 里的数字先跑这个，比接板子试快得多、也安全得多。
    if args.show:
        print_pose_table(poses, g)
        bad_all: List[str] = []
        for key in POSE_ORDER:
            if key not in poses:
                continue
            for b in check_angles(poses[key]["angles"]):
                bad_all.append("姿态 %s（%s）：%s" % (key, poses[key].get("name", ""), b))
        print("")
        if bad_all:
            print("限位体检: 有问题 —— 下面这些角度固件会回 $ERR,03 拒收：")
            for b in bad_all:
                print("   X %s" % b)
            print("\n改 %s 里的数字，或改 pc/core/kinematics.py 的 SERVO_LIMITS"
                  "（须与固件 0stm/app/main.h 同步）" % args.poses_file)
            return 2
        print("限位体检: 全部通过，可以上真机了。")
        print("下一步: python tools/pose_demo.py --mock        （空跑看要发什么帧）")
        return 0

    ser_cfg = cfg.get("serial") or {}
    port = args.port or str(ser_cfg.get("port", "COM3"))
    baud = int(args.baud or ser_cfg.get("baud", 115200))
    timeout = float(ser_cfg.get("timeout", 3.0))

    # ---- 打印计划 ----
    print_pose_table(poses, g)

    # --only 可以写多个，也认 all；编号 A~D 和 1~4 都行
    keys = list(POSE_ORDER)
    if args.pose:
        want = [_norm_key(x) for x in args.pose]
        if "ALL" not in want:
            keys = [k for k in want if k in poses]
            unknown = [x for x in want if x not in poses]
            if unknown:
                print("[警告] 姿态表里没有 %s，已忽略（可用 %s 或 1~4）"
                      % ("/".join(unknown), "/".join(POSE_ORDER)))
            if not keys:
                print("[错误] --only/--pose 指定的编号一个都不认识，退出。")
                return 1

    print("\n将要执行:")
    print("  串口     : %s @ %d  %s" % (port, baud, "(空跑模式)" if args.mock else ""))
    print("  运动时长 : %d ms" % args.tt)
    print("  读回角度 : %s" % ("开" if args.readback else "关"))
    if args.angles:
        print("  模式     : 直接下发 %d 组角度（不改文件）" % len(args.angles))
        for spec in args.angles:
            print("             %s" % spec)
    elif args.cycle:
        print("  模式     : 自动循环 %s × %d 轮，每个停 %.1f s"
              % ("→".join(keys), args.repeat, args.dwell))
    elif args.pose:
        print("  模式     : 只跑 %s（%s），每个停 %.1f s"
              % ("/".join(keys), "→".join(keys), args.dwell))
    else:
        print("  模式     : 交互（进去按 a/b/c/d 试，用 + / - 微调）")
        print("             ★ 每个舵机往哪转，进去第一屏就有速查表")
    if args.cycle or args.pose:
        print("  走法     : %s" % ("每个姿态「复位 → 姿态 → 复位」（--no-home-each 可关）"
                                   if args.home_each else
                                   "姿态之间直接切换（末端会大范围扫掠，注意周围）"))

    # ---- 安全确认 ----
    if not args.mock and not args.yes:
        print("""
--------------------------------------------------------------------------------
 安全检查（真机操作，请逐条确认）
   1. 机械臂已**空载**放在桌子中间，扇形工作范围内没有障碍物、没人手。
   2. 舵机电源已接好（不要用 USB 供电带 4 个舵机），急停/断电开关在手边。
   3. 机械臂**没有卡在某个姿态上**（卡着上电会让舵机一上来就堵转）。
   4. 万一听到舵机持续嗡鸣（顶到限位堵转），**立刻断电**。
--------------------------------------------------------------------------------""")
        try:
            input("确认无误后按回车开始（Ctrl-C 取消）> ")
        except (EOFError, KeyboardInterrupt):
            print("\n已取消。")
            return 1

    # ---- 连接 ----
    arm = ArmController(port=port, baud=baud, timeout=timeout, mock=args.mock,
                        verbose=True)
    if not arm.connect():
        print("\n[错误] 串口打不开。先跑一次：")
        print("       python tools/pose_demo.py --list-ports")
        print("       把结论里那个 USB 转串口设备（不是蓝牙口）填到 --port。")
        return 1

    rc = 0
    try:
        if not args.mock and not arm.wait_ready(timeout=6.0):
            print("[警告] 没等到 $READY。继续发指令也能试，但说明板子可能没跑对固件。")

        demo = PoseDemo(arm, poses, g, _abs(args.poses_file),
                        tt=args.tt, readback=args.readback)
        if args.angles:
            # 直接下发若干组角度：先回零，再一组一组走，中间停 dwell 秒。
            # 用途是「不想动配置文件，只想马上试一组数字」。
            demo.home()
            time.sleep(0.3)
            for spec in args.angles:
                a = _parse_angles(spec)
                if a is None:
                    rc = 2
                    continue
                if not demo.set_all(a):
                    rc = 2
                time.sleep(max(0.0, args.dwell))
        elif args.cycle or args.pose:
            demo.run_sequence(keys, args.repeat, args.dwell,
                              home_first=True, home_each=args.home_each)
        else:
            demo.run_interactive()
    except KeyboardInterrupt:
        print("\n\n!! 收到 Ctrl-C，发急停…")
        try:
            arm.estop()
        except Exception:
            pass
        rc = 130
    finally:
        try:
            arm.close()
        except Exception:
            pass
        print("\n串口已关闭。若机械臂停在奇怪的位置，下次运行会先 $HOM 回零位。")
    return rc


def _abs(p: str) -> str:
    """相对路径按「项目根」解析，这样从 PyCharm 或任意目录跑都一样"""
    return p if os.path.isabs(p) else os.path.join(_ROOT, p)


def _parse_angles(spec: str) -> Optional[List[float]]:
    """
    解析 --angles 的一串文本，例："180,15,150,70" 或 "180 15 150"。

    夹爪可以省略（只给三个时，第四个沿用当前值，由 set_all 补）。
    解析不出来就打印原因并返回 None。
    """
    txt = str(spec).replace("，", ",").replace(" ", ",").replace(";", ",")
    items = [x for x in txt.split(",") if x.strip()]
    try:
        vals = [float(x) for x in items]
    except ValueError:
        print("[错误] --angles 解析失败：%r（要的是数字，例：180,15,150,70）" % spec)
        return None
    if len(vals) < 3:
        print("[错误] --angles 至少要给 s0,s1,s2 三个数（例：180,15,150,70），收到：%r" % spec)
        return None
    return vals[:4]


if __name__ == "__main__":
    sys.exit(main())
