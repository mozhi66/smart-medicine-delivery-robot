#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Keil 固件静态一致性检查：协议表 / 头文件函数 / 引脚资源 / 中断入口。"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parent
ok = True

def fail(msg):
    global ok
    ok = False
    print("[FAIL]", msg)

def okp(msg):
    print("[PASS]", msg)

# 1) 协议枚举 vs s_cmd_names
ph = (ROOT / "app" / "protocol.h").read_text(encoding="utf-8", errors="ignore")
pc = (ROOT / "app" / "protocol.c").read_text(encoding="utf-8", errors="ignore")
enum = re.findall(r"PROTO_CMD_(\w+)\s*=\s*(\d+)", ph)
m = re.search(r"s_cmd_names\[\]\s*=\s*\{([^}]+)\}", pc, re.S)
ns = re.findall(r"\"([A-Z]+)\"", m.group(1)) if m else []
pairs = [(n, int(i)) for n, i in enum if n != "NONE"]
if len(pairs) != len(ns):
    fail(f"cmd count enum={len(pairs)} names={len(ns)}")
for name, idx in pairs:
    if idx >= len(ns) or ns[idx] != name:
        fail(f"cmd table mismatch {name}@{idx} -> {ns[idx] if idx < len(ns) else 'OOB'}")
if ok:
    okp(f"protocol cmd table 1:1 ({len(ns)} cmds: {','.join(ns)})")

# 融合版必须有的底盘指令
for need in ("MD", "MV", "MS", "MC", "MQ"):
    if need not in ns:
        fail(f"missing chassis cmd {need}")
for bad in ("MVS", "MVC", "MVQ"):
    if bad in ns:
        fail(f"legacy cmd {bad} should be removed")

# 2) chassis.h 声明 vs chassis.c 定义（兼容 CHASSIS_* 与 Chassis_*）
ch = (ROOT / "app" / "chassis.h").read_text(encoding="utf-8", errors="ignore")
cc = (ROOT / "app" / "chassis.c").read_text(encoding="utf-8", errors="ignore")
decls = re.findall(r"\b((?:CHASSIS_|Chassis_)\w+)\s*\(", ch)
for d in sorted(set(decls)):
    if f"{d}(" not in cc and f"{d} (" not in cc:
        fail(f"chassis missing def {d}")
    else:
        okp(f"chassis {d}")

# 3) protocol.h 发送函数
for fn in [
    "PROTO_Init", "PROTO_FeedByte", "PROTO_SendPayload", "PROTO_SendReady",
    "PROTO_SendAck", "PROTO_SendDone", "PROTO_SendPose", "PROTO_SendErr",
    "PROTO_SendHb", "PROTO_SendChassis", "PROTO_ParseCmd", "PROTO_GetArg",
]:
    if f"{fn}(" not in ph:
        fail(f"protocol.h missing {fn}")
    elif f"{fn}(" not in pc and fn != "PROTO_FeedByte":
        if f"{fn}(" not in pc:
            fail(f"protocol.c missing {fn}")
okp("protocol send/parse symbols present")

# 4) main.c handlers all defined
mc = (ROOT / "user" / "main.c").read_text(encoding="utf-8", errors="ignore")
for h in [
    "HandleCmd_SA", "HandleCmd_SS", "HandleCmd_GC", "HandleCmd_RUN",
    "HandleCmd_HOM", "HandleCmd_QRY", "HandleCmd_EST", "HandleCmd_CFG",
    "HandleCmd_MD", "HandleCmd_MV", "HandleCmd_MS", "HandleCmd_MC", "HandleCmd_MQ",
    "CHASSIS_Init", "CHASSIS_Update", "CHASSIS_Move", "CHASSIS_Stop",
]:
    if h not in mc:
        fail(f"main.c missing {h}")
okp("main.c handlers/init present")

# 5) 中断入口
it = (ROOT / "user" / "stm32f10x_it.c").read_text(encoding="utf-8", errors="ignore")
if "TIM3_IRQHandler" not in it:
    fail("missing TIM3_IRQHandler")
else:
    okp("TIM3_IRQHandler in it.c")
if "USART3_IRQHandler" not in cc and "USART3_IRQHandler" not in mc:
    fail("missing USART3_IRQHandler")
else:
    okp("USART3_IRQHandler present")
if "USART2_IRQHandler" in cc:
    fail("stale USART2_IRQHandler")

# 6) 引脚/定时器冻结
mh = (ROOT / "app" / "main.h").read_text(encoding="utf-8", errors="ignore")
checks = [
    ("TIM2", mh, "servo uses TIM2"),
    ("TIM3", mh, "ctrl uses TIM3"),
    ("GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3", mh, "servo pins PA0-3"),
    ("TIM4", cc, "motor PWM TIM4"),
    ("GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9", cc, "motor PWM PB6-9"),
    ("USART3", cc, "HC-05 on USART3"),
    ("115200", mh, "USART1 115200"),
    ("9600", mh + ch, "HC-05 9600"),
    ("CHASSIS_DEADMAN_AUTO_MS", mh + ch + cc, "deadman AUTO"),
    ("CHASSIS_MODE_AUTO", ch + cc + mc, "work mode AUTO"),
]
for needle, text, label in checks:
    if needle not in text:
        fail(f"pin/timer check: {label} ({needle!r})")
    else:
        okp(f"pin/timer: {label}")

# 7) 方向脚
for pin in ["GPIO_Pin_4", "GPIO_Pin_5", "GPIO_Pin_6", "GPIO_Pin_7",
            "GPIO_Pin_0", "GPIO_Pin_1", "GPIO_Pin_12", "GPIO_Pin_13"]:
    if pin not in cc:
        fail(f"chassis.c missing dir pin {pin}")
okp("chassis dir pins present")

# 8) uvprojx 收录
uv = (ROOT / "template.uvprojx").read_text(encoding="utf-8", errors="ignore")
for f in ["chassis.c", "chassis.h", "servo.c", "protocol.c", "main.c"]:
    if f not in uv:
        fail(f"uvprojx missing {f}")
okp("uvprojx includes chassis + app sources")

# 9) 麦轮公式
for s in ["y + x + rx", "y - x - rx", "y - x + rx", "y + x - rx"]:
    if s not in cc:
        fail(f"kinematics missing {s}")
okp("mecanum inverse present")

# 10) 错误码 06
if "PROTO_ERR_LOCKED" not in ph and "ERR,06" not in mc and "06" not in ph:
    fail("missing locked/error 06")
else:
    okp("error code 06 defined")

# 11) 不回 DONE,MV（审查 A3）
if "SendDone(\"MV\")" in mc or 'SendDone("MV")' in mc:
    fail("MV must not send DONE (PC must not wait_done)")
else:
    okp("MV does not send DONE")

print("=" * 40)
print("firmware_static_check:", "ALL OK" if ok else "HAS FAILURES")
sys.exit(0 if ok else 1)
