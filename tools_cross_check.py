
import re, pathlib
root = pathlib.Path(__file__).resolve().parent
ph = (root/"0stm/app/protocol.h").read_text(encoding="utf-8", errors="replace")
pc = (root/"0stm/app/protocol.c").read_text(encoding="utf-8", errors="replace")
mc = (root/"0stm/user/main.c").read_text(encoding="utf-8", errors="replace")
cc = (root/"0stm/app/chassis.c").read_text(encoding="utf-8", errors="replace")
ch = (root/"0stm/app/chassis.h").read_text(encoding="utf-8", errors="replace")
py = (root/"智能药箱系统/pc/comm/chassis_controller.py").read_text(encoding="utf-8", errors="replace")
sm = (root/"智能药箱系统/pc/core/state_machine.py").read_text(encoding="utf-8", errors="replace")
errs=[]
names = re.findall(r'"(\w+)"', re.search(r"s_cmd_names\[\]\s*=\s*\{([^}]+)\}", pc, re.S).group(1))
enum_names = [e[0] for e in re.findall(r"PROTO_CMD_(\w+)\s*=\s*(\d+)", ph) if e[0]!="NONE"]
if enum_names != names: errs.append(f"enum/name {enum_names} vs {names}")
for n in ["MD","MV","MS","MC","MQ"]:
    if n not in names: errs.append(f"missing {n}")
if set(names) & {"MVS","MVC","MVQ"}: errs.append("legacy cmds left")
for h in ["HandleCmd_MD","HandleCmd_MV","HandleCmd_MS","HandleCmd_MC","HandleCmd_MQ","SendChassisState"]:
    if h not in mc: errs.append(f"main missing {h}")
if "PROTO_SendMvState" in mc+ph+pc: errs.append("SendMvState left")
if "PROTO_SendChassis" not in ph+pc: errs.append("SendChassis missing")
if "CHASSIS_SetMode" not in ch+cc: errs.append("SetMode missing")
if "DEADMAN" not in (ch+cc+(root/"0stm/app/main.h").read_text(encoding="utf-8", errors="replace")): errs.append("deadman missing")
if re.search(r"def move\(.*wait_done=True", py, re.S): errs.append("move wait_done")
for bad in ['"MVS"', '"MVC"']:
    if bad in py: errs.append(f"PC still {bad}")
if '"MS"' not in py: errs.append("PC no MS")
if "MC,%d" not in py: errs.append("PC no MC")
if "set_mode" not in sm: errs.append("SM no set_mode sync")
if "move_forward" not in sm and "lift_dz" not in sm: errs.append("no pull-out path")
if "PROTO_ERR_LOCKED" not in ph: errs.append("no ERR 06")
print("CROSSCHECK:", "OK" if not errs else errs)
raise SystemExit(1 if errs else 0)
