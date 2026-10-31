#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
================================================================================
 舵机扫角 / 防堵转烧机工具 —— servo_sweep.py
================================================================================
为什么需要这个
--------------
第 4 路夹爪是小舵机（SG90 一类）。指令角度一旦顶到【机械限位】，舵机还在
持续通电试图转过去 → 堵转 → 电流拉满 → 几分钟就烧。

本工具干两件事：
  1. **慢慢扫角**：一步一步走，人能听/能看，顶到限位立刻停；
  2. **随时急停**：任意时刻发 $EST，冻结当前位置，绝不“卡住了还继续转”。

摸出安全角之后，把数字改进 `0stm/app/main.h` 的
    SERVO3_LIMIT_MIN_DEG / SERVO3_LIMIT_MAX_DEG
再烧录，固件软限位就会在源头拦住越界指令。

默认目标：第 4 路（夹爪，内部编号 3）。也可扫 1~3 路。

安全约定（与固件一致）
--------------------
  1 号 底座俯仰 / 2 号 大臂 / 3 号 小臂 / 4 号 夹爪
  夹爪：0° = 夹紧，180° = 全开（★ 角度大 = 张开）

★ 听到「嗡——」连续鸣叫 = 堵转，立刻按 Enter（或敲 e / stop）急停。
★ 每一步都默认走 800ms，宁可慢，不要快。

用法
----
  # 1) 不接硬件，先看会发什么帧
  python tools/servo_sweep.py --mock

  # 2) 真机：扫夹爪（第 4 路），一次 2°，到头会停
  python tools/servo_sweep.py --port COM3

  # 3) 只走一小步一小步（找限位最稳）
  python tools/servo_sweep.py --port COM3 --mode step

  # 4) 扫别的舵机 / 改步长与范围
  python tools/servo_sweep.py --port COM3 --servo 2 --from 20 --to 140 --step 2

  # 5) 自动慢扫（无需回车），按 Enter 随时打断
  python tools/servo_sweep.py --port COM3 --mode auto

交互命令（--mode step / 交互默认）
--------------------------------
  +        向角度增大方向走一步（夹爪 = 再张开一点）
  -        向角度减小方向走一步（夹爪 = 再夹紧一点）
  + 5      一步走 5°
  = 90     直接走到 90°
  g        查询当前四路角度（$QRY）
  auto     开始自动扫角（等回车，可随时按 Enter 打断）
  e / s    立刻急停 $EST（堵转时用这个）
  h        回 HOME
  o / f    夹爪张开 / 夹紧（只对第 4 路有意义）
  q        退出（先急停）
================================================================================
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from typing import List, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, os.path.join(_ROOT, "pc")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from comm.arm_controller import ArmController, MockArmController  # noqa: E402


# 固件默认软限位（0stm/app/main.h），仅作参考提示
DEFAULT_LIMITS = {
    1: (0.0, 180.0),
    2: (15.0, 165.0),
    3: (0.0, 150.0),
    4: (0.0, 180.0),
}
SERVO_NAMES = {1: "底座俯仰", 2: "大臂", 3: "小臂", 4: "夹爪"}


class SweepSession:
    """一次扫角会话：只动一个舵机，全程可急停。"""

    def __init__(self, arm: ArmController, servo_no: int,
                 angle: float, step: float, tt: int,
                 lo: float, hi: float, verbose: bool = True):
        self.arm = arm
        self.no = int(servo_no)            # 1~4，用户可见编号
        self.sid = self.no - 1             # 0~3，协议编号
        self.angle = float(angle)
        self.step = float(step)
        self.tt = int(tt)
        self.lo = float(lo)
        self.hi = float(hi)
        self.verbose = verbose
        self.estopped = False
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    def _say(self, msg: str) -> None:
        if self.verbose:
            print(msg)

    def estop(self) -> None:
        """急停：无条件立刻发，不等待任何回复。"""
        with self._lock:
            self.estopped = True
        try:
            self.arm.estop()
            self._say("  [ESTOP] 已发送 $EST —— 舵机冻结在当前位置")
        except Exception as e:
            self._say("  [ESTOP] 发送失败: %s" % e)

    def query(self) -> Optional[List[float]]:
        try:
            return self.arm.query_pose()
        except Exception as e:
            self._say("  查询失败: %s" % e)
            return None

    def goto(self, target: float, tt: Optional[int] = None) -> bool:
        """走绝对角。超出本会话范围的直接拒绝，不发给下位机。"""
        if self.estopped:
            self._say("  已急停，请先重新上电或重启本工具再动")
            return False
        target = float(target)
        if target < self.lo - 1e-6 or target > self.hi + 1e-6:
            self._say("  [拒绝] %.1f° 超出本会话范围 [%.1f, %.1f]（防堵转）"
                      % (target, self.lo, self.hi))
            return False
        t = int(tt if tt is not None else self.tt)
        self._say("  → %s(%d) 到 %.1f°，时长 %d ms"
                  % (SERVO_NAMES.get(self.no, "?"), self.no, target, t))
        ok = self.arm.move_servo(self.sid, target, tt=t)
        if ok:
            self.angle = target
            self._say("  ← 到位 %.1f°" % self.angle)
        else:
            self._say("  ← 失败（超时/堵转/限位）。若在嗡鸣请立刻急停 e")
        return ok

    def nudge(self, delta: float) -> bool:
        return self.goto(self.angle + float(delta))

    def grip(self, open_it: bool) -> bool:
        if self.no != 4:
            self._say("  o/f 只对 4 号夹爪有意义；当前是 %d 号" % self.no)
            return False
        if self.estopped:
            self._say("  已急停")
            return False
        target = 180.0 if open_it else 0.0
        self._say("  夹爪 → %s（目标 %.0f°）" % ("张开" if open_it else "夹紧", target))
        ok = self.arm.grip(open_it, tt=max(self.tt, 400))
        if ok:
            self.angle = target
        return ok

    def home(self) -> bool:
        if self.estopped:
            self._say("  已急停")
            return False
        self._say("  回 HOME…")
        ok = self.arm.home()
        return ok

    def auto_sweep(self, wait_key=None) -> None:
        """
        从当前角向 hi 慢扫，到头再扫回 lo。
        wait_key() 若提供，则每步之间调用它；返回 True 表示用户要求打断。
        """
        self._say("\n>>> 开始自动扫角：%s(%d)  %.1f° → %.1f° → %.1f°，步长 %.1f°"
                  % (SERVO_NAMES.get(self.no, "?"), self.no,
                     self.angle, self.hi, self.lo, self.step))
        self._say("    听到持续嗡鸣 = 顶到机械限位，立刻按 Enter 急停\n")

        targets = []
        a = self.angle
        if abs(a - self.hi) > 1e-6:
            step = abs(self.step) if self.hi > a else -abs(self.step)
            while (step > 0 and a < self.hi - 1e-6) or (step < 0 and a > self.hi + 1e-6):
                a = min(self.hi, a + step) if step > 0 else max(self.hi, a + step)
                targets.append(round(a, 1))
        a = targets[-1] if targets else self.angle
        if abs(a - self.lo) > 1e-6:
            step = abs(self.step) if self.lo > a else -abs(self.step)
            while (step > 0 and a < self.lo - 1e-6) or (step < 0 and a > self.lo + 1e-6):
                a = min(self.lo, a + step) if step > 0 else max(self.lo, a + step)
                targets.append(round(a, 1))

        for i, tgt in enumerate(targets, 1):
            if self.estopped:
                self._say("  已急停，中止扫角")
                return
            if wait_key is not None and wait_key():
                self._say("  收到打断请求")
                self.estop()
                return
            self._say("[%d/%d]" % (i, len(targets)))
            if not self.goto(tgt):
                self._say("  运动失败，自动急停（防堵转）")
                self.estop()
                return
            # 步与步之间留出观察/急停窗口
            if wait_key is not None:
                # 有键盘打断时，短等即可
                t0 = time.time()
                while time.time() - t0 < 0.25:
                    if wait_key():
                        self.estop()
                        return
            else:
                time.sleep(0.35)

        self._say("\n>>> 扫角结束，停在 %.1f°" % self.angle)


# ---------------------------------------------------------------------------
def _wait_enter_breaker(stop_event: threading.Event) -> threading.Thread:
    """后台读一行：用户按 Enter 就置位 stop_event（auto 模式可随时打断）。"""
    def _loop():
        try:
            input()
        except Exception:
            pass
        stop_event.set()
    th = threading.Thread(target=_loop, daemon=True)
    th.start()
    return th


def _print_banner(sess: SweepSession) -> None:
    print("""
================================================================================
 舵机扫角 —— %s（第 %d 路 / 协议号 %d）
 当前角度 %.1f°   本会话允许范围 [%.1f, %.1f]   步长 %.1f°   单步时长 %d ms
--------------------------------------------------------------------------------
  + / -     一步（+ 5 可指定步长）
  = 90      走到 90°
  g         查当前角度
  auto      自动来回扫（按 Enter 随时打断）
  e  / s    ★ 急停 ★（听到嗡鸣立刻用）
  o / f     夹爪张开 / 夹紧
  h         回 HOME
  q         退出
================================================================================""" % (
        SERVO_NAMES.get(sess.no, "?"), sess.no, sess.sid,
        sess.angle, sess.lo, sess.hi, sess.step, sess.tt))


def run_interactive(sess: SweepSession) -> None:
    _print_banner(sess)
    while True:
        try:
            line = input("sweep> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            sess.estop()
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0]

        if cmd in ("q", "quit", "exit"):
            sess.estop()
            print("已退出（已急停）")
            break
        if cmd in ("e", "s", "stop", "estop"):
            sess.estop()
            continue
        if cmd == "g":
            p = sess.query()
            if p:
                print("  当前四路: %s" % ", ".join("%.1f" % v for v in p))
            continue
        if cmd == "h":
            sess.home()
            continue
        if cmd == "o":
            sess.grip(True)
            continue
        if cmd == "f":
            sess.grip(False)
            continue
        if cmd in ("+", "++"):
            d = float(parts[1]) if len(parts) > 1 else sess.step
            sess.nudge(abs(d))
            continue
        if cmd in ("-", "--"):
            d = float(parts[1]) if len(parts) > 1 else sess.step
            sess.nudge(-abs(d))
            continue
        if cmd in ("=", "goto") and len(parts) >= 2:
            try:
                sess.goto(float(parts[1]))
            except ValueError:
                print("  用法: = 90")
            continue
        if cmd == "step" and len(parts) >= 2:
            try:
                sess.step = abs(float(parts[1]))
                print("  步长改为 %.1f°" % sess.step)
            except ValueError:
                print("  用法: step 2")
            continue
        if cmd == "auto":
            stop = threading.Event()
            _wait_enter_breaker(stop)
            sess.auto_sweep(wait_key=stop.is_set)
            # auto 结束后继续交互（若已急停可 q 退出）
            continue

        print("  未知命令。常用: +  -  = 90  g  auto  e  o  f  h  q")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="舵机慢速扫角 / 防堵转烧机（默认第 4 路夹爪）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="例: python tools/servo_sweep.py --mock\n"
               "    python tools/servo_sweep.py --port COM3\n"
               "    python tools/servo_sweep.py --port COM3 --mode auto\n"
               "    python tools/servo_sweep.py --port COM3 --servo 2 --step 1")
    ap.add_argument("--port", default=None, help="串口号，如 COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mock", action="store_true", help="空跑，只打印帧")
    ap.add_argument("--servo", type=int, default=4, choices=(1, 2, 3, 4),
                    help="要扫的舵机号 1~4（默认 4=夹爪）")
    ap.add_argument("--mode", choices=("step", "auto"), default="step",
                    help="step=回车驱动（默认，最稳）；auto=自动扫")
    ap.add_argument("--from", dest="a0", type=float, default=None,
                    help="起始角（默认取当前角）")
    ap.add_argument("--to", dest="a1", type=float, default=None,
                    help="终止角（默认该舵机软限位上界）")
    ap.add_argument("--step", type=float, default=2.0,
                    help="步进角度，默认 2°（找限位建议 1°）")
    ap.add_argument("--tt", type=int, default=800,
                    help="单步运动时长 ms，默认 800（越大越慢越安全）")
    ap.add_argument("--min", dest="lo", type=float, default=None,
                    help="本会话下界（默认取固件软限位）")
    ap.add_argument("--max", dest="hi", type=float, default=None,
                    help="本会话上界（默认取固件软限位）")
    args = ap.parse_args()

    if args.mock:
        arm = MockArmController("MOCK", verbose=True)
        arm.connect()
    else:
        port = args.port
        if not port:
            print("请指定串口，例如: --port COM3   （或先用 --mock 空跑）")
            print(ArmController.format_ports())
            return 2
        arm = ArmController(port, baud=args.baud, mock=False, verbose=True)
        if not arm.connect():
            print("串口打开失败")
            return 2
        if not arm.wait_ready(timeout=6.0):
            print("下位机未就绪（可继续试，但建议先确认供电/接线）")

    dlo, dhi = DEFAULT_LIMITS[args.servo]
    lo = args.lo if args.lo is not None else dlo
    hi = args.hi if args.hi is not None else dhi
    if lo > hi:
        lo, hi = hi, lo

    # 起始角：命令行 > 查询当前 > 限位中点
    angle = args.a0
    if angle is None:
        try:
            pose = arm.query_pose()
            if pose and len(pose) >= 4:
                angle = float(pose[args.servo - 1])
                print("当前 %d 号角 = %.1f°" % (args.servo, angle))
        except Exception:
            angle = None
    if angle is None:
        angle = max(lo, min(hi, (lo + hi) / 2.0))
        print("拿不到当前角，从 %.1f° 起步" % angle)

    if args.a1 is not None:
        hi = max(lo, min(max(hi, args.a1), args.a1 if args.a1 >= lo else hi))
        # 允许 --to 超出默认软限位，但收进本会话
        hi = args.a1 if args.a1 >= lo else hi

    sess = SweepSession(arm, args.servo, angle=angle,
                        step=args.step, tt=args.tt, lo=lo, hi=hi)

    print("\n★ 提醒：%s 顶到机械限位还继续通电就会烧。听到嗡鸣立刻急停。"
          % SERVO_NAMES.get(args.servo, "舵机"))
    print("★ 摸出安全角后，请改 0stm/app/main.h 的 SERVO%d_LIMIT_MIN/MAX_DEG 并重新烧录。\n"
          % (args.servo - 1))

    try:
        if args.mode == "auto":
            stop = threading.Event()
            _wait_enter_breaker(stop)
            # auto 也允许用 --to 只扫一段
            if args.a1 is not None:
                # 扫到 --to 为止
                targets_hi = args.a1
                sess.hi = targets_hi if targets_hi >= sess.lo else sess.hi
            sess.auto_sweep(wait_key=stop.is_set)
        else:
            run_interactive(sess)
    except KeyboardInterrupt:
        print("\nCtrl-C → 急停")
        sess.estop()
    finally:
        try:
            arm.close()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
