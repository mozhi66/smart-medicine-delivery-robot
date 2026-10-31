# -*- coding: utf-8 -*-
"""
USB-TTL 测底盘横移（$MV 协议，不是 HC-05 的 G/H 单字符）

用法：
  python tools/test_chassis_lateral.py --mock
  python tools/test_chassis_lateral.py --port COM3            # 左 0.6s 右 0.6s
  python tools/test_chassis_lateral.py --port COM3 --dir left --ms 800
"""
from __future__ import annotations

import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, os.path.join(_ROOT, "pc")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from comm.chassis_controller import ChassisController, MockChassisController  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="USB-TTL 底盘横移测试（$MV）")
    ap.add_argument("--port", default=None, help="串口号，如 COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mock", action="store_true", help="空跑，只打印帧")
    ap.add_argument("--dir", choices=("left", "right", "both"), default="both",
                    help="left=向左 G 语义 / right=向右 H 语义 / both=左右各一次")
    ap.add_argument("--ms", type=int, default=600, help="每次横移时长 ms")
    ap.add_argument("--speed", type=float, default=1.0, help="横移速度 0~1")
    args = ap.parse_args()

    if args.mock:
        ch = MockChassisController("MOCK", verbose=True)
    else:
        if not args.port:
            print("请指定 --port COM3，或用 --mock 空跑")
            return 2
        ch = ChassisController(port=args.port, baud=args.baud, verbose=True)

    if not ch.connect():
        print("串口打开失败")
        return 2

    print("\n★ 说明：")
    print("  USB-TTL 走 $ 协议，横移用 $MV,y,x,rx,tt")
    print("  向左 = x 负 → $MV,0,-1,0,%d   （HC-05 蓝牙上是 G）" % args.ms)
    print("  向右 = x 正 → $MV,0,+1,0,%d   （HC-05 蓝牙上是 H）" % args.ms)
    print("  G/H 单字符只在 HC-05 蓝牙口有效，USB-TTL 不认。\n")

    s = max(0.05, min(1.0, float(args.speed)))
    ok = True

    def do_left():
        print(">>> 向左横移 %.2f / %d ms" % (-s, args.ms))
        r = ch.move_lateral(-s, args.ms)
        print("   ", "OK" if r else "FAIL", ch.last_error or "")
        time.sleep(0.2)
        ch.stop()
        return r

    def do_right():
        print(">>> 向右横移 %.2f / %d ms" % (s, args.ms))
        r = ch.move_lateral(s, args.ms)
        print("   ", "OK" if r else "FAIL", ch.last_error or "")
        time.sleep(0.2)
        ch.stop()
        return r

    if args.dir in ("left", "both"):
        ok &= do_left()
    if args.dir in ("right", "both"):
        ok &= do_right()

    try:
        ch.close()
    except Exception:
        pass

    print("\n结果:", "全部成功" if ok else "有失败")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
