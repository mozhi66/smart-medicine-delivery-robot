#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 serial_listen.py —— 串口旁听/只读探测诊断工具
================================================================================
用途
----
排查"PC 连得上串口但收不到 STM32 回帧"一类问题：

  被动旁听（默认，零风险，一个字节都不发）:
      打开串口 -> 原样打印收到的每一个字节 -> 用项目同款 StreamParser
      验证哪些能构成合法帧。
      判读：
        看到 $HB,..      -> 板子活着、TX 通路 OK（问题在握手时序/固件版本）
        看到 $READY,..   -> 板子刚重启过（READY 是上电一次性帧）
        什么都看不到     -> TX 通路断了 / 板子没运行 / 波特率不符
        看到乱码         -> 波特率不对或接线接触不良

  只读探测（--probe，不发任何运动指令）:
      追加发送 $QRY,POSE 与 $MQ（纯查询，舵机/底盘都不会动），
      验证 PC->STM32 方向是否通畅。禁止在本工具里放 HOM/SA/SS 等运动指令。

用法
----
    python tools/serial_listen.py --port COM12 --seconds 10
    python tools/serial_listen.py --port COM12 --seconds 10 --probe
================================================================================
"""

from __future__ import annotations

import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import serial                                   # noqa: E402
from pc.comm.serial_protocol import (StreamParser, build_frame,  # noqa: E402
                                     verify_line)


def main() -> int:
    ap = argparse.ArgumentParser(description="串口旁听/只读探测诊断")
    ap.add_argument("--port", default="COM12", help="串口号，默认 COM12")
    ap.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    ap.add_argument("--seconds", type=float, default=10.0, help="旁听秒数，默认 10")
    ap.add_argument("--probe", action="store_true",
                    help="旁听结束后追加只读探测 $QRY,POSE / $MQ（不产生任何运动）")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2, dsrdtr=False)
    except Exception as e:
        print("[错误] 打不开 %s: %s" % (args.port, e))
        return 1
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass

    print("[listen] %s @ %d 已打开，被动旁听 %.1f 秒"
          "（DTR/RTS 已禁用，与主程序 connect() 行为一致；本阶段不发送任何数据）"
          % (args.port, args.baud, args.seconds))

    t0 = time.time()
    parser = StreamParser()
    frames = []
    raw_total = 0

    try:
        while time.time() - t0 < args.seconds:
            n = ser.in_waiting
            if n:
                data = ser.read(n)
                raw_total += len(data)
                print("[%6.2fs] RAW  %r" % (time.time() - t0, data))
                for body in parser.feed(data):
                    print("[%6.2fs] FRAME $%s" % (time.time() - t0, body))
                    frames.append(body)
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[listen] 手动中断")

    # ---------- 判读 ----------
    print("-" * 70)
    print("旁听 %.1fs：共收到 %d 字节，%d 个合法帧" % (args.seconds, raw_total, len(frames)))
    has_hb = any(f.upper().startswith("HB") for f in frames)
    has_ready = any(f.upper().startswith("READY") for f in frames)
    if has_ready:
        print("结论 A：收到 $READY —— 板子在旁听期间重启过（READY 是上电一次性帧）")
    if has_hb:
        print("结论 B：收到 $HB 心跳 —— 板子活着、USART1 TX 通路 OK")
        if not has_ready:
            print("       但没有 $READY —— 与『READY 一次性、板子上电时早已发完』吻合")
    if not frames:
        print("结论 C：一个合法帧都没收到 —— 可能：")
        print("  1) STM32 PA9(TX) -> CH340 RX 接线断了/虚接")
        print("  2) 板子没在跑这个固件（没烧录/变砖）")
        print("  3) 波特率不符（板子不是 115200）")
        print("  4) 板子没供电")
        print("  辅助判断：看板载 LED(PC13) 是否每 0.5s 闪烁 —— 闪 = 主循环在跑")

    # ---------- 只读探测 ----------
    if args.probe:
        print("-" * 70)
        print("[probe] 发送只读查询（不产生任何运动）...")
        for body, want in (("QRY,POSE", "POSE"), ("MQ", "CHS")):
            ser.write(build_frame(body))
            print("[probe] >> $%s" % body)
            t1 = time.time()
            hit = None
            while time.time() - t1 < 2.0:
                n = ser.in_waiting
                if n:
                    for b2 in parser.feed(ser.read(n)):
                        print("[probe] << $%s" % b2)
                        if b2.upper().startswith(want) and hit is None:
                            hit = b2
                else:
                    time.sleep(0.05)
            if hit:
                print("[probe] %s 双向通路 OK（PC->STM32->PC 都通）" % body)
            else:
                print("[probe] %s 未收到 %s 回应 —— PC->STM32 方向可能不通"
                      "（TX/RX 接反、RX 线断）" % (body, want))

    ser.close()
    print("[listen] 串口已关闭")
    return 0


if __name__ == "__main__":
    sys.exit(main())
