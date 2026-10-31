#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 isp_flash.py —— 通过 STM32 串口 ROM bootloader 读取 Flash（默认纯只读）
================================================================================
用途
----
板上固件协议（$-帧）没有任何"读 Flash / 读限位"的指令（$CFG 只写不读），
所以要看到板内实际存的舵机限位/home 配置，只能走 STM32 出厂 ROM bootloader
（USART1，正好就是接 CH340 的那个串口）把配置页 dump 出来离线解码。

★ 前置物理操作（必须人工完成）★
    1) 把 BOOT0 跳线接到 1（3.3V）
    2) 按一下 RESET 键（板子进入 bootloader，此时主程序不跑、PWM 停止输出，
       ★ 舵机会失去保持力矩变"软"，请先用手扶住机械臂防止垂落砸坏 ★）
    3) 运行本工具 dump
    4) 完事后把 BOOT0 跳回 0，再按 RESET（主程序恢复，会执行上电归位）

用法
----
    # 只读：dump 配置页（页62, 0x0800F800, 1KB）并自动解码舵机限位/home
    python tools/isp_flash.py --port COM12 --dump

    # 只读：dump 原始字节存文件，不 解码
    python tools/isp_flash.py --port COM12 --dump --raw flash_cfg.bin

    # 写操作（危险，二次确认才执行）：擦掉配置页 62
    #   → 下次上电固件发现配置无效，自动回落 main.h 编译默认值
    python tools/isp_flash.py --port COM12 --erase-page 62 --confirm-erase

技术要点
--------
    - STM32 ROM bootloader 固定 **8E1**（8 数据位/偶校验/1 停止位），不是 8N1！
    - 同步：上电后第一字节发 0x7F，收到 0x79=ACK
    - Read Memory(0x11)：地址大端 + 各字节 XOR 校验，单次最多读 256 字节
    - 页擦除：优先 legacy(0x43)，不支持则用 extended(0x44)
    - ServoCfgStore_t 布局见 0stm/app/servo.h（76 字节，无填充）
    - 校验和 = sum(bytes[8:76]) & 0xFFFF（与 servo.c Servo_CalcChecksum 一致）
================================================================================
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

import serial                                   # pyserial

# ---- 与 0stm/app/main.h 保持一致 ----
CFG_ADDR   = 0x08000000 + 62 * 1024             # 配置页：页 62
ACT_ADDR   = 0x08000000 + 63 * 1024             # 动作表页：页 63
PAGE_SIZE  = 1024
CFG_MAGIC  = 0x3163424D                          # "MBc1"
CFG_VER    = 3                                   # main.h: CFG_STORE_VERSION

# ---- main.h 编译默认值（用于对照） ----
DEF_LIMITS = [(0.0, 180.0), (15.0, 165.0), (0.0, 150.0), (45.0, 90.0)]
DEF_HOME   = [180.0, 120.0, 120.0, 90.0]
DEF_GRIP   = (90.0, 45.0)

ACK, NACK = 0x79, 0x1F

SERVO_NAMES = ["s0(底座)", "s1(大臂)", "s2(小臂)", "s3(夹爪)"]


class Bootloader:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, bytesize=8, parity=serial.PARITY_EVEN,
                                 stopbits=1, timeout=1.0)
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except Exception:
            pass

    def _read1(self, timeout_s: float = 1.0) -> int:
        end = time.time() + timeout_s
        while time.time() < end:
            b = self.ser.read(1)
            if b:
                return b[0]
        return -1

    def _cmd(self, code: int) -> bool:
        """发命令字节 + 反码，等 ACK"""
        self.ser.write(bytes([code, (~code) & 0xFF]))
        return self._read1() == ACK

    def sync(self, attempts: int = 8) -> bool:
        """发 0x7F 同步字节，等 ACK。板子必须在 bootloader 里"""
        for i in range(attempts):
            self.ser.reset_input_buffer()
            self.ser.write(bytes([0x7F]))
            r = self._read1(0.8)
            if r == ACK:
                return True
            print("  同步第 %d 次失败（收到 0x%02X）" % (i + 1, r & 0xFF if r >= 0 else 0))
            time.sleep(0.3)
        return False

    def get_info(self) -> None:
        """GET(0x00)：报告 bootloader 版本与支持的命令表"""
        if not self._cmd(0x00):
            print("GET 失败")
            return
        n = self._read1()                      # 后续数据字节数
        data = self.ser.read(n if n > 0 else 0)
        end = self._read1()
        if len(data) >= 3:
            print("[bl] bootloader 版本 %d.%d，支持命令: %s"
                  % (data[0] >> 4, data[0] & 0xF,
                     " ".join("%02X" % b for b in data[1:])))

    def read_mem(self, addr: int, size: int) -> bytes:
        """Read Memory(0x11)，内部按 256 字节分块"""
        out = bytearray()
        done = 0
        while done < size:
            n = min(256, size - done)
            if not self._cmd(0x11):
                raise RuntimeError("Read Memory 命令被 NACK")
            a = bytes([(addr >> 24) & 0xFF, (addr >> 16) & 0xFF,
                       (addr >> 8) & 0xFF, addr & 0xFF])
            cs = a[0] ^ a[1] ^ a[2] ^ a[3]
            self.ser.write(a + bytes([cs]))
            if self._read1() != ACK:
                raise RuntimeError("Read Memory 地址被拒 0x%08X" % addr)
            self.ser.write(bytes([(n - 1) & 0xFF, (~(n - 1)) & 0xFF]))
            chunk = self.ser.read(n)
            if len(chunk) != n:
                raise RuntimeError("Read Memory 数据不足（%d/%d）" % (len(chunk), n))
            out += chunk
            done += n
            addr += n
        return bytes(out)

    def erase_page(self, page: int) -> None:
        """擦一页：先试 legacy(0x43)，NACK 再试 extended(0x44)"""
        # ---- legacy: [页数-1][页号...][XOR] ----
        if self._cmd(0x43):
            payload = bytes([0x00, page & 0xFF])            # 1 页
            cs = 0
            for b in payload:
                cs ^= b
            self.ser.write(payload + bytes([cs]))
            if self._read1() == ACK:
                print("[erase] legacy(0x43) 擦除页 %d 成功" % page)
                return
        # ---- extended: [页数-1(16bit)][页号(16bit)...][XOR] ----
        print("[erase] legacy 不支持，改用 extended(0x44)")
        if not self._cmd(0x44):
            raise RuntimeError("Extended Erase 命令被 NACK（页可能写保护）")
        payload = bytes([0x00, 0x00, 0x00, page & 0xFF])    # 1 页, 16 位页号
        cs = 0
        for b in payload:
            cs ^= b
        self.ser.write(payload + bytes([cs]))
        if self._read1() != ACK:
            raise RuntimeError("Extended Erase 失败（页可能写保护，需先 Write Unprotect）")
        print("[erase] extended(0x44) 擦除页 %d 成功" % page)


# ============================================================================
# 解码 ServoCfgStore_t（servo.h：76 字节，无填充）
# ============================================================================
def decode_cfg(page: bytes) -> None:
    print("=" * 70)
    print("配置页解码（ServoCfgStore_t @ 0x%08X）" % CFG_ADDR)
    magic, version, checksum = struct.unpack_from("<IHH", page, 0)
    print("magic    = 0x%08X %s" % (magic, "(应为 0x3163424D \"MBc1\")" if magic != CFG_MAGIC else "OK"))
    print("version  = %d %s" % (version, "(应为 %d)" % CFG_VER if version != CFG_VER else "OK"))
    calc = sum(page[8:76]) & 0xFFFF
    print("checksum = 0x%04X %s（重算 0x%04X）"
          % (checksum, "OK" if calc == checksum else "不一致！", calc))

    if magic != CFG_MAGIC or version != CFG_VER or calc != checksum:
        print("\n>>> 三重校验【不通过】→ 固件上电会忽略此页、使用 main.h 编译默认值。")
        print(">>> 若如此，ERR,03,LIM 另有原因（需再查），把这行结果发我。")
        return

    lim_min = struct.unpack_from("<4f", page, 8)
    lim_max = struct.unpack_from("<4f", page, 24)
    home    = struct.unpack_from("<4f", page, 40)
    g_open, g_close = struct.unpack_from("<2f", page, 56)
    trim    = struct.unpack_from("<4h", page, 64)
    inv     = struct.unpack_from("<4B", page, 72)

    print("\n%-10s %-14s %-14s %-14s %-8s %-6s %s"
          % ("通道", "limit_min", "limit_max", "home", "grip", "trim", "inv"))
    for i in range(4):
        print("%-10s %-14.1f %-14.1f %-14.1f %-8s %-6d %d"
              % (SERVO_NAMES[i], lim_min[i], lim_max[i], home[i],
                 ("open" if g_open == home[i] else "-") if i == 3 else "-",
                 trim[i], inv[i]))
    print("\ngrip_open=%.1f  grip_close=%.1f" % (g_open, g_close))

    # ---- 与编译默认值 / PC 端认知对照，直接给出判读 ----
    print("-" * 70)
    mismatch = []
    for i in range(4):
        if (lim_min[i], lim_max[i]) != DEF_LIMITS[i]:
            mismatch.append("%s 限位 [%.1f, %.1f] ≠ 默认 [%.1f, %.1f]"
                            % (SERVO_NAMES[i], lim_min[i], lim_max[i], *DEF_LIMITS[i]))
        if home[i] != DEF_HOME[i]:
            mismatch.append("%s home %.1f ≠ 默认 %.1f"
                            % (SERVO_NAMES[i], home[i], DEF_HOME[i]))
    if (g_open, g_close) != DEF_GRIP:
        mismatch.append("grip open/close (%.1f, %.1f) ≠ 默认 (%.1f, %.1f)"
                        % (g_open, g_close, *DEF_GRIP))

    if mismatch:
        print(">>> 与 main.h 编译默认值不一致的项（这就是 ERR,03,LIM 的来源）：")
        for m in mismatch:
            print("    - " + m)
        print(">>> 处理二选一：")
        print("    a) 若这些值是你有意调的 → 告诉我，我把 PC 端点位生成改到这套限位内")
        print("    b) 若是旧调参残留 → 用 --erase-page 62 擦掉此页，")
        print("       下次上电自动回落默认值，与 PC 完全一致（无需重编译固件）")
    else:
        print(">>> 配置与编译默认值完全一致 → ERR,03,LIM 另有原因，把本页发我复核")


def main() -> int:
    ap = argparse.ArgumentParser(description="STM32 串口 bootloader 读/擦配置页")
    ap.add_argument("--port", default="COM12")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dump", action="store_true", help="读取配置页（1KB）并解码")
    ap.add_argument("--raw", default=None, help="dump 原始字节另存到该文件")
    ap.add_argument("--erase-page", type=int, default=None, metavar="N",
                    help="擦除 Flash 第 N 页（写操作！需配合 --confirm-erase）")
    ap.add_argument("--confirm-erase", action="store_true",
                    help="擦除操作的二次确认开关")
    args = ap.parse_args()

    if not args.dump and args.erase_page is None:
        ap.print_help()
        return 2
    if args.erase_page is not None and not args.confirm_erase:
        print("擦除是写操作且不可恢复：请再加 --confirm-erase 明确执行")
        return 2

    print("[isp] 打开 %s @ %d 8E1（bootloader 用偶校验，与主程序 8N1 不同）"
          % (args.port, args.baud))
    try:
        bl = Bootloader(args.port, args.baud)
    except Exception as e:
        print("[错误] 打不开串口: %s" % e)
        return 1

    try:
        print("[isp] 同步 bootloader（0x7F）... 若一直失败：确认 BOOT0=1 且已按 RESET")
        if not bl.sync():
            print("[错误] 同步失败。检查：BOOT0 跳线=1？按过 RESET？")
            return 1
        print("[isp] 已进入 bootloader")
        bl.get_info()

        if args.dump:
            print("[isp] 读取配置页 0x%08X（1KB）..." % CFG_ADDR)
            page = bl.read_mem(CFG_ADDR, PAGE_SIZE)
            if args.raw:
                with open(args.raw, "wb") as f:
                    f.write(page)
                print("[isp] 原始字节已存 %s" % args.raw)
            decode_cfg(page)

        if args.erase_page is not None:
            bl.erase_page(args.erase_page)
            print("[isp] 完成。请把 BOOT0 跳回 0 并按 RESET —— 固件将使用编译默认配置")

        print("[isp] 请把 BOOT0 跳回 0，按 RESET 恢复主程序"
              "（会执行一次上电归位，扶好机械臂）")
        return 0
    except Exception as e:
        print("[错误] %s" % e)
        return 1
    finally:
        bl.ser.close()


if __name__ == "__main__":
    sys.exit(main())
