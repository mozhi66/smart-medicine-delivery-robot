#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 串口协议层 (serial_protocol.py)
================================================================================
职责：定义并实现对 STM32 通信的帧格式处理（纯逻辑，不碰硬件，方便单元测试）。

帧格式（NMEA 风格 ASCII，两端必须完全一致）
--------------------------------------------
    $<BODY>*<CS>\\r\\n

    $      帧头，固定
    BODY   指令内容，形如 "SA,90,75,60,40,800"，字段之间用英文逗号分隔
    *      校验分隔符
    CS     BODY 每个字符的 ASCII 码逐字节 XOR，结果用【2 位大写十六进制】表示
    \\r\\n   帧尾，固定

示例
----
    $SA,90,75,60,40,800*5E\\r\\n     四个舵机在 800ms 内同步运动到目标角
    $HOM*1C\\r\\n                     回零位
    $EST*07\\r\\n                     急停

为什么用 ASCII 而不是二进制？
    1) 串口助手直接看得懂，现场调硬件不用写解码器
    2) 答辩时能当着手敲指令演示
    3) 毕设阶段调试效率 >> 通信效率（本项目一秒才几条指令，115200 绰绰有余）

为什么必须有校验？
    舵机电源线、USB 线捆在一起时串口会有偶发误码。没有校验就可能收到
    "$SA,900,75,..."，舵机直接冲到限位憋到堵转烧掉。校验是最后一道保险。
================================================================================
"""

from __future__ import annotations

from typing import List, Optional, Tuple


def xor_checksum(body: str) -> int:
    """
    计算帧体校验值：BODY 每个字符的 ASCII 码逐字节 XOR。

    参数
    ----
    body : 帧体字符串（不含 $ 和 * 和 \\r\\n）

    返回
    ----
    0~255 的整数校验值
    """
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return cs & 0xFF


def build_frame(body: str) -> bytes:
    """
    把帧体打包成完整的一帧（含帧头、校验、帧尾）。

    参数
    ----
    body : 例如 "SA,90,75,60,40,800"

    返回
    ----
    bytes，可直接 ser.write()
    """
    return ("$%s*%02X\r\n" % (body, xor_checksum(body))).encode("ascii")


def verify_line(line: str) -> Optional[str]:
    """
    校验并解析一行文本。

    参数
    ----
    line : 不含 \\r\\n 的一行，例如 "$DONE,SA*2F"

    返回
    ----
    校验通过返回帧体（如 "DONE,SA"）；校验失败/格式不对返回 None
    """
    if not line:
        return None
    line = line.strip()
    if not line.startswith("$"):
        return None
    if "*" not in line:
        return None
    body, _, cs_hex = line[1:].rpartition("*")
    if len(cs_hex) != 2:
        return None
    try:
        cs = int(cs_hex, 16)
    except ValueError:
        return None
    if cs != xor_checksum(body):
        return None
    return body


def split_cmd(body: str) -> Tuple[str, List[str]]:
    """
    把帧体拆成 (指令, 参数列表)。

    参数
    ----
    body : 例如 "SA,90,75,60,40,800"

    返回
    ----
    ("SA", ["90", "75", "60", "40", "800"])
    """
    parts = body.split(",")
    return parts[0].strip().upper(), [p.strip() for p in parts[1:]]


class StreamParser:
    """
    串口字节流解析器：把"流"切成"帧"。

    串口是字节流，一次 read() 可能读到半帧、一帧半、或三帧，
    所以必须有一个能累积缓冲区的解析器，不能直接按行假设。

    用法
    ----
        p = StreamParser()
        for body in p.feed(ser.read(n)):
            cmd, args = split_cmd(body)
    """

    def __init__(self, max_buffer: int = 4096):
        """
        参数
        ----
        max_buffer : 缓冲区上限，防止对端发疯导致内存暴涨
        """
        self._buf = bytearray()
        self.max_buffer = int(max_buffer)

    def feed(self, data: bytes) -> List[str]:
        """
        喂入新收到的字节，返回本次凑齐的所有"合法帧体"。

        参数
        ----
        data : 新收到的字节（可以是 b"" ）

        返回
        ----
        合法帧体字符串列表（已通过校验）；非法/不完整的不返回
        """
        out: List[str] = []
        if not data:
            return out

        self._buf.extend(data)

        # 缓冲区过长保护：只保留最后一个帧尾之后的内容
        if len(self._buf) > self.max_buffer:
            idx = self._buf.rfind(b"\r\n")
            if idx >= 0:
                del self._buf[:idx + 2]
            else:
                self._buf.clear()

        while True:
            idx = self._buf.find(b"\r\n")
            if idx < 0:
                break
            raw = bytes(self._buf[:idx])
            del self._buf[:idx + 2]
            try:
                line = raw.decode("ascii", errors="ignore")
            except Exception:
                continue
            body = verify_line(line)
            if body is not None:
                out.append(body)
        return out

    def reset(self) -> None:
        """清空缓冲区（换端口/重连时调用）"""
        self._buf.clear()


# ============================================================================
# 错误码文字说明（与 stm32/Core/Inc/protocol.h 保持一致）
# ============================================================================
ERROR_CODES = {
    "01": "校验失败（STM32 收到的帧校验不对，应重发）",
    "02": "未知指令",
    "03": "角度越限（超过该舵机配置的 min/max）",
    "04": "执行超时",
    "05": "舵机异常/堵转",
}


def describe_error(code: str) -> str:
    """把错误码翻译成人话（用于日志和 TTS 提示）"""
    return ERROR_CODES.get(str(code).zfill(2), "未知错误 %s" % code)
