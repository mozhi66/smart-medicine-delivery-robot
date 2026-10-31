#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 麦轮底盘控制器（融合版）
================================================================================
各取优势：
  · 文件夹 0 —— 与机械臂共串口（继承 ArmController）、定时短移、
                闭环对列/拔瓶所需 API、MC 限速、PC 优先于手机
  · 文件夹 1 —— 指令命名消歧（MS=停 / MC=限速 / MQ=查 / MD=模式）、
                双工作模式、HC-05 单字符手动路径、失联自停语义

协议（与 0stm/app/protocol.h 一致）：
    $MD,AUTO|MAN|Q      工作模式
    $MV,y,x,rx[,tt]     运动 y/x/rx∈[-1,1]；tt=ms（省略/0=连续，靠 MS 或 deadman 停）
    $MS                 立即停车
    $MC,pct             速度上限 0~100（0=锁死）
    $MQ                 查询 -> $CHS,mode,moving,y,x,rx,pct

★ 底盘运动帧只回 $ACK + $CHS，**不回 $DONE** —— 本类绝不 wait_done。
★ 安全互斥：底盘动时机械臂锁定（固件 SERVO_IsBusy + PC 双重检查）。
================================================================================
"""
from __future__ import annotations

import time
from typing import List, Optional, Tuple

from .arm_controller import ArmController


class ChassisController(ArmController):
    """麦轮底盘控制器（与机械臂共用 USART1 一条串口 / 一套帧协议）。"""

    # 逆解公式（标准 FTC/ROS 麦轮惯例，★ 不可改 ★，与固件 chassis.c 一致）
    MIX = ((1.0, 1.0, 1.0),      # FL
           (1.0, -1.0, -1.0),    # FR
           (1.0, -1.0, 1.0),     # RL
           (1.0, 1.0, -1.0))     # RR

    # HC-05 单字符（手动路径，与固件 HC05_HandleChar 一致）
    BT_MAP = {
        "forward": "F", "back": "B", "left": "L", "right": "R",
        "slide_left": "G", "slide_right": "H",
        "stop": "S", "faster": "+", "slower": "-",
    }

    def __init__(self, port: str = "COM3", baud: int = 115200,
                 timeout: float = 3.0, mock: bool = False,
                 verbose: bool = True,
                 bt_port: Optional[str] = None, bt_baud: int = 9600):
        super().__init__(port, baud=baud, timeout=timeout,
                         mock=mock, verbose=verbose)
        self.mode = "manual_phone"          # 'auto' | 'manual_phone'
        self.bt_port = bt_port              # 可选：独立 HC-05 串口（手动路径）
        self.bt_baud = int(bt_baud)
        self.bt_ser = None
        self.last_error = ""

    # ------------------------------------------------------------------
    # 模式（融合自文件夹 1）
    # ------------------------------------------------------------------
    def set_mode(self, mode: str) -> bool:
        if mode not in ("auto", "manual_phone"):
            raise ValueError("mode 必须是 auto 或 manual_phone")
        self.mode = mode
        # 同步固件模式（AUTO / MAN）
        tag = "AUTO" if mode == "auto" else "MAN"
        ok = self._send_and_wait("MD,%s" % tag, "MD", 2.0, wait_done=False)
        self.stop()
        return ok

    def is_auto(self) -> bool:
        return self.mode == "auto"

    # ------------------------------------------------------------------
    # 运动学（纯计算，方便单测）
    # ------------------------------------------------------------------
    @classmethod
    def mix(cls, y: float, x: float, rx: float) -> List[float]:
        out: List[float] = []
        for ky, kx, kr in cls.MIX:
            v = ky * float(y) + kx * float(x) + kr * float(rx)
            if v > 1.0:
                v = 1.0
            elif v < -1.0:
                v = -1.0
            out.append(v)
        return out

    # ------------------------------------------------------------------
    # USB $ 协议业务指令（auto 主路径）
    # ------------------------------------------------------------------
    def move(self, y: float = 0.0, x: float = 0.0, rx: float = 0.0,
             ms: int = 0, timeout: Optional[float] = None) -> bool:
        """
        $MV,y,x,rx[,tt] —— 按麦轮逆解运动。

        ms=0   连续运动，必须再调 stop()（或依赖 deadman）
        ms>0   定时运动，到点固件自停

        ★ 不等 DONE：固件只回 $ACK + $CHS。
        """
        body = "MV,%.3f,%.3f,%.3f" % (y, x, rx)
        if ms and int(ms) > 0:
            body += ",%d" % int(ms)
        if timeout is None:
            # 定时：多留一点 ACK 时间；连续：短超时
            timeout = (int(ms) / 1000.0 + 1.5) if ms and int(ms) > 0 else 2.0
        ok = self._send_and_wait(body, "MV", timeout, wait_done=False)
        if not ok:
            self.last_error = "MV timeout/ack fail"
        return ok

    def move_lateral(self, x: float, ms: int,
                     timeout: Optional[float] = None) -> bool:
        """纯横移：x>0 右移 / x<0 左移，持续 ms 毫秒。"""
        return self.move(y=0.0, x=float(x), rx=0.0, ms=int(ms), timeout=timeout)

    def move_forward(self, y: float, ms: int,
                     timeout: Optional[float] = None) -> bool:
        """纯前后：y>0 前进 / y<0 后退，持续 ms 毫秒。拔瓶用 y<0。"""
        return self.move(y=float(y), x=0.0, rx=0.0, ms=int(ms), timeout=timeout)

    def set_speed(self, pct: int) -> bool:
        """$MC,pct —— 速度上限（★ 不是停车）。pct=0 锁死底盘。"""
        p = max(0, min(100, int(pct)))
        return self._send_and_wait("MC,%d" % p, "MC", 2.0, wait_done=False)

    def stop(self) -> bool:
        """$MS —— 立即停车（★ 不等 DONE，急停越快越好）。"""
        try:
            self._send("MS")
            return True
        except Exception as e:
            self.last_error = str(e)
            return False

    def query(self, timeout: float = 2.0) -> Optional[Tuple[str, int, float, float, float, int]]:
        """$MQ —— 返回 (mode, moving, y, x, rx, pct)；失败返回 None。"""
        if self.mock:
            return ("M", 0, 0.0, 0.0, 0.0, 60)
        with self._lock:
            self._drain()
            self._send("MQ")
            end = time.time() + timeout
            skipped = []
            while time.time() < end:
                try:
                    body = self._frames.get(timeout=0.15)
                except Exception:
                    continue
                if body.upper().startswith("CHS"):
                    self._requeue(skipped)
                    parts = body.split(",")
                    # CHS,mode,moving,y,x,rx,pct
                    if len(parts) >= 7:
                        try:
                            return (parts[1], int(parts[2]),
                                    float(parts[3]), float(parts[4]), float(parts[5]),
                                    int(float(parts[6])))
                        except Exception:
                            continue
                elif body.upper().startswith("ERR"):
                    self._requeue(skipped)
                    self.last_error = body
                    return None
                else:
                    skipped.append(body)
            self._requeue(skipped)
        self.last_error = "MQ timeout"
        return None

    def estop(self) -> bool:
        """
        全局急停：臂 + 底盘。
        ★ 补一条 $MS（停车），不要发裸 MC（会被当参数错误）。
        """
        try:
            self._send("EST")
        except Exception:
            pass
        return self.stop()

    # ------------------------------------------------------------------
    # HC-05 单字符手动路径（融合自文件夹 1；manual_phone 用）
    # ------------------------------------------------------------------
    def connect_bt(self) -> bool:
        if self.bt_port is None or self.mock:
            return False
        try:
            import serial  # noqa
            self.bt_ser = serial.Serial(self.bt_port, self.bt_baud, timeout=0.2)
            try:
                self.bt_ser.dtr = False
                self.bt_ser.rts = False
            except Exception:
                pass
            return True
        except Exception as e:
            self.last_error = "bt: %s" % e
            self.bt_ser = None
            return False

    def bt_cmd(self, name: str) -> bool:
        """发送 HC-05 单字符（forward/back/slide_left/.../stop）。manual 才允许动。"""
        ch = self.BT_MAP.get(name)
        if not ch:
            return False
        if self.mode != "auto" and name != "stop":
            # manual_phone：PC 不生成移动指令，只允许 stop
            return False
        if self.bt_ser is None:
            return False
        try:
            self.bt_ser.write(ch.encode("ascii"))
            return True
        except Exception as e:
            self.last_error = str(e)
            return False

    def close(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
        if self.bt_ser is not None:
            try:
                self.bt_ser.close()
            except Exception:
                pass
            self.bt_ser = None
        try:
            super().close()
        except Exception:
            pass


class _MockBTSerial:
    """假 HC-05 串口：记录写出的单字符（供 mock 模式验证手动路径）"""

    def __init__(self, sink: List[str]):
        self._sink = sink
        self.is_open = True

    def write(self, data):
        if isinstance(data, (bytes, bytearray)):
            self._sink.append(bytes(data).decode("ascii", "ignore"))
        else:
            self._sink.append(str(data))

    def close(self):
        self.is_open = False


class MockChassisController(ChassisController):
    """不碰串口的底盘控制器（与 MockArmController 同风格，供 --mock / 测试用）。"""

    def __init__(self, *a, **kw):
        kw["mock"] = True
        super().__init__(*a, **kw)
        self.history: List[str] = []
        self.bt_written: List[str] = []       # HC-05 单字符路径的发出记录

    def _send(self, body: str) -> None:
        self.history.append(body)
        print("[CHASSIS-MOCK] >> $%s" % body)

    def connect_bt(self) -> bool:
        """mock 版：给一个假 BT 串口，让 bt_cmd() 手动路径可被验证。
        规则 4（manual_phone 禁止 PC 发移动指令）仍由父类 bt_cmd 把关。"""
        self.bt_ser = _MockBTSerial(self.bt_written)
        return True

    def close(self) -> None:
        self.bt_ser = None
        try:
            super().close()
        except Exception:
            pass


# 自检：python -m pc.comm.chassis_controller
if __name__ == "__main__":
    ch = ChassisController(mock=True)
    print("mix(1,0,0) =", ch.mix(1, 0, 0))
    print("mix(0,1,0) =", ch.mix(0, 1, 0))
    print("mix(0,0,1) =", ch.mix(0, 0, 1))
    assert ch.mix(1, 0, 0) == [1.0, 1.0, 1.0, 1.0]
    assert ch.mix(0, 1, 0) == [1.0, -1.0, -1.0, 1.0]
    assert ch.mix(0, 0, 1) == [1.0, -1.0, 1.0, -1.0]
    assert ch.set_mode("auto") in (True, False)
    print("chassis_controller fusion self-check OK")
