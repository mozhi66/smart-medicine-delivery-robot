#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 机械臂控制器 (arm_controller.py)
================================================================================
职责：把"我要让机械臂做某个动作"翻译成串口帧、发出去、并【等到 STM32 确认做完】。

三条铁律（违反任何一条都会出事）
--------------------------------
1. 发了指令必须等 ACK：否则你不知道 STM32 有没有收到。
2. 运动指令必须等 DONE：否则机械臂还在半路，你就开始拍照/夹取了。
3. 一切都有超时：超时后重发，再失败就进异常处理，绝不无限等待。

Mock 模式
---------
mock=True 时不碰串口，只把指令打印出来并模拟延时返回成功。
用途：没有硬件时跑通全流程、单元测试、答辩前预演。
本项目 tests/test_pipeline.py 的端到端仿真就靠它。
================================================================================
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Dict, List, Optional, Sequence

from .serial_protocol import StreamParser, build_frame, describe_error, split_cmd

try:                                   # pyserial 可能没装，允许在无硬件环境下 import
    import serial
    import serial.tools.list_ports as list_ports
    _HAS_SERIAL = True
except Exception:                      # pragma: no cover
    serial = None
    list_ports = None
    _HAS_SERIAL = False


class ArmController:
    """
    机械臂串口控制器。

    用法
    ----
        arm = ArmController("COM3", baud=115200)
        if arm.connect() and arm.wait_ready():
            arm.home()
            arm.move_all(90, 132, 114, 40, tt=800)   # 四个舵机同步运动
            arm.grip(True)                            # 夹爪张开
        arm.close()
    """

    def __init__(self, port: str = "COM3", baud: int = 115200,
                 timeout: float = 3.0, mock: bool = False,
                 verbose: bool = True):
        """
        参数
        ----
        port    : 串口号，Windows 形如 'COM3'，Linux 形如 '/dev/ttyUSB0'
        baud    : 波特率，必须与 STM32 一致，默认 115200
        timeout : 等待 ACK/DONE 的默认超时（秒）
        mock    : True 时不真实发串口，只打印（无硬件也能跑）
        verbose : 是否打印收发日志
        """
        self.port = port
        self.baud = int(baud)
        self.timeout = float(timeout)
        self.mock = bool(mock)
        self.verbose = bool(verbose)

        self.ser = None
        self._rx_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._frames: "queue.Queue[str]" = queue.Queue()
        self._parser = StreamParser()
        self._lock = threading.RLock()

        self.last_error: Optional[str] = None
        self.last_pose: Optional[List[float]] = None
        self._heartbeat = 0.0

    # ==================================================================
    # 连接管理
    # ==================================================================
    @staticmethod
    def list_ports() -> List[str]:
        """列出当前电脑上所有可用串口，方便用户确认 COM 号"""
        if not _HAS_SERIAL or list_ports is None:
            return []
        try:
            return [p.device for p in list_ports.comports()]
        except Exception:
            return []

    @staticmethod
    def format_ports() -> str:
        """
        带分类说明的串口清单（给 --list-ports 用）。
        按 VID/ 描述把口分成 [USB 转串口] 和 [蓝牙] 两类 ——
        蓝牙虚拟口（耳机/音箱）经常排在 COM3 前面，新手很容易选错。
        """
        if not _HAS_SERIAL or list_ports is None:
            return "未安装 pyserial，无法枚举串口（pip install pyserial）"
        try:
            ports = list(list_ports.comports())
        except Exception:
            ports = []
        if not ports:
            return "没有发现任何串口。检查：\n" \
                   "  1) USB 线插好没（要插数据线，不能是纯充电线）\n" \
                   "  2) CH340 驱动装了没（设备管理器里应出现 USB-SERIAL CH340）"
        lines = []
        usb_found = []
        for i, p in enumerate(ports, 1):
            desc = (p.description or "").strip()
            low = desc.lower()
            is_bt = ("bluetooth" in low) or ("蓝牙" in desc)
            tag = "蓝牙" if is_bt else "USB "
            note = ""
            if is_bt:
                note = "蓝牙虚拟口（耳机/音箱等），不是 STM32，忽略"
            elif "ch340" in low or "ch341" in low:
                note = "USB 转串口：CH340/CH341 (沁恒)"
            elif "cp210" in low:
                note = "USB 转串口：CP210x (芯科)"
            elif "ftdi" in low or "ft232" in low:
                note = "USB 转串口：FTDI"
            else:
                note = "串口设备（若非 STM32 请忽略）"
            lines.append("  %d) [%s] %-6s %s\n        %s (%s)"
                         % (i, tag, p.device, note, desc, p.device))
            if not is_bt:
                usb_found.append(p.device)
        out = "\n".join(lines)
        if usb_found:
            out += ("\n\n结论：发现 %d 个 USB 转串口设备 → %s\n"
                    "      大概率就是你的 STM32，把 config/app.yaml 里的 serial.port 改成它，\n"
                    "      或运行时加参数 --port %s"
                    % (len(usb_found), "、".join(usb_found), usb_found[0]))
        else:
            out += "\n\n结论：没有 USB 转串口设备 —— STM32 没插好或驱动没装"
        return out

    def connect(self) -> bool:
        """
        打开串口并启动接收线程。成功返回 True。

        注意：这里显式关掉 DTR/RTS —— CH340 等芯片在 DTR 变化时会把
        STM32 复位，表现为"Python 一打开串口，板子就重启"。
        """
        if self.mock:
            self._log("[MOCK] 已连接（虚拟串口）")
            return True
        if not _HAS_SERIAL:
            self._log("[错误] 未安装 pyserial，请执行: pip install pyserial")
            return False
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2,
                                     dsrdtr=False)
            try:
                self.ser.dtr = False
                self.ser.rts = False
            except Exception:
                pass
        except Exception as e:
            self._log("[错误] 打开串口 %s 失败: %s" % (self.port, e))
            self.ser = None
            return False

        self._stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self._log("串口已打开: %s @ %d" % (self.port, self.baud))
        return True

    def close(self) -> None:
        """关闭串口和接收线程"""
        self._stop.set()
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    # ==================================================================
    # 接收线程
    # ==================================================================
    def _rx_loop(self) -> None:
        """后台线程：持续读串口，把合法帧体放进队列"""
        while not self._stop.is_set():
            if self.ser is None or not self.ser.is_open:
                time.sleep(0.05)
                continue
            try:
                n = self.ser.in_waiting or 1
                data = self.ser.read(n)
            except Exception:
                time.sleep(0.05)
                continue
            if not data:
                continue
            for body in self._parser.feed(data):
                self._on_frame(body)

    def _on_frame(self, body: str) -> None:
        """处理一个收到的帧体：更新状态 + 入队供 wait_* 消费"""
        cmd, args = split_cmd(body)
        if cmd == "POSE" and len(args) >= 4:
            try:
                self.last_pose = [float(a) for a in args[:4]]
            except ValueError:
                pass
        elif cmd == "ERR":
            self.last_error = describe_error(args[0]) if args else "未知错误"
            self._log("[STM32 报错] %s" % self.last_error)
        elif cmd == "HB":
            self._heartbeat = time.time()
        if self.verbose and cmd not in ("HB",):
            self._log("<< %s" % body)
        self._frames.put(body)

    # ==================================================================
    # 发送与等待
    # ==================================================================
    def _log(self, msg: str) -> None:
        if self.verbose:
            print("[ARM] %s" % msg)

    def _send(self, body: str) -> None:
        """发出一帧（不等待回应）。写入失败自动重连一次（USB 瞬断容错）。"""
        if self.verbose:
            print("[ARM] >> $%s" % body)
        if self.mock:
            return
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("串口未打开，请先 connect()")
        try:
            self.ser.write(build_frame(body))
            return
        except Exception as e:
            # ★ 底盘电机启动的电流涌动/EMI 可能让 CH340 从 USB 总线瞬断
            #   （WriteFile 报"拒绝访问"），重开一次端口通常能恢复。
            self._log("warn", "串口写入失败（%s），尝试重连一次…" % e)
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        time.sleep(0.5)
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2,
                                     dsrdtr=False)
            self.ser.dtr = False
            self.ser.rts = False
        except Exception as e2:
            self._log("error", "串口重连失败：%s —— 请检查 USB 线/供电/是否被占用"
                      % e2)
            raise
        self.ser.write(build_frame(body))

    def _wait_for(self, prefix: str, timeout: Optional[float] = None) -> bool:
        """
        等待某一类帧（按前缀匹配），超时返回 False。

        参数
        ----
        prefix  : 帧体前缀，如 "DONE,SA" / "ACK" / "READY"
        timeout : 超时秒数，None 用默认 self.timeout

        返回
        ----
        是否等到（True/False）
        """
        end = time.time() + (self.timeout if timeout is None else timeout)
        skipped: List[str] = []

        while True:
            remain = end - time.time()
            if remain <= 0:
                break
            try:
                body = self._frames.get(timeout=min(remain, 0.2))
            except queue.Empty:
                continue
            if body.upper().startswith(prefix.upper()):
                self._requeue(skipped)
                return True
            # 收到 ERR 直接判失败，不用干等超时
            if body.upper().startswith("ERR"):
                self._requeue(skipped)
                return False
            skipped.append(body)

        self._requeue(skipped)
        return False

    def _requeue(self, items: List[str]) -> None:
        """把"路上捡到但不是我要的"帧塞回队列，避免丢帧"""
        for b in reversed(items):
            self._frames.put(b)

    def _send_and_wait(self, body: str, cmd_tag: str,
                       timeout: Optional[float] = None,
                       wait_done: bool = True) -> bool:
        """
        发一条指令并等待 ACK（+DONE）。

        参数
        ----
        body     : 帧体
        cmd_tag  : 指令名（用于匹配 ACK,DONE 的后缀），如 'SA'
        timeout  : 单次等待超时
        wait_done: 是否还要等 DONE（纯查询类指令不需要）

        返回
        ----
        成功 True，失败 False（失败原因见 self.last_error）
        """
        with self._lock:
            if self.mock:
                # 走 _send：真·ArmController 只打印不写串口；
                # MockArmController 会记录 history，供测试断言流程。
                self._send(body)
                time.sleep(0.02)
                return True
            self._drain()
            self._send(body)
            if not self._wait_for("ACK,%s" % cmd_tag, timeout):
                self.last_error = "等待 ACK 超时，STM32 可能未收到或掉线"
                self._log(self.last_error)
                return False
            if wait_done and not self._wait_for("DONE,%s" % cmd_tag, timeout):
                self.last_error = "等待 DONE 超时，机械臂可能未到位"
                self._log(self.last_error)
                return False
            return True

    def _drain(self) -> None:
        """清空待处理帧队列，避免上一条指令的残留 ACK 干扰本次判断"""
        try:
            while True:
                self._frames.get_nowait()
        except queue.Empty:
            pass

    # ==================================================================
    # 业务指令
    # ==================================================================
    def wait_ready(self, timeout: float = 6.0) -> bool:
        """
        等待 STM32 就绪（$READY 或等价的活性证明）。

        为什么不能只被动等 $READY
        ------------------------
        $READY 是【上电一次性帧】（0stm/user/main.c：归位结束后发一次，
        g_ready_sent 锁存后再不发）。板子比 PC 先上电是常态（板子上电跑了
        十分钟，你才启动 PC），这帧早就发完了 —— 被动等永远等不到，表现为
        "串口能打开、6 秒后报未收到 READY"，但板子其实是好的（1Hz $HB 照发）。
        2026-10-22 实测确诊：旁听只见 $HB 不见 $READY，心跳计数证明板子
        已连续运行 ~11 分钟。

        因此分两步：
          1) 先被动等 $READY 至多 2s（覆盖"刚按过复位键/板子刚好重启"，
             此时 READY 会正常到达，走严格路径）；
          2) 等不到就发【只读】探测 $QRY,POSE（不产生任何运动）：
             收到 $ACK,QRY / $POSE 说明主循环在跑、串口双向通畅 -> 判定
             就绪；连探测都没回应才判失败。

        参数
        ----
        timeout : 总超时（秒），默认 6.0
        """
        if self.mock:
            self._log("[MOCK] STM32 已就绪")
            return True

        # ---- 第一步：被动等 $READY（给"板子刚重启"的情况留窗口）----
        if self._wait_for("READY", min(float(timeout), 2.0)):
            return True

        # ---- 第二步：只读探测 $QRY,POSE ----
        self._drain()
        try:
            self._send("QRY,POSE")
        except Exception as e:
            self._log("发送探测帧失败: %s" % e)
            return False
        remain = float(timeout) - 2.0
        if remain <= 0:
            remain = 2.0
        end = time.time() + remain
        while time.time() < end:
            try:
                body = self._frames.get(timeout=min(end - time.time(), 0.2))
            except queue.Empty:
                continue
            u = body.upper()
            if u.startswith("ACK,QRY") or u.startswith("POSE"):
                self._log("未收到 $READY（上电一次性帧，板子应早于 PC 上电），"
                          "但 $QRY,POSE 探测有响应 -> 判定就绪")
                self._frames.put(body)      # 放回队列，不吞后续逻辑要用的帧
                return True
            if u.startswith("ERR"):
                self._log("探测收到 $ERR: %s" % body)
                return False
            # 其它帧（如 $HB）忽略，继续等
        self.last_error = "等待 READY/探测响应超时，STM32 可能未收到或掉线"
        self._log(self.last_error)
        return False

    def move_all(self, a1: float, a2: float, a3: float, a4: float,
                 tt: int = 800, timeout: Optional[float] = None) -> bool:
        """
        四个舵机【同步】运动到目标角度。

        参数
        ----
        a1..a4 : 四个舵机目标角（度，0~180）
        tt     : 运动总时长 ms，越大越慢越稳。抓取建议 800~1200
        timeout: 超时秒数；None 时自动按 tt/1000 + 3 秒计算

        返回
        ----
        是否成功到位
        """
        if timeout is None:
            timeout = tt / 1000.0 + 3.0
        body = "SA,%.1f,%.1f,%.1f,%.1f,%d" % (a1, a2, a3, a4, int(tt))
        return self._send_and_wait(body, "SA", timeout, wait_done=True)

    def move_servo(self, servo_id: int, angle: float, tt: int = 500,
                   timeout: Optional[float] = None) -> bool:
        """单个舵机运动（调试用），servo_id 0~3"""
        if timeout is None:
            timeout = tt / 1000.0 + 2.0
        body = "SS,%d,%.1f,%d" % (int(servo_id), float(angle), int(tt))
        return self._send_and_wait(body, "SS", timeout, wait_done=True)

    def grip(self, open_it: bool, tt: int = 400) -> bool:
        """
        夹爪开合。

        参数
        ----
        open_it : True 张开，False 夹紧
        tt      : 动作时长 ms
        """
        body = "GC,%s" % ("open" if open_it else "close")
        return self._send_and_wait(body, "GC", tt / 1000.0 + 2.0, wait_done=True)

    def home(self, timeout: float = 6.0) -> bool:
        """回到初始安全位（上电/异常恢复/每次取药前都建议先回位）"""
        return self._send_and_wait("HOM", "HOM", timeout, wait_done=True)

    def estop(self) -> bool:
        """
        急停：立即停止所有舵机运动。
        注意：急停不等 DONE，越快越好，所以单独走 _send。
        """
        try:
            self._send("EST")
            return True
        except Exception as e:
            self._log("急停发送失败: %s" % e)
            return False

    def run_slot(self, slot_id: int, timeout: float = 8.0) -> bool:
        """
        执行 STM32 Flash 中预存的动作序列（点位表烧在下位机里时用）。
        本项目默认走 move_all（点位在 PC 上更好改），这个接口作为备用。
        """
        return self._send_and_wait("RUN,%d" % int(slot_id), "RUN", timeout, wait_done=True)

    def query_pose(self, timeout: float = 2.0) -> Optional[List[float]]:
        """查询当前四个舵机角度，失败返回 None"""
        if self.mock:
            return [90.0, 132.0, 114.0, 40.0]
        with self._lock:
            self._drain()
            self.last_pose = None
            self._send("QRY,POSE")
            end = time.time() + timeout
            while time.time() < end:
                try:
                    body = self._frames.get(timeout=0.2)
                except queue.Empty:
                    continue
                if body.upper().startswith("POSE"):
                    return self.last_pose
                self._frames.put(body)
            return None

    def is_alive(self, max_age: float = 3.0) -> bool:
        """根据心跳判断 STM32 是否还活着（掉线检测）"""
        if self.mock:
            return True
        if self._heartbeat <= 0:
            return False
        return (time.time() - self._heartbeat) < max_age

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *a):
        self.close()
        return False


# ============================================================================
# Mock 控制器：完全离线，用于流程仿真
# ============================================================================
class MockArmController(ArmController):
    """
    不碰串口的机械臂控制器，行为与 ArmController 完全一致。

    额外能力：记录所有"发出过"的指令，方便测试里断言流程是否正确。
    """

    def __init__(self, *a, **kw):
        kw["mock"] = True
        super().__init__(*a, **kw)
        self.history: List[str] = []

    def _send(self, body: str) -> None:
        self.history.append(body)
        print("[ARM-MOCK] >> $%s" % body)
