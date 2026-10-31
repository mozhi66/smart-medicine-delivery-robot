#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESC 退出监测（Windows 控制台）

主程序默认「一直循环听下一单」。本模块在后台线程轮询键盘：
用户按 **ESC** 时置退出标志 —— 录音/等语音/等机械臂应答/主循环都会尽快停。

设计要点
--------
· 后台 daemon 线程 + msvcrt.kbhit()，不阻塞业务逻辑。
· 只认 ESC（\\x1b）。Ctrl+C 仍走 KeyboardInterrupt，两条路都能退。
· 非 Windows 无 msvcrt 时自动空实现（仍可 Ctrl+C）。
· 线程安全：只写一个 bool，业务侧随时 requested() 查询。
"""
from __future__ import annotations

import os
import threading
import time

_stop = False
_hit = False
_thr = None
_lock = threading.Lock()


def start() -> None:
    """启动 ESC 监听（幂等，可重复调用）。"""
    global _thr
    with _lock:
        if _thr is not None or os.name != "nt":
            return
        t = threading.Thread(target=_loop, name="EscKeyMonitor", daemon=True)
        _thr = t
        t.start()


def stop() -> None:
    """停止监听（进程退出前可调；daemon 线程不调也可）。"""
    global _stop
    _stop = True


def requested() -> bool:
    """是否已按下 ESC（不消费）。"""
    return _hit


def consume() -> bool:
    """读取并清除退出标志。"""
    global _hit
    with _lock:
        r = _hit
        _hit = False
        return r


def reset() -> None:
    """清除退出标志（单轮开始前可调）。"""
    global _hit
    with _lock:
        _hit = False


def _mark_hit() -> None:
    global _hit
    with _lock:
        if not _hit:
            _hit = True
            try:
                print("\n[退出] 检测到 ESC —— 将在当前步骤结束后停止循环…")
            except Exception:
                pass


def _loop() -> None:
    try:
        import msvcrt
    except Exception:
        return
    while not _stop:
        try:
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                # ESC 单独出现是 '\\x1b'；若是方向键等扩展键会先来 '\\x00'/'\\xe0'
                if ch == "\x1b":
                    _mark_hit()
                    return
                if ch in ("\x00", "\xe0"):
                    try:
                        msvcrt.getwch()          # 丢掉扩展键第二字节
                    except Exception:
                        pass
        except Exception:
            return
        time.sleep(0.05)
