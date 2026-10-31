#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 统一日志 (logger.py)
================================================================================
职责：把系统运行过程同时输出到终端和日志文件。

为什么单独做一个？
    毕设答辩和现场调试时，"刚才到底发生了什么"全靠日志。
    日志里带时间戳和步骤编号，出问题能一眼定位到是哪一步、隔了多久。
    同时这份日志也是论文里"系统运行实例"章节的素材。
================================================================================
"""

from __future__ import annotations

import os
import threading
import time
from typing import Optional


class Logger:
    """
    线程安全的简易日志器。

    用法
    ----
        log = Logger("logs/run.log")
        log.info("开始取药")
        log.warn("视觉确认失败，重试")
        log.error("急停触发")
    """

    def __init__(self, path: Optional[str] = None, echo: bool = True,
                 keep_open: bool = True):
        """
        参数
        ----
        path      : 日志文件路径；None 表示不写文件
        echo      : 是否同时打印到终端
        keep_open : True 时保持文件句柄（快），False 时每次追加后关闭（安全）
        """
        self.path = path
        self.echo = bool(echo)
        self.keep_open = bool(keep_open)
        self._fp = None
        self._lock = threading.Lock()
        self._t0 = time.time()

        if path:
            d = os.path.dirname(os.path.abspath(path))
            if d:
                os.makedirs(d, exist_ok=True)
            if keep_open:
                self._fp = open(path, "a", encoding="utf-8")

    # ------------------------------------------------------------------
    def log(self, level: str, msg: str) -> None:
        """写一条日志。level 为 INFO/WARN/ERROR/STEP"""
        t = time.time() - self._t0
        stamp = time.strftime("%H:%M:%S")
        line = "[%s +%7.2fs] %-5s %s" % (stamp, t, level, msg)
        with self._lock:
            if self.echo:
                print(line)
            if self.path:
                if self.keep_open and self._fp:
                    self._fp.write(line + "\n")
                    self._fp.flush()
                else:
                    with open(self.path, "a", encoding="utf-8") as f:
                        f.write(line + "\n")

    # ------------------------------------------------------------------
    def info(self, msg: str) -> None:
        """普通信息"""
        self.log("INFO", msg)

    def step(self, msg: str) -> None:
        """流程节点（每一步动作前打一条，便于定位卡在哪）"""
        self.log("STEP", msg)

    def warn(self, msg: str) -> None:
        """警告：能继续跑，但要留意"""
        self.log("WARN", msg)

    def error(self, msg: str) -> None:
        """错误：这一步失败了"""
        self.log("ERROR", msg)

    # ------------------------------------------------------------------
    def close(self) -> None:
        """关闭文件句柄"""
        with self._lock:
            if self._fp:
                try:
                    self._fp.close()
                except Exception:
                    pass
                self._fp = None
