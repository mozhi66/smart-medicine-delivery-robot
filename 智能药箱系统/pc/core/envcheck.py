#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 运行环境自检 (envcheck.py)
================================================================================
职责：程序启动时把"用的是哪个 python、关键依赖齐不齐、缺了会怎样"直接摊开讲。
专治"选了错的解释器 → 报 No module named 'cv2'"这类迷之红字。

用法（main.py）::

    from core.envcheck import report
    report(require=["numpy", "yaml", "cv2", "serial", "sounddevice", "funasr", "torch"])

设计要点：
· 只校验【本次运行真正需要的包】（require 传入），--mock 时不会冲着一堆
  用不上的包刷屏报警。
· 每个包给一行结论（版本 / MISSING），缺了还给一句"装它"的命令。
· 永不抛异常 —— 自检挂了也要让主程序继续跑。
================================================================================
"""

from __future__ import annotations

import importlib
import shutil
import sys


def _module_version(mod) -> str:
    """尽力取模块版本号，取不到返回 '?'。"""
    for attr in ("__version__", "VERSION", "version"):
        v = getattr(mod, attr, None)
        if isinstance(v, str) and v:
            return v
        if isinstance(v, (tuple, list)):
            try:
                return ".".join(str(x) for x in v)
            except Exception:
                pass
    return "?"


def report(require, python_exe: str = "") -> None:
    """
    打印环境自检报告。

    参数
    ----
    require    : 本次运行需要的包名列表，如 ["numpy", "cv2", "funasr"]
    python_exe : 当前解释器路径（None 时自动取 sys.executable）
    """
    try:
        exe = python_exe or sys.executable
        print("-" * 70)
        print("[环境自检] 解释器: %s" % exe)

        # ① 解释器黑名单提醒：ANACONDA 基础环境缺 funasr/torch，本机已知坑
        exe_low = exe.replace("\\", "/").lower()
        if "anaconda" in exe_low or "miniconda" in exe_low:
            print("[环境自检] ★ 警告：当前是 ANACONDA 解释器 —— 本机实测它缺")
            print("           funasr/torch/sounddevice，语音链路会挂。")
            print("           请改用独立安装的 CPython 3.10 解释器（非 conda 环境）")

        # ② 逐包检查
        miss = []
        for name in require:
            try:
                mod = importlib.import_module(name)
                print("  [OK]      %-12s %s" % (name, _module_version(mod)))
            except Exception as e:
                print("  [MISSING] %-12s %s" % (name, e))
                miss.append(name)

        if miss:
            print("[环境自检] 缺 %d 个包，安装命令：" % len(miss))
            print('  "%s" -m pip install %s' % (exe, " ".join(miss)))
            print("[环境自检] ★ 装完确认用的还是同一个解释器再运行！")
        else:
            print("[环境自检] 本次所需依赖齐全。")
        print("-" * 70)
    except Exception as e:
        print("[环境自检] 自身出错(不影响主流程): %r" % e)


def check_tools(tool_names) -> dict:
    """检查外部命令行工具是否在 PATH 里（如 ffmpeg）。返回 {name: path|None}。"""
    out = {}
    try:
        for t in tool_names:
            out[t] = shutil.which(t)
    except Exception:
        pass
    return out


if __name__ == "__main__":
    report(require=["numpy", "yaml", "cv2", "serial", "sounddevice", "funasr", "torch", "pyttsx3"])
