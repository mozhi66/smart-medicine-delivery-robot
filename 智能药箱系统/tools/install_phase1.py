#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 Phase-1 环境一键安装 (install_phase1.py)
================================================================================
职责：把「跑语音全链路需要的 Python 依赖」一次装齐。
Phase-1 = 环境与模型就绪（装包 + 下模型），之后才能 phase1_smoke 彩排。

本机既定事实（写死在脚本里的经验）：
  ★ 涉及真麦克风 / 真 ASR，必须用 E:\\software\\Python 3.10 解释器。
    Anaconda 基础环境缺 funasr/torch，装不上也别硬刚。
  ★ opencv 建议用 opencv-python（无 GTK 依赖，Windows 开箱即用）。
  ★ torch 体积大，CPU 版即可（本项目 ASR/逆解都吃 CPU）。

用法：
  python tools/install_phase1.py            # 装全部（含 torch 大块头）
  python tools/install_phase1.py --no-torch # 跳过 torch（已装过/想手动）
  python tools/install_phase1.py --dry-run  # 只打印要执行的命令，不真装

注意：脚本只调用 pip，不修改系统环境变量；装完自行用 check_env.py 验收。
================================================================================
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)

# (pip 包名, 是否大块头, 用途)
PACKAGES = [
    ("numpy",            False, "数值计算"),
    ("pyyaml",           False, "读 config/app.yaml"),
    ("opencv-python",    False, "摄像头/二维码/中文渲染"),
    ("pyserial",         False, "STM32 串口"),
    ("requests",         False, "LLM 云端调用"),
    ("sounddevice",      False, "真麦克风录音"),
    ("pyttsx3",          False, "离线 TTS（SAPI5）"),
    ("edge-tts",         False, "联网 TTS（可选，音质更好）"),
    ("funasr",           False, "FunASR 语音识别"),
    ("torch",            True,  "FunASR 推理引擎（CPU 版）"),
    ("matplotlib",       False, "运动学检查画图（可选）"),
    ("python-docx",      False, "药品对照表生成（可选）"),
]

# 已知坑解释器
KNOWN_GOOD_HINT = "python"


def _pip_install(pkg: str, dry: bool) -> bool:
    cmd = [sys.executable, "-m", "pip", "install", pkg]
    print(">>", " ".join(cmd))
    if dry:
        return True
    try:
        r = subprocess.run(cmd)
        return r.returncode == 0
    except Exception as e:
        print("[FAIL] 执行出错: %s" % e)
        return False


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Phase-1 环境一键安装")
    ap.add_argument("--no-torch", action="store_true", help="跳过 torch")
    ap.add_argument("--dry-run", action="store_true", help="只打印命令不执行")
    a = ap.parse_args(argv)

    print("=" * 70)
    print("Phase-1 环境安装（智能药箱语音全链路）")
    print("=" * 70)
    print("[信息] 当前解释器: %s" % sys.executable)
    low = sys.executable.replace("\\", "/").lower()
    if "anaconda" in low or "miniconda" in low:
        print("[WARN] 你在用 Anaconda 基础环境——它缺 funasr/torch 的兼容构建，")
        print("       强烈建议改用 %s 再跑本脚本！" % KNOWN_GOOD_HINT)

    targets = [p for p in PACKAGES if not (a.no_torch and p[0] == "torch")]
    print("[信息] 待装 %d 项（torch %s）" % (len(targets), "跳过" if a.no_torch else "包含"))

    failed = []
    for i, (pkg, big, why) in enumerate(targets, 1):
        print("-" * 70)
        print("(%d/%d) %s —— %s%s" % (i, len(targets), pkg, why,
                                        "（大块头，耐心等）" if big else ""))
        if not _pip_install(pkg, a.dry_run):
            failed.append(pkg)

    print("=" * 70)
    if a.dry_run:
        print("[结论] dry-run 结束（未真装）。去掉 --dry-run 再跑一遍即开始安装。")
        return 0
    if failed:
        print("[结论] %d 项失败: %s" % (len(failed), ", ".join(failed)))
        print("       国内网络建议加镜像：pip install -i https://pypi.tuna.tsinghua.edu.cn/simple <包>")
        return 1
    print("[结论] 全部安装完成。下一步：")
    print("       1) python tools/dl_models.py      # 下 FunASR 三件套模型")
    print("       2) python tools/check_env.py      # 验收环境")
    print("       3) python tools/phase1_smoke.py   # Phase-1 冒烟彩排")
    return 0


if __name__ == "__main__":
    sys.exit(main())
