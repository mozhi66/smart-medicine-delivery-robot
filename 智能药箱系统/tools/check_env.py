#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 运行环境体检 (check_env.py)
================================================================================
职责：开跑主程序之前的「一站式体检」——把解释器、依赖、麦克风、摄像头、
串口一次查清，输出阻塞项 / 警告项 / OK 项，避免跑到一半才红字。

与 pc/core/envcheck.py 的分工：
  envcheck.py  ：主程序启动时的轻量自检（只查 require 列表里的包）。
  check_env.py ：本工具，独立命令行深度体检（硬件 + 全部依赖 + 配置）。

判定分级：
  [ OK ]    通过
  [WARN]    不阻塞主流程，但功能受限（如缺 edge-tts 只是没联网语音）
  [FAIL]    阻塞项——不解决跑不起来

用法：
  python tools/check_env.py                # 全项体检
  python tools/check_env.py --no-hardware  # 跳过麦克风/摄像头/串口探测
  python tools/check_env.py --quick        # 只查依赖，最快

退出码：0 = 无阻塞项；1 = 有阻塞项。
================================================================================
"""
from __future__ import annotations

import argparse
import importlib
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, os.path.join(_ROOT, "pc")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# 依赖分级：(模块名, 级别, 说明, 安装提示)
DEPS = [
    ("numpy",       "FAIL", "数值计算（视觉/逆解全靠它）",        "pip install numpy"),
    ("yaml",        "FAIL", "读 config/app.yaml",                "pip install pyyaml"),
    ("cv2",         "FAIL", "摄像头 / 二维码 / 中文渲染",         "pip install opencv-python"),
    ("serial",      "FAIL", "STM32 串口通信",                    "pip install pyserial"),
    ("requests",    "FAIL", "LLM 云端调用",                      "pip install requests"),
    ("sounddevice", "WARN", "真麦克风录音（没有它只能文本输入）",  "pip install sounddevice"),
    ("funasr",      "WARN", "FunASR 语音识别（没有它 ASR 不可用）", "pip install funasr"),
    ("torch",       "WARN", "FunASR 的推理引擎",                  "pip install torch"),
    ("pyttsx3",     "WARN", "离线 TTS（SAPI5）",                 "pip install pyttsx3"),
    ("edge_tts",    "WARN", "联网 TTS（微软神经语音，音质更好）",   "pip install edge-tts"),
    ("matplotlib",  "WARN", "kinematics_check 画图用",           "pip install matplotlib"),
    ("docx",        "WARN", "生成药品对照表 docx",               "pip install python-docx"),
]

# 已知坑：Anaconda 基础环境缺 funasr/torch，真机必须用 Python 3.10
BAD_EXE_HINTS = ("anaconda", "miniconda")


def _ver(mod):
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


def check_interpreter() -> int:
    """解释器检查：版本 + 已知坑提醒。返回阻塞计数。"""
    blocked = 0
    print("-" * 70)
    v = sys.version_info
    print("[信息] 解释器: %s" % sys.executable)
    print("[信息] Python: %d.%d.%d" % (v.major, v.minor, v.micro))
    if (v.major, v.minor) < (3, 8):
        print("[FAIL] Python 需 >= 3.8（本项目开发用的是 3.10）")
        blocked += 1
    else:
        print("[ OK ] Python 版本满足 >= 3.8")
    low = sys.executable.replace("\\", "/").lower()
    if any(h in low for h in BAD_EXE_HINTS):
        print("[WARN] 正在用 Anaconda 基础环境——它缺 funasr/torch，")
        print("       真麦克风 + 真 ASR 必须改用 Python 3.10 解释器！")
    return blocked


def check_deps() -> int:
    """依赖检查。返回阻塞计数。"""
    print("-" * 70)
    print("[依赖] 共 %d 项" % len(DEPS))
    blocked = 0
    for name, level, why, fix in DEPS:
        try:
            mod = importlib.import_module(name)
            print("[ OK ] %-14s %-10s %s" % (name, _ver(mod), why))
        except Exception:
            if level == "FAIL":
                print("[FAIL] %-14s %-10s %s" % (name, "缺失", why))
                print("       装它: %s" % fix)
                blocked += 1
            else:
                print("[WARN] %-14s %-10s %s（%s）" % (name, "缺失", why, fix))
    return blocked


def check_config() -> int:
    """配置文件存在性与基本结构。返回阻塞计数。"""
    print("-" * 70)
    blocked = 0
    cfg = os.path.join(_ROOT, "config", "app.yaml")
    if not os.path.exists(cfg):
        print("[FAIL] 找不到 config/app.yaml")
        return 1
    print("[ OK ] config/app.yaml 存在")
    try:
        import yaml
        d = yaml.safe_load(open(cfg, encoding="utf-8"))
        llm = d.get("llm") or {}
        if not llm.get("provider"):
            print("[FAIL] app.yaml 的 llm.provider 为空")
            blocked += 1
        else:
            print("[ OK ] llm.provider = %s" % llm["provider"])
        if not (llm.get("api_key") or os.environ.get(llm.get("api_key_env") or "")):
            print("[WARN] LLM API Key 未设置（app.yaml 的 api_key 或环境变量 %s）"
                  % llm.get("api_key_env"))
            print("       没 Key 也能跑——自动回退本地关键词规则（mock）")
        else:
            print("[ OK ] LLM API Key 已设置")
    except Exception as e:
        print("[FAIL] app.yaml 解析失败: %s" % e)
        blocked += 1
    med = os.path.join(_ROOT, "config", "medicines.json")
    if os.path.exists(med):
        print("[ OK ] config/medicines.json 存在")
    else:
        print("[FAIL] 找不到 config/medicines.json")
        blocked += 1
    return blocked


def check_mic() -> int:
    """麦克风探测。返回阻塞计数（麦克风缺失不算阻塞，只 WARN）。"""
    print("-" * 70)
    try:
        import sounddevice as sd
    except Exception:
        print("[WARN] sounddevice 未装，跳过麦克风探测（pip install sounddevice）")
        return 0
    try:
        devs = sd.query_devices()
        mics = [d for d in devs if d.get("max_input_channels", 0) > 0]
        if not mics:
            print("[WARN] 未发现任何输入设备——真麦克风录音不可用")
            return 0
        print("[ OK ] 麦克风可用：发现 %d 个输入设备" % len(mics))
        for i, d in enumerate(mics[:5]):
            print("       #%d %s（%d 通道，%d Hz）"
                  % (i, d.get("name", "?"), d.get("max_input_channels", 0),
                     int(d.get("default_samplerate", 0))))
        return 0
    except Exception as e:
        print("[WARN] 麦克风探测失败: %s" % e)
        return 0


def check_camera() -> int:
    """摄像头探测。返回阻塞计数（摄像头缺失只 WARN——扫码功能不可用）。"""
    print("-" * 70)
    try:
        import cv2
    except Exception:
        print("[WARN] cv2 未装，跳过摄像头探测")
        return 0
    found = []
    for idx in (0, 1, 2):
        try:
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                ok, frame = cap.read()
                if ok and frame is not None:
                    h, w = frame.shape[:2]
                    found.append((idx, w, h))
            cap.release()
        except Exception:
            pass
    if not found:
        print("[WARN] 未发现可用摄像头——过检扫码功能不可用")
        return 0
    print("[ OK ] 摄像头可用：%d 个" % len(found))
    for idx, w, h in found:
        print("       index %d：%dx%d" % (idx, w, h))
    return 0


def check_serial() -> int:
    """串口探测。返回阻塞计数（没串口只 WARN——mock 模式可跑）。"""
    print("-" * 70)
    try:
        import serial.tools.list_ports as lp
    except Exception:
        print("[WARN] pyserial 未装，跳过串口探测")
        return 0
    try:
        ports = list(lp.comports())
        if not ports:
            print("[WARN] 未发现任何串口——真机控制不可用（mock 模式不受影响）")
            return 0
        print("[ OK ] 串口：发现 %d 个" % len(ports))
        for p in ports:
            print("       %s：%s" % (p.device, p.description))
        return 0
    except Exception as e:
        print("[WARN] 串口探测失败: %s" % e)
        return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="智能药箱运行环境体检")
    ap.add_argument("--no-hardware", action="store_true", help="跳过硬件探测")
    ap.add_argument("--quick", action="store_true", help="只查依赖与配置")
    a = ap.parse_args(argv)

    print("=" * 70)
    print("智能药箱 · 运行环境体检")
    print("=" * 70)

    blocked = 0
    blocked += check_interpreter()
    blocked += check_deps()
    blocked += check_config()
    if not a.quick:
        if not a.no_hardware:
            blocked += check_mic()
            blocked += check_camera()
            blocked += check_serial()
        else:
            print("-" * 70)
            print("[信息] 已按 --no-hardware 跳过硬件探测")

    print("=" * 70)
    if blocked:
        print("[结论] 有 %d 个阻塞项，按上面提示逐条解决后再跑主程序" % blocked)
        return 1
    print("[结论] 没有阻塞项 + [ OK ] 关键项可用，可以开跑")
    print("       下一步：python pc/main.py（或先 python tools/phase1_smoke.py 彩排）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
