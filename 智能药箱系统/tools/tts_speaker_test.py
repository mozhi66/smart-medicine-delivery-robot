#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 TTS 喇叭测试 (tts_speaker_test.py)
================================================================================
职责：单独验证「语音播报链路」——不碰串口、不碰机械臂、不连 LLM，
只让 TTS 引擎说几句话，用来排查：没声音 / 音质怪 / 后端选错。

后端（与 pc/audio/tts.py 一致）：
  pyttsx3  离线 SAPI5（Windows 自带，断网可用，机械音）
  edge     联网微软神经语音（音质好，需网络；首次有建立连接开销）
  auto     优先 pyttsx3，失败回退 edge
  mock     只打印不发声（无音频设备环境）

用法：
  python tools/tts_speaker_test.py                  # auto 后端说三句
  python tools/tts_speaker_test.py --backend edge   # 指定后端
  python tools/tts_speaker_test.py --text "你好呀"   # 自定义文本
  python tools/tts_speaker_test.py --mock           # 只打印（CI/无喇叭环境）

退出码：0 = 至少一个后端发声成功；1 = 全失败。
================================================================================
"""
from __future__ import annotations

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, os.path.join(_ROOT, "pc")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

DEFAULT_TEXTS = [
    "智能药箱语音自检开始",
    "您需要的药品是感冒灵颗粒，请按时服用",
    "智能药箱语音自检结束",
]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="TTS 喇叭测试")
    ap.add_argument("--backend", default="auto",
                    choices=["auto", "pyttsx3", "edge", "mock"], help="TTS 后端")
    ap.add_argument("--text", default=None, help="自定义测试文本（默认三句）")
    ap.add_argument("--rate", type=int, default=0, help="语速增减（-50~+50）")
    ap.add_argument("--mock", action="store_true", help="强制 mock（只打印）")
    a = ap.parse_args(argv)

    print("=" * 70)
    print("TTS 喇叭测试")
    print("=" * 70)

    try:
        from audio.tts import TTSEngine
    except Exception as e:
        print("[FAIL] 导不入 TTSEngine：%s" % e)
        print("       确认在 智能药箱系统/ 目录下运行，且 pc/ 可导入")
        return 1

    backend = "mock" if a.mock else a.backend
    texts = [a.text] if a.text else DEFAULT_TEXTS

    print("[信息] 请求后端: %s" % backend)
    try:
        eng = TTSEngine(backend=backend, rate=a.rate, mock=a.mock)
    except Exception as e:
        print("[FAIL] 引擎初始化失败: %s" % e)
        return 1

    actual = getattr(eng, "backend", "?")
    print("[信息] 实际后端: %s" % actual)
    if not a.mock and actual == "mock":
        print("[WARN] 请求 %s 但回退到了 mock —— 多半是依赖缺失或音频设备不可用" % backend)
        print("       pyttsx3: pip install pyttsx3 ；edge: pip install edge-tts")

    ok = 0
    for i, t in enumerate(texts, 1):
        print("-" * 70)
        print("(%d/%d) 播报: %s" % (i, len(texts), t))
        try:
            if eng.speak(t, wait=True):
                ok += 1
                print("[ OK ] 播报完成")
            else:
                print("[FAIL] 播报返回 False（引擎报错，看上面日志）")
        except Exception as e:
            print("[FAIL] 播报异常: %s" % e)

    print("=" * 70)
    if ok == len(texts):
        print("[结论] TTS 链路 OK（%d/%d）——能听到声音就说明喇叭/驱动没问题" % (ok, len(texts)))
        return 0
    if ok:
        print("[结论] 部分成功（%d/%d）——检查失败那几句的报错" % (ok, len(texts)))
        return 1
    print("[结论] 全部失败。排查顺序：")
    print("       1) 系统音量/默认输出设备；2) pip install pyttsx3 edge-tts；")
    print("       3) 换 --backend pyttsx3 或 --backend edge 各试一次；4) --mock 确认代码通路")
    return 1


if __name__ == "__main__":
    sys.exit(main())
