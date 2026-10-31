#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 语音包 (pc/audio)
================================================================================
职责：智能药箱的"嘴巴和耳朵"。

    asr.py : ASREngine    语音识别（听）——麦克风 + FunASR + 端点检测
    tts.py : TTSEngine    语音合成（说）——pyttsx3 / edge-tts / mock

用法
----
    from audio.asr import ASREngine
    from audio.tts import TTSEngine

两个模块的第三方依赖（funasr / sounddevice / pyttsx3 / edge_tts）都是可选的：
缺失时自动降级到 mock，只打印不发声，绝不会让主控程序在 import 阶段崩溃。
================================================================================
"""

from __future__ import annotations

__all__: list = []          # 故意不在包初始化时 import 子模块（避免拖慢启动、避免 torch 被拉起）

# 说明：这里不写 "from .asr import ASREngine" 是刻意的。
# asr 会在加载模型时 import funasr -> torch，代价好几秒；
# 而很多场景（比如只做视觉标定）根本用不到语音。
# 请按需显式导入：from audio.asr import ASREngine
