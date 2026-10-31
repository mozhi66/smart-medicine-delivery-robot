#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 语音合成模块 (tts.py)
================================================================================
职责：把文本读出来，用于"取到感冒药了，请服用""这个药一天最多两次"等反馈。

三种后端，按可用性自动选择
--------------------------
    pyttsx3 : 离线首选。Windows 走 SAPI5，Linux 走 espeak。
              优点：完全离线、延迟低、无需联网。
              坑：中文需要系统装了中文语音包（Windows 默认可能只有英文），
                  而且 pyttsx3 不是线程安全的，必须加锁串行调用。
    edge    : edge-tts（微软 Edge 的在线语音），音质远好于 SAPI5。
              需要联网；实现方式是合成出 mp3 再调用系统默认播放器播放。
    mock    : 只打印不发声。用于无音频设备的服务器/沙盒自测。

铁律
----
    语音反馈是"锦上添花"，绝不能因为它失败而中断取药主流程。
    所以 speak() 永远不抛异常，只返回 True/False。
================================================================================
"""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import threading
import time
from typing import List, Optional

# ---------------------------------------------------------------------------
# 可选依赖：全部 try-import，缺了只是换个后端，不让 import 阶段崩
# ---------------------------------------------------------------------------
try:
    import pyttsx3                      # pip install pyttsx3
except Exception:                       # pragma: no cover
    pyttsx3 = None

try:
    import comtypes.client as _sapi     # Windows SAPI5 COM 直连（pyttsx3 底层同款）
except Exception:                       # pragma: no cover
    _sapi = None

try:
    import edge_tts                     # pip install edge-tts
except Exception:                       # pragma: no cover
    edge_tts = None


class TTSEngine:
    """
    语音合成引擎（pyttsx3 / edge-tts / mock 三选一，自动降级）。

    典型用法
    --------
        tts = TTSEngine()                 # auto：优先 pyttsx3，失败转 edge，再失败转 mock
        tts.speak("已为您取出感冒灵颗粒")   # 阻塞读完
        tts.speak_async("请注意用药剂量")   # 开线程读，不卡主流程
        tts.close()

    主控程序里的推荐写法（取药动作和语音播报并行）：
        tts.speak_async("正在取药，请稍候")
        arm.pick(slot)
        tts.speak("取药完成")             # 这句会等上一句读完（内部有锁）
    """

    # edge-tts 默认音色（中文女声，自然度最好之一）
    EDGE_VOICE = "zh-CN-XiaoxiaoNeural"

    # 判断"这是不是一个中文语音"的关键词（pyttsx3 的 voice 信息各系统写法不同）
    CN_HINTS = ("zh", "chinese", "cn", "中文", "普通话",
                "huihui", "yaoyao", "xiaoxiao", "yunyang", "kangkang")

    def __init__(self, backend: str = "auto", rate: int = 0,
                 volume: float = 1.0, mock: bool = False,
                 want_real: bool = False):
        """
        参数
        ----
        backend   : 'pyttsx3'(离线) | 'edge'(需联网) | 'auto'(优先 pyttsx3) | 'mock'(只打印)
        rate      : 语速相对调整量（在 pyttsx3 默认 200 wpm 的基础上增减）。
                    0=不变，-50=慢一点，+50=快一点。内部会钳到 [60, 400] wpm。
        volume    : 音量 0.0~1.0
        mock      : True 时强制只打印不发声（无音频设备环境用）
        want_real : ★ mock 模式下也真发声（--real-tts）。mock=True 但 want_real=True
                    时按真实后端初始化 —— 让"假机械臂 + 真播报"的彩排模式成立。
        """
        self.rate = int(rate)
        self.volume = max(0.0, min(1.0, float(volume)))
        self.backend = "mock"             # 实际生效的后端
        self._engine = None               # pyttsx3 引擎实例（仅保留兼容，不再复用）
        self._voice_id = None             # 选定的中文语音 id（pyttsx3 路径用）
        self._sapi_voice = None           # Windows SAPI5 SpVoice（COM 直连，连播稳定）
        self._lock = threading.RLock()    # 语音后端非线程安全，必须串行
        self._tmp_files: List[str] = []   # edge 后端产生的临时 mp3
        self._last_error: Optional[Exception] = None

        # ★ --real-tts：mock 只代表"硬件是假的"，播报仍要真出声
        effective_mock = bool(mock) and not bool(want_real)

        want = "mock" if effective_mock else str(backend).lower()
        if want == "mock":
            self.backend = "mock"
            print("[TTS] mock 模式：speak() 只打印文本，不发声")
        elif want == "pyttsx3":
            if not self._init_pyttsx3():
                self.backend = "mock"
        elif want == "edge":
            if not self._init_edge():
                self.backend = "mock"
        else:                              # auto
            if not self._init_pyttsx3():
                if not self._init_edge():
                    self.backend = "mock"
                    print("[TTS] 所有后端都不可用 -> 已降级为 mock（只打印，不影响主流程）")

    # ======================================================================
    # 后端初始化
    # ======================================================================
    def _init_pyttsx3(self) -> bool:
        """
        初始化离线 SAPI5 后端。

        ★ Windows 首选路径：绕开 pyttsx3，直接用 comtypes 建 SAPI5 的
          SpVoice COM 对象。原因：pyttsx3 的 runAndWait() 事件循环在 Windows
          上只有第一次有效，复用引擎后续全部静音（pyttsx3.init 还返回模块级
          缓存单例，"每句新建引擎"也拿不到新对象）；而 SpVoice.Speak() 同步
          调用天然支持连续播报。pyttsx3 路径保留给非 Windows 平台。

        返回
        ----
        True 成功；False 失败（已打印原因）
        """
        # ---- ① Windows：SAPI5 COM 直连 ----
        if sys.platform.startswith("win"):
            if _sapi is None:
                print("[TTS] 未安装 comtypes（pyttsx3 依赖它，正常已装）："
                      "pip install comtypes")
            else:
                try:
                    sapi = _sapi.CreateObject("SAPI.SpVoice")
                    self._sapi_voice = sapi
                    self._apply_sapi_voice(sapi)
                    self.backend = "pyttsx3"      # 对外仍叫 pyttsx3（离线 SAPI5）
                    print("[TTS] SAPI5 就绪（comtypes 直连，支持连续播报）")
                    return True
                except Exception as e:
                    print("[TTS] SAPI5 直连失败，退回 pyttsx3 引擎：%r" % (e,))

        # ---- ② 回退：pyttsx3 引擎（非 Windows / comtypes 失败时）----
        if pyttsx3 is None:
            print("[TTS] 未安装 pyttsx3：pip install pyttsx3")
            return False

        try:
            # Windows 上显式指定 sapi5 更快也更稳；其它平台用默认驱动
            if sys.platform.startswith("win"):
                eng = pyttsx3.init("sapi5")
            else:
                eng = pyttsx3.init()
        except Exception as e:
            self._last_error = e
            print("[TTS] pyttsx3 初始化失败：%r" % (e,))
            print("      Windows 需要 SAPI5；Linux 需要 espeak：")
            print("      sudo apt install espeak libespeak1")
            return False

        try:
            cur_rate = eng.getProperty("rate") or 200
            eng.setProperty("rate", int(max(60, min(400, cur_rate + self.rate))))
            eng.setProperty("volume", self.volume)
        except Exception as e:
            print("[TTS] 设置语速/音量失败（用默认值继续）：%r" % (e,))

        voice = self._pick_chinese_voice(eng)
        if voice is not None:
            try:
                eng.setProperty("voice", voice.id)
                self._voice_id = voice.id          # ★ 缓存，供每次新建引擎时套用
                print("[TTS] pyttsx3 就绪，中文语音：%s" % (voice.name,))
            except Exception as e:
                print("[TTS] 切换中文语音失败，用系统默认：%r" % (e,))
        else:
            self._warn_no_chinese_voice(eng)

        self._engine = eng
        self.backend = "pyttsx3"
        return True

    def _init_edge(self) -> bool:
        """
        初始化 edge-tts 在线后端（只检查包是否可用，真正合成在 speak 时做）。

        返回
        ----
        True 可用；False 不可用（已打印安装提示）
        """
        if edge_tts is None:
            print("[TTS] 未安装 edge-tts：pip install edge-tts（且需联网）")
            return False
        self.backend = "edge"
        print("[TTS] edge-tts 就绪，音色：%s（需要联网）" % self.EDGE_VOICE)
        return True

    def _pick_chinese_voice(self, eng):
        """
        在系统已安装的语音里挑一个中文语音。

        参数
        ----
        eng : pyttsx3 引擎实例

        返回
        ----
        匹配的 voice 对象；找不到返回 None（此时不切换语音，用系统默认继续跑）
        """
        try:
            voices = eng.getProperty("voices") or []
        except Exception:
            return None
        for v in voices:
            blob = " ".join([str(getattr(v, "id", "") or ""),
                             str(getattr(v, "name", "") or ""),
                             " ".join(str(x) for x in (getattr(v, "languages", None) or []))
                             ]).lower()
            for hint in self.CN_HINTS:
                if hint in blob:
                    return v
        return None

    def _warn_no_chinese_voice(self, eng) -> None:
        """
        没找到中文语音时给出可操作的指引（而不是崩溃或用英文念中文）。

        参数
        ----
        eng : pyttsx3 引擎实例，用于列出当前可用语音
        """
        print("-" * 72)
        print("[TTS] 警告：系统里没有检测到中文语音，中文会被念成英文/直接跳过。")
        print("      Windows 解决：设置 -> 时间和语言 -> 语音(语言和区域) ->")
        print("                    添加中文(简体)语言包并勾选【语音识别/朗读】。")
        print("      Linux  解决：sudo apt install espeak-ng-data 或安装中文 espeak 语音。")
        try:
            names = [str(getattr(v, "name", "?")) for v in (eng.getProperty("voices") or [])]
            print("      当前可用语音（%d 个）：%s" % (len(names), ", ".join(names[:8])))
        except Exception:
            pass
        print("      备选方案：pip install edge-tts 后改用 TTSEngine(backend='edge')")
        print("-" * 72)

    def _apply_sapi_voice(self, sapi) -> None:
        """给 SpVoice 设置中文音色 / 语速 / 音量（失败静默，用默认值继续）"""
        try:
            sapi.Rate = int(max(-10, min(10, round(self.rate / 20.0))))
            sapi.Volume = int(round(self.volume * 100))
        except Exception:
            pass
        try:
            voices = sapi.GetVoices()
            for i in range(voices.Count):
                v = voices.Item(i)
                desc = str(v.GetDescription() or "").lower()
                if any(h in desc for h in self.CN_HINTS):
                    sapi.Voice = v
                    print("[TTS] SAPI5 中文语音：%s" % v.GetDescription())
                    break
        except Exception:
            pass

    # ======================================================================
    # 对外接口
    # ======================================================================
    def speak(self, text: str, wait: bool = True) -> bool:
        """
        朗读文本。

        参数
        ----
        text : 要朗读的文本（中文）
        wait : True 阻塞到读完；False 立刻返回，后台读

        返回
        ----
        True 成功；False 失败（任何异常都在内部兜住，不影响主控流程）
        """
        text = (text or "").strip()
        if not text:
            return False
        try:
            if self.backend == "pyttsx3":
                return self._speak_pyttsx3(text, wait)
            if self.backend == "edge":
                return self._speak_edge(text, wait)
            print("[TTS-MOCK] %s" % text)          # mock：只打印
            return True
        except Exception as e:
            # 最后一道保险：语音失败绝不能打断取药
            print("[TTS] 朗读失败（已忽略，不影响主流程）：%r" % (e,))
            print("[TTS] 本应播报：%s" % text)
            return False

    def speak_async(self, text: str) -> None:
        """
        非阻塞朗读：开一个守护线程去读，主线程立刻返回。

        适合"正在取药，请稍候"这种提示，避免卡住机械臂动作。

        参数
        ----
        text : 要朗读的文本
        """
        t = threading.Thread(target=self.speak, args=(text, True),
                             name="TTS-Speak", daemon=True)
        t.start()

    def close(self) -> None:
        """停止朗读并释放资源（可重复调用）。"""
        try:
            if self._engine is not None:
                self._engine.stop()
        except Exception:
            pass
        self._engine = None
        self._sapi_voice = None          # SpVoice COM 对象交给 GC 释放
        # 清理 edge 后端产生的临时 mp3
        for p in list(self._tmp_files):
            try:
                if os.path.exists(p):
                    os.remove(p)
            except Exception:
                pass
        self._tmp_files = []

    # ======================================================================
    # 上下文管理器 / 调试信息
    # ======================================================================
    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()
        return False

    @property
    def is_mock(self) -> bool:
        """当前是否处于只打印不发声的 mock 状态"""
        return self.backend == "mock"

    def __repr__(self) -> str:
        return "TTSEngine(backend=%s, rate=%d, volume=%.2f)" % (
            self.backend, self.rate, self.volume)

    # ======================================================================
    # 各后端的实际发声
    # ======================================================================
    def _new_pyttsx3_engine(self):
        """
        新建一个一次性 pyttsx3 引擎（读一句话的生命周期）。

        ★ 为什么不复用引擎：Windows SAPI5 驱动下 pyttsx3 的 runAndWait()
          只有第一次真正出声，之后再调事件循环已经"坏了"——表现为
          "只有第一句有声音，后面全部静音"。每句话新建引擎是标准解。
        """
        try:
            if sys.platform.startswith("win"):
                eng = pyttsx3.init("sapi5")
            else:
                eng = pyttsx3.init()
        except Exception as e:
            self._last_error = e
            print("[TTS] pyttsx3 引擎创建失败：%r" % (e,))
            return None
        try:
            cur_rate = eng.getProperty("rate") or 200
            eng.setProperty("rate", int(max(60, min(400, cur_rate + self.rate))))
            eng.setProperty("volume", self.volume)
            if self._voice_id:
                eng.setProperty("voice", self._voice_id)
        except Exception:
            pass
        return eng

    def _speak_pyttsx3(self, text: str, wait: bool) -> bool:
        """
        离线发声。

        Windows：SAPI5 COM 直连（SpVoice.Speak 同步调用，天然支持连续播报，
        彻底绕开 pyttsx3 runAndWait 复用失效问题）。
        其它平台 / COM 不可用：pyttsx3 每句新建引擎。

        参数
        ----
        text : 文本
        wait : 是否阻塞到读完

        返回
        ----
        True/False
        """
        # ---- ① Windows SAPI5 直连 ----
        if self._sapi_voice is not None or (_sapi is not None and
                                            sys.platform.startswith("win")):

            def _run_sapi() -> bool:
                with self._lock:
                    v = self._sapi_voice
                    if v is None:
                        return False
                    try:
                        v.Speak(text)          # 同步：读完才返回
                        return True
                    except Exception as e:
                        print("[TTS] SAPI5 朗读异常，重建 SpVoice 重试：%r" % (e,))
                    try:
                        v2 = _sapi.CreateObject("SAPI.SpVoice")
                        self._sapi_voice = v2
                        self._apply_sapi_voice(v2)
                        v2.Speak(text)
                        return True
                    except Exception as e2:
                        print("[TTS] SAPI5 重试仍失败：%r" % (e2,))
                        return False

            if wait:
                return _run_sapi()
            threading.Thread(target=_run_sapi, name="TTS-Speak",
                             daemon=True).start()
            return True

        # ---- ② 回退：pyttsx3 每句新建引擎 ----
        if pyttsx3 is None:
            return False

        def _run() -> bool:
            with self._lock:
                eng = self._new_pyttsx3_engine()
                if eng is None:
                    return False
                try:
                    eng.say(text)
                    eng.runAndWait()
                finally:
                    try:
                        eng.stop()
                    except Exception:
                        pass
                return True

        if wait:
            return _run()
        # 非阻塞：另起线程完整走一遍"建引擎-读-销毁"
        threading.Thread(target=_run, name="TTS-RunAndWait", daemon=True).start()
        return True

    def _speak_edge(self, text: str, wait: bool) -> bool:
        """
        用 edge-tts 合成 mp3 并调用系统默认播放器播放。

        注意：edge 后端依赖外部播放器，wait=True 只能按字数粗略等待，
        做不到精确阻塞。要求严格同步的场合请用 pyttsx3 后端。

        参数
        ----
        text : 文本
        wait : 是否粗略等待播放结束

        返回
        ----
        True/False
        """
        import asyncio

        rate_pct = int(self.rate / 200.0 * 100)          # wpm 增量换算成百分比
        volume_pct = int((self.volume - 1.0) * 100)

        fd, path = tempfile.mkstemp(prefix="medbox_tts_", suffix=".mp3")
        os.close(fd)

        async def _synth() -> None:
            comm = edge_tts.Communicate(
                text, self.EDGE_VOICE,
                rate="%+d%%" % rate_pct,
                volume="%+d%%" % volume_pct)
            await comm.save(path)

        try:
            asyncio.run(_synth())
        except Exception as e:
            print("[TTS] edge-tts 合成失败（检查网络）：%r" % (e,))
            try:
                os.remove(path)
            except Exception:
                pass
            return False

        self._tmp_files.append(path)
        if not self._play_file(path):
            print("[TTS] 无法自动播放，音频已保存：%s" % path)
            return False

        if wait:
            # 中文约 4~5 字/秒，粗估等待；不精确但够用
            time.sleep(max(0.5, len(text) / 4.5))
        return True

    @staticmethod
    def _play_file(path: str) -> bool:
        """
        用系统默认程序播放音频文件（不引入额外依赖）。

        参数
        ----
        path : 音频文件路径

        返回
        ----
        True 已调用播放器；False 调用失败
        """
        try:
            if sys.platform.startswith("win"):
                os.startfile(path)                       # noqa: S606
                return True
            if sys.platform == "darwin":
                subprocess.Popen(["open", path])
                return True
            subprocess.Popen(["xdg-open", path],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except Exception as e:
            print("[TTS] 调用系统播放器失败：%r" % (e,))
            return False


# ===========================================================================
# 自测：python3.11 -c "import sys; sys.path.insert(0,'pc'); from audio.tts import TTSEngine; ..."
# ===========================================================================
def _selftest() -> None:
    """无音频设备环境下的最小自测"""
    tts = TTSEngine(mock=True)
    print("engine =", tts)
    print("speak ->", tts.speak("已为您取出感冒灵颗粒"))
    tts.speak_async("请注意用药剂量")
    time.sleep(0.2)
    tts.close()

    auto = TTSEngine()          # 真实环境下会自动挑后端
    print("auto  ->", auto)
    auto.close()


if __name__ == "__main__":
    _selftest()
    sys.exit(0)
