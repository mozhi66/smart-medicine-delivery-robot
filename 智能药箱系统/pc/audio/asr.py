#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 语音识别模块 (asr.py)
================================================================================
职责：把"用户对着药箱说的一句话"变成"一段中文文本"，交给云端大模型做意图映射。

链路位置：
    麦克风 -> 端点检测(VAD) -> FunASR 推理 -> 文本 -> 云端大模型 -> 药品/槽位

设计要点（都是真实项目里会踩的坑，不是教科书内容）
------------------------------------------------
1) 三级降级，任何一级失败都不能让主控程序崩
      一级：funasr 真模型（联网/已装 torch 时）
      二级：用户显式 mock=True（无模型环境跑通全流程）
      三级：模型加载失败自动降级到 mock（funasr 没装、没网、显存不够）
   答辩现场环境不可控，主控程序必须"能说话、能降级、不崩"。

2) 端点检测（简化版 VAD）
   不用"固定录 3 秒"——用户说半句就要干等，说长了又被截断，体验极差。
   这里用【短时能量 + 自适应噪声阈值】判断什么时候开始说、什么时候说完：
       - 开头 0.3 秒当作静音期，用来估计当前环境的底噪 RMS
       - 阈值 = max(底噪*3, 底噪+3σ+ε, 底噪+0.01, 0.006)
         三个式子各有作用：3 倍底噪抗一般噪声；+3σ 抗忽大忽小的环境噪声；
         +0.01 和 0.006 是绝对下限，防止在极安静房间里阈值趋近 0 导致
         呼吸声被当成说话（这是自适应阈值最典型的翻车点）
       - 连续 2 帧超阈值才算"开始说话"（抗键盘声、桌面的单次敲击）
       - 说完之后静音超过 phrase_timeout 秒就结束
       - 保留 1 秒预缓冲（pre-roll），避免第一个字被切掉

3) 热词 hotwords
   药品名（"布洛芬""感冒灵颗粒"）是低频专有名词，通用模型极易识别成
   同音词。FunASR 的 generate() 支持 hotword 参数，注入后可显著提升命中率。

4) 采样率
   模型要 16k，但不少 Windows 麦克风不支持 16000Hz（只支持 44100/48000）。
   这里先按 16k 开流，失败就用设备默认采样率录音，最后线性重采样到 16k。
   对 ASR 来说线性插值的失真完全可以接受。

依赖（都是可选的，缺了会自动降级，不会在 import 阶段崩）
------------------------------------------------
    pip install sounddevice        # 录音（推荐，自带 PortAudio，无需手动装）
    pip install -U funasr          # 识别（会连带装 torch，比较大）
================================================================================
"""

from __future__ import annotations

import collections
import os
import sys
import tempfile
import time
import wave
from typing import Deque, List, Optional

import numpy as np

# ---------------------------------------------------------------------------
# 可选依赖：录音。缺失时不影响 import，只在实际录音时提示并降级。
# ---------------------------------------------------------------------------
try:
    import sounddevice as sd          # pip install sounddevice
except Exception:                     # pragma: no cover
    sd = None

# funasr 故意【不在模块顶层 import】：
#   它会连带 import torch，加载一次要好几秒，会让 "import audio.asr" 变慢，
#   而绝大多数 mock 场景根本用不到它。放到真正加载模型时再 import。


# ===========================================================================
# 常量
# ===========================================================================
_FRAME_SEC = 0.03          # VAD 每帧 30ms（语音处理常用帧长，兼顾灵敏度与抖动）
_CALIB_SEC = 0.3           # 开头用于估计底噪的静音时长
_PREROLL_SEC = 1.0         # 保留的预缓冲时长，防止第一个字被切掉
_MAX_RECORD_SEC = 15.0     # 单次录音硬上限，防止一直说导致内存/等待失控


def _print_funasr_help(err: Exception, model_name: str, import_error: bool) -> None:
    """
    打印 FunASR 不可用时的排查指引。

    参数
    ----
    err          : 捕获到的异常对象
    model_name   : 当前请求的模型名，用于提示下载哪个模型
    import_error : True 表示连 import 都失败（没装包）；False 表示加载/下载失败
    """
    print("=" * 72)
    print("[ASR] FunASR 不可用 -> 已自动降级为 mock 模式（程序不会中断）")
    print("      原因：%r" % (err,))
    print("      解决办法：")
    if import_error:
        print("        1) 安装：pip install -U funasr")
        print("           （funasr 会依赖 torch/torchaudio，CPU 版直接装即可；")
        print("            CUDA 版请先装 torch：")
        print("            pip install torch torchaudio --index-url "
              "https://download.pytorch.org/whl/cu118）")
    print("        2) 模型：首次运行会从 ModelScope 自动下载 '%s'（约数百 MB），"
          % model_name)
    print("           需要联网。离线环境请先在能联网的机器上跑一次，")
    print("           再把 ~/.cache/modelscope（Windows: C:/Users/<你>/.cache/modelscope）")
    print("           整个目录拷到目标机器。")
    print("        3) 现在的表现：recognize_once() 用 input() 键盘输入代替麦克风，")
    print("           其余流程（大模型映射/取药/反馈）完全不受影响。")
    print("=" * 72)


def parse_funasr_result(res) -> Optional[str]:
    """
    把 FunASR generate() 的返回值统一解析成一个字符串。

    FunASR 各版本返回结构不完全一致，常见有：
        [{'key': 'xxx', 'text': '我感冒了'}]
        [{'key': 'xxx', 'value': '我感冒了'}]
        '我感冒了'（直接字符串）

    参数
    ----
    res : generate() 的原始返回值

    返回
    ----
    识别文本；无法解析或文本为空时返回 None
    """
    if res is None:
        return None

    # 情况1：直接给字符串
    if isinstance(res, str):
        return res.strip() or None

    # 情况2：list[dict] / tuple（最常见）
    if isinstance(res, (list, tuple)):
        texts: List[str] = []
        for item in res:
            if isinstance(item, dict):
                # 注意：不要用 'key'，那是音频文件名/utt id，不是识别结果
                t = item.get("text") or item.get("value") or item.get("preds")
                if isinstance(t, str) and t.strip():
                    texts.append(t.strip())
            elif isinstance(item, str) and item.strip():
                texts.append(item.strip())
        if texts:
            return "".join(texts) if _is_mostly_chinese(texts[0]) else " ".join(texts)

    # 情况3：dict
    if isinstance(res, dict):
        t = res.get("text") or res.get("value")
        if isinstance(t, str) and t.strip():
            return t.strip()
    return None


def _is_mostly_chinese(s: str) -> bool:
    """判断字符串是否以中文为主（中文之间不该插空格）。内部小工具。"""
    if not s:
        return False
    cn = sum(1 for ch in s if "\u4e00" <= ch <= "\u9fff")
    return cn >= max(1, len(s) * 0.5)


class ASREngine:
    """
    语音识别引擎（FunASR + 简化端点检测 + 全自动降级）。

    典型用法
    --------
        asr = ASREngine()                       # 真机：自动加载 paraformer-zh
        text = asr.recognize_once()             # 录一段话 -> 文本（超时返回 None）
        if text:
            print("用户说：", text)
        asr.close()

        # 没有模型 / 没有麦克风的环境（比如答辩前的笔记本、CI、沙盒）：
        asr = ASREngine(mock=True)              # 或让它在加载失败时自动降级
        text = asr.recognize_once()             # 这里会提示你用键盘敲一句话

    主控程序里建议这样写（保证任何环境都能跑）：
        asr = ASREngine(mock=not HAS_MODEM, hotwords="感冒灵 布洛芬 阿莫西林")
    """

    def __init__(self, model: str = "paraformer-zh", device: str = "cpu",
                 mock: bool = False, sample_rate: int = 16000,
                 vad_model: str = "fsmn-vad", punc_model: str = "ct-punc",
                 input_device=None, hotwords: str = ""):
        """
        参数
        ----
        model        : FunASR 模型名。默认 'paraformer-zh'（中文非流式，识别率好）。
                       想做实时上屏可换 'paraformer-zh-streaming'。
        device       : 'cpu' 或 'cuda'（有 N 卡就写 'cuda:0'）
        mock         : True 时完全不加载模型，recognize_once() 用 input() 读一行
                       键盘输入代替识别，方便无模型环境跑通全流程
        sample_rate  : 模型要求的采样率，paraformer 系列固定 16000
        vad_model    : 模型侧 VAD（切长音频），如 'fsmn-vad'。留空 = 不用
        punc_model   : 标点恢复模型，如 'ct-punc'，让文本带标点。留空 = 不用
        input_device : 录音输入设备：int 序号或设备名子串；None = 系统默认。
                       用 ASREngine.list_input_devices() 查看可选设备
        hotwords     : 热词，空格分隔，例如 "感冒灵 布洛芬 阿莫西林 硝苯地平"。
                       也可以是热词文件路径（.txt，每行一个，可带权重），
                       只要该路径真实存在就会按文件处理。
        """
        self.model_name = str(model)
        self.device = str(device)
        self.sample_rate = int(sample_rate)
        self.hotwords = (hotwords or "").strip()

        # 融合自另一套方案：模型侧 VAD + 标点恢复（加载失败自动降级，见 _make_model）
        self.vad_model = (vad_model or "").strip()
        self.punc_model = (punc_model or "").strip()
        # 录音设备选择：None=系统默认；int=设备序号；str=名字子串（resolve 时已转成 int）
        self.input_device = input_device

        self.mock = bool(mock)
        self._model = None                  # funasr AutoModel 实例
        self._stream = None                 # sounddevice 输入流
        self._stream_sr = self.sample_rate  # 实际录音采样率（可能与请求值不同）
        self._last_error: Optional[Exception] = None

        self.noise_floor = 0.0              # 最近一次估计的底噪 RMS（调试用）
        self.speech_threshold = 0.02        # 最近一次使用的能量阈值（调试用）

        # 没装 sounddevice 时也只能走键盘输入，这里一次性提示清楚
        if sd is None and not self.mock:
            print("[ASR] 未检测到 sounddevice，麦克风不可用 -> 录音将退化为键盘输入")
            print("      安装：pip install sounddevice")

        if self.mock:
            print("[ASR] mock 模式：不加载模型，recognize_once() 用键盘输入代替识别")
        else:
            self._load_model()

    # ======================================================================
    # 模型加载
    # ======================================================================
    def _load_model(self) -> bool:
        """
        加载 FunASR 模型。

        返回
        ----
        True 加载成功；False 表示已降级为 mock（异常信息已打印）
        """
        try:
            # 延迟 import：funasr 会拉起 torch，放这里避免拖慢模块导入
            from funasr import AutoModel
        except Exception as e:
            self._last_error = e
            _print_funasr_help(e, self.model_name, import_error=True)
            self.mock = True
            return False

        # 第一次：主模型 + （可选）模型侧 VAD / 标点恢复
        try:
            self._model = self._make_model(AutoModel)
            print("[ASR] 模型已加载：%s（device=%s）" % (self.model_name, self.device))
            if self.hotwords:
                print("[ASR] 已启用热词：%s" % (self.hotwords[:60],))
            return True
        except Exception as e:
            self._last_error = e
            # fsmn-vad / ct-punc 要额外下载，网络不通或缓存不全时会把【整个 ASR】
            # 拖成 mock —— 代价太大（识别本身明明是好的）。所以降级重试一次：
            # 去掉附加模型，只加载主模型，保住"能识别"这个底线能力。
            if self.vad_model or self.punc_model:
                dropped = [x for x in (self.vad_model, self.punc_model) if x]
                print("[ASR] 带附加模型加载失败（%r）" % (e,))
                print("      -> 降级重试：只加载主模型（暂时没有 %s，识别不受影响）"
                      % "/".join(dropped))
                self.vad_model = ""
                self.punc_model = ""
                try:
                    self._model = self._make_model(AutoModel)
                    print("[ASR] 模型已加载：%s（device=%s，无 VAD/标点）"
                          % (self.model_name, self.device))
                    return True
                except Exception as e2:
                    self._last_error = e2
                    _print_funasr_help(e2, self.model_name, import_error=False)
                    self._model = None
                    self.mock = True
                    return False
            _print_funasr_help(e, self.model_name, import_error=False)
            self._model = None
            self.mock = True
            return False

    def _make_model(self, AutoModel):
        """
        按当前参数构造 FunASR AutoModel。

        单独抽出来是为了让"附加模型（fsmn-vad / ct-punc）加载失败时，
        去掉它们再试一次"这个兜底逻辑能复用同一段构造代码。
        """
        kwargs = {"model": self.model_name, "device": self.device}
        # 融合自另一套方案：加上模型侧 VAD 与标点恢复，文本更干净
        if self.vad_model:
            kwargs["vad_model"] = self.vad_model
        if self.punc_model:
            kwargs["punc_model"] = self.punc_model
        try:
            # disable_update=True：跳过"检查 funasr 是否有新版本"（联网+慢+没用）
            return AutoModel(**kwargs, disable_update=True)
        except TypeError:                       # 老版本 funasr 不认识这个参数
            return AutoModel(**kwargs)

    # ======================================================================
    # 输入设备选择（--list-mics / --mic-device 用）
    # ======================================================================
    @staticmethod
    def list_input_devices() -> List[dict]:
        """
        列出所有可用的录音输入设备。

        返回
        ----
        [{"index": int, "name": str, "sr": int, "ch": int}, ...]；无设备返回 []
        """
        if sd is None:
            return []
        out: List[dict] = []
        try:
            default_idx = -1
            try:
                d = sd.query_devices(kind="input")
                default_idx = int(d.get("index", -1))
            except Exception:
                pass
            for i, dev in enumerate(sd.query_devices()):
                if int(dev.get("max_input_channels", 0)) <= 0:
                    continue
                out.append({
                    "index": i,
                    "name": str(dev.get("name", "?")).strip(),
                    "sr": int(float(dev.get("default_samplerate", 0) or 0)),
                    "ch": int(dev.get("max_input_channels", 0)),
                })
                # query_devices 枚举序号即设备号；默认设备已在上面记下
                if i == default_idx and out:
                    out[-1]["_is_default"] = True
        except Exception:
            pass
        return out

    @staticmethod
    def default_input_device() -> Optional[dict]:
        """返回系统默认输入设备的信息 dict；没有返回 None"""
        if sd is None:
            return None
        try:
            d = sd.query_devices(kind="input")
            idx = int(d.get("index", -1))
            for x in ASREngine.list_input_devices():
                if x["index"] == idx:
                    return x
            if idx >= 0:
                return {"index": idx, "name": str(d.get("name", "?")),
                        "sr": int(float(d.get("default_samplerate", 0) or 0)),
                        "ch": int(d.get("max_input_channels", 0))}
        except Exception:
            pass
        return None

    @staticmethod
    def resolve_input_device(spec) -> Optional[int]:
        """
        把 --mic-device 参数解析成设备序号。

        spec 可以是：
          int / 数字字符串  -> 直接当设备序号（校验存在性）
          名字子串          -> 第一个名字包含它的输入设备（大小写不敏感）

        返回设备序号 int；找不到返回 None 并打印候选清单。
        """
        devs = ASREngine.list_input_devices()
        if not devs:
            print("[ASR] 没有检测到任何输入设备")
            return None
        # 数字 -> 序号
        try:
            idx = int(str(spec).strip())
            for x in devs:
                if x["index"] == idx:
                    return idx
            print("[ASR] 设备序号 %d 不存在。可用设备：" % idx)
            for x in devs:
                print("  %3d  %s" % (x["index"], x["name"]))
            return None
        except ValueError:
            pass
        # 名字子串
        key = str(spec).strip().lower()
        for x in devs:
            if key in x["name"].lower():
                return x["index"]
        print("[ASR] 没有名字包含 %r 的输入设备。可用设备：" % spec)
        for x in devs:
            print("  %3d  %s" % (x["index"], x["name"]))
        return None

    # ======================================================================
    # 对外接口
    # ======================================================================
    def recognize_once(self, timeout: float = 6.0,
                       phrase_timeout: float = 1.2) -> Optional[str]:
        """
        从麦克风录一段话并返回识别文本。

        流程：打开麦克风 -> 端点检测自动判断起止 -> 送模型 -> 返回文本

        参数
        ----
        timeout       : 等待"用户开始说话"的最长秒数，超时返回 None
        phrase_timeout: 说话过程中，静音超过这么久就认为说完了

        返回
        ----
        识别出的文本（已 strip）；超时/没声音/识别失败返回 None。
        mock 模式或缺麦克风时用 input() 读一行代替。
        """
        # 键盘兜底：mock 模式 或 没装 sounddevice
        if self.mock or sd is None:
            return self._read_from_keyboard()

        audio = self._record_until_silence(timeout=timeout,
                                           phrase_timeout=phrase_timeout)
        if audio is None or audio.size == 0:
            return None

        text = self._infer_array(audio)
        if text:
            print("[ASR] 识别结果：%s" % text)
        return text

    def recognize_file(self, path: str) -> Optional[str]:
        """
        识别一个 wav 文件的内容。

        参数
        ----
        path : wav 文件路径（16k 单声道最佳，其它采样率 FunASR 会自动处理）

        返回
        ----
        识别文本；文件不存在 / 无模型 / 识别失败返回 None
        """
        if not path or not os.path.isfile(path):
            print("[ASR] 文件不存在：%r" % (path,))
            return None
        if self._model is None:
            print("[ASR] 当前无可用模型（mock 模式），无法识别文件：%s" % path)
            return None
        try:
            res = self._generate(path)
            text = parse_funasr_result(res)
            print("[ASR] 文件识别结果：%s" % text)
            return text
        except Exception as e:
            print("[ASR] 文件识别失败（已忽略）：%r" % (e,))
            return None

    def close(self) -> None:
        """释放模型与音频流资源（可重复调用）。"""
        self._close_stream()
        self._model = None

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
        """当前是否处于 mock（无真实模型）状态"""
        return self.mock or self._model is None

    def __repr__(self) -> str:
        return "ASREngine(model=%s, device=%s, mock=%s, sr=%d)" % (
            self.model_name, self.device, self.is_mock, self.sample_rate)

    # ======================================================================
    # 键盘兜底（mock 模式）
    # ======================================================================
    def _read_from_keyboard(self) -> Optional[str]:
        """
        用键盘输入代替麦克风识别，保证无模型/无麦克风环境也能跑通全流程。

        返回
        ----
        输入的一行文本；空行或标准输入不可用时返回 None
        """
        try:
            s = input("[ASR] 请用键盘输入一句话代替语音（直接回车=放弃）> ")
        except EOFError:
            # 非交互环境（如管道/沙盒）没有 stdin，不能让程序崩
            print("[ASR] 标准输入不可用，返回 None")
            return None
        s = (s or "").strip()
        return s or None

    # ======================================================================
    # 录音 + 端点检测
    # ======================================================================
    def _open_stream(self, frame_len: int):
        """
        打开麦克风输入流。

        先按 self.sample_rate（16k）尝试；设备不支持时退到设备默认采样率，
        并在录音结束后重采样（见 _resample）。

        参数
        ----
        frame_len : 每次 read() 读取的采样点数

        返回
        ----
        已启动的 sounddevice.InputStream；失败返回 None
        """
        last_err: Optional[Exception] = None
        for sr in (self.sample_rate, self._device_default_sr()):
            if sr is None:
                continue
            try:
                st = sd.InputStream(samplerate=int(sr), channels=1,
                                    dtype="float32", blocksize=frame_len,
                                    device=self.input_device)
                st.start()
                self._stream_sr = int(sr)
                if sr != self.sample_rate:
                    print("[ASR] 麦克风不支持 %dHz，已按 %dHz 录音，"
                          "识别前会重采样到 %dHz" % (self.sample_rate, sr, self.sample_rate))
                return st
            except Exception as e:
                last_err = e
                continue
        print("[ASR] 无法打开麦克风：%r" % (last_err,))
        print("      检查：1) 系统是否允许 Python 使用麦克风（Windows 隐私设置）")
        print("            2) 是否被其它软件独占占用")
        print("            3) pip install sounddevice 是否成功")
        return None

    def _device_default_sr(self) -> Optional[int]:
        """查询默认输入设备的采样率；查询失败返回 None"""
        try:
            info = sd.query_devices(kind="input")
            return int(float(info.get("default_samplerate", 0))) or None
        except Exception:
            return None

    def _close_stream(self) -> None:
        """关闭并释放音频输入流（异常静默）"""
        if self._stream is None:
            return
        try:
            self._stream.stop()
        except Exception:
            pass
        try:
            self._stream.close()
        except Exception:
            pass
        self._stream = None

    def _record_until_silence(self, timeout: float,
                              phrase_timeout: float) -> Optional["np.ndarray"]:
        """
        带端点检测的录音：一直录到"用户说完"为止，而不是固定时长。

        参数
        ----
        timeout       : 等待开始说话的最长秒数（超时返回 None）
        phrase_timeout: 说完后静音多久判定结束（秒）

        返回
        ----
        float32 一维数组（已重采样到 self.sample_rate）；
        没等到说话 / 录音失败返回 None
        """
        frame_len = max(1, int(self._stream_sr * _FRAME_SEC))
        stream = self._open_stream(frame_len)
        if stream is None:
            return None
        self._stream = stream

        n_calib = max(1, int(round(_CALIB_SEC / _FRAME_SEC)))          # 底噪帧数
        preroll_len = max(1, int(round(_PREROLL_SEC / _FRAME_SEC)))    # 预缓冲帧数
        preroll: Deque["np.ndarray"] = collections.deque(maxlen=preroll_len)

        calib: List[float] = []
        threshold: Optional[float] = None
        speech: List["np.ndarray"] = []
        started = False
        loud_streak = 0            # 连续超阈值的帧数（抗单次噪声）
        start_at = 0.0
        last_loud_at = 0.0
        t0 = time.time()

        print("[ASR] 请说话……（正在估计环境底噪）")
        try:
            while True:
                now = time.time()
                if not started and (now - t0) > timeout:
                    print("[ASR] 等待说话超时（%.1fs），返回 None" % timeout)
                    return None
                if started and (now - start_at) > _MAX_RECORD_SEC:
                    print("[ASR] 达到单次录音上限 %.0fs，强制结束" % _MAX_RECORD_SEC)
                    break
                if started and (now - last_loud_at) > phrase_timeout:
                    break

                data, _overflowed = stream.read(frame_len)
                x = np.asarray(data, dtype=np.float32).reshape(-1)
                if x.size == 0:
                    continue
                rms = float(np.sqrt(np.mean(x * x)))
                preroll.append(x)

                # ---- 阶段1：估计底噪，确定自适应阈值 ----
                if threshold is None:
                    calib.append(rms)
                    if len(calib) >= n_calib:
                        noise = float(np.mean(calib))
                        sigma = float(np.std(calib))
                        # 三个下限各有作用，详见文件头"设计要点 2"
                        threshold = max(noise * 3.0,
                                        noise + 3.0 * sigma + 1e-9,
                                        noise + 0.01,
                                        0.006)
                        self.noise_floor = noise
                        self.speech_threshold = threshold
                        print("[ASR] 底噪 RMS=%.5f -> 说话阈值=%.5f（开始监听）"
                              % (noise, threshold))
                    continue

                # ---- 阶段2：等待开始说话 ----
                if not started:
                    if rms > threshold:
                        loud_streak += 1
                        if loud_streak >= 2:          # 连续 2 帧才算开始，抗敲击声
                            started = True
                            start_at = time.time()
                            last_loud_at = start_at
                            speech.extend(preroll)    # 补上预缓冲，保住第一个字
                            print("[ASR] 检测到说话，开始录音……")
                    else:
                        loud_streak = 0
                    continue

                # ---- 阶段3：正在说话，记录直到静音超时 ----
                speech.append(x)
                if rms > threshold:
                    last_loud_at = time.time()

        except Exception as e:
            print("[ASR] 录音过程异常（已忽略）：%r" % (e,))
            return None
        finally:
            self._close_stream()

        if not speech:
            return None

        audio = np.concatenate(speech).astype(np.float32)
        dur = audio.size / float(self._stream_sr)
        print("[ASR] 录音结束：%.2f 秒" % dur)
        if dur < 0.2:                      # 太短多半是误触发（咳嗽/碰撞）
            print("[ASR] 录音过短（<0.2s），判定为噪声，返回 None")
            return None

        if self._stream_sr != self.sample_rate:
            audio = self._resample(audio, self._stream_sr, self.sample_rate)
        return audio

    @staticmethod
    def _resample(data: "np.ndarray", src_sr: int, dst_sr: int) -> "np.ndarray":
        """
        线性插值重采样（stdlib 之外不引入依赖）。

        参数
        ----
        data   : 一维 float32 音频
        src_sr : 原始采样率
        dst_sr : 目标采样率

        返回
        ----
        重采样后的一维 float32 数组。语音识别用，线性插值的失真可忽略。
        """
        if src_sr == dst_sr or data.size == 0:
            return data.astype(np.float32)
        n_out = int(round(data.size * float(dst_sr) / float(src_sr)))
        if n_out <= 1:
            return data.astype(np.float32)
        x_old = np.linspace(0.0, 1.0, num=data.size, endpoint=False, dtype=np.float64)
        x_new = np.linspace(0.0, 1.0, num=n_out, endpoint=False, dtype=np.float64)
        return np.interp(x_new, x_old, data.astype(np.float64)).astype(np.float32)

    # ======================================================================
    # 推理
    # ======================================================================
    def _generate(self, input_data):
        """
        调用 FunASR 的 generate()，并兼容"老版本不支持 hotword 参数"的情况。

        参数
        ----
        input_data : wav 文件路径，或 float32 音频数组

        返回
        ----
        generate() 的原始返回值
        """
        kwargs = {}
        if self.hotwords:
            kwargs["hotword"] = self.hotwords
        try:
            return self._model.generate(input=input_data, **kwargs)
        except TypeError:
            # 少数版本的 generate 不接受 hotword，去掉再试一次
            return self._model.generate(input=input_data)

    def _infer_array(self, audio: "np.ndarray") -> Optional[str]:
        """
        识别一段内存中的音频。

        为什么要先写成临时 wav 再送进去，而不是直接传 ndarray？
        因为 FunASR 各版本对 ndarray 输入的采样率参数（fs）支持不一致，
        传路径是唯一在所有版本上都稳的写法，顺带还能留个调试音频。

        参数
        ----
        audio : float32 一维数组，采样率 self.sample_rate

        返回
        ----
        识别文本或 None
        """
        if self._model is None:
            return None
        tmp_path = None
        try:
            tmp_path = self._write_temp_wav(audio, self.sample_rate)
            res = self._generate(tmp_path)
            return parse_funasr_result(res)
        except Exception as e:
            print("[ASR] 推理失败（已忽略）：%r" % (e,))
            return None
        finally:
            if tmp_path and os.path.exists(tmp_path):
                try:
                    os.remove(tmp_path)
                except Exception:
                    pass

    @staticmethod
    def _write_temp_wav(audio: "np.ndarray", sample_rate: int) -> str:
        """
        把 float32 音频写成 16bit PCM 临时 wav 文件（只用标准库 wave）。

        参数
        ----
        audio       : float32 音频，取值建议在 [-1, 1]
        sample_rate : 采样率

        返回
        ----
        临时文件路径（调用方负责删除）
        """
        fd, path = tempfile.mkstemp(prefix="medbox_asr_", suffix=".wav")
        os.close(fd)
        # 限幅后转 int16，避免溢出导致爆音
        pcm = np.clip(audio, -1.0, 1.0)
        pcm = (pcm * 32767.0).astype("<i2")
        with wave.open(path, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(int(sample_rate))
            wf.writeframes(pcm.tobytes())
        return path


# ===========================================================================
# 自测：python3.11 -c "import sys; sys.path.insert(0,'pc'); from audio.asr import ASREngine; ..."
# ===========================================================================
def _selftest() -> None:
    """无硬件环境下的最小自测：构造引擎 -> 解析各种 FunASR 返回结构"""
    eng = ASREngine(mock=True)
    print("engine =", eng)
    print("is_mock =", eng.is_mock)

    # 三种真实存在的 FunASR 返回结构都要能解析
    print(parse_funasr_result([{"key": "wav1", "text": "我感冒了"}]))
    print(parse_funasr_result([{"key": "wav1", "value": "头疼"}]))
    print(parse_funasr_result("拿一盒布洛芬"))
    print(parse_funasr_result(None))

    # 重采样自检
    x = np.sin(np.linspace(0, 20, 4800)).astype(np.float32)
    print("resample:", x.size, "->", ASREngine._resample(x, 48000, 16000).size)
    eng.close()


if __name__ == "__main__":
    _selftest()
    sys.exit(0)
