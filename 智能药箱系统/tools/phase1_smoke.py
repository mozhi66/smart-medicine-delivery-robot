#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 Phase-1 冒烟测试 (phase1_smoke.py)
================================================================================
职责：环境与模型装好之后的「彩排」——不碰串口、不碰机械臂、不碰摄像头，
按顺序验证四段链路，任何一段挂都能精确定位到环节：

  0) 配置与解释器   —— app.yaml 可读、药品库可载、不是错的解释器
  1) 大模型客户端   —— 供应商/模型/Key/本地规则兜底（★ 09-29 原版逻辑）
  2) 麦克风 + ASR   —— 选设备、录一小段、FunASR 认出文字（可跳过）
  3) TTS 播报       —— 说一句话确认喇叭（可跳过）
  4) 知识库         —— 类别白名单、库存扣减模拟

用法：
  python tools/phase1_smoke.py                 # 全流程（会问你选麦克风）
  python tools/phase1_smoke.py --no-ask        # 不交互，用默认设备
  python tools/phase1_smoke.py --offline       # 强制本地规则（没 Key 时）
  python tools/phase1_smoke.py --skip-mic      # 跳过麦克风/ASR 段
  python tools/phase1_smoke.py --skip-tts      # 跳过 TTS 段

退出码：0 = 全过（或跳过的段不算失败）；1 = 有失败段。
================================================================================
"""
from __future__ import annotations

import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, os.path.join(_ROOT, "pc")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

_CFG = os.path.join(_ROOT, "config", "app.yaml")


# --------------------------------------------------------------------------
# 配置读取小组件
# --------------------------------------------------------------------------
def _read_llm_cfg() -> dict:
    """读 app.yaml 的 llm 段（缺 yaml 库时返回空 dict）。"""
    try:
        import yaml
        d = yaml.safe_load(open(_CFG, encoding="utf-8")) or {}
        return d.get("llm") or {}
    except Exception:
        return {}


def _read_provider():
    """返回 (provider, key_env, endpoint, model)——换供应商后这里跟着变。"""
    llm = _read_llm_cfg()
    return (llm.get("provider", "zhipu"),
            llm.get("api_key_env", "ZHIPU_API_KEY"),
            llm.get("base_url", ""),
            llm.get("model", "glm-4-flash"))


def _inline_key() -> str:
    """app.yaml 里内联的 api_key（没有则空串）。"""
    return str(_read_llm_cfg().get("api_key") or "")


def _line(title: str) -> None:
    print()
    print("-" * 70)
    print(title)
    print("-" * 70)


def choose_mic_interactive() -> int:
    """列出输入设备让用户选，返回设备 index。"""
    import sounddevice as sd
    devs = [d for d in sd.query_devices() if d.get("max_input_channels", 0) > 0]
    if not devs:
        print("[设备] 未发现输入设备，用系统默认")
        return 0
    print("[设备] 可用麦克风：")
    for i, d in enumerate(devs):
        print("   %d) %s（%d 通道）" % (i, d.get("name", "?"),
                                        d.get("max_input_channels", 0)))
    try:
        sel = input("请选择麦克风编号 [0]: ").strip()
        return int(sel) if sel else 0
    except Exception:
        return 0


# --------------------------------------------------------------------------
# 主流程
# --------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Phase-1 冒烟测试")
    ap.add_argument("--offline", action="store_true", help="强制本地规则（mock）")
    ap.add_argument("--no-ask", action="store_true", help="不交互选设备")
    ap.add_argument("--mic-device", type=int, default=None, help="直接指定麦克风 index")
    ap.add_argument("--skip-mic", action="store_true", help="跳过麦克风/ASR 段")
    ap.add_argument("--skip-tts", action="store_true", help="跳过 TTS 段")
    args = ap.parse_args(argv)

    print("=" * 70)
    print("Phase-1 冒烟测试（不碰硬件外设）")
    print("=" * 70)
    failed = []

    # ---------------- 0) 配置与解释器 ----------------
    _line("0. 配置与解释器")
    if not os.path.exists(_CFG):
        print("[FAIL] 找不到 config/app.yaml")
        return 1
    print("[ OK ] app.yaml 存在")
    low = sys.executable.replace("\\", "/").lower()
    if "anaconda" in low or "miniconda" in low:
        print("[WARN] 正在用 Anaconda 基础环境——真 ASR 请换 Python 3.10")
    else:
        print("[ OK ] 解释器: %s" % sys.executable)

    try:
        from core.medicine_kb import MedicineKB
        from llm.glm_client import GLMClient
    except Exception as e:
        print("[FAIL] PC 模块导入失败: %s" % e)
        return 1
    print("[ OK ] PC 模块可导入")

    # ---------------- 麦克风设备选择（供第 2 段用）----------------
    mic_dev = None
    if not args.skip_mic:
        if args.mic_device is not None:
            mic_dev = args.mic_device
            print("[设备] 已按 --mic-device 指定：%s 号" % mic_dev)
        elif not args.no_ask:
            mic_dev = choose_mic_interactive()
        else:
            print("[设备] 跳过选择（--no-ask），用系统默认输入设备")

    # ---------------- 1) 大模型侧 ----------------
    kb = MedicineKB.load(os.path.join(_ROOT, "config", "medicines.json"))
    cats = kb.categories()

    # 读取当前配置的供应商（换 DeepSeek/通义/… 后这里会跟着变）
    provider, key_env, endpoint, model = _read_provider()
    has_key = bool(os.environ.get(key_env)) or bool(_inline_key())
    use_mock = bool(args.offline) or not has_key

    _line("1. 大模型客户端")
    print("  候选类别白名单（%d）: %s" % (len(cats), "、".join(cats)))
    print("  供应商  : %s   （config/app.yaml 的 llm.provider）" % provider)
    print("  接口    : %s" % (endpoint or "(空！)"))
    print("  模型    : %s" % (model or "(空)"))
    print("  API Key : %s" % ("已设置" if has_key else "未设置（将走本地规则）"))
    print("  模式    : %s" % ("本地规则(mock)" if use_mock else "云端真实调用（%s）" % provider))
    if use_mock and not args.offline:
        print("  [提示] 没设 %s，本次只验证本地规则；设好 Key 去掉 --offline 再跑。"
              % key_env)

    glm = GLMClient(categories=cats, model=model or None, provider=provider,
                    base_url=endpoint.rsplit("/chat/completions", 1)[0] or None,
                    api_key=(_inline_key() or None),
                    api_key_env=key_env,
                    timeout=float(_read_llm_cfg().get("timeout") or 12),
                    retries=int(_read_llm_cfg().get("retries") or 2),
                    mock=use_mock)
    try:
        r = glm.map_symptom("我感冒了，鼻子不通气")
        if r.get("ok") and r.get("category"):
            print("[ OK ] 语义映射: 「感冒」-> %s（source=%s）"
                  % (r["category"], r.get("source")))
        else:
            print("[WARN] 映射结果不可用: %s" % r)
    except Exception as e:
        print("[FAIL] 语义映射异常: %s" % e)
        failed.append("大模型")

    # ---------------- 2) 麦克风 + ASR ----------------
    if args.skip_mic:
        _line("2. 麦克风 + ASR（已按 --skip-mic 跳过）")
    else:
        _line("2. 麦克风 + ASR")
        try:
            from audio.asr import ASREngine
            print("[信息] 加载 FunASR 模型（首次较慢，耐心等）...")
            t0 = time.time()
            asr = ASREngine()
            print("[ OK ] 模型就绪（%.1fs）" % (time.time() - t0))
            print("[信息] 请对着麦克风说一句，比如「我感冒了」（6 秒内）...")
            text = asr.recognize_once(timeout=6.0)
            if text:
                print("[ OK ] 识别结果: %s" % text)
            else:
                print("[WARN] 没听到有效语音（可能是设备选错或环境太吵）")
        except Exception as e:
            print("[WARN] ASR 段失败: %s" % e)
            print("       不影响后面段落；真 ASR 需要 funasr/torch + Python 3.10")

    # ---------------- 3) TTS 播报 ----------------
    if args.skip_tts:
        _line("3. TTS 播报（已按 --skip-tts 跳过）")
    else:
        _line("3. TTS 播报")
        try:
            from audio.tts import TTSEngine
            tts = TTSEngine(backend="auto")
            if tts.speak(" Phase 1 冒烟测试，语音播报正常 ", wait=True):
                print("[ OK ] 播报完成（后端: %s）" % getattr(tts, "backend", "?"))
            else:
                print("[WARN] 播报返回 False（没装 pyttsx3/edge-tts 或无声卡）")
        except Exception as e:
            print("[WARN] TTS 段失败: %s" % e)

    # ---------------- 4) 知识库 ----------------
    _line("4. 知识库")
    try:
        if not cats:
            print("[FAIL] 类别白名单为空——检查 config/medicines.json")
            failed.append("知识库")
        else:
            print("[ OK ] 类别 %d 个: %s" % (len(cats), "、".join(cats)))
        meds = getattr(kb, "medicines", [])
        print("[ OK ] 药品 %d 条" % len(meds))
    except Exception as e:
        print("[FAIL] 知识库异常: %s" % e)
        failed.append("知识库")

    # ---------------- 总结 ----------------
    print("=" * 70)
    if failed:
        print("[结论] 失败段: %s —— 按上面日志逐段排查" % ", ".join(failed))
        return 1
    print("[结论] Phase-1 冒烟通过：配置/大模型/知识库全 OK")
    print("       下一步接硬件：python pc/main.py（或先 check_env.py 验收）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
