#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 FunASR 模型下载 (dl_models.py)
================================================================================
职责：把 ASR 三件套模型提前下到本地缓存，避免首次录音时才现下
（现下要等几分钟，用户体验极差）。

模型清单（与 pc/audio/asr.py 的默认参数一一对应）：
  paraformer-zh   语音识别主干（中文非流式）
  fsmn-vad        语音活动检测（切长音频、去静音）
  ct-punc         标点恢复（让识别文本带标号）

下载通道：ModelScope（modelscope.cn，国内快；FunASR 的默认 hub）。
模型缓存位置：ModelScope 默认 ~/.cache/modelscope/hub（可用
HF_HUB_CACHE / MODELSCOPE_CACHE 环境变量改）。

用法：
  python tools/dl_models.py                # 下全部三件套
  python tools/dl_models.py --only punc    # 只补某一个
  python tools/dl_models.py --list         # 只看清单不下载

退出码：0 = 全部就绪；1 = 有失败。
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

# 模型清单：key -> (ModelScope 模型 id, 用途, 体积量级)
MODELS = {
    "asr":  ("paraformer-zh", "语音识别主干（中文）", "~1GB"),
    "vad":  ("fsmn-vad",      "语音活动检测",         "~2MB"),
    "punc": ("ct-punc",       "标点恢复",             "~1GB"),
}


def _download(model_id: str) -> bool:
    """走 ModelScope 下载（FunASR 默认 hub）。"""
    try:
        from modelscope import snapshot_download
    except Exception as e:
        print("[FAIL] 未装 modelscope：%s" % e)
        print("       先跑：pip install modelscope")
        return False
    print(">> snapshot_download('%s') ..." % model_id)
    try:
        path = snapshot_download(model_id)
        print("[ OK ] 已缓存到: %s" % path)
        return True
    except Exception as e:
        print("[FAIL] 下载失败: %s" % e)
        return False


def _already_cached(model_id: str) -> bool:
    """粗判模型是否已在 ModelScope 缓存里（按目录名含模型名）。"""
    cache = os.environ.get("MODELSCOPE_CACHE") or os.path.join(
        os.path.expanduser("~"), ".cache", "modelscope", "hub")
    if not os.path.isdir(cache):
        return False
    tail = model_id.split("/")[-1]
    for root, dirs, _files in os.walk(cache):
        for d in dirs:
            if tail.lower() in d.lower():
                return True
    return False


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="FunASR 模型下载")
    ap.add_argument("--only", choices=sorted(MODELS), default=None,
                    help="只下载某一个（asr/vad/punc）")
    ap.add_argument("--list", action="store_true", help="只看清单")
    a = ap.parse_args(argv)

    print("=" * 70)
    print("FunASR 模型下载（ModelScope 通道）")
    print("=" * 70)
    keys = [a.only] if a.only else sorted(MODELS)
    for k in keys:
        mid, why, size = MODELS[k]
        cached = "（已缓存）" if _already_cached(mid) else ""
        print("  %-5s %-18s %-22s ~%s %s" % (k, mid, why, size, cached))

    if a.list:
        return 0

    failed = []
    for k in keys:
        mid, why, size = MODELS[k]
        print("-" * 70)
        print("[%s] %s（%s，~%s）" % (k, mid, why, size))
        if _already_cached(mid):
            print("[ OK ] 已在缓存，跳过（想强制重下就删 ~/.cache/modelscope 下对应目录）")
            continue
        if not _download(mid):
            failed.append(mid)

    print("=" * 70)
    if failed:
        print("[结论] %d 个模型失败: %s" % (len(failed), ", ".join(failed)))
        print("       国内网络可设镜像：export MODELSCOPE_DOMAIN=www.modelscope.cn")
        return 1
    print("[结论] 模型全部就绪。首次跑 ASR 还会现解压/校验，稍安勿躁。")
    print("       验收：python tools/check_env.py --quick")
    return 0


if __name__ == "__main__":
    sys.exit(main())
