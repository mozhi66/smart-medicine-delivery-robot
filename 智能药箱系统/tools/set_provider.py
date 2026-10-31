#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 LLM 供应商切换 (set_provider.py)
================================================================================
职责：一条命令把 config/app.yaml 的 llm 段切成指定供应商预设，
不用手翻 YAML、不用记哪个模型配哪家。

预设来源：pc/llm/providers.py 的 PRESETS（与代码共用一份，绝不漂移）。
切换内容：provider / base_url / model / api_key_env 四行；
          api_key（密钥）不动——它不属于预设，避免误覆盖你的 Key。

用法：
  python tools/set_provider.py                 # 看当前供应商 + 预设列表
  python tools/set_provider.py deepseek        # 切到 DeepSeek
  python tools/set_provider.py zhipu --model glm-4-flash   # 切并微调模型
  python tools/set_provider.py --set-key sk-xxxx  # 只写 api_key（不切供应商）

退出码：0 = 成功；1 = 失败。
================================================================================
"""
from __future__ import annotations

import argparse
import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, os.path.join(_ROOT, "pc")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

APP_YAML = os.path.join(_ROOT, "config", "app.yaml")

try:
    from llm.providers import PRESETS, DEFAULT_PROVIDER
except Exception:
    # 兼容包内/包外两种导入方式
    from pc.llm.providers import PRESETS, DEFAULT_PROVIDER


def _read_yaml_text() -> str:
    with open(APP_YAML, encoding="utf-8") as f:
        return f.read()


def _write_yaml_text(text: str) -> None:
    with open(APP_YAML, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def _current_llm(text: str) -> dict:
    """粗解析 llm: 段的键值（够用，不引 yaml 依赖）。"""
    out = {}
    m = re.search(r"^llm:\s*$", text, re.M)
    if not m:
        return out
    for line in text[m.end():].split("\n"):
        if line.strip() and not line.startswith((" ", "\t", "#")):
            break  # 下一个顶级段
        km = re.match(r"\s+([a-z_]+)\s*:\s*(.*)$", line)
        if km and not km.group(2).lstrip().startswith("#"):
            out[km.group(1)] = km.group(2).strip()
    return out


def _set_llm_field(text: str, key: str, value: str) -> str:
    """替换 llm: 段内某个键的值；键不存在则插入到 llm: 行之后。"""
    m = re.search(r"^llm:\s*$", text, re.M)
    if not m:
        raise RuntimeError("app.yaml 里找不到 llm: 段")
    seg_end = m.end()
    for line in text[m.end():].split("\n"):
        if line.strip() and not line.startswith((" ", "\t", "#")):
            break
        seg_end += len(line) + 1
    seg = text[m.end():seg_end]
    pat = re.compile(r"^(\s*%s\s*:\s*).*$" % re.escape(key), re.M)
    if pat.search(seg):
        seg = pat.sub(lambda mm: mm.group(1) + value, seg, count=1)
    else:
        seg = seg.rstrip("\n") + "\n  %s: %s\n" % (key, value)
    return text[:m.end()] + seg + text[seg_end:]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="LLM 供应商切换")
    ap.add_argument("provider", nargs="?", default=None,
                    help="目标供应商（%s）" % "/".join(sorted(PRESETS)))
    ap.add_argument("--model", default=None, help="覆盖预设的模型名")
    ap.add_argument("--set-key", default=None, help="只写 api_key，不切供应商")
    a = ap.parse_args(argv)

    if not os.path.exists(APP_YAML):
        print("[FAIL] 找不到 %s" % APP_YAML)
        return 1
    text = _read_yaml_text()
    cur = _current_llm(text)

    print("=" * 70)
    print("LLM 供应商切换")
    print("=" * 70)
    print("[当前] provider=%s model=%s base_url=%s"
          % (cur.get("provider", "?"), cur.get("model", "?"), cur.get("base_url", "?")))
    print("[当前] api_key: %s" % ("已设置" if cur.get("api_key") else "未设置（走环境变量）"))

    if a.set_key:
        text = _set_llm_field(text, "api_key", a.set_key)
        _write_yaml_text(text)
        print("[ OK ] api_key 已写入 app.yaml")
        return 0

    if not a.provider:
        print("-" * 70)
        print("[预设] 可用供应商：")
        for name in sorted(PRESETS):
            p = PRESETS[name]
            mark = "  <= 当前" if name == cur.get("provider") else ""
            print("   %-12s %-24s model=%-22s key环境变量=%s%s"
                  % (name, p["doc"], p["model"], p["api_key_env"] or "(不需要)", mark))
        print("-" * 70)
        print("[用法] python tools/set_provider.py <供应商名> [--model 模型名]")
        return 0

    name = a.provider.strip().lower()
    if name not in PRESETS:
        print("[FAIL] 未知供应商 '%s'（可选: %s）" % (name, "/".join(sorted(PRESETS))))
        return 1
    p = PRESETS[name]
    model = a.model or p["model"]

    try:
        for key, val in (("provider", name), ("base_url", p["base_url"]),
                         ("model", model), ("api_key_env", p["api_key_env"] or '""')):
            if val == "":
                continue
            text = _set_llm_field(text, key, val)
        _write_yaml_text(text)
    except Exception as e:
        print("[FAIL] 写入失败: %s" % e)
        return 1

    print("-" * 70)
    print("[ OK ] 已切换到 %s（%s）" % (name, p["doc"]))
    print("       model     = %s" % model)
    print("       base_url  = %s" % p["base_url"])
    print("       key环境变量 = %s" % (p["api_key_env"] or "(不需要)"))
    if p["api_key_env"]:
        print("[提示] 记得设置环境变量 %s 或用 --set-key 写入 app.yaml" % p["api_key_env"])
    print("[提示] 换完跑一下 phase1_smoke.py 验证连通性")
    return 0


if __name__ == "__main__":
    sys.exit(main())
