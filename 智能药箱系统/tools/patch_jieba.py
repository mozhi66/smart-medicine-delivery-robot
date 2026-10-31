#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 jieba 药品词库补丁 (patch_jieba.py)
================================================================================
职责：把 config/medicines.json 里的【药品名 / 类别 / 症状口语说法】写进
jieba 自定义词典，让「症状 -> 药品」的中文分词不把药名切碎。

典型场景：
  用户说「我头痛还有点咳嗽」——若 jieba 把「感冒灵」切成「感/冒/灵」，
  知识库匹配就会 miss。本脚本生成/更新 userdict 文件并给出加载方式。

产物：智能药箱系统/userdict.txt（jieba 格式：词条 词频 词性）
     词性统一用 nz（专有名词），不与 jieba 默认词典打架。

用法：
  python tools/patch_jieba.py            # 依据 medicines.json 生成 userdict
  python tools/patch_jieba.py --show     # 只打印将写入的词条，不落盘
  python tools/patch_jieba.py --test     # 落盘后做一次分词验证

退出码：0 = 成功；1 = 失败（如 medicines.json 不存在）。
================================================================================
"""
from __future__ import annotations

import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, os.path.join(_ROOT, "pc")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

MED_JSON = os.path.join(_ROOT, "config", "medicines.json")
USERDICT = os.path.join(_ROOT, "userdict.txt")

# 症状口语说法 -> 也进词典（ medicines.json 的 symptoms/aliases 字段若存在则取）
EXTRA_WORDS = ("感冒", "发烧", "头痛", "咳嗽", "拉肚子", "过敏", "擦伤", "消化不良")


def _load_medicines() -> list:
    with open(MED_JSON, encoding="utf-8") as f:
        d = json.load(f)
    return d.get("medicines") or []


def build_entries() -> list:
    """收集全部词条 -> [(词, 词频)]，按项目内出现重要性给权。"""
    freq = {}
    try:
        meds = _load_medicines()
    except Exception as e:
        print("[FAIL] 读不到 %s：%s" % (MED_JSON, e))
        raise
    for m in meds:
        name = str(m.get("name") or "").strip()
        if name:
            freq[name] = 200
        cat = str(m.get("category") or "").strip()
        if cat:
            freq[cat] = 120
        # 症状/口语说法（字段名兼容几种可能）
        for key in ("symptoms", "aliases", "say"):
            vals = m.get(key)
            if isinstance(vals, str):
                vals = [vals]
            for v in (vals or []):
                v = str(v).strip()
                if v:
                    freq[v] = 160
        note = str(m.get("note") or "").strip()
        # note 里太长的句子不进词典，只截前 8 字的短语（若有）
        if note and len(note) <= 8:
            freq[note] = 60
    for w in EXTRA_WORDS:
        freq.setdefault(w, 150)
    return sorted(freq.items(), key=lambda kv: (-kv[1], kv[0]))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="jieba 药品词库补丁")
    ap.add_argument("--show", action="store_true", help="只打印词条不落盘")
    ap.add_argument("--test", action="store_true", help="落盘后分词验证")
    a = ap.parse_args(argv)

    print("=" * 70)
    print("jieba 药品词库补丁")
    print("=" * 70)

    try:
        entries = build_entries()
    except Exception:
        return 1
    print("[信息] 收集词条 %d 个（来源: %s）" % (len(entries), MED_JSON))

    if a.show:
        for w, f in entries:
            print("   %-16s %d nz" % (w, f))
        return 0

    # 落盘（jieba 格式：词条 词频 词性）
    lines = ["%s %d nz" % (w, f) for w, f in entries]
    with open(USERDICT, "w", encoding="utf-8") as fp:
        fp.write("\n".join(lines) + "\n")
    print("[ OK ] 已写入 %s（%d 行）" % (USERDICT, len(lines)))

    if a.test:
        print("-" * 70)
        print("[验证] 加载 userdict 后分词：")
        try:
            import jieba
            jieba.load_userdict(USERDICT)
            demo = "我头痛还有点咳嗽，需要感冒药"
            print("   输入: %s" % demo)
            print("   输出: / ".join(jieba.lcut(demo)))
        except ImportError:
            print("[WARN] 未装 jieba，跳过分词验证（pip install jieba）")
            return 0
        except Exception as e:
            print("[FAIL] 分词验证出错: %s" % e)
            return 1
    print("-" * 70)
    print("[用法] 在 PC 代码里：jieba.load_userdict('userdict.txt')")
    return 0


if __name__ == "__main__":
    sys.exit(main())
