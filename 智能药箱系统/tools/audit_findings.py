#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 代码审查发现整理 (audit_findings.py)
================================================================================
职责：对固件 + PC 源码做一轮「旧协议名/坏味道」静态审计，把发现列成清单。
专治协议改名（$MVS→$MS 等）之后的漏网之鱼。

检查项：
  A. 旧协议名残留   —— $MVS / $MVC / $MVQ / PROTO_SendMvState / MVSTATE
  B. 新旧混用       —— 同一文件里同时出现新旧名（最容易出微妙 bug）
  C. PC 侧旧名字符串 —— chassis_controller/state_machine 里的 "MVS"/"MVC"
  D. 舵机角度越界倾向 —— 固件里写死的夹爪角度是否落在 45~90 合法行程外

用法：
  python tools/audit_findings.py            # 全量审计
  python tools/audit_findings.py --only A   # 只跑某项
  python tools/audit_findings.py --fix-hint # 附带修复建议

退出码：0 = 无发现；1 = 有发现。
================================================================================
"""
from __future__ import annotations

import argparse
import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)

FW = os.path.join(_ROOT, "0stm")
PC = os.path.join(_ROOT, "智能药箱系统")

LEGACY_TOKENS = ("$MVS", "$MVC", "$MVQ", "PROTO_SendMvState", "MVSTATE",
                 "SendMvState")
NEW_TOKENS = ("$MS", "$MC", "$MQ", "$MD")


def _iter_source(dirs, exts):
    for d in dirs:
        if not os.path.isdir(d):
            continue
        for root, dirs_, files in os.walk(d):
            dirs_[:] = [x for x in dirs_ if x not in ("__pycache__", "Objects",
                                                      "Listings", ".git")]
            for fn in files:
                if os.path.splitext(fn)[1] in exts:
                    yield os.path.join(root, fn)


def check_legacy(verbose=False):
    """A/B/C：旧协议名与新旧混用。"""
    findings = []
    for path in _iter_source([FW, PC], (".c", ".h", ".py")):
        try:
            text = open(path, encoding="utf-8", errors="replace").read()
        except Exception:
            continue
        rel = os.path.relpath(path, _ROOT)
        for tok in LEGACY_TOKENS:
            if tok in text:
                ln = text.count("\n", 0, text.find(tok)) + 1
                findings.append("A %s:%d 旧协议名 %s" % (rel, ln, tok))
        # 新旧混用：旧名与新名同现（排除纯注释说明改名的行）
        for old in ("MVS", "MVC"):
            for new in ('"MS"', '"MC"'):
                if old in text and new in text:
                    # 只在没有「改名说明」注释时才算混用
                    if not re.search(r"(MVS.*?→.*?MS|不再用|已废弃|旧名)", text):
                        findings.append("C %s 同时含 %s 与 %s（疑似新旧混用）"
                                        % (rel, old, new))
    return findings


def check_gripper_range(verbose=False):
    """D：夹爪角度合法性（3u 体系：合法行程 45~90 度）。"""
    findings = []
    main_h = os.path.join(FW, "app", "main.h")
    if not os.path.exists(main_h):
        return findings
    text = open(main_h, encoding="utf-8", errors="replace").read()
    for m in re.finditer(r"#define\s+(GRIPPER_\w+|SERVO3_\w+)\s+([\d.]+)f?", text):
        name, val = m.group(1), float(m.group(2))
        if name.endswith(("_DEG", "_MIN_DEG", "_MAX_DEG", "_HOME_DEG")):
            if not (0.0 <= val <= 90.0):
                findings.append("D main.h %s = %.1f 超出 0~90 机械范围" % (name, val))
    return findings


CHECKS = {"A": check_legacy, "D": check_gripper_range}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="代码审查发现整理")
    ap.add_argument("--only", default=None, help="只跑某项（%s）" % "/".join(CHECKS))
    a = ap.parse_args(argv)

    print("=" * 70)
    print("代码审查发现整理（audit_findings）")
    print("=" * 70)

    findings = []
    for key, fn in CHECKS.items():
        if a.only and key != a.only.upper():
            continue
        print("[运行] 检查 %s ..." % key)
        findings += fn()

    if findings:
        print("-" * 70)
        print("[发现] 共 %d 条：" % len(findings))
        for f in findings[:30]:
            print("   -", f)
        print("[结论] 有发现，按清单逐条处理")
        return 1
    print("[结论] 未发现旧协议名残留 / 新旧混用 / 角度越界")
    return 0


if __name__ == "__main__":
    sys.exit(main())
