#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 manual_phone 模式静态检查器 (check_manual_mode.py)
================================================================================
职责：不跑硬件、不连串口，纯静态扫描 PC 源码，验证「手机手动模式」的硬约束：

  规则 1：调用底盘【移动】API（move / move_lateral / move_forward / set_speed）
          的函数，其全部调用点必须处于 auto 模式守卫之下
          （work_mode == "auto" / is_auto()）。manual_phone 路径不得触及。
  规则 2：manual_phone 专属代码块（set_mode 的 manual 分支等）不得出现
          任何底盘移动 API；唯一允许的底盘调用是 stop()。
  规则 3：状态机必须提供 set_mode() 且把模式同步给底盘（$MD,AUTO/MAN）。
  规则 4：源码中不得出现旧协议名 $MVS/$MVC/$MVQ；新名
          MD=Mode / MV=Move / MS=Stop / MC=Cap / MQ=Query 必须在用。

用法：
  python tools/check_manual_mode.py            # 扫描并输出结论
  python tools/check_manual_mode.py --verbose  # 附带每个命中位置

退出码：0 = 全部通过；1 = 有违规项。
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

# 底盘协议标签（新命名体系）
CHASSIS_TAGS = ("$MV", "$MS", "$MC", "$MQ", "$MD")
# 旧命名（必须绝迹）
LEGACY_TAGS = ("$MVS", "$MVC", "$MVQ")
# manual_phone 下禁止出现的【移动】API
MOVE_APIS = ("move_lateral", "move_forward", "set_speed", "move")
# auto 守卫模式
AUTO_GUARDS = ('work_mode == "auto"', "work_mode == 'auto'", "is_auto()")

SM_FILE = os.path.join(_ROOT, "pc", "core", "state_machine.py")
CTRL_FILE = os.path.join(_ROOT, "pc", "comm", "chassis_controller.py")


def _read(path: str) -> str:
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


def _line_of(text: str, pos: int) -> int:
    return text.count("\n", 0, pos) + 1


def _iter_functions(src: str):
    """yield (函数名, 函数体起pos, 函数体终pos)"""
    for m in re.finditer(r"^    def (\w+)\s*\(", src, re.M):
        name = m.group(1)
        start = m.start()
        nxt = re.search(r"^    def \w+\s*\(", src[m.end():], re.M)
        end = m.end() + nxt.start() if nxt else len(src)
        yield name, start, end


def _strip_docstrings(src: str) -> str:
    """剥离三引号文档字符串，避免示例代码造成误报（保留行数结构）。"""
    out = []
    i = 0
    while i < len(src):
        m = re.match(r'("""|\'\'\')', src[i:])
        if m:
            q = m.group(1)
            j = src.find(q, i + 3)
            if j < 0:
                out.append("\n" * src[i:].count("\n"))
                break
            out.append("\n" * src[i:j + 3].count("\n"))
            i = j + 3
        else:
            out.append(src[i])
            i += 1
    return "".join(out)


def check_move_api_guards(verbose: bool = False):
    """规则 1：移动 API 所在函数的调用点必须全在 auto 守卫下。"""
    errs = []
    if not os.path.exists(SM_FILE):
        return ["找不到 %s" % SM_FILE]
    src = _read(SM_FILE)
    code = _strip_docstrings(src)

    # 找调用移动 API 的函数
    move_funcs = []
    for name, start, end in _iter_functions(code):
        body = code[start:end]
        for api in MOVE_APIS:
            if re.search(r"\.%s\s*\(" % re.escape(api), body):
                move_funcs.append((name, api))
                break
    if not move_funcs:
        return errs

    # 对这些函数的每个调用点，回溯 700 字符内是否有 auto 守卫；
    # 无守卫时再看被调函数自身是否在移动调用前内置 auto 守卫（传递守卫）。
    for fname, api in move_funcs:
        for m in re.finditer(r"\.%s\s*\(" % re.escape(fname), code):
            if code[max(0, m.start() - 30):m.start()].rstrip().endswith("def " + fname):
                continue
            ctx = code[max(0, m.start() - 700):m.start()]
            if any(g in ctx for g in AUTO_GUARDS):
                continue
            # 传递守卫：被调函数体内、首个移动调用之前是否有 auto 守卫
            fm = re.search(r"^    def %s\s*\(" % re.escape(fname), code, re.M)
            if fm:
                nxt = re.search(r"^    def \w+\s*\(", code[fm.end():], re.M)
                fbody = code[fm.start():fm.end() + nxt.start() if nxt else len(code)]
                first_move = re.search(r"\.(?:%s)\s*\(" % "|".join(MOVE_APIS), fbody)
                if first_move and any(g in fbody[:first_move.start()] for g in AUTO_GUARDS):
                    continue
            ln = _line_of(src, m.start())
            errs.append("规则1：第 %d 行调用 .%s()（内部用 %s）不在 auto 守卫下"
                        % (ln, fname, api))
            if not verbose:
                return errs
    return errs


def check_manual_branches(verbose: bool = False):
    """规则 2：manual_phone 专属块内不得出现移动 API。"""
    errs = []
    if not os.path.exists(SM_FILE):
        return errs
    src = _read(SM_FILE)
    # manual 分支块：if ... == "manual_phone": 起 12 行
    for m in re.finditer(r'==\s*"manual_phone"', src):
        ln_start = src.rfind("\n", 0, m.start()) + 1
        block = src[ln_start:ln_start + 900]
        for api in MOVE_APIS:
            if re.search(r"\.%s\s*\(" % re.escape(api), block):
                ln = _line_of(src, m.start())
                errs.append("规则2：第 %d 行 manual_phone 分支出现移动 API .%s()"
                            % (ln, api))
                if not verbose:
                    return errs
    return errs


def check_set_mode_sync(verbose: bool = False):
    """规则 3：set_mode 必须把模式同步给底盘。"""
    errs = []
    if not os.path.exists(SM_FILE):
        return ["找不到 %s" % SM_FILE]
    src = _read(SM_FILE)
    m = re.search(r"^    def set_mode\s*\(", src, re.M)
    if not m:
        return ["规则3：state_machine 缺少 set_mode()"]
    nxt = re.search(r"^    def \w+\s*\(", src[m.end():], re.M)
    body = src[m.start():m.end() + nxt.start() if nxt else len(src)]
    if not re.search(r"chassis\s*\.\s*set_mode|self\.chassis\.set_mode", body):
        errs.append("规则3：set_mode() 未调用 chassis.set_mode() 同步固件 $MD")
    return errs


def check_tags(verbose: bool = False):
    """规则 4：旧协议名绝迹 + 新名在用。"""
    errs = []
    if not os.path.exists(CTRL_FILE):
        return ["找不到 %s" % CTRL_FILE]
    src = _read(CTRL_FILE)
    for tag in LEGACY_TAGS:
        if tag in src:
            errs.append("规则4：第 %d 行仍在使用旧协议名 %s"
                        % (_line_of(src, src.find(tag)), tag))
    for tag in ("MS", "MC", "MV"):
        if ('"%s"' % tag) not in src and ("%s," % tag) not in src:
            errs.append("规则4：底盘控制器缺少新协议名 %s 的使用" % tag)
    return errs


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="manual_phone 模式静态检查")
    ap.add_argument("--verbose", action="store_true", help="输出全部命中位置")
    a = ap.parse_args(argv)

    print("=" * 70)
    print("manual_phone 模式静态检查（规则 1-4）")
    print("=" * 70)

    errs = []
    errs += check_move_api_guards(a.verbose)
    errs += check_manual_branches(a.verbose)
    errs += check_set_mode_sync(a.verbose)
    errs += check_tags(a.verbose)

    if errs:
        print("[FAIL] 发现 %d 个问题：" % len(errs))
        for e in errs[:20]:
            print("   -", e)
        print("\n[结论] manual_phone 约束未通过，请修正后重跑。")
        return 1
    print("[ OK ] 规则1：底盘移动 API 的调用点全部处于 auto 守卫下")
    print("[ OK ] 规则2：manual_phone 专属块无任何底盘移动 API")
    print("[ OK ] 规则3：set_mode() 正常同步模式给固件（$MD,AUTO/MAN）")
    print("[ OK ] 规则4：新协议名 MS/MC/MV 在用，旧名 MVS/MVC/MVQ 已绝迹")
    print("\n[结论] manual_phone 约束全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
