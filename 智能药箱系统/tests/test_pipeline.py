#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 端到端流程测试 (test_pipeline.py)
================================================================================
目的：在【没有任何硬件】的情况下，证明整条链路真的能跑通。

覆盖范围
--------
  1. 串口协议：帧构建 / 校验 / 流解析（半帧、粘包、脏数据）
  2. 运动学：FK / IK 往返一致性
  3. 知识库：类别查询、选药、库存扣减
  4. 语义映射：mock 规则把"我感冒了"映射到"感冒药"
  5. 二维码：多后端融合识别
  6. 主控流程（核心）：
       - 正常取药成功
       - 高风险药二次确认（用户确认 / 用户取消）
       - 抓错药 -> 过检位发现 -> 放回（安全闭环）
       - 槽位空 -> 提前告知
       - 未听懂 -> 提示重说

运行
----
    cd /workspace/smart_medbox
    python3.11 tests/test_pipeline.py
    python3.11 tests/test_pipeline.py -v        # 打印详细过程
退出码 0 表示全部通过。
================================================================================
"""

from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, "pc"))

import numpy as np                                        # noqa: E402

from comm.serial_protocol import (StreamParser, build_frame,  # noqa: E402
                                  split_cmd, verify_line, xor_checksum)
from comm.arm_controller import MockArmController            # noqa: E402
from comm.chassis_controller import MockChassisController    # noqa: E402
from core.kinematics import ArmGeometry, fk, ik              # noqa: E402
from core.medicine_kb import MedicineKB                      # noqa: E402
from core.pose_table import PoseTable                        # noqa: E402
from core.logger import Logger                               # noqa: E402
from core.state_machine import MedicineBoxSystem             # noqa: E402
from llm.glm_client import GLMClient                         # noqa: E402
from vision.qr_detector import QRDetector, make_qr_image     # noqa: E402

PASS, FAIL = "PASS", "FAIL"
_results = []


def check(name: str, cond: bool, detail: str = "") -> bool:
    """记录一条断言结果"""
    ok = bool(cond)
    _results.append((ok, name, detail))
    print("  [%s] %s%s" % (PASS if ok else FAIL, name,
                           ("  -- " + detail) if detail else ""))
    return ok


# ============================================================================
# 假设备：可编程的 ASR / TTS / 相机 / 槽位监测器
# ============================================================================
class ScriptedASR:
    """按脚本返回识别结果；脚本用完后返回 None"""

    is_mock = True          # 产品契约：state_machine 据此跳过真人等待环节

    def __init__(self, answers):
        self.answers = list(answers)
        self.log = []

    def recognize_once(self, timeout=6.0, **kw):
        if not self.answers:
            return None
        a = self.answers.pop(0)
        self.log.append(a)
        print("    (ASR) 用户说: %s" % a)
        return a

    def recognize_file(self, path):
        return None

    def close(self):
        pass


class CollectTTS:
    """收集所有播报内容，不真的发声"""

    is_mock = True          # 产品契约：state_machine 据此跳过等待语音播报完毕

    def __init__(self):
        self.said = []

    def speak(self, text, wait=True):
        self.said.append(text)
        print("    (TTS) %s" % text)
        return True

    def speak_async(self, text):
        self.said.append(text)

    def close(self):
        pass


class DynamicCamera:
    """
    动态假相机：画面里真的渲染出指定药品的二维码。
    这样过检位扫码是【真的在解码】，不是假的返回值。

    参数
    ----
    code : 当前"被抓在手上"的药品二维码内容

    set_expected(want)
    ----------------
    产品代码（state_machine）的契约：列二维码校正 / 过检位扫码前会调用本方法，
    通知"现在应该看到哪个码"。

    follow_expected=False 时【故意不跟随】：相机坚持渲染"实际抓在手上"的码
    （构造函数传入的 code），用来模拟"抓错药 -> 过检发现"的安全闭环场景。
    """

    def __init__(self, code: str = "MED001", follow_expected: bool = True):
        self.med_code = code                 # 实际抓在手上的药（恒定不变）
        self.follow_expected = bool(follow_expected)
        self.code = code
        self._frame = None

    def set_expected(self, want: str) -> None:
        """被告知"接下来应该看到 want 这个码"，分两种语义处理：

        - COL 开头的列码：贴在货架上的绝对位置标记，底盘到位就该看到，
          **始终跟随**（否则 auto 对位永远过不了）；
        - MED 药品码（过检位）：follow_expected=True 时渲染"期望的药"
          （演示全流程走通）；False 时坚持渲染手上真正的药，用来模拟
          "抓错药 -> 过检发现"的安全闭环。
        """
        if not want:
            return
        if want.startswith("COL"):
            target = want                    # 列码：始终跟随
        elif self.follow_expected:
            target = want                    # 药品码：演示模式，跟随期望
        else:
            target = self.med_code           # 药品码：闭环模式，坚持真药
        if target != self.code:
            self.code = target
            self._frame = None               # 失效缓存，下次 read() 重渲染

    def _render(self):
        import cv2
        img = np.full((420, 420, 3), 205, np.uint8)
        qr = make_qr_image(self.code, size=200)
        if qr is None:
            return img
        q = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
        q = cv2.resize(q, (260, 260), interpolation=cv2.INTER_NEAREST)
        img[80:340, 80:340] = q
        return img

    def read(self):
        if self._frame is None:
            self._frame = self._render()
        return True, self._frame.copy()

    def release(self):
        pass


class FakeMonitor:
    """假槽位监测器：可指定每个槽位的有无状态"""

    def __init__(self, state: dict):
        self.state = dict(state)

    def check(self, frame):
        return dict(self.state)


# ============================================================================
# 创建被测试的系统
# ============================================================================
def make_system(asr_answers, camera_code="MED001", slot_state=None,
                verbose=False, chassis=None, camera=None):
    """组装一个全假设备的系统，返回 (system, tts, camera)

    chassis : 传 None 时自动装配 MockChassisController 并走 auto 模式，
              让"底盘对位"这一步也能在无硬件环境下验证（发帧序列可查）。
    camera  : 传 None 时用 DynamicCamera(camera_code)（跟随 set_expected，
              演示全流程走通）；测"抓错药"时传 follow_expected=False 的实例。
    """
    kb = MedicineKB.load(os.path.join(_ROOT, "config", "medicines.json"))
    poses = PoseTable.load(os.path.join(_ROOT, "config", "poses.json"))
    arm = MockArmController("MOCK", verbose=False)
    asr = ScriptedASR(asr_answers)
    tts = CollectTTS()
    glm = GLMClient(categories=kb.categories(), mock=True, verbose=False)
    if camera is None:
        camera = DynamicCamera(camera_code)
    qr = QRDetector()
    monitor = FakeMonitor(slot_state) if slot_state is not None else None
    if chassis is None:
        chassis = MockChassisController("MOCK", verbose=False)

    cfg = {
        "timing": {"settle_ms": 10, "move_ms": 100, "scan_frames": 3},
        "retry": {"max_attempts": 1, "delay_s": 0.05},
        "vision": {"require_slot_check": slot_state is not None,
                   "inspect_roi": None},
        "work_mode": {"mode": "auto"},
    }
    log = Logger(None, echo=False)
    sys_ = MedicineBoxSystem(cfg, arm=arm, asr=asr, tts=tts, glm=glm, kb=kb,
                             poses=poses, camera=camera, qr=qr,
                             monitor=monitor, log=log, chassis=chassis)
    return sys_, tts, camera, kb


# ============================================================================
# 各测试项
# ============================================================================
def test_protocol():
    print("\n[1] 串口协议")
    body = "SA,90,75,60,40,800"
    f = build_frame(body)
    # 帧头/帧尾/帧体（不硬编码校验值，只验证结构自洽）
    line = f.decode("ascii").strip()
    check("帧以 $ 开头并以 \\r\\n 结尾", f.startswith(b"$") and f.endswith(b"\r\n"), str(f))
    check("帧体包含指令字段", line.startswith("$" + body + "*"), line)
    cs_hex = line.split("*", 1)[1]
    check("校验值与 xor_checksum 自洽",
          int(cs_hex, 16) == xor_checksum(body),
          "%s vs %02X" % (cs_hex, xor_checksum(body)))
    check("合法帧能被校验通过", verify_line(line) == body, str(verify_line(line)))
    check("校验错误被拒绝", verify_line("$SA,90,75*00") is None)
    check("无帧头被拒绝", verify_line("SA,90*00") is None)
    cmd, args = split_cmd("SA,90,75,60,40,800")
    check("指令拆分正确", cmd == "SA" and args[0] == "90" and len(args) == 5)

    p = StreamParser()
    # 用真实 build_frame 构造帧，故意拆成半帧 + 粘完整帧 + 脏数据 + 多余换行
    f1 = build_frame("SA,10")
    f2 = build_frame("DONE,SA")
    f3 = build_frame("ERR,03")
    raw = b"hello\r\n" + f1[:6] + f1[6:] + f2 + b"\r\n" + f3
    got = p.feed(raw)
    check("流解析能处理半帧/粘包/脏数据",
          got == ["SA,10", "DONE,SA", "ERR,03"], str(got))


def test_kinematics():
    print("\n[2] 运动学")
    # core.kinematics 是平面模型：ik(x, z, phi_deg)，fk() 返回 ((x, z), (phi1, phi2, phi3))
    # y 方向（货架列间）由底盘横移负责，不在本模块职责内。
    g = ArmGeometry()
    tx, tz, tphi = 150.0, 80.0, -90.0        # 前伸 150mm、高 80mm、夹爪竖直朝下
    sol = ik(tx, tz, phi_deg=tphi, g=g)
    check("逆解有解", sol is not None,
          "目标 (%g, %g, %g)" % (tx, tz, tphi))
    (fx, fz), _ = fk(sol["s0"], sol["s1"], sol["s2"], g)
    err = float(abs(fx - tx) + abs(fz - tz))
    check("FK/IK 往返一致", err < 0.01, "误差 %.6f mm" % err)
    check("超出臂展返回 None", ik(900, 0, g=g) is None)
    check("角度在安全区内", sol["in_limits"],
          "s=%.1f/%.1f/%.1f" % (sol["s0"], sol["s1"], sol["s2"]))
    check("末端姿态即夹爪朝向", abs(sol["phi3_deg"] - tphi) < 0.01,
          "%.1f 度（-90 = 竖直朝下）" % sol["phi3_deg"])


def test_kb():
    print("\n[3] 药品知识库")
    kb = MedicineKB.load(os.path.join(_ROOT, "config", "medicines.json"))
    check("药品加载成功", len(kb.all()) == 9, "%d 种" % len(kb.all()))
    check("类别提取正确", "感冒药" in kb.categories(), str(kb.categories()))
    m = kb.pick_one("感冒药")
    check("按类别选药", m is not None and m.code == "MED001", str(m))
    check("按码反查", kb.by_code("MED006").name == "阿莫西林胶囊")
    check("高风险标记", kb.by_code("MED006").high_risk is True)
    before = kb.by_code("MED001").stock
    kb.consume("MED001", 1)
    check("库存扣减", kb.by_code("MED001").stock == before - 1)
    check("类别白名单校验", kb.is_valid_category("感冒药")
          and not kb.is_valid_category("长生不老药"))


def test_llm_mock():
    print("\n[4] 语义映射（mock 规则）")
    kb = MedicineKB.load(os.path.join(_ROOT, "config", "medicines.json"))
    glm = GLMClient(categories=kb.categories(), mock=True, verbose=False)
    r = glm.map_symptom("我感冒了，鼻子不通气")
    check("感冒映射正确", r["ok"] and r["category"] == "感冒药", str(r))
    r = glm.map_symptom("有点发烧，头也疼")
    check("发烧映射正确", r["category"] in ("退烧药", "止痛药"), str(r))
    r = glm.map_symptom("今天天气不错")
    check("无关话题判为 unknown", not r["ok"] or r["intent"] == "unknown", str(r))


def test_qr():
    print("\n[5] 二维码识别")
    det = QRDetector()
    print("    解码后端: %s" % det.backend_name)
    img = make_qr_image("MED003", size=200)
    check("二维码能生成", img is not None)
    res = det.detect_best(img)
    check("二维码能识别", res is not None and res.code == "MED003",
          str(res))


def test_flow_success():
    print("\n[6] 主控流程：正常取药")
    sys_, tts, cam, kb = make_system(["我感冒了"], camera_code="MED001")
    r = sys_.run_once()
    check("取药成功", r["status"] == "success", str(r))
    check("取的是感冒灵", r["medicine"] is not None
          and r["medicine"].code == "MED001", str(r.get("medicine")))
    check("播报了药品名", any("感冒灵" in s for s in tts.said), str(tts.said))
    check("有 TTS 反馈", len(tts.said) >= 2, "%d 条" % len(tts.said))


def test_flow_high_risk_confirm():
    print("\n[7] 主控流程：高风险药二次确认（用户确认）")
    sys_, tts, cam, kb = make_system(["我发烧了", "是"], camera_code="MED002")
    r = sys_.run_once()
    check("高风险药触发确认",
          any("确认" in s for s in tts.said), str(tts.said))
    check("确认后取药成功", r["status"] == "success", str(r))


def test_flow_high_risk_cancel():
    print("\n[8] 主控流程：高风险药二次确认（用户取消）")
    sys_, tts, cam, kb = make_system(["我头疼得厉害", "不要"],
                                     camera_code="MED004")
    r = sys_.run_once()
    check("用户否认后取消", r["status"] == "cancel", str(r))
    check("播报了取消提示", any("取消" in s for s in tts.said), str(tts.said))


def test_flow_mismatch():
    print("\n[9] 主控流程：抓错药 -> 过检发现 -> 放回（安全闭环）")
    # 用户要感冒药(MED001)，但机械臂抓起来的药扫码是 MED005。
    # follow_expected=False：过检位相机不理会"期望码"通知，坚持渲染手上的
    # 真药（错药），这样才能真的触发 mismatch 安全闭环。
    cam = DynamicCamera("MED005", follow_expected=False)
    sys_, tts, cam, kb = make_system(["我感冒了"], camera=cam)
    r = sys_.run_once()
    check("检测到抓错药", r["status"] == "mismatch", str(r))
    check("触发放回动作",
          any("放回" in s or "错误" in s for s in tts.said), str(tts.said))
    check("没有错误交付", not any("已为您取到" in s for s in tts.said))


def test_flow_empty_slot():
    print("\n[10] 主控流程：槽位为空提前告知")
    sys_, tts, cam, kb = make_system(["我感冒了"], camera_code="MED001",
                                     slot_state={1: False, 2: True})
    r = sys_.run_once()
    check("识别为空槽", r["status"] == "empty_slot", str(r))
    check("提示缺药", any("没有检测到药品" in s for s in tts.said), str(tts.said))


def test_flow_unknown():
    print("\n[11] 主控流程：没听懂")
    sys_, tts, cam, kb = make_system(["今天天气不错"])
    r = sys_.run_once()
    check("判为未知需求", r["status"] == "unknown", str(r))
    check("提示用户重说", any("没有听懂" in s or "请告诉我" in s
                              for s in tts.said), str(tts.said))


def test_arm_sequence():
    print("\n[12] 机械臂动作序列完整性")
    sys_, tts, cam, kb = make_system(["我感冒了"], camera_code="MED001")
    arm = sys_.arm
    r = sys_.run_once()
    hist = [h.split(",")[0] for h in arm.history]
    check("发送了归位指令", "HOM" in hist, str(hist))
    check("发送了同步运动指令", "SA" in hist, str(hist))
    check("发送了夹爪指令", "GC" in hist, str(hist))
    check("夹爪有开有合",
          any("GC,open" in h for h in arm.history)
          and any("GC,close" in h for h in arm.history), str(arm.history[-4:]))
    check("结尾回到安全位", arm.history[-1] == "HOM", arm.history[-1])


# ============================================================================
def main():
    print("=" * 74)
    print("智能药箱 —— 端到端流程测试（全仿真，无需硬件）")
    print("=" * 74)
    for fn in (test_protocol, test_kinematics, test_kb, test_llm_mock,
               test_qr, test_flow_success, test_flow_high_risk_confirm,
               test_flow_high_risk_cancel, test_flow_mismatch,
               test_flow_empty_slot, test_flow_unknown, test_arm_sequence):
        try:
            fn()
        except Exception as e:
            import traceback
            traceback.print_exc()
            check("%s 抛出异常" % fn.__name__, False, str(e))

    n_ok = sum(1 for ok, _, _ in _results if ok)
    n_all = len(_results)
    print("\n" + "=" * 74)
    print("测试结果: %d/%d 通过" % (n_ok, n_all))
    for ok, name, detail in _results:
        if not ok:
            print("  [失败] %s  %s" % (name, detail))
    print("=" * 74)
    return 0 if n_ok == n_all else 1


if __name__ == "__main__":
    sys.exit(main())
