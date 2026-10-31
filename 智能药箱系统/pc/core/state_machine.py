#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 主控状态机 (state_machine.py) —— 整个系统的大脑
================================================================================
职责：把语音、大模型、知识库、视觉、机械臂串成一条完整流程，并负责
      高风险药二次确认、异常重试、急停保护。

完整流程（每一步失败都有对应处理）
----------------------------------
    ┌─ IDLE ────────────────────────────────────────────────────────┐
    │                                                                │
    │  ① 听        ASR 识别用户说的话                                │
    │     失败 -> 重听（最多 N 次）-> 放弃并提示                     │
    │                                                                │
    │  ② 理解      GLM 把"我感冒了"映射成类别"感冒药"                 │
    │     失败/未知 -> 语音请用户再说一遍                             │
    │                                                                │
    │  ③ 决策      本地知识库：类别 -> 具体药品 + 槽位                 │
    │     缺货/没有 -> 语音告知并结束                                 │
    │                                                                │
    │  ④ 高风险确认 若 high_risk，语音询问 + 听回复确认                │
    │     用户否认 -> 取消                                            │
    │                                                                │
    │  ⑤ 取前确认   视觉检查目标槽位"有药"                            │
    │     空槽 -> 告知用户缺药，结束                                   │
    │                                                                │
    │  ⑥ 抓取      home -> approach -> grasp -> 夹紧 -> lift          │
    │     串口超时 -> 重试一次 -> 仍失败进 FAULT                       │
    │                                                                │
    │  ⑦ 过检（关键）把药举到相机前扫码，核对是不是刚才要的那种        │
    │     不一致 -> 放回原槽 + 报警（这是系统的安全闭环）              │
    │                                                                │
    │  ⑧ 交付      送到出药位 -> 松爪 -> 回 home                      │
    │                                                                │
    │  ⑨ 反馈      TTS 播报药品名 + 服用提示，扣减库存                 │
    └────────────────────────────────────────────────────────────────┘

    任何一步出现"机械臂通信异常/超时/识别不出" -> 重试 -> 仍失败 -> FAULT:
        立即急停 -> 回安全位 -> 语音报警 -> 等人工处理

为什么"过检位"（第⑦步）必须做？
    机械臂是开环舵机，可能抓空、可能抓错、可能在半路掉了。
    槽位里的二维码还常常被药瓶挡住，取药前根本读不到。
    所以"抓起后送到相机前扫一次"是唯一可靠的确认手段，
    也是答辩时被问"抓错了怎么办"时的标准答案。
================================================================================
"""

from __future__ import annotations

import time
from typing import Callable, Dict, List, Optional

from .medicine_kb import Medicine, MedicineKB
from .pose_table import PoseTable


class MedicineBoxSystem:
    """
    智能药箱主控系统。

    用法
    ----
        sys = MedicineBoxSystem(cfg, arm=arm, asr=asr, tts=tts,
                                glm=glm, kb=kb, poses=poses,
                                camera=cam, qr=qr, monitor=mon, log=log)
        result = sys.run_once()          # 处理一轮用户请求
        print(result["status"])          # 'success' / 'cancel' / 'fault' / ...
    """

    def __init__(self, cfg: dict, *,
                 arm, asr, tts, glm, kb: MedicineKB, poses: PoseTable,
                 camera=None, qr=None, monitor=None, log=None,
                 chassis=None,
                 show_scan_window: bool = False,
                 scan_hold_s: float = 60.0):
        """
        参数
        ----
        cfg     : 配置字典（见 config/app.yaml）
        arm     : ArmController 机械臂控制器
        asr     : ASREngine 语音识别
        tts     : TTSEngine 语音合成
        glm     : GLMClient 语义映射
        kb      : MedicineKB 药品知识库
        poses   : PoseTable 预设点位表
        camera  : Camera / MockCamera（可选，无视觉时跳过视觉确认）
        qr      : QRDetector（可选）
        monitor : SlotMonitor（可选）
        log     : Logger（可选）
        chassis : ChassisController 麦轮底盘控制器（可选）。
                  ★ 只有 auto 模式才用得到：manual_phone 模式绝不发底盘指令。
                  为 None 时 auto 模式会退化为"只提示不动底盘"并给出告警，
                  而不是静默跳过 —— 否则用户会以为底盘动了。
        show_scan_window : 过检位扫码时是否弹实时预览窗口（默认关）。
                   开了之后用户能看见画面、知道什么时候举标签、有没有扫上。
                   用真摄像头人工举标签时**强烈建议开**（pc/main.py --show）。
                   没有桌面环境时自动退化为不弹窗，不影响流程。
        scan_hold_s : 弹窗时，正式投票扫码前的"举标签倒计时"秒数。
        """
        self.cfg = cfg or {}
        self.arm = arm
        self.asr = asr
        self.tts = tts
        self.glm = glm
        self.kb = kb
        self.poses = poses
        self.camera = camera
        self.qr = qr
        self.monitor = monitor
        self.log = log
        self.chassis = chassis

        t = self.cfg.get("timing") or {}
        vis = self.cfg.get("vision") or {}
        self.settle_ms = int(t.get("settle_ms", 350))        # 舵机到位后等待抖动消失
        self.retry_max = int((self.cfg.get("retry") or {}).get("max_attempts", 2))
        self.retry_delay = float((self.cfg.get("retry") or {}).get("delay_s", 0.5))
        self.move_ms_default = int(t.get("move_ms", 900))
        # 过检扫码：默认按【时间窗】扫（给人举手机/标签留时间），不是固定 5 帧 0.3s
        self.scan_frames = int(vis.get("scan_frames", t.get("scan_frames", 80)))
        self.scan_timeout_s = float(vis.get(
            "scan_timeout_s", t.get("scan_timeout_s", 12.0)))
        self.scan_interval_s = float(vis.get(
            "scan_interval_s", t.get("scan_interval_s", 0.0)))
        self.scan_min_votes = int(vis.get("scan_min_votes", 2))
        # false=整帧识别（与 tools/camera_preview.py 一致，手举码更稳）
        self.inspect_use_roi = bool(vis.get("inspect_use_roi", False))
        self.require_slot_check = bool(vis.get("require_slot_check", True))
        # ★ 过检位扫码确认开关（默认 True 保留安全闭环）。
        #   设 False：夹紧后机械臂【保持姿态不动】，直接底盘退离 -> HC05 人工放药。
        #   扫码出库由人工环节完成（二次人工出库）。跳过时打 WARN 保持可见性。
        self.require_scan_confirm = bool(vis.get("require_scan_confirm", True))

        # ---- 工作模式（规则 1 / 规则 3 / 规则 4）----
        wm = self.cfg.get("work_mode") or {}
        explicit = wm.get("mode", None)
        self.work_mode = str(explicit if explicit is not None else "auto").strip().lower()
        if self.work_mode not in self.MODES:
            self._log_safe("warn", "work_mode '%s' 不认识，已回退为 auto"
                           % self.work_mode)
            self.work_mode = "auto"
            explicit = "auto"
        # ★ 安全兜底：没显式配模式、又没接底盘时，auto 模式一上来就会
        #   "no_chassis" 直接失败（用户会以为程序坏了）。这种情况自动退成
        #   manual_phone —— 不依赖底盘，靠人推车，功能上仍然完整可用。
        if explicit is None and chassis is None and self.work_mode == "auto":
            self.work_mode = "manual_phone"
            self._log_safe("warn", "未显式配置 work_mode 且未装配底盘，"
                                   "已自动回退为 manual_phone（不生成底盘指令）")
        self.allow_voice_switch = bool(wm.get("allow_voice_switch", True))
        self.manual_confirm_timeout_s = float(
            wm.get("manual_confirm_timeout_s", 180.0))
        self.manual_prompt_repeat = int(wm.get("manual_prompt_repeat", 2))

        # ---- 底盘对位参数 ----
        ch = self.cfg.get("chassis") or {}
        self.chassis_cfg = ch
        self.chassis_speed = float(ch.get("speed", 0.35))
        self.chassis_speed_scale = float(ch.get("speed_scale_mm_s", 100.0))
        self.chassis_col_pitch_mm = float(ch.get("col_pitch_mm", 600.0))
        self.chassis_move_timeout_s = float(ch.get("move_timeout_s", 12.0))
        self.chassis_col_prefix = str(ch.get("col_qr_prefix", "COL"))
        # ★ 抓完自动退离开关（2026-10-22 用户要求去掉：夹住药后姿态冻结，
        #   出槽由人工 HC05 开车完成）。true=恢复原来的自动 $MV 后退。
        self.chassis_auto_retreat = bool(ch.get("auto_retreat", False))
        # ★ 分段横移（解决长行程电池压降 → 右前轮堵转）：
        #   每段最多走 lateral_pulse_mm 的时长，段间停 lateral_pause_ms
        #   让电池回血。第 2 列 = 1 段、第 3 列 = 2 段（即用户说的
        #   "HC05 打一次/两次右移命令"）。0 或 null = 不分段（旧行为）。
        self.chassis_lateral_pulse_mm = float(ch.get("lateral_pulse_mm", 0) or 0)
        self.chassis_lateral_pause_ms = float(ch.get("lateral_pause_ms", 800))
        self.chassis_align_tol_mm = float(ch.get("align_tol_mm", 6.0))
        self.chassis_align_retry = int(ch.get("align_retry", 2))
        # 每层的底盘偏移（升/降层取药时底盘相对第1层要挪多少 mm）
        self.layer_offset_mm = {
            int(k): float(v)
            for k, v in ((self.cfg.get("box_layout") or {})
                         .get("layer_offset_mm") or {}).items()
            if str(k).lstrip("-").isdigit()
        }
        self.current_col = 1        # 底盘当前对准的列（1 起）。1 = 基准位

        # ---- 过检位扫码实时窗口 ----
        self.show_scan_window = bool(show_scan_window)
        # 举标签准备时间：配置可覆盖构造参数（默认给人 5 秒举稳）
        self.scan_hold_s = float(
            (self.cfg.get("vision") or {}).get("scan_hold_s", scan_hold_s))
        self._scan_win = None

        self.emergency = False
        self.stats = {"success": 0, "fault": 0, "cancel": 0, "unknown": 0}

    # ==================================================================
    # 工作模式（规则 3 / 规则 4 —— 两种互斥模式）
    # ==================================================================
    MODES = ("auto", "manual_phone")

    def set_mode(self, mode: str) -> bool:
        """
        切换工作模式。返回是否切换成功。

        ★ 两种模式【互斥】：同一时刻只能处于其中一种。
          mode='auto'         ：PC 通过蓝牙控底盘横移对位，之后机械臂抓取
          mode='manual_phone' ：不生成任何底盘指令，靠人工用手机蓝牙把底盘
                                移到目标列并锁死，再执行机械臂抓取
        """
        m = str(mode).strip().lower()
        if m not in self.MODES:
            self._log("warn", "未知工作模式 '%s'（可选: %s）" % (mode, "/".join(self.MODES)))
            return False
        if m == self.work_mode:
            self._log("info", "已经在 %s 模式" % m)
            return True
        old = self.work_mode
        self.work_mode = m
        self._log("info", "工作模式切换: %s -> %s" % (old, m))
        # 同步固件 $MD（融合自文件夹 1）：AUTO / MAN
        if self.chassis is not None:
            try:
                self.chassis.set_mode(m)
            except Exception as e:
                self._log("warn", "底盘模式同步失败: %s" % e)
        if m == "manual_phone":
            self._say("已切换到手机手动模式，请用手机蓝牙操控底盘", wait=True)
        else:
            self._say("已切换到全自动模式", wait=True)
        return True

    def _log_safe(self, level: str, msg: str) -> None:
        """_log 的安全版：__init__ 期间 self.log 可能还没赋值"""
        try:
            self._log(level, msg)
        except Exception:
            print("[%s] %s" % (level, msg))

    # ==================================================================
    # 底盘对位（规则 3 的核心：底盘横移到目标列）
    # ==================================================================
    def _target_layer_of(self, med: Medicine) -> int:
        """
        药品所在层号（1 起）。优先用知识库派生的 pos_z；
        没有布局信息（pos_z=0）时从槽位号反算，保证 auto 模式仍能工作。
        """
        lz = int(getattr(med, "pos_z", 0) or 0)
        if lz > 0:
            return lz
        cols = max(1, int((self.cfg.get("box_layout") or {}).get("cols", 3)))
        return (int(med.slot) - 1) // cols + 1

    def _target_col_of(self, med: Medicine) -> int:
        """药品所在列号（1 起）。同上，优先用 pos_x。"""
        c = int(getattr(med, "pos_x", 0) or 0)
        if c > 0:
            return c
        cols = max(1, int((self.cfg.get("box_layout") or {}).get("cols", 3)))
        return (int(med.slot) - 1) % cols + 1

    def _column_qr(self, col: int) -> str:
        """第 col 列的二维码内容，如 COL1 / COL2 / COL3"""
        return "%s%d" % (self.chassis_col_prefix, int(col))

    def align_chassis(self, med: Medicine) -> Dict:
        """
        【仅 auto 模式】把底盘移动到目标药品所在的列，并做二维码校正。

        流程（规则 3）
        -------------
        ① 算出目标列与层偏移
        ② Math：需要横移多少（列差 × 列间距）
        ③ 发 $MV 横移；运动期间机械臂锁定（固件 + PC 双重检查）
        ④ 到位后读该列的二维码 $COLn 校正；不一致就按容差重试
        ⑤ 确认到位后返回，让调用方开始抓取

        返回
        ----
        dict:
          ok      : 是否确认到位
          reason  : 'ok' / 'no_chassis' / 'move_failed' /
                    'align_mismatch' / 'not_supported'
          col     : 目标列号
          layer   : 目标层号
          shift_mm: 本次横移量（正 = 向右）
        """
        col = self._target_col_of(med)
        layer = self._target_layer_of(med)
        # ★ 列间距必须用 chassis.col_pitch_mm（货架【列】间距，实测约 600mm），
        #   不是 box_layout.pitch_x——那是机械臂九宫格【药槽】间距(12mm)，相差 50 倍！
        #   （历史坑：旧代码错用 box_layout.pitch_x，导致拿第 2/3 列几乎不横移。）
        pitch_x = self.chassis_col_pitch_mm

        # 列位移：底盘当前列 -> 目标列。列号从左到右递增（1 是最左）。
        # 本项目约定 x>0 = 向右横移，所以列号变大 = 向右：
        #   拿第 2 列 → 右移 ~600mm；拿第 3 列 → 右移 ~1200mm。
        shift_cols = col - self.current_col
        shift_mm = shift_cols * pitch_x

        out = {"ok": False, "reason": "ok", "col": col, "layer": layer,
               "shift_mm": shift_mm, "layer_offset_mm": 0.0}

        self._log("step", "底盘对位：目标第 %d 列 / 第 %d 层，需横移 %.1f mm"
                  % (col, layer, shift_mm))

        if self.chassis is None:
            # ★ 不要静默跳过：用户会以为底盘动了。明确报出来。
            self._log("error", "auto 模式需要底盘控制器，但当前没有装配（chassis=None）。"
                               "请用配置里的 work_mode.mode=manual_phone，"
                               "或先接好底盘再跑 auto")
            self._say("底盘未连接，无法自动对位", wait=True)
            out["reason"] = "no_chassis"
            return out

        # ---- 移动到目标列（纯横移）----
        if abs(shift_mm) < 1e-6:
            self._log("info", "已在目标列，无需横移")
        else:
            direction = 1.0 if shift_mm > 0 else -1.0
            # 时间 = 距离 / (速度 × 速度系数)。系数 100mm/s 是标称值，
            # 真机需实测标定（见 app.yaml 的 chassis.speed 注释）。
            speed_mm_s = self.chassis_speed * self.chassis_speed_scale
            ms = int(abs(shift_mm) / max(speed_mm_s, 1.0) * 1000.0)
            # ★ 上限 30s 不够第 3 列：1200mm @ 35mm/s ≈ 34s，截短会欠走 15cm。
            ms = max(120, min(ms, 60000))
            self._log("step", "横移 %s %.1f mm（%d ms @ %.0f mm/s）"
                      % ("向右" if direction > 0 else "向左",
                         abs(shift_mm), ms, speed_mm_s))
            # 机械臂锁定检查：运动期间绝不允许抓取（规则 3）
            if not self._arm_locked_and_idle():
                self._log("error", "机械臂正忙，底盘移动被拒绝（规则 3：移动期间锁定）")
                out["reason"] = "move_failed"
                return out
            # ★ 分段横移（用户 2026-10-22 提出）：长行程连续横移会让电池持续
            #   大电流放电、电压垮 → 右前轮堵转；HC05 短脉冲一直很顺。
            #   lateral_pulse_mm > 0 时拆成多段：每段一个 $MV（到点自停），
            #   段间停 lateral_pause_ms 让电池回血。第 2 列=1 段、第 3 列=2 段。
            pulse_mm = float(self.chassis_lateral_pulse_mm or 0)
            pause_s = self.chassis_lateral_pause_ms / 1000.0
            n_seg = 1
            if pulse_mm > 0 and abs(shift_mm) > pulse_mm:
                n_seg = int((abs(shift_mm) + pulse_mm - 1) // pulse_mm)
            seg_ms = max(120, ms // n_seg)

            for seg in range(1, n_seg + 1):
                dur = seg_ms if seg < n_seg else max(120, ms - seg_ms * (n_seg - 1))
                try:
                    ok = self.chassis.move_lateral(direction, dur)
                except Exception as e:
                    # 串口被 USB 瞬断/重枚举破坏且重连失败——优雅降级成 FAULT，
                    # 不让异常裸奔把整个程序带走（退出后还能继续语音服务）。
                    self._log("error", "横移串口异常：%s" % e)
                    self.fault("底盘串口异常（疑似 USB 掉线，检查线材/供电）")
                    out["reason"] = "serial_lost"
                    return out
                if not ok:
                    self._log("error", "底盘横移失败: %s" % getattr(self.chassis, "last_error", ""))
                    out["reason"] = "move_failed"
                    return out
                # ★ $MV 带 tt 是"定时运动，到点固件自停"——绝不能紧跟 $MS 截杀！
                #   （实测：$MV 后 0.2s 发 $MS，60cm 只走了 0.2 秒。）
                self._log("info", "横移中…第 %d/%d 段（%d ms，到点自停，勿动机械臂）"
                          % (seg, n_seg, dur))
                time.sleep(dur / 1000.0 + (pause_s if seg < n_seg else 0.5))
        # ---- 二维码校正（规则 3：靠列二维码做位置校正，不靠步数累计）----
        aligned = self._verify_column_qr(col)
        for attempt in range(self.chassis_align_retry):
            if aligned:
                break
            self._log("warn", "第 %d 次列二维码校正未通过，微调重试" % (attempt + 1))
            # 微调方向：读了码但不对 -> 说明偏了，先往右小步试；
            # 完全没读到码 -> 可能整列都不在视野，也给一次小步机会。
            if not self.chassis.move_lateral(1.0, 200):
                break
            time.sleep(0.5)      # 走完 200ms + 停稳（定时运动自停，不发 $MS 截杀）
            time.sleep(0.3)
            aligned = self._verify_column_qr(col)

        if not aligned:
            # 校正失败：保守停车并让调用方决定怎么办，不要硬着头皮抓药
            self._log("error", "底盘列校正失败（目标第 %d 列），为安全起见停止" % col)
            self.chassis.stop()
            self._say("底盘未对准目标列，已停止，请检查", wait=True)
            out["reason"] = "align_mismatch"
            return out

        self.current_col = col
        out["ok"] = True
        self._log("info", "底盘已对准第 %d 列（本次横移 %.1f mm）"
                  % (col, shift_mm))
        return out

    def _arm_locked_and_idle(self) -> bool:
        """
        底盘运动前的机械臂互斥检查（规则 3）。

        返回 True = 机械臂空闲、可以动底盘。
        固件侧 CHASSIS_Move 也会查 SERVO_IsBusy；这里再查一次是为了
        在发指令之前就拦下来，省得发出去被拒、还要处理 $ERR。
        """
        busy = getattr(self.arm, "is_busy", None)
        if callable(busy):
            try:
                return not bool(busy())
            except Exception:
                pass
        return True

    def _verify_column_qr(self, col: int) -> bool:
        """
        读相机画面里的列二维码，判断底盘是不是真在第 col 列。

        ★ 为什么必须用二维码校正而不是"走够步数"
        ------------------------------------------
        麦轮打滑是常态（地面有灰/有坡/加速太猛），按时间折算的距离
        累积误差会越来越大，跑几趟就偏出一整列。二维码是"绝对位置"，
        每次都对一下，误差不累积。

        视觉不可用时**返回 True**（不阻断流程），但会在日志里说清楚 ——
        因为底盘定位是"辅助确认"，不像过检位扫码那样是安全闭环。
        """
        if self.qr is None or self.camera is None:
            self._log("warn", "无视觉，跳过列二维码校正（仅按时间估算位置，"
                              "可能累积误差）")
            return True
        frame = self._grab_frame()
        if frame is None:
            self._log("warn", "读帧失败，跳过列二维码校正")
            return True
        want = self._column_qr(col)
        # 通知视觉层"现在应该看到哪一列的码"。真实相机没这个方法（忽略）；
        # mock 假相机据此渲染出 COLn，让 --mock 能把 auto 模式跑通。
        # 与 inspect_grabbed() 里对药品码做的处理完全一致。
        set_expected = getattr(self.camera, "set_expected", None)
        if callable(set_expected):
            try:
                set_expected(want)
            except Exception:
                pass
        try:
            res = self.qr.robust_scan(self.camera, n=3, interval=0.05)
        except Exception as e:
            self._log("warn", "列二维码扫描异常: %s" % e)
            return True
        if res is None or not getattr(res, "code", None):
            self._log("warn", "未读到列二维码（期望 %s），跳过校正" % want)
            return True
        got = str(res.code).strip().upper()
        if got == want.upper():
            self._log("info", "列二维码校正通过: 读到 %s" % got)
            return True
        self._log("warn", "列二维码不一致: 读到 %s，期望 %s" % (got, want))
        return False

    # ==================================================================
    # 工具
    # ==================================================================
    # 模式切换的口语说法 -> 目标模式。只认明确提到"模式"的说法，
    # 避免把"我手动拿"之类的正常需求误判成切换模式。
    _MODE_WORDS = {
        "auto": ("自动模式", "全自动", "自动取药", "切到自动", "切换到自动",
                 "用自动", "自动吧", "自动模式吧"),
        "manual_phone": ("手动模式", "手机模式", "手机手动", "手动吧",
                         "切到手动", "切换到手动", "用手机控制", "自己开",
                         "手动取药"),
    }

    def parse_mode_switch(self, text: str) -> Optional[str]:
        """
        从用户说的话里识别"切换工作模式"的意图。

        返回目标模式字符串（'auto' / 'manual_phone'），没有切换意图返回 None。

        ★ 只在句子里出现"模式"或明确的口语说法时才认，避免误判：
              "切换到手动模式"     -> manual_phone
              "用自动模式"         -> auto
              "我手动拿一下药"     -> None（这是正常需求，不是切模式）
        """
        if not text:
            return None
        s = text.strip()
        # 必须提到"模式"或"切换"，否则不认为是模式切换请求
        if not any(k in s for k in ("模式", "切换", "换成", "改用")):
            return None
        # 手动 优先判（因为"手动模式"里也含"模式"，但与"自动"互斥）
        for m, words in self._MODE_WORDS.items():
            if any(w in s for w in words):
                return m
        # 只说了"切换模式"但没说切成哪个，再宽松抓一次关键字
        if "手动" in s or "手机" in s:
            return "manual_phone"
        if "自动" in s:
            return "auto"
        return None

    def _log(self, level: str, msg: str) -> None:
        if self.log is None:
            print("[%s] %s" % (level, msg))
            return
        {"info": self.log.info, "step": self.log.step,
         "warn": self.log.warn, "error": self.log.error}.get(
            level, self.log.info)(msg)

    def _say(self, text: str, wait: bool = True) -> None:
        """语音播报（带日志，且绝不抛异常）"""
        try:
            if hasattr(self.tts, "speak"):
                # 先说清楚是“真出声”还是“只打印”，避免日志像播了其实没有
                mock = bool(getattr(self.tts, "is_mock", False))
                ok = bool(self.tts.speak(text, wait=wait))
                tag = "TTS" if (not mock and ok) else ("TTS(仅打印)" if mock else "TTS(失败)")
                if mock:
                    self._log("info", "%s: %s  ← 未出声，加 --real-tts 才走扬声器" % (tag, text))
                elif not ok:
                    self._log("warn", "%s: %s" % (tag, text))
                else:
                    self._log("info", "%s: %s" % (tag, text))
            elif callable(self.tts):
                self.tts(text)
                self._log("info", "TTS: %s" % text)
        except Exception as e:
            self._log("warn", "TTS 失败: %s  （原文: %s）" % (e, text))

    def _grab_frame(self):
        """取一帧画面；没有相机返回 None"""
        if self.camera is None:
            return None
        try:
            ok, frame = self.camera.read()
            return frame if ok else None
        except Exception as e:
            self._log("warn", "读帧失败: %s" % e)
            return None

    def _settle(self, extra_ms: int = 0) -> None:
        """
        等待机械臂稳定。

        为什么要等？舵机到位瞬间会有 0.2~0.5 秒的抖动，
        这时候拍照必然是运动模糊 —— 仿真已经证明运动模糊会让
        二维码识别率直接归零（0/8）。这一步不是可选项。
        """
        time.sleep((self.settle_ms + extra_ms) / 1000.0)

    # ==================================================================
    # ① 听
    # ==================================================================
    def listen(self, timeout: float = 8.0) -> Optional[str]:
        """
        听用户说一句话，返回识别文本；没听清返回 None。
        内部带重试。按 ESC 会尽快放弃并返回 None（主循环随后退出）。
        """
        for i in range(self.retry_max + 1):
            try:
                from core.exit_key import requested as _esc
                if _esc():
                    self._log("info", "收到 ESC，停止等待语音")
                    return None
            except Exception:
                pass
            self._log("step", "等待用户说话（第 %d 次）" % (i + 1))
            try:
                text = self.asr.recognize_once(timeout=timeout)
            except Exception as e:
                self._log("error", "ASR 异常: %s" % e)
                text = None
            try:
                from core.exit_key import requested as _esc2
                if _esc2():
                    self._log("info", "收到 ESC，停止等待语音")
                    return None
            except Exception:
                pass
            if text:
                self._log("info", "识别结果: %s" % text)
                return text.strip()
            if i < self.retry_max:
                self._say("没听清，请再说一次", wait=True)
        return None

    # ==================================================================
    # ④ 高风险二次确认
    # ==================================================================
    def confirm_with_user(self, question: str) -> bool:
        """
        语音询问并等待用户确认。

        参数
        ----
        question : 要问的话，如 "你要取的是阿莫西林，属于处方药，请确认"

        返回
        ----
        True 用户确认 / False 否认或超时
        """
        yes_words = ("是", "对", "确认", "好的", "可以", "没错", "嗯", "要", "拿",
                     "是的", "确定", "行")
        no_words = ("不", "取消", "不要", "算了", "错", "不用", "否")

        for i in range(self.retry_max + 1):
            self._say(question, wait=True)
            ans = self.listen(timeout=6.0)
            if not ans:
                continue
            if any(w in ans for w in no_words):
                self._log("info", "用户否认: %s" % ans)
                return False
            if any(w in ans for w in yes_words):
                self._log("info", "用户确认: %s" % ans)
                return True
            self._say("请回答 是 或 不是", wait=True)
        return False

    # ==================================================================
    # ⑤ 取药前：确认槽位有药
    # ==================================================================
    def check_slot_has_medicine(self, slot_id: int) -> Optional[bool]:
        """
        视觉检查某个槽位是否还有药。

        返回
        ----
        True 有药 / False 空槽 / None 无法判断（没配视觉）
        """
        if self.monitor is None or self.camera is None:
            return None
        frame = self._grab_frame()
        if frame is None:
            return None
        try:
            state = self.monitor.check(frame)
            has = bool(state.get(int(slot_id), False))
            self._log("info", "槽位 %d 视觉检测: %s" % (slot_id, "有药" if has else "空"))
            return has
        except Exception as e:
            self._log("warn", "槽位检测异常: %s" % e)
            return None

    # ==================================================================
    # ⑦ 过检位扫码确认（安全闭环的核心）
    # ==================================================================
    def inspect_grabbed(self, expect_code: str,
                        move_to_inspect: bool = True) -> Dict:
        """
        扫码核对药品身份。

        move_to_inspect=True  ：先把药品举到过检位再扫（原抓取验药流程）。
        move_to_inspect=False ：★【原地扫码】——不发送任何机械臂指令，
                                用于"二次人工出库"：人工已用 HC05 把车/臂
                                开到出库位并确认，摄像头对准夹爪位置扫一次。

        参数
        ----
        expect_code : 期望的二维码内容，如 'MED003'

        返回
        ----
        dict:
          ok      : 是否确认一致
          code    : 实际读到的码（None 表示没读到）
          reason  : 'match' / 'mismatch' / 'no_code' / 'no_vision'
        """
        if self.qr is None or self.camera is None:
            return {"ok": True, "code": None, "reason": "no_vision"}

        # 通知视觉层"即将核对的药"。真实相机没有这个方法，直接忽略；
        # mock 假相机据此渲染对应的二维码，让 --mock 演示能走通全流程。
        set_expected = getattr(self.camera, "set_expected", None)
        if callable(set_expected):
            try:
                set_expected(expect_code)
            except Exception:
                pass

        if move_to_inspect:
            insp = self.poses.inspect
            self._log("step", "移动到过检位 %s" % insp)
            if not self.arm.move_all(insp[0], insp[1], insp[2], insp[3],
                                     tt=self.move_ms_default):
                return {"ok": False, "code": None, "reason": "no_code"}
        else:
            self._log("step", "原地扫码核验（不发送任何机械臂指令，当前姿态保持）")

        # mock 假相机有 set_expected()，会自己渲染出二维码，不用人配合；
        # 真实相机 / 图片回放没有这个方法，得靠人把标签举到镜头前 —— 说清楚，
        # 否则用户根本不知道"什么时候举、举到哪"。
        roi = (self.cfg.get("vision") or {}).get("inspect_roi")
        manual = not callable(getattr(self.camera, "set_expected", None))
        # 图片回放（ImageCamera）画面已是成品标签，不必等人举
        is_playback = type(self.camera).__name__ == "ImageCamera"
        win = self._ensure_scan_window(roi) if manual else None

        # 识别区域：默认整帧（与 camera_preview 一致）。ROI 只作瞄准框。
        # 只在 ROI 内解码时，手举手机稍微偏一点就扫不到 —— 这正是
        # "预览很准、过检很飘"的主要原因。
        detect_roi = tuple(roi) if (roi and self.inspect_use_roi) else None
        # 真摄像头人工举码：等人按空格再扫，上限默认 60 秒
        # 原地扫码（二次人工出库）不等人举标签：药在夹爪里，直接定时扫
        hold_s = 0.0 if (is_playback or not move_to_inspect) else max(float(self.scan_hold_s or 60.0), 8.0)
        timeout_s = self.scan_timeout_s if self.scan_timeout_s > 0 else 12.0

        if manual and move_to_inspect:
            self._log("step",
                      "【请把药品/手机标签举到镜头前】"
                      "窗口里举稳后按【空格】开始识别；最长扫 %.1f 秒…"
                      % timeout_s)

        self._settle(extra_ms=150)          # 过检位多等一会，确保画面清晰

        # 过检前再跑一遍自适应曝光：换场地/换灯后不用改代码。
        # ★ 用户在扫码窗手动调过曝光（win.manual_exposure）时【不再覆盖】，
        #   否则辛苦调好的画面会被 auto_expose 拉回去。
        user_locked_exp = bool(getattr(win, "manual_exposure", False)) if win is not None else False
        # 画面已经够亮时跳过 auto_expose（它要试很多档，体感“卡、帧率低”）
        need_ae = True
        try:
            fr = self._grab_frame()
            if fr is not None:
                from vision.camera import brightness as _br
                b = _br(fr)
                need_ae = (b < 70.0 or b > 200.0)
        except Exception:
            need_ae = True
        if (not is_playback and not user_locked_exp and need_ae
                and callable(getattr(self.camera, "auto_expose", None))
                and getattr(self.camera, "cap", None) is not None):
            try:
                ae = self.camera.auto_expose()
                if ae is not None:
                    self._log("info", "过检自适应曝光: exposure=%g 亮度=%.1f"
                              % (self.camera.exposure, ae[1]))
            except Exception as e:
                self._log("warn", "过检自动曝光失败（继续扫码）: %s" % e)
        elif user_locked_exp:
            self._log("info", "过检沿用手动曝光 exposure=%s（扫码窗 [ ] 调过的）"
                      % getattr(self.camera, "exposure", "?"))
        elif not need_ae:
            self._log("info", "过检亮度已合适，跳过自动曝光（保持当前 exposure=%s）"
                      % getattr(self.camera, "exposure", "?"))

        # 先等人举好码：真摄像头按【空格】才开始识别，避免“还没举起就结束”
        if win is not None and win.is_open and hold_s > 0:
            win.hold(self.camera, hold_s,
                     status="请把手机/标签举到镜头前，准备好后按【空格】",
                     subtitle="举稳后按空格开始识别；偏暗按 ] 或 a",
                     hint="空格=开始识别 · q=立刻扫 · [ 调暗 ] 调亮 · a 自动曝光",
                     wait_space=True)
        elif manual and move_to_inspect and hold_s > 0:
            # 只有"真人举码"才需要等人；mock 相机（有 set_expected）画面
            # 已经渲染好，立即开扫，别白等 —— 与上面 manual 的判定保持一致。
            self._log("step", "请举好标签…（无预览窗，等待 %.0f 秒）" % min(hold_s, 15.0))
            time.sleep(min(hold_s, 15.0))

        n_frames = self.scan_frames
        if timeout_s > 0:
            n_frames = max(n_frames, int(timeout_s / max(self.scan_interval_s, 0.01)) + 20)

        try:
            res = self.qr.robust_scan(
                self.camera, n=n_frames,
                interval=self.scan_interval_s,
                min_votes=self.scan_min_votes,
                accept_votes=self.scan_min_votes,
                roi=detect_roi,
                timeout_s=timeout_s,
                on_frame=(win.on_frame if (win is not None and win.is_open) else None))
        except Exception as e:
            self._log("error", "扫码异常: %s" % e)
            res = None

        # 结论定住 1.8 秒：让用户看清"扫到的是哪个码、对不对"
        if win is not None and win.is_open:
            win.verdict(res.code if res is not None else None, expect_code)

        if res is None:
            self._log("warn", "过检位未识别到二维码")
            return {"ok": False, "code": None, "reason": "no_code"}

        self._log("info", "过检位读到: %s（期望 %s）" % (res.code, expect_code))
        if res.code == expect_code:
            return {"ok": True, "code": res.code, "reason": "match"}
        return {"ok": False, "code": res.code, "reason": "mismatch"}

    # ------------------------------------------------------------------
    def _ensure_scan_window(self, roi):
        """
        惰性创建"过检位扫码"实时窗口（整个进程只开一次，跨重试复用）。

        返回 ScanWindow 或 None（未开启 / 无桌面环境 / 创建失败）。
        """
        if not self.show_scan_window or self.camera is None:
            return None
        if self._scan_win is None:
            try:
                from vision.scan_view import ScanWindow   # 延迟导入，避免无 GUI 依赖
            except Exception:
                try:
                    from pc.vision.scan_view import ScanWindow
                except Exception as e:
                    self._log("warn", "扫码窗口不可用: %s" % e)
                    self.show_scan_window = False
                    return None
            self._scan_win = ScanWindow(roi=roi)
            if not self._scan_win.enabled:
                self._log("warn", "当前环境不能弹窗，扫码仍会进行，只是没有预览窗口")
                self.show_scan_window = False
                self._scan_win = None
                return None
            self._scan_win.open()
        return self._scan_win

    def close_scan_window(self) -> None:
        """关闭过检位扫码窗口（幂等；流程结束时调用）"""
        if self._scan_win is not None:
            try:
                self._scan_win.close()
            except Exception:
                pass
            self._scan_win = None

    # ==================================================================
    # ⑥ 抓取动作序列
    # ==================================================================
    def pick_from_slot(self, slot_id: int) -> bool:
        """
        从指定槽位抓取药品（含完整动作序列）。

        序列：回位 -> 接近位(张爪) -> 抓取位(张爪) -> 夹紧 -> 抬起位

        参数
        ----
        slot_id : 槽位号

        返回
        ----
        是否成功（夹爪夹住并抬起）
        """
        approach = self.poses.get(slot_id, "approach")
        grasp = self.poses.get(slot_id, "grasp")
        lift = self.poses.get(slot_id, "lift")
        if not (approach and grasp and lift):
            self._log("error", "槽位 %d 未标定点位" % slot_id)
            return False

        tt = self.move_ms_default
        self._log("step", "抓取槽位 %d：回位" % slot_id)
        if not self.arm.home():
            return False

        self._log("step", "打开夹爪")
        if not self.arm.grip(True, tt=400):
            return False

        self._log("step", "接近位 %s" % approach)
        if not self.arm.move_all(*approach[:3], approach[3], tt=tt):
            return False
        self._settle()

        self._log("step", "下降至抓取位 %s" % grasp)
        if not self.arm.move_all(*grasp[:3], grasp[3], tt=int(tt * 0.7)):
            return False
        self._settle()

        self._log("step", "夹紧")
        if not self.arm.grip(False, tt=400):
            return False
        self._settle(extra_ms=200)

        self._log("step", "抬起 %s" % lift)
        if not self.arm.move_all(*lift[:3], lift[3], tt=tt):
            return False
        self._settle()
        return True

    def return_to_slot(self, slot_id: int) -> bool:
        """
        把抓错的药放回原槽位（过检失败时调用）。

        序列：接近位 -> 抓取位 -> 张开夹爪 -> 抬起 -> 回位
        """
        approach = self.poses.get(slot_id, "approach")
        grasp = self.poses.get(slot_id, "grasp")
        if not (approach and grasp):
            return False
        tt = self.move_ms_default
        ok = True
        ok &= self.arm.move_all(*approach[:3], approach[3], tt=tt)
        self._settle()
        ok &= self.arm.move_all(*grasp[:3], grasp[3], tt=int(tt * 0.7))
        self._settle()
        ok &= self.arm.grip(True, tt=400)          # 松爪放回
        self._settle(extra_ms=200)
        ok &= self.arm.home()
        return bool(ok)

    def prompt_hc05_place(self, scan_code: Optional[str] = None):
        """
        人工用 HC05 放药 + 出库扫码（预览窗实时可见）。

        ★ 线程模型（重要）：cv2 窗口在 Windows 上只能在【主线程】渲染
          （imshow/waitKey 跨线程不显示），所以：
            主线程   = 预览窗渲染 + 扫码解码（~10-30fps，流畅）
            后台线程 = 语音监听（好了/取消），结果写 voice["cmd"]，
                       主循环每帧都检查，随时可提前结束。
          与上一版（扫码在子线程）相反——那版窗口永远画不出来。

        返回
        ----
        (confirmed: bool, scanned_code: Optional[str])
          scanned_code 非 None = 人工阶段已成功读到期望的码。
        """
        win = None
        if scan_code and self.qr is not None and self.camera is not None:
            set_expected = getattr(self.camera, "set_expected", None)
            if callable(set_expected):
                try:
                    set_expected(scan_code)         # mock 相机据此渲染二维码
                except Exception:
                    pass
            roi = (self.cfg.get("vision") or {}).get("inspect_roi")
            win = self._ensure_scan_window(roi)
            if win is not None:
                # ★ 必须绑定相机引用：否则窗口里的 [ ] a 调曝光全部无效
                win.set_camera(self.camera)
            if win is None:
                self._log("warn", "扫码预览窗未开启（开关关闭或创建失败）——"
                                  "扫码继续，只是没有画面")

        if getattr(self.asr, "is_mock", False):
            self._log("info", "模拟模式：跳过 HC05 人工放药确认")
            return True, None

        # ★ 先把亮度调到位再开始 TTS 指导，别让人对着黑窗口干等
        if scan_code and self.camera is not None:
            try:
                fr = self._grab_frame()
                if fr is not None:
                    from vision.camera import brightness as _br
                    b = _br(fr)
                    if (b < 70.0 or b > 200.0) and callable(
                            getattr(self.camera, "auto_expose", None)) \
                            and getattr(self.camera, "cap", None) is not None:
                        ae = self.camera.auto_expose()
                        if ae is not None:
                            self._log("info", "出库扫码自适应曝光: exposure=%g 亮度=%.1f"
                                      % (self.camera.exposure, ae[1]))
            except Exception as e:
                self._log("warn", "出库扫码自动曝光失败（继续）: %s" % e)

        self._say("请用手机蓝牙控制底盘和舵机放药，药瓶标签对着摄像头。"
                  "移动底盘用 F B G H，选舵机按 1 2 3 4，加减号微调，"
                  "O 张开夹爪。完成后说好了", wait=True)

        # ---- 后台语音线程：听"好了 / 取消"，写 voice["cmd"] ----
        import threading
        voice = {"cmd": None}
        stop_voice = threading.Event()

        def _voice_worker():
            # ★ 用 asr.recognize_once 静默直调，不走 self.listen()——
            #   listen 内部是"3 次尝试 + 每次失败 TTS 播报没听清"的重型封装，
            #   放后台会 TTS 刷屏 + 抢 CPU 拖垮预览帧率。
            while not stop_voice.is_set():
                try:
                    ans = self.asr.recognize_once(timeout=2.0)
                except Exception:
                    ans = None
                if stop_voice.is_set():
                    break
                if not ans:
                    continue
                s = str(ans).strip().lower()
                if any(k in s for k in ("取消", "不要", "算了")):
                    voice["cmd"] = "cancel"
                    return
                if any(k in s for k in ("好了", "完成", "放下", "ok", "okay", "可以了")):
                    voice["cmd"] = "ok"
                    return
                # 其它话：静默忽略（HC05 阶段不播报"没听清"）

        threading.Thread(target=_voice_worker, name="HC05-Voice",
                         daemon=True).start()

        # ---- 主线程循环：预览渲染 + 扫码 + 检查语音指令 ----
        deadline = time.time() + float(self.manual_confirm_timeout_s)
        result_code = None
        miss_at = 0.0
        # ★ 预览与解码解耦：渲染每帧都做（跟相机帧率走），二维码解码降频到
        #   每 0.4s 一次——CPU 上 aruco+builtin 融合解码约 0.2-0.4s/帧，
        #   逐帧解码会把预览拖到 2-3fps（实测）。扫静止的码 2.5Hz 绰绰有余。
        decode_interval = 0.4
        last_decode = 0.0
        while time.time() < deadline:
            frame = self._grab_frame()
            if frame is None:
                if voice["cmd"]:
                    break
                time.sleep(0.1)
                continue

            if win is not None and win.is_open:
                try:
                    # 主线程渲染（on_frame 内含 imshow+waitKey 泵）——每帧都画
                    win.on_frame(frame, None, 0, 0)
                except Exception:
                    pass

            now = time.time()
            if scan_code and result_code is None and now - last_decode >= decode_interval:
                last_decode = now
                code = None
                try:
                    res = self.qr.detect_best(frame)
                    code = getattr(res, "code", None)
                except Exception:
                    code = None
                if code and code == scan_code:
                    result_code = code
                    if win is not None and win.is_open:
                        try:
                            win.verdict(code, scan_code, seconds=1.8)
                        except Exception:
                            pass
                    self._log("info", "出库扫码读到期望码: %s（窗口已显示判定）" % code)
                    # ★ 用户流程：扫到码 = 出库确认完成，立即播报用法并收尾，
                    #   不再要求说"好了"
                    break
                elif code and now - miss_at > 5.0:
                    miss_at = now
                    self._log("warn", "出库扫码读到 %s（期望 %s），继续扫…"
                              % (code, scan_code))

            if voice["cmd"] == "ok":
                stop_voice.set()
                self._say("好的，放药完成", wait=True)
                return True, result_code
            if voice["cmd"] == "cancel":
                stop_voice.set()
                return False, result_code

        stop_voice.set()
        if scan_code and result_code == scan_code:
            # 扫码驱动的正常完成路径：由 ⑧ 播报"核验通过 + 用法"并关窗收尾
            return True, result_code
        self._log("warn", "人工放药超时，按完成处理")
        return True, result_code

    def deliver(self) -> bool:
        """把药送到出药位并松开夹爪，然后回安全位"""
        dv = self.poses.deliver
        self._log("step", "送至出药位 %s" % dv)
        if not self.arm.move_all(*dv[:3], dv[3], tt=self.move_ms_default):
            return False
        self._settle(extra_ms=200)
        self._log("step", "松开夹爪")
        if not self.arm.grip(True, tt=400):
            return False
        self._settle(extra_ms=300)
        self._log("step", "回安全位")
        self.arm.home()
        return True

    # ==================================================================
    # 异常处理
    # ==================================================================
    def fault(self, msg: str) -> None:
        """进入故障态：急停 -> 语音报警 -> 记录"""
        self._log("error", "FAULT: %s" % msg)
        self.stats["fault"] += 1
        # ★ 规则 5「全局急停优先级最高」：机械臂 + 底盘一起停。
        #   ChassisController.estop() 会同时发 $EST（停舵机）和 $MS（停底盘）；
        #   这里再加一次显式底盘停止，防止 arm 是纯 ArmController（没有底盘急停）。
        try:
            self.arm.estop()
        except Exception:
            pass
        if self.chassis is not None:
            try:
                self.chassis.stop()
            except Exception:
                pass
        self._say("系统出现异常，已停止，请检查设备", wait=True)

    # ==================================================================
    # ④' manual_phone 模式：提示人工移动底盘并对位
    # ==================================================================
    def prompt_manual_align(self, med: Medicine) -> bool:
        """
        【仅 manual_phone 模式】语音提示操作人员用手机蓝牙把底盘移到目标列，
        然后等人工确认"已对位锁死"。

        ★ 规则 4：本模式**不生成任何底盘移动指令**。
          底盘完全由人用手机蓝牙（HC05）操控，PC 只负责"说清去哪、等确认"。

        返回
        ----
        True  = 人工确认已对位，可以开始抓取
        False = 超时 / 用户否认，应当取消本轮
        """
        col = self._target_col_of(med)
        layer = self._target_layer_of(med)
        qr = self._column_qr(col)

        prompt = ("请用手机蓝牙把小车移动到第 %d 列，也就是货架上贴着 %s "
                  "的那一列；对准后锁死底盘，然后对我说 好了" % (col, qr))
        self._log("step", "[manual_phone] 提示人工对位：第 %d 列（%s）/ 第 %d 层"
                  % (col, qr, layer))

        # 规则 4 要求 TTS 提示；念 manual_prompt_repeat 遍（人工可能没听清）
        for i in range(max(1, self.manual_prompt_repeat)):
            self._say(prompt, wait=True)

        # 等人工确认。给足时间（默认 180 秒），超时按取消处理。
        self._log("step", "等待人工确认已对位锁死（最多 %.0f 秒）"
                  % self.manual_confirm_timeout_s)
        deadline = time.time() + self.manual_confirm_timeout_s
        while time.time() < deadline:
            ans = self.listen(timeout=min(10.0, max(1.0, deadline - time.time())))
            if not ans:
                continue
            low = ans.strip()
            if any(w in low for w in ("取消", "不取", "算了", "不要")):
                self._log("info", "用户取消人工对位")
                return False
            if any(w in low for w in ("好了", "好", "对好", "对位", "已锁死",
                                      "锁死", "到位", "可以", "对准", "是", "ok")):
                self._log("info", "用户确认已对位: %s" % low)
                self._say("已确认对位，开始取药", wait=True)
                self.current_col = col
                return True
            self._say("请对准第 %d 列后说 好了，或者说 取消" % col, wait=True)

        self._log("warn", "等待人工对位超时")
        self._say("等待超时，本轮取消", wait=True)
        return False

    # ==================================================================
    # ⑦' manual_phone 模式：请人工把小车往后开，把药瓶拔出来
    # ==================================================================
    def wait_human_takeaway(self) -> bool:
        """
        【仅 manual_phone 模式】药瓶已经夹在夹爪里，但还卡在货架槽中。

        为什么需要这一步？
            本项目 `lift_dz = 0`（不抬高，靠底盘退开把药瓶拔出来，见 app.yaml）。
            auto 模式由 PC 发 $MV 后退完成；manual_phone 模式**不能发底盘指令**
            （规则 4），所以只能请人用手机蓝牙把小车往后开一点。

        返回
        ----
        True  人已确认退开（或有超时兜底）
        False 人明确说取消
        """
        prompt = ("药品已夹住，请用手机蓝牙把小车【向后退约 10 厘米】，"
                  "把药瓶从货架上取出来，然后对我说 好了")
        self._log("step", "[manual_phone] 提示人工退开车，取出药瓶")
        self._say(prompt, wait=True)

        deadline = time.time() + min(self.manual_confirm_timeout_s, 120.0)
        while time.time() < deadline:
            ans = self.listen(timeout=min(10.0, max(1.0, deadline - time.time())))
            if not ans:
                continue
            low = ans.strip()
            if any(w in low for w in ("取消", "不取", "算了", "不要")):
                self._log("info", "用户取消取药（退开车阶段）")
                return False
            if any(w in low for w in ("好了", "好", "退了", "退开", "取出来",
                                      "拿出来", "行", "可以", "是", "ok")):
                self._log("info", "用户确认已退开: %s" % low)
                self._settle(extra_ms=200)
                return True
            self._say("请把小车往后退一点，取出药瓶后说 好了，或者说 取消", wait=True)

        # 超时不直接判失败：药已经夹住，继续走完流程比半路扔掉更安全。
        # 但要留下明确日志，方便事后追查为什么过检位扫不到码。
        self._log("warn", "等待人工退开车超时，继续执行后续流程")
        return True

    # ==================================================================
    # 一轮完整流程
    # ==================================================================
    def run_once(self, user_text: Optional[str] = None) -> Dict:
        """
        跑一轮完整流程：听 -> 理解 -> 决策 -> 确认 -> 底盘对位 -> 取药
                       -> 过检 -> 交付。

        ★ 底盘对位这一步按工作模式分叉（规则 3 / 规则 4）：
            auto         ：PC 控底盘横移到目标列 + 列二维码校正
            manual_phone ：只语音提示人工用手机移动底盘，并等人工确认；
                           全程不发任何底盘指令

        参数
        ----
        user_text : 直接给定用户说的话（测试用）；None 时调用 ASR 听

        返回
        ----
        dict:
          status  : 'success' | 'cancel' | 'unknown' | 'empty_slot'
                    | 'no_medicine' | 'mismatch' | 'fault' | 'no_input'
                    | 'align_failed' | 'align_timeout'
          medicine: Medicine 对象或 None
          detail  : 补充说明
          mode    : 本轮用的工作模式
        """
        # ---------- ① 听 ----------
        if user_text is None:
            user_text = self.listen()
        if not user_text:
            self.stats["unknown"] += 1
            self._say("没有听到您的指令，下次再见", wait=True)
            return {"status": "no_input", "medicine": None,
                    "detail": "ASR 无输入", "mode": self.work_mode}

        self._log("info", "用户输入: %s（模式 %s）" % (user_text, self.work_mode))

        # ---------- ①' 运行时模式切换（规则 1：按当前模式解析输入）----------
        if self.allow_voice_switch:
            m = self.parse_mode_switch(user_text)
            if m is not None:
                self.set_mode(m)
                return {"status": "mode_switch", "medicine": None,
                        "detail": "切换到 %s" % m, "mode": self.work_mode}

        # ---------- ② 理解 ----------
        self._log("step", "调用大模型做语义映射")
        r = self.glm.map_symptom(user_text)
        self._log("info", "大模型返回: %s" % r)

        # ---------- ②' 药名直取（最高优先级）----------
        # 用户直接报药名（"布洛芬缓释胶囊"/"拿盒感冒灵颗粒"）时绕过类别映射，
        # 直接锁定该药品。本地字符串匹配，零延迟、离线也可用。
        med = self.kb.find_by_text(user_text)
        if med is not None:
            if med.stock <= 0:
                self.stats["unknown"] += 1
                self._say("%s已经没有库存了，请换一种药" % med.name, wait=True)
                return {"status": "no_medicine", "medicine": med,
                        "detail": "药名直取但库存为 0", "mode": self.work_mode}
            self._log("info", "药名直取命中: %s（槽位 %d）" % (med.name, med.slot))
        else:
            # ---- 症状 -> 类别 -> 选药（原路径）----
            if not r.get("ok") or r.get("intent") != "medicine" or not r.get("category"):
                self.stats["unknown"] += 1
                if r.get("intent") == "chitchat":
                    self._say("我是智能药箱，请告诉我您哪里不舒服", wait=True)
                else:
                    self._say("没有听懂您的需求，请描述您的症状，比如感冒、发烧", wait=True)
                return {"status": "unknown", "medicine": None,
                        "detail": r.get("reason", ""), "mode": self.work_mode}

            category = r["category"]

            # ---------- ③ 决策 ----------
            med = self.kb.pick_one(category)
            if med is None:
                self.stats["unknown"] += 1
                self._say("药品库中没有%s" % category, wait=True)
                return {"status": "no_medicine", "medicine": None, "detail": category}
        self._log("info", "选定药品: %s（槽位 %d）" % (med, med.slot))
        # ★ 统一播报【具体药名】（用户要求："你需要的是xx药品"），不说类别
        self._say("您需要的是%s，正在为您取药" % med.name, wait=True)

        # ---------- ④ 高风险二次确认 ----------
        if med.high_risk:
            self._log("step", "高风险药品，请求二次确认")
            ok = self.confirm_with_user(
                "您要取的是%s，属于需要谨慎使用的药品，请确认是否取用" % med.name)
            if not ok:
                self.stats["cancel"] += 1
                self._say("已取消取药", wait=True)
                return {"status": "cancel", "medicine": med,
                        "detail": "用户取消", "mode": self.work_mode}

        # ---------- ⑤ 底盘对位（★ 两种模式的分叉点，规则 3 / 规则 4）----------
        #   底盘必须先到位、并锁死，机械臂才允许抓取。
        #   顺序放在"抓取"之前是硬要求：底盘动的时候机械臂绝不能动。
        if self.work_mode == "auto":
            # 规则 3：PC 控底盘横移到目标列 + 列二维码校正
            self._log("step", "[auto] 底盘自动对位到第 %d 列"
                      % self._target_col_of(med))
            al = self.align_chassis(med)
            if not al.get("ok"):
                reason = al.get("reason")
                # 底盘没到位就【绝不抓药】—— 会抓空、会撞货架
                self.fault("底盘对位失败（%s）" % reason)
                return {"status": "align_failed", "medicine": med,
                        "detail": reason, "mode": self.work_mode}
            self._log("info", "[auto] 底盘已到位并锁死，开始机械臂流程")
        else:
            # 规则 4：不生成底盘指令，TTS 提示人工用手机蓝牙移动并锁死
            if self.chassis is None:
                # 没接底盘 = 没有可移动的机构，无需对位，直接抓取。
                # （纯机械臂台架调试、单元测试走的就是这条路）
                self._log("info", "[manual_phone] 未装配底盘，跳过人工对位，直接抓取")
            elif not self.prompt_manual_align(med):
                self.stats["cancel"] += 1
                return {"status": "align_timeout", "medicine": med,
                        "detail": "人工对位未确认", "mode": self.work_mode}
            else:
                self._log("info", "[manual_phone] 人工确认已对位锁死，开始机械臂流程")

        # ---------- ⑤' 取前确认槽位有药 ----------
        if self.require_slot_check:
            has = self.check_slot_has_medicine(med.slot)
            if has is False:
                self._say("槽位 %d 没有检测到药品，可能已经用完" % med.slot, wait=True)
                return {"status": "empty_slot", "medicine": med,
                        "detail": "视觉判定空槽", "mode": self.work_mode}
            if has is None:
                self._log("warn", "无法确认槽位状态（视觉不可用），继续按预设点位抓取")

        # ---------- ⑥ 抓取（带重试，规则 5：最多重试 2 次） ----------
        picked = False
        for attempt in range(self.retry_max + 1):
            self._log("step", "第 %d 次尝试抓取槽位 %d" % (attempt + 1, med.slot))
            if self.pick_from_slot(med.slot):
                picked = True
                break
            self._log("warn", "抓取失败，%s" % ("重试" if attempt < self.retry_max else "放弃"))
            time.sleep(self.retry_delay)
        if not picked:
            self.fault("抓取槽位 %d 失败" % med.slot)
            return {"status": "fault", "medicine": med,
                    "detail": "抓取失败", "mode": self.work_mode}

        # ---------- ⑦ 过检位确认（可经 vision.require_scan_confirm 关闭） ----------
        if not self.require_scan_confirm:
            # 用户流程：夹住药 = PC 任务结束，姿态保持不变，扫码留给人工出库环节。
            self._log("warn", "过检位扫码确认已关闭（vision.require_scan_confirm=false），"
                              "机械臂保持夹持姿态；药品身份由人工出库环节核对")
        else:
            self._log("step", "过检位扫码确认")
            insp = self.inspect_grabbed(med.code)
            if not insp.get("ok"):
                reason = insp.get("reason")
                if reason == "mismatch":
                    self._log("error", "抓错药：读到 %s，期望 %s" % (insp.get("code"), med.code))
                    self._say("检测到取药错误，正在放回", wait=True)
                    self.return_to_slot(med.slot)
                    self._say("取药错误已恢复，请重试或检查药品摆放", wait=True)
                    return {"status": "mismatch", "medicine": med,
                            "detail": "读到 %s" % insp.get("code"),
                            "mode": self.work_mode}
                # 没读到码：再试一次（可能是角度/光照问题），仍不行就报警
                self._log("warn", "过检未读到码，重扫一次")
                time.sleep(0.4)
                insp = self.inspect_grabbed(med.code)
                if not insp.get("ok"):
                    self._say("无法确认药品身份，已放回", wait=True)
                    self.return_to_slot(med.slot)
                    self.fault("过检位无法识别二维码")
                    return {"status": "fault", "medicine": med,
                            "detail": "二维码识别失败", "mode": self.work_mode}

        # ---------- ⑦' 底盘退出货架（★ 本项目的取药动作，见 app.yaml 的 lift_dz）----------
        #   夹爪已经夹紧药瓶，但药瓶还在货架的槽里。机械臂【不抬高】
        #   （上层抬高会超出臂展），而是让底盘退开，药瓶就被从槽里"拔"出来。
        #   这是"底盘代替机械臂补齐自由度"最直接的体现。
        #   ★ 必须在过检位扫码【之前】做：药还在槽里时举到过检位会把药瓶刮掉。
        #
        #   ★ 两种模式在这里同样分叉（规则 4：manual_phone 不生成底盘指令）：
        #     auto         -> PC 发 $MV 让底盘后退，把药瓶拔出来
        #     manual_phone -> 只提示操作人员"往后退一点"，由人来开底盘
        if (self.work_mode == "auto" and self.chassis is not None
                and not self.chassis_auto_retreat):
            # ★ 用户流程（2026-10-22）：夹住药后姿态冻结、不发任何指令，
            #   出槽由人工 HC05 开车完成（车一后退药瓶即出槽）。
            self._log("step", "跳过自动退离（chassis.auto_retreat=false）——"
                              "保持夹持姿态，人工 HC05 接管")
        elif self.work_mode == "auto" and self.chassis is not None:
            self._log("step", "底盘退离货架（把药瓶从槽中取出）")
            # 后退 = 远离货架方向。距离取抓取位到安全距离的估计值，
            # 用速度为 1.0 的短脉冲，真机按实际货架深度调（见 chassis 段注释）。
            back_ms = int((self.cfg.get("chassis") or {}).get("retreat_ms", 600))
            if not self.chassis.move_forward(-1.0, back_ms):
                self._log("warn", "底盘退离失败，但在架外用机械臂搬运仍可继续")
            else:
                time.sleep(back_ms / 1000.0 + 0.3)   # 到点自停，不发 $MS 截杀
                self._settle(extra_ms=200)
        elif self.work_mode == "manual_phone" and self.chassis is not None:
            # 规则 4：不生成底盘指令。药瓶已经夹在夹爪里，但人必须把小车
            # 【往后开一点】，药瓶才会从槽里脱出来。这一步由人来做。
            self._log("step", "[manual_phone] 请人工把小车向后移动约 10cm，把药瓶从槽中取出")
            if not self.wait_human_takeaway():
                # 此刻药已夹在爪里，不能就这么走人 —— 先放回槽位再退出。
                self.stats["cancel"] += 1
                self._say("已取消，正在把药品放回", wait=True)
                self.return_to_slot(med.slot)
                return {"status": "cancel", "medicine": med,
                        "detail": "人工退开车阶段取消", "mode": self.work_mode}
        else:
            self._log("warn", "无底盘控制器，跳过「退离货架」动作；"
                              "如果药瓶仍卡在槽里，过检位扫码会失败")

        # ---------- ⑧ 出库播报 + HC05 人工放药 + 原地扫码核验 ----------
        # 流程（2026-10-22 用户定义）：夹住药 = PC 停止控制机械臂 -> 人工 HC05
        # 把小车/臂开到出库位放药 -> 说"好了" -> 摄像头【原地】扫码核验
        # -> 播报该药的服用/使用方法 -> 出库完成。
        self._say("已为您取到%s。请用手机蓝牙控制底盘移动，再控制舵机放下药品"
                  % med.name, wait=True)
        self._log("info", "已夹取药品: %s。等待 HC05 人工放药，放好后原地扫码出库…"
                  % med.name)

        confirmed, scanned = self.prompt_hc05_place(
            scan_code=(med.code if not self.require_scan_confirm else None))
        if not confirmed:
            self.stats["cancel"] += 1
            self._say("已取消放药", wait=True)
            self.close_scan_window()
            self.arm.home()
            return {"status": "cancel", "medicine": med,
                    "detail": "人工放药阶段取消", "mode": self.work_mode}

        # ★ 二次人工出库扫码核验（require_scan_confirm=false 时执行）：
        #   HC05 阶段预览窗常开、后台线程持续扫当前画面，读到期望码即锁定，
        #   用户在窗口里能实时看到"扫没扫上"；说"好了"时还没读到的话，
        #   再给一次原地补扫机会（零机械臂指令，姿态保持不变）。
        #   （require_scan_confirm=true = 原设计，抓取阶段已在过检位扫过）
        if not self.require_scan_confirm:
            if scanned == med.code:
                self._log("info", "出库扫码核验通过（HC05 阶段读到）: %s" % scanned)
            else:
                insp = self.inspect_grabbed(med.code, move_to_inspect=False)
                if insp.get("ok") and insp.get("code"):
                    self._log("info", "出库扫码核验通过（放好后再扫）: %s" % insp["code"])
                elif insp.get("code"):
                    self._log("warn", "出库扫码读到 %s，与期望 %s 不一致"
                              "（人工出库流程，以人工核对为准）"
                              % (insp.get("code"), med.code))
                else:
                    self._log("warn", "出库扫码未读到码"
                              "（人工出库流程，不影响放药完成）")

        # ★ 出库完成，关闭扫码预览窗（用户要求：扫到码播报后窗口关闭）
        self.close_scan_window()

        # ★ 出库时播报该药的服用/使用方法（用户流程定义：扫码出库时播报）
        if med.note:
            self._say("%s的用法：%s" % (med.name, med.note), wait=True)

        self.kb.consume(med.code, 1)
        self.arm.home()
        self.stats["success"] += 1
        self._log("info", "取药成功: %s（剩余 %d）" % (med.name, med.stock))
        return {"status": "success", "medicine": med,
                "detail": "OK", "mode": self.work_mode}

    # ==================================================================
    def emergency_stop(self) -> None:
        """外部调用：立即急停（规则 5：机械臂 + 底盘一起停）"""
        self.emergency = True
        try:
            self.arm.estop()
        except Exception:
            pass
        if self.chassis is not None:
            try:
                self.chassis.stop()
            except Exception:
                pass
        self._log("warn", "外部触发急停（机械臂 + 底盘）")
