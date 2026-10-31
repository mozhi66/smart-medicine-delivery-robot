#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
"""
================================================================================
 智能药箱主控程序 (main.py) —— PyCharm 里直接运行这个
================================================================================
用法
----
    # 全自动真机（真麦克风 + 真摄像头 + 真 TTS + STM32@COM3）
    python pc/main.py --real-mic --real-tts --real-cam --show --port COM3

    # 全仿真模式（不接任何硬件，用假串口/假相机/假语音，跑通全流程）
    python pc/main.py --mock

    # 真相机 + 假机械臂（扫码链路彩排，不动硬件）
    python pc/main.py --mock --real-cam --real-tts --show --once

    # 只处理一句话就退出（自动化测试用）
    python pc/main.py --mock --once --text "我感冒了"

    # 查看可用串口 / 麦克风
    python pc/main.py --list-ports      python pc/main.py --list-mics

设计说明
--------
本文件只做"组装"：读配置 -> 造各个模块 -> 交给状态机。
业务逻辑全在 core/state_machine.py，改流程不要动这里。

--mock 模式的意义
-----------------
它让整套系统在没有任何硬件的情况下完整跑一遍：
语音用键盘输入代替、相机用合成图片代替、串口只打印不发。
第一次拿到代码、答辩前一天晚上、换了台电脑，都应该先跑 --mock。

--real-cam 与扫码预览窗（--show）
---------------------------------
真摄像头下扫码是"人举标签"的交互，没有窗口就不知道有没有扫上。
所以 --real-cam 时默认弹扫码预览窗（--no-show 可关）；
图片回放（--cam-image）画面是钉死的，默认不弹。
================================================================================
"""

from __future__ import annotations

import argparse
import os
import sys

# ---- 保证无论从哪里启动都能 import 到 pc 包 ----
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import numpy as np                                    # noqa: E402

from core.logger import Logger                        # noqa: E402
from core.medicine_kb import MedicineKB               # noqa: E402
from core.pose_table import PoseTable, generate_poses  # noqa: E402
from core.state_machine import MedicineBoxSystem      # noqa: E402
from core.envcheck import report as env_report        # noqa: E402
from comm.arm_controller import ArmController, MockArmController  # noqa: E402
from comm.chassis_controller import (ChassisController,           # noqa: E402
                                     MockChassisController)
from llm.glm_client import GLMClient                  # noqa: E402

try:
    import yaml
    _HAS_YAML = True
except Exception:                                     # pragma: no cover
    yaml = None
    _HAS_YAML = False


def load_config(path: str) -> dict:
    """
    读取 YAML 配置。文件不存在或没装 pyyaml 时返回一套内置默认值，
    保证程序不会在启动阶段就崩。
    """
    if _HAS_YAML and os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    print("[配置] 未找到 %s，使用内置默认配置" % path)
    return {
        "serial": {"port": "COM3", "baud": 115200, "timeout": 3.0},
        "camera": {"index": 0, "width": 1280, "height": 720,
                   "fps": 30, "exposure": -6},
        "vision": {"require_slot_check": True,
                   "inspect_roi": [340, 120, 600, 480],
                   "scan_hold_s": 60.0, "scan_timeout_s": 12.0,
                   "scan_frames": 80, "scan_interval_s": 0.0,
                   "scan_min_votes": 2},
        "chassis": {"speed": 0.35, "col_pitch_mm": 600.0,
                    "speed_scale_mm_s": 100.0, "move_timeout_s": 30.0,
                    "col_qr_prefix": "COL", "align_tol_mm": 6.0,
                    "align_retry": 2, "retreat_ms": 600},
        "work_mode": {"mode": "auto"},
        "timing": {"move_ms": 900, "settle_ms": 350, "scan_frames": 80},
        "retry": {"max_attempts": 2, "delay_s": 0.5},
        "llm": {"provider": "zhipu", "base_url": None, "api_key": None,
                "api_key_env": "ZHIPU_API_KEY", "model": "glm-4-flash",
                "timeout": 45, "retries": 2},
        "audio": {"asr_model": "paraformer-zh", "vad_model": "fsmn-vad",
                  "punc_model": "ct-punc", "tts_backend": "auto"},
        "paths": {"medicines": "config/medicines.json",
                  "poses": "config/poses.json",
                  "slots": "config/slots.yaml",
                  "log": "logs/run.log"},
    }


def _abs(path: str) -> str:
    """把配置里的相对路径解析成相对项目根目录的绝对路径"""
    return path if os.path.isabs(path) else os.path.join(_ROOT, path)


def _medicine_names(cfg: dict):
    """读取药品名列表，用作 ASR 热词，提升识别率"""
    try:
        kb = MedicineKB.load(_abs((cfg.get("paths") or {}).get(
            "medicines", "config/medicines.json")))
        return [m.name for m in kb.all()]
    except Exception:
        return []


# ============================================================================
# 构建各模块
# ============================================================================
def build_asr(cfg: dict, mock: bool, input_device=None):
    """构造语音识别引擎。失败时降级为键盘输入（mock）"""
    try:
        from audio.asr import ASREngine
        a = cfg.get("audio") or {}
        return ASREngine(model=a.get("asr_model", "paraformer-zh"),
                         vad_model=a.get("vad_model", "fsmn-vad") or "",
                         punc_model=a.get("punc_model", "ct-punc") or "",
                         mock=mock,
                         input_device=input_device,
                         hotwords=" ".join(
                             m for m in _medicine_names(cfg)))
    except TypeError:
        # 上游 ASREngine 不认识 vad/punc/input_device 时逐级降级
        try:
            from audio.asr import ASREngine
            a = cfg.get("audio") or {}
            return ASREngine(model=a.get("asr_model", "paraformer-zh"),
                             mock=mock,
                             hotwords=" ".join(
                                 m for m in _medicine_names(cfg)))
        except Exception as e:
            print("[启动] ASR 初始化失败（%s），降级为键盘输入模式" % e)
            return _KeyboardASR()
    except Exception as e:
        print("[启动] ASR 初始化失败（%s），降级为键盘输入模式" % e)
        return _KeyboardASR()


def build_tts(cfg: dict, mock: bool, want_real: bool = False):
    """构造语音合成引擎。失败时降级为只打印"""
    try:
        from audio.tts import TTSEngine
        a = cfg.get("audio") or {}
        try:
            return TTSEngine(backend=a.get("tts_backend", "auto"),
                             mock=mock, want_real=want_real)
        except TypeError:
            # 上游签名没有 want_real：--real-tts 时强制非 mock
            return TTSEngine(backend=a.get("tts_backend", "auto"),
                             mock=mock and not want_real)
    except Exception as e:
        print("[启动] TTS 初始化失败（%s），降级为打印模式" % e)
        return _PrintTTS()


class _KeyboardASR:
    """兜底 ASR：用键盘输入代替语音识别（无麦克风也能跑）"""

    def recognize_once(self, timeout: float = 6.0, **kw):
        try:
            return input("请输入指令 > ").strip() or None
        except EOFError:
            return None

    def recognize_file(self, path: str):
        return None

    def close(self):
        pass


class _PrintTTS:
    """兜底 TTS：只打印不发声"""

    def speak(self, text: str, wait: bool = True) -> bool:
        print("[TTS] %s" % text)
        return True

    def speak_async(self, text: str) -> None:
        print("[TTS] %s" % text)

    def close(self):
        pass


def build_camera(cfg: dict, mock: bool, cam_real: bool = False,
                 cam_images=None, expect_code=None):
    """
    构造相机。优先级：图片回放(--cam-image) > 真实相机(--real-cam) > 合成。

    mock 且无 --real-cam/--cam-image 时返回"合成药瓶场景"假相机，
    画面里真的有二维码，所以过检位扫码能真正跑通。
    """
    # ---- ① 图片回放（钉死输入，排除硬件干扰）----
    if cam_images:
        from vision.image_cam import ImageCamera
        from vision.qr_detector import make_qr_image

        def _resolve(item: str):
            """PATH 或 药品码（自动去 output/qr_labels/ 找 label_CODE.png）"""
            if os.path.isfile(item):
                return item
            cand = os.path.join(_ROOT, "output", "qr_labels",
                                "label_%s.png" % item)
            if os.path.isfile(cand):
                return cand
            return item          # 交给 ImageCamera 报读不了

        paths = [_resolve(str(x)) for x in cam_images]
        return ImageCamera.from_paths(paths, loop=True)

    # ---- ② 真实相机 ----
    if cam_real or not mock:
        from vision.camera import Camera, parse_backend
        c = cfg.get("camera") or {}
        try:
            return Camera(index=int(c.get("index", 0)),
                          width=int(c.get("width", 1280)),
                          height=int(c.get("height", 720)),
                          fps=int(c.get("fps", 30)),
                          exposure=c.get("exposure", -6),
                          backend=parse_backend(c.get("backend", "auto")),
                          auto_exposure=bool(c.get("auto_exposure", True)),
                          ae_target_lo=c.get("ae_target_lo", 90),
                          ae_target_hi=c.get("ae_target_hi", 160),
                          ae_min=c.get("ae_min", -13),
                          ae_max=c.get("ae_max", 0),
                          ae_settle_frames=c.get("ae_settle_frames", 5),
                          auto_wb=bool(c.get("auto_wb", True)),
                          wb_settle_frames=c.get("wb_settle_frames", 45))
        except TypeError:
            # 上游 Camera 签名较简：退回基础参数
            return Camera(index=c.get("index", 0),
                          width=c.get("width", 1280),
                          height=c.get("height", 720),
                          fps=c.get("fps", 30),
                          exposure=c.get("exposure", -6))

    # ---- ③ 合成二维码（mock）----
    from vision.camera import MockCamera
    from vision.qr_detector import make_qr_image
    import cv2
    code = expect_code or "MED001"

    def _scene():
        img = np.full((420, 420, 3), 205, np.uint8)
        qr = make_qr_image(code, size=170)
        if qr is None:
            return img
        q = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
        q = cv2.resize(q, (240, 240))
        img[90:330, 90:330] = q
        return img

    return MockCamera([_scene()], loop=True)


def _camera_is_real(mock: bool, cam_real: bool, cam_images) -> bool:
    """本次运行是否用真实相机（决定要不要显式 open + 打日志）"""
    if cam_images:
        return False
    return bool(cam_real) or not mock


def build_system(cfg: dict, mock: bool = False, port: str = None,
                 verbose: bool = True, llm_real: bool = False,
                 asr_real: bool = False, tts_real: bool = False,
                 mic_device=None, cam_real: bool = False,
                 cam_images=None, show_scan_window: bool = False,
                 scan_hold_s: float = 60.0,
                 mode: str = None, chassis_port: str = None) -> MedicineBoxSystem:
    """
    组装整个系统。

    参数
    ----
    cfg        : 配置字典
    mock       : 全仿真模式
    port       : 机械臂串口号（覆盖配置）
    llm_real   : mock 模式下也走真实大模型（--real-llm）
    asr_real   : mock 模式下也用真实麦克风（--real-mic）
    tts_real   : mock 模式下也真发声（--real-tts）
    mic_device : 麦克风设备（序号或名字子串；None=系统默认）
    cam_real   : 用真实摄像头（--real-cam）
    cam_images : 图片回放列表（--cam-image，优先级高于 cam_real）
    show_scan_window : 过检位扫码时弹实时预览窗口（--show）
    scan_hold_s : 弹窗时，正式扫码前的"举标签倒计时"秒数
    mode       : 工作模式覆盖（auto / manual_phone）
    chassis_port : 底盘串口号（默认与机械臂同口，同一块 STM32）
    """
    paths = cfg.get("paths") or {}

    log = Logger(_abs(paths.get("log", "logs/run.log")), echo=verbose)

    # --- 知识库 ---
    kb = MedicineKB.load(_abs(paths.get("medicines", "config/medicines.json")))
    if not kb.all():
        log.warn("药品库为空，请检查 config/medicines.json")
    log.info("药品库加载完成：%d 种药品，类别 %s"
             % (len(kb.all()), kb.categories()))

    # --- 点位表 ---
    poses = PoseTable.load(_abs(paths.get("poses", "config/poses.json")))
    if not poses.slots:
        log.warn("点位表为空，将使用内置默认点位（建议先运行 tools/gen_poses.py）")
        from core.kinematics import ArmGeometry
        positions = _default_slot_positions(len(kb.all()) or 8)
        poses = PoseTable(generate_poses(positions, ArmGeometry()))
    log.info("点位表加载完成：%d 个槽位" % len(poses.slots))

    # --- 机械臂（与底盘共用一块 STM32：USART1 $-帧）---
    s = cfg.get("serial") or {}
    if mock:
        arm = MockArmController(port or s.get("port", "COM3"),
                                baud=s.get("baud", 115200), verbose=verbose)
        log.info("机械臂: 仿真（指令只打印不发串口）")
    else:
        arm = ArmController(port or s.get("port", "COM3"),
                            baud=s.get("baud", 115200),
                            timeout=s.get("timeout", 3.0), verbose=verbose)
        if not arm.connect():
            log.error("串口连接失败，请检查 COM 号和接线；可用端口:\n%s"
                      % ArmController.format_ports())
            raise SystemExit(1)
        if not arm.wait_ready():
            log.error("STM32 无响应：既没收到 READY，$QRY,POSE 探测也无回帧。"
                      "请检查固件/波特率/接线；可先运行 "
                      "python tools/serial_listen.py --port %s 旁听定位" % arm.port)
            raise SystemExit(1)
        log.info("STM32 已就绪")

    # --- 底盘（融合固件：与机械臂同一块 STM32，默认共用串口）---
    chassis = None
    if mock:
        chassis = MockChassisController(port or s.get("port", "COM3"),
                                        baud=s.get("baud", 115200),
                                        verbose=verbose)
        chassis.connect()
        log.info("麦轮底盘: 仿真（不接硬件）")
    else:
        cport = chassis_port or port or s.get("port", "COM3")
        # 若底盘与机械臂是同一串口，就复用 arm 上的连接（同一个 ser 对象），
        # 避免两个线程抢同一个 COM 口 —— 那是必然收不到正确回帧的。
        same_port = (cport == getattr(arm, "port", None))
        if same_port and getattr(arm, "ser", None) is not None:
            chassis = ChassisController.__new__(ChassisController)
            # 浅共享 arm 的串口/线程/队列：两块硬件共用一条链路
            chassis.__dict__.update(arm.__dict__)
            chassis.verbose = verbose
            chassis.work_mode = "auto"
            chassis.last_error = ""
            log.info("麦轮底盘: 与机械臂共用串口 %s（合并固件，同一块 STM32）" % cport)
        else:
            chassis = ChassisController(cport, baud=s.get("baud", 115200),
                                        timeout=s.get("timeout", 3.0),
                                        verbose=verbose)
            if not chassis.connect():
                log.warn("底盘串口 %s 打不开 —— auto 模式将无法自动对位。"
                         "manual_phone 模式不受影响（不控底盘）" % cport)
                chassis = None
            else:
                log.info("麦轮底盘: 独立串口 %s" % cport)

        cap = (cfg.get("chassis") or {}).get("speed_cap_pct")
        if chassis is not None and cap:
            # 可选：运行时用 $MC 调速度上限（固件默认 60%）。横移吃电流，
            # 若轮子偶发堵转可提到 80~100 增大扭矩；但会加大电池压降，
            # 供电不稳时慎用。
            try:
                chassis.set_speed(int(cap))
                log.info("底盘速度上限已设为 %d%%（$MC）" % int(cap))
            except Exception as e:
                log.warn("设置速度上限失败（不影响运行）: %s" % e)

    # --- 语音 / 大模型 ---
    # 注意：mock 只决定"用不用假硬件"，语音/大模型各自可以被 xxx_real 单独拉回真实。
    asr = build_asr(cfg, mock and not asr_real, input_device=mic_device)
    tts = build_tts(cfg, mock and not tts_real, want_real=bool(tts_real) or not mock)
    if mock and asr_real:
        log.info("语音输入: 真实麦克风 + FunASR（--real-mic）")
    elif mock:
        log.info("语音输入: 键盘输入（mock）")
    if mock and tts_real:
        log.info("语音播报: 真实扬声器（--real-tts）")
    if mock and asr_real and getattr(asr, "is_mock", False):
        log.warn("你要的是真实麦克风，但 FunASR 没起来 -> 已退化成键盘输入。"
                 "先跑 `python tools/check_env.py` 看缺什么")
    l = cfg.get("llm") or {}
    if not isinstance(l, dict):
        # 常见于把 `model: glm-4-flash` 误写成 `model:glm-4-flash`（冒号后缺空格），
        # 整段会被 YAML 当成一个字符串，不会报错但读不出任何字段。
        log.warn("config/app.yaml 的 llm 段不是键值对（多半是冒号后漏了空格），"
                 "已改用默认配置：provider=zhipu / glm-4-flash")
        l = {}
    # ★ 供应商 / 地址 / 密钥 / 模型 全部来自 config/app.yaml，代码不写死。
    #   换模型只改 app.yaml（或用 tools/set_provider.py 一键切）。
    _glm_kw = dict(model=l.get("model") or "glm-4-flash",
                   timeout=l.get("timeout", 12),
                   retries=l.get("retries", 2),
                   categories=kb.categories(),
                   mock=(mock and not llm_real), verbose=verbose)
    if l.get("api_key"):
        _glm_kw["api_key"] = l["api_key"]
    glm = None
    try:
        # 融合版 GLMClient：支持多供应商（provider/base_url/prompt_template）
        glm = GLMClient(provider=l.get("provider", "zhipu"),
                        base_url=l.get("base_url") or None,
                        api_key_env=l.get("api_key_env") or None,
                        prompt_template=l.get("prompt_template") or None,
                        **_glm_kw)
    except TypeError:
        # 上游 GLMClient（单供应商智谱，端点硬编码）：退回基础签名
        try:
            glm = GLMClient(**_glm_kw)
        except Exception as e:
            log.warn("大模型客户端初始化失败（%s）-> 本地关键词规则" % e)
            glm = GLMClient(categories=kb.categories(), mock=True, verbose=verbose)
    if glm is None:
        glm = GLMClient(categories=kb.categories(), mock=True, verbose=verbose)
    if verbose:
        try:
            log.info("大模型: %s / %s  (%s)"
                     % (getattr(glm, "provider", "zhipu"),
                        getattr(glm, "model", "?"),
                        getattr(glm, "endpoint", "智谱官方端点")))
        except Exception:
            pass

    # --- 视觉 ---
    camera = None
    qr = None
    monitor = None
    try:
        camera = build_camera(cfg, mock, cam_real=cam_real,
                              cam_images=cam_images)
        from vision.qr_detector import QRDetector
        qr = QRDetector()
        log.info("二维码解码后端: %s" % qr.backend_name)
        if cam_images:
            log.info("相机来源: 图片回放（%s）—— 画面已钉死，"
                     "排除光照/手抖/帧率干扰" % camera.describe())
        elif cam_real or not mock:
            log.info("相机来源: 真实 USB 相机 (index %s, %s)"
                     % (getattr(camera, "index", "?"),
                        "DSHOW" if getattr(camera, "backend", None) is not None
                        else "auto"))
        else:
            log.info("相机来源: 合成二维码（mock 假相机）")

        # ★ 真实相机必须**显式 open()** —— Camera.__init__ 只存参数，不打开设备。
        #   漏掉这一步的后果是彻底静默的：
        #     cam.cap 恒为 None → read() 恒返回 (False, None)
        #     → 过检位永远"未识别到二维码" → 每单都走 FAULT
        #   而日志里只会写"无法确认药品身份"，看不出是相机根本没开。
        #   这里**故意不把 camera 置 None**：置 None 会让 inspect_grabbed
        #   返回 no_vision 并【判定通过】，等于静默跳过安全闭环（fail-open）。
        #   保持现状则扫码必然失败 → FAULT（fail-closed），这才是对的。
        if _camera_is_real(mock, cam_real, cam_images) and hasattr(camera, "open"):
            if camera.open():
                log.info("相机已打开: index=%s 后端=%s 曝光=%s 白平衡=%s"
                         % (getattr(camera, "index", "?"),
                            getattr(camera, "backend_name", None)
                            or ("DSHOW" if getattr(camera, "backend_used", None)
                                else "auto"),
                            getattr(camera, "exposure", None),
                            "自动" if getattr(camera, "auto_wb", True) else "手动"))
            else:
                log.error("相机打不开（index %s）—— 过检位扫码必然失败。"
                          "先跑 tools/camera_check.py --list 确认序号，"
                          "并关掉可能占用摄像头的软件（会议/相机 App）"
                          % getattr(camera, "index", "?"))
    except Exception as e:
        if isinstance(e, ImportError) or "cv2" in str(e) or "numpy" in str(e):
            # 这是最常见的"解释器选错"症状：视觉整块不可用，但程序照样能跑，
            # 于是很容易一路跑到答辩才发现"过检位从来没生效过"。所以要说重。
            log.warn("视觉模块不可用（%s）" % e)
            log.warn("  → 取药流程仍能跑，但【过检位二次确认 / 空槽检测会失效】，"
                     "安全闭环缺一环")
            log.warn("  → 多半是解释器选错了。当前: %s" % sys.executable)
            log.warn("  → 推荐: python")
        else:
            log.warn("视觉模块初始化失败（%s），将跳过视觉确认" % e)
        camera = qr = monitor = None

    if camera is not None and _camera_is_real(mock, cam_real, cam_images):
        try:
            from vision.slot_monitor import build_from_config
            slots_cfg = {}
            if _HAS_YAML and os.path.isfile(_abs(paths.get("slots", ""))):
                with open(_abs(paths["slots"]), "r", encoding="utf-8") as f:
                    slots_cfg = yaml.safe_load(f) or {}
            if slots_cfg.get("slots"):
                monitor = build_from_config(slots_cfg)
                log.info("槽位 ROI 加载完成：%d 个" % len(slots_cfg["slots"]))
            else:
                log.warn("未找到槽位标定文件 config/slots.yaml，"
                         "槽位有无检测将跳过（请运行 tools/calibrate_slots.py）")
        except Exception as e:
            log.warn("槽位监测器初始化失败: %s" % e)

    # --- 工作模式（--mode 覆盖配置文件；两种模式互斥）---
    if mode:
        cfg = dict(cfg)
        wm = dict(cfg.get("work_mode") or {})
        wm["mode"] = str(mode)
        cfg["work_mode"] = wm
    _wm = (cfg.get("work_mode") or {}).get("mode", "auto")
    if _wm == "auto":
        if chassis is None:
            log.warn("当前是 auto 模式但底盘控制器不可用 —— 取药会在"
                     "「底盘对位」这一步失败。改用 --mode manual_phone "
                     "可跳过底盘自动控制，由人工用手机对位")
        else:
            log.info("工作模式: auto 全自动（PC 控底盘横移对位 + 列二维码校正）")
    else:
        log.info("工作模式: manual_phone 手机手动"
                 "（不生成底盘指令，语音提示人工对位）")

    return MedicineBoxSystem(cfg, arm=arm, asr=asr, tts=tts, glm=glm,
                             kb=kb, poses=poses, camera=camera,
                             qr=qr, monitor=monitor, log=log,
                             chassis=chassis,
                             show_scan_window=show_scan_window,
                             scan_hold_s=scan_hold_s)


def _default_slot_positions(n: int = 8, radius: float = 250.0,
                            span_deg: float = 105.0, z: float = 65.0) -> dict:
    """
    默认的弧形槽位坐标（与 tools/kinematics_check.py 的推荐布局一致）。
    真实项目里应该用实测值，这里只作为"没标定时"的兜底。
    """
    import math
    out = {}
    if n <= 1:
        return {1: (radius, 0.0, z)}
    step = span_deg / (n - 1)
    half = span_deg / 2.0
    for i in range(n):
        a = math.radians(-half + i * step)
        out[i + 1] = (radius * math.cos(a), radius * math.sin(a), z)
    return out


# ============================================================================
# 入口
# ============================================================================
def main():
    ap = argparse.ArgumentParser(
        description="智能药箱主控程序（语音 -> 大模型 -> 抓药 -> 扫码出库 -> 播报）")
    ap.add_argument("--config", default="config/app.yaml", help="配置文件路径")
    ap.add_argument("--port", default=None, help="串口号，如 COM3，覆盖配置文件")
    ap.add_argument("--chassis-port", default=None,
                    help="底盘串口号。默认与机械臂同口（合并固件，同一块 STM32）；"
                         "底盘单独接板时才需要指定")
    ap.add_argument("--mode", default=None, choices=["auto", "manual_phone"],
                    help="工作模式覆盖：auto=PC控底盘对位；"
                         "manual_phone=人工用手机对位（不发底盘指令）")
    ap.add_argument("--mock", action="store_true",
                    help="全仿真模式，不接硬件（串口只打印、相机合成、语音键盘）")
    ap.add_argument("--real-llm", dest="real_llm", action="store_true",
                    help="mock 模式下也调用真实大模型（默认 mock 用本地关键词规则）")
    ap.add_argument("--real-mic", dest="real_mic", action="store_true",
                    help="mock 模式下也用真实麦克风（FunASR 识别）")
    ap.add_argument("--real-tts", dest="real_tts", action="store_true",
                    help="mock 模式下也用真实扬声器播报")
    ap.add_argument("--real-cam", dest="real_cam", action="store_true",
                    help="mock 模式下也用真实 USB 摄像头"
                         "（演示「真摄像头识别 + 假机械臂」）")
    ap.add_argument("--cam-image", dest="cam_image", action="append", default=None,
                    metavar="PATH|CODE",
                    help="用图片当相机画面（可重复给多张，循环播放）。可写完整路径，"
                         "也可直接写药品码如 MED003（自动去 output/qr_labels/ "
                         "找 label_MED003.png）。优先级高于 --real-cam。"
                         "用途：把识别链路钉死在已知输入上，排除光照/手抖/帧率干扰")
    ap.add_argument("--mic-device", default=None,
                    help="指定录音用的输入设备：序号（如 2）或名字子串（如 TWS）；"
                         "不填 = 系统默认输入设备")
    ap.add_argument("--list-mics", action="store_true",
                    help="列出所有可录音的输入设备后退出")
    ap.add_argument("--once", action="store_true", help="只处理一轮就退出")
    ap.add_argument("--text", default=None, help="直接给定用户说的话（跳过语音输入）")
    ap.add_argument("--list-ports", action="store_true", help="列出可用串口后退出")
    ap.add_argument("--show", dest="show", action="store_true", default=None,
                    help="过检位扫码时弹出**实时预览窗口**，让你看得见画面、"
                         "知道什么时候举标签、有没有扫上。"
                         "默认：用真摄像头（--real-cam）时自动开，其余情况不开")
    ap.add_argument("--no-show", dest="show", action="store_false",
                    help="强制不弹扫码预览窗口（无桌面/CI 环境用）")
    ap.add_argument("--hold", type=float, default=60.0, metavar="SEC",
                    help="举标签准备上限秒数（默认 60；有窗口时按【空格】立刻开始识别）")
    ap.add_argument("--quiet", action="store_true", help="减少日志输出")
    args = ap.parse_args()

    # 扫码预览窗口：
    #   显式 --show / --no-show 优先；
    #   否则【真机联调默认开】——没有窗口就不知道有没有扫上码。
    #   仅图片回放（--cam-image）默认关，因为画面是钉死的。
    if args.show is None:
        show_win = (bool(args.real_cam) or (not args.mock)) and (not args.cam_image)
    else:
        show_win = bool(args.show)

    cfg = load_config(_abs(args.config))

    if args.list_ports:
        print(ArmController.format_ports())
        return 0

    if args.list_mics:
        from audio.asr import ASREngine
        devs = ASREngine.list_input_devices()
        d = ASREngine.default_input_device()
        if not devs:
            print("没有检测到输入设备（sounddevice 没装？跑 check_env.py 看看）")
            return 0
        print("可用的输入设备（用 --mic-device 序号或名字子串来选）：")
        for x in devs:
            mark = "   <-- 当前默认" if d and x["index"] == d["index"] else ""
            print("  %3d  %s  (%dHz, %dch)%s"
                  % (x["index"], x["name"], x["sr"], x["ch"], mark))
        print("\n示例：--mic-device 2   或   --mic-device TWS")
        return 0

    mic_dev = None
    if args.mic_device:
        from audio.asr import ASREngine
        mic_dev = ASREngine.resolve_input_device(args.mic_device)
        if mic_dev is None:
            return 2
        print("[启动] 录音设备已指定：%s 号" % mic_dev)

    hw = "全仿真 MOCK" if args.mock else "真实硬件"
    brain = "真实调用" if (args.real_llm or not args.mock) else "本地规则(mock)"
    if args.mock:
        mouth = "真实扬声器" if args.real_tts else "打印(mock)"
        ear = "真实麦克风" if args.real_mic else "键盘输入(mock)"
        if args.cam_image:
            eye = "图片回放(%d 张)" % len(args.cam_image)
        elif args.real_cam:
            eye = "真实摄像头"
        else:
            eye = "合成二维码(mock)"
    else:
        ear, mouth, eye = "真实麦克风", "真实扬声器", "真实摄像头"

    _wm_show = (args.mode
                or (cfg.get("work_mode") or {}).get("mode", "auto"))
    _mode_txt = ("auto 全自动（PC 控底盘对位）" if _wm_show == "auto"
                 else "manual_phone 手机手动（人工对位）")
    print("=" * 70)
    print("智能药箱控制系统   硬件: %s   大模型: %s" % (hw, brain))
    print("                   语音输入: %s   语音播报: %s" % (ear, mouth))
    print("                   相机: %s" % eye)
    print("                   工作模式: %s" % _mode_txt)
    print("                   扫码预览窗口: %s" % ("开（--show）" if show_win else "关"))
    print("=" * 70)
    if show_win:
        print("提示：走到『过检位扫码确认』时会弹出一个预览窗口，"
              "里面画着黄色 ROI 框；")
        print("      把药品标签举进黄框、举稳，倒计时结束后自动识别，"
              "窗口里会显示结果。")
        print()

    # 环境自检：把"我用的是哪个 python、缺什么、该换成哪个"直接摊开讲。
    # 这一步专治"选了错的解释器 → 报 No module named 'cv2'"这类迷之红字。
    # 只校验【本次运行真正需要的包】，避免 --mock 时冲着一堆用不上的包报警。
    if not args.quiet:
        need = ["numpy", "yaml", "cv2"]
        if not args.mock:
            need += ["serial", "sounddevice", "funasr", "torch"]
        if args.real_mic:
            need += ["sounddevice", "funasr", "torch"]
        if args.real_tts:
            need += ["pyttsx3"]
        env_report(require=need)

    system = build_system(cfg, mock=args.mock, port=args.port,
                          verbose=not args.quiet, llm_real=args.real_llm,
                          asr_real=args.real_mic, tts_real=args.real_tts,
                          mic_device=mic_dev, cam_real=args.real_cam,
                          cam_images=args.cam_image,
                          show_scan_window=show_win,
                          scan_hold_s=args.hold,
                          mode=args.mode,
                          chassis_port=args.chassis_port)

    try:
        from core.exit_key import start as esc_start
        esc_start()
    except Exception:
        pass

    if args.once:
        try:
            r = system.run_once(user_text=args.text)
        finally:
            system.close_scan_window()      # 别让预览窗口留在屏幕上
        print("\n[结果] %s" % r)
        system.log.info("单轮结束: %s" % r["status"])
        system.log.close()
        return 0 if r["status"] == "success" else 2

    # 后台监测 ESC：没按就一直循环服务，按了就干净退出
    try:
        from core.exit_key import start as esc_start, requested as esc_req
    except Exception:
        def esc_start():
            pass

        def esc_req():
            return False
    esc_start()

    print("\n系统就绪。说出您的需求（ESC 或 Ctrl+C 退出；不按 ESC 会一直值班）\n")
    try:
        while True:
            if esc_req():
                print("\n收到 ESC，退出值班循环")
                break
            r = system.run_once(user_text=args.text)
            print("\n[本轮结果] %s\n" % r["status"])
            if esc_req():
                print("\n收到 ESC，退出值班循环")
                break
            if args.text:            # 测试模式给了固定文本，跑一轮就停
                break
    except KeyboardInterrupt:
        print("\n已退出")
    finally:
        try:
            from core.exit_key import stop as esc_stop
            esc_stop()
        except Exception:
            pass
        system.close_scan_window()
        try:
            system.arm.home()
            system.arm.close()
        except Exception:
            pass
        system.log.info("程序退出，统计: %s" % system.stats)
        system.log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
