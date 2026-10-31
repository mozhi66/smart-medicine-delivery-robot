#!/usr/bin/env python3.11
# -*- coding: utf-8 -*-
# =============================================================================
# ⚠ 本文件为片段留存，内容不完整
#    自 _verdict 函数起至 step_list 函数体中部为止；
#    不含 main 入口，无法独立运行，仅作参考。
# =============================================================================
def _verdict(b: float) -> Tuple[str, str]:
    """返回 (判定文字, 严重级别)，级别用于最后汇总：ok / warn / bad"""
    if b < DARK_BLACK:
        return "全黑（视觉链路必然失效）", "bad"
    if b < DARK_LOW:
        return "偏暗（识别率会明显下降）", "warn"
    if b > BRIGHT_HOT:
        return "过曝（高光溢出，二维码易糊）", "bad"
    if b > BRIGHT_OK:
        return "偏亮", "warn"
    return "正常", "ok"


def _open_capture(index: int, backend: Optional[int] = None,
                  width: int = 0, height: int = 0,
                  fps: Optional[float] = None) -> Optional[cv2.VideoCapture]:
    """
    开一个裸 VideoCapture（用于探测/扫描，不走 Camera 类）。

    与 pc/vision/camera.py._open_capture 同策略：先锁 MJPG，再设分辨率；
    backend=None 时 Windows 上优先 MSMF，失败再 DSHOW。
    """
    if backend is not None:
        candidates = ((backend, "forced"),)
    elif os.name == "nt":
        candidates = (
            (getattr(cv2, "CAP_MSMF", 1400), "MSMF"),
            (getattr(cv2, "CAP_DSHOW", 700), "DSHOW"),
        )
    else:
        candidates = ((None, "AUTO"),)

    for api, _name in candidates:
        try:
            if api is None:
                cap = cv2.VideoCapture(int(index))
            else:
                cap = cv2.VideoCapture(int(index), api)
        except TypeError:
            cap = cv2.VideoCapture(int(index))
        except Exception:
            continue
        if cap is None or not cap.isOpened():
            try:
                cap.release()
            except Exception:
                pass
            continue
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        except Exception:
            pass
        if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        if fps:
            cap.set(cv2.CAP_PROP_FPS, float(fps))
        ok, _ = cap.read()
        if ok:
            return cap
        try:
            cap.release()
        except Exception:
            pass
    return None


def resolve_backend(name: str) -> Optional[int]:
    """
    把 --backend 的名字解析成 cv2 常量。'auto' 返回 None（交给 OpenCV 选）。

    为什么默认用 DSHOW（Windows）
    -----------------------------
    实测（本机内置相机，1280x720）：
        MSMF  : cap.get(CAP_PROP_EXPOSURE) 恒返回 -5.0（读回不可靠），
                但画面亮度随设置正确变化；
        DSHOW : 读回值精确跟随设置值（-8/-6/-5/-4/-2/0 逐档对应），
                画面亮度与 MSMF 完全一致。
    也就是说**曝光效果**两个后端相同，但只有 DSHOW 能让你验证
    "我设的曝光到底生效了没有"。

    ⚠️ 但白平衡完全不同（2026-09-13 实测，1280x720，exposure=-1）：
        | 后端  | auto_wb=1 | auto_wb=0 |
        | MSMF  | R-B -32.0 | R-B -31.8 |   ← 设 1 和设 0 一样，AUTO_WB 是哑的
        | DSHOW | R-B +11.7 | R-B -31.4 |   ← 自动白平衡真正工作
    所以 MSMF 下画面**恒定偏蓝**，依赖 HSV 色相的颜色判据会整体失效。
    （二维码走灰度，不受影响。）

    因此 `pc/vision/camera.py` 也已在 Windows 上默认解析为 DSHOW
    （`resolve_backend(None)` → CAP_DSHOW），与本工具保持一致。
    """
    n = (name or "auto").lower()
    if n in ("dshow", "directshow"):
        return getattr(cv2, "CAP_DSHOW", None)
    if n in ("msmf",):
        return getattr(cv2, "CAP_MSMF", None)
    if n in ("auto", "default", "any"):
        return None
    return None


def default_backend_name() -> str:
    """本平台下工具默认使用的后端名（仅用于打印）"""
    return "DSHOW" if os.name == "nt" else "auto"


def default_backend() -> Optional[int]:
    """本平台下工具默认使用的后端常量"""
    return resolve_backend("dshow") if os.name == "nt" else None



def _warmup(cap: cv2.VideoCapture, n: int = 10) -> Optional[np.ndarray]:
    """预热若干帧后取一帧（相机刚开时前几帧参数没生效、画面偏暗）"""
    f = None
    for _ in range(n):
        ok, fr = cap.read()
        if ok and fr is not None:
            f = fr
    return f


# ============================================================================
# 步骤 1：枚举摄像头
# ============================================================================

def enumerate_cameras(max_index: int = 6,
                      backend: Optional[int] = None) -> List[dict]:
    """
    逐个 index 试开，返回可用摄像头列表。

    返回: [{'index','width','height','fps','backend'}, ...]
    """
    found: List[dict] = []
    for i in range(max_index):
        cap = _open_capture(i, backend)
        if cap is None:
            continue
        f = _warmup(cap, 6)
        if f is None:
            cap.release()
            continue
        h, w = f.shape[:2]
        try:
            be = cap.getBackendName()
        except Exception:
            be = "?"
        found.append({
            "index": i,
            "width": w,
            "height": h,
            "fps": float(cap.get(cv2.CAP_PROP_FPS) or 0.0),
            "backend": be,
        })
        cap.release()
    return found


def step_list(cams: List[dict]) -> None:
    _hr("① 摄像头枚举")
    if not cams:
        print("  未发现任何摄像头。")
        print("  → 检查：USB 是否插好 / 摄像头是否被其他程序占用（钉钉、腾讯会议、相机 App）")
        return
    print("  发现 %d 个可用摄像头：" % len(cams))
