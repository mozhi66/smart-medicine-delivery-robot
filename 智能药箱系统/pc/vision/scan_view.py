#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 扫码确认实时窗口 (scan_view.py)
================================================================================
职责：在**取药流程**（pc/main.py 的 ⑦ 过检位确认）里弹出一个实时窗口，
     让用户看得见画面，从而知道两件事：

       1) 什么时候该举二维码 —— 窗口里画着黄色 ROI 框 + 倒计时提示
       2) 有没有扫上 —— 每帧实时显示识别结果，最后给一个大字结论

为什么必须有它
--------------
取药流程默认是"无窗口"的：机械臂（哪怕是 mock）走到过检位后，
程序只会在终端打印一行"请把标签举到镜头前"，然后 0.3 秒内扫完 5 帧。
人根本来不及反应，也不知道举没举对位置 —— 体验极差。

而 USB 摄像头**不能被两个进程同时打开**，所以"一边开 tools/camera_preview.py
预览、一边跑 pc/main.py"是行不通的。预览必须由取药流程自己弹。

设计要点
--------
· 只在 `gui_available()` 为真时才真的开窗；否则所有方法退化成空操作，
  取药流程照常跑（容器/CI/无桌面环境不会崩）。
· 所有 cv2 调用都包在 try 里：**显示失败绝不影响识别与取药主流程**。
· 准备阶段（hold）给用户 3 秒倒计时举标签；投票阶段（on_frame）逐帧实时显示；
  结论阶段（verdict）把结果定住 1.8 秒，让人看清扫到的是哪个码。
================================================================================
"""

from __future__ import annotations

import time
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np

from .gui import TextRenderer, gui_available

# ---------------------------------------------------------------------------
# 配色（BGR）
# ---------------------------------------------------------------------------
C_OK = (80, 220, 80)         # 绿：成功 / 匹配
C_WARN = (60, 200, 255)      # 橙：警告 / 需要操作
C_BAD = (80, 80, 240)        # 红：失败 / 不匹配
C_ROI = (0, 220, 255)        # 黄：ROI 框
C_QR = (80, 255, 80)         # 亮绿：二维码框
C_DIM = (185, 185, 185)      # 灰：次要文字
C_WHITE = (245, 245, 245)


class ScanWindow:
    """
    取药流程里的"扫码确认"实时窗口。

    典型用法（state_machine.inspect_grabbed 内）::

        win = ScanWindow(roi=roi)
        win.open()
        win.hold(cam, 3.0, "请把药品标签举进黄框", "对准后举稳，马上开始识别")
        res = qr.robust_scan(cam, n=5, roi=roi, on_frame=win.on_frame)
        win.verdict(res.code if res else None, expect_code)
        win.close()

    任何一个环节失败（没有桌面 / cv2 报错）都不会抛异常。
    """

    TITLE = "智能药箱 - 扫码确认"

    def __init__(self, roi: Optional[Sequence[int]] = None,
                 font_size: int = 26, win_w: int = 1000,
                 enabled: bool = True):
        self.roi: Optional[Tuple[int, int, int, int]] = (
            tuple(int(v) for v in roi) if roi else None)
        self.win_w = int(win_w)
        self.tr = TextRenderer(size=font_size)
        self.tr_s = TextRenderer(size=max(15, int(font_size) - 9))
        self.enabled = bool(enabled) and gui_available()
        self._open = False
        self._frame_seen = 0
        self._last_good = None
        self._last_result = None
        self._camera = None          # hold() 时记下，供 [ ] 调曝光
        self._note = ""              # 按键反馈（曝光 -> x 等）
        self.exposure_note = ""
        #: 用户手动调过曝光后为 True —— 过检流程不要再自动覆盖
        self.manual_exposure = False
        #: 曝光搜索范围（与 camera_preview / Camera.auto_expose 一致）
        self.exp_min = -13.0
        self.exp_max = 0.0
        self._last_ui_t = 0.0
        self._ui_fps = 0.0
        self._space_pressed = False

    # ==================================================================
    # 生命周期
    # ==================================================================
    def set_camera(self, cam) -> None:
        """
        绑定相机引用（公开接口）。

        hold() 会隐式绑定；但外部自建循环（如 HC05 出库扫码）直接调
        on_frame 渲染时，必须先调本方法，否则 [ ] a 调曝光全部无效
        （内部 _camera 为 None，按键只会提示"没有相机"）。
        """
        self._camera = cam

    def open(self) -> bool:
        """创建窗口。返回是否真的打开了。"""
        if not self.enabled or self._open:
            return self._open
        try:
            cv2.namedWindow(self.TITLE, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.TITLE, self.win_w, int(self.win_w * 9 / 16))
            cv2.moveWindow(self.TITLE, 60, 60)
            self._open = True
        except Exception:
            self.enabled = False
            self._open = False
        return self._open

    def close(self) -> None:
        """关闭窗口（幂等，绝不抛异常）"""
        if not self._open:
            return
        try:
            cv2.destroyWindow(self.TITLE)
            cv2.waitKey(1)
        except Exception:
            pass
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open

    def __enter__(self) -> "ScanWindow":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ==================================================================
    # 绘制
    # ==================================================================
    def _draw_roi(self, img: np.ndarray) -> None:
        """画黄色 ROI 框（带四角加粗，和 camera_preview 风格一致）"""
        if not self.roi:
            return
        H, W = img.shape[:2]
        x, y, w, h = self.roi
        x = max(0, min(x, W - 1))
        y = max(0, min(y, H - 1))
        x2 = max(0, min(x + w, W - 1))
        y2 = max(0, min(y + h, H - 1))

        cv2.rectangle(img, (x, y), (x2, y2), C_ROI, 2)
        # 四角加粗，远处也能一眼看到框在哪
        tick = max(18, min(40, (x2 - x) // 6, (y2 - y) // 6))
        for px, py, dx, dy in ((x, y, 1, 1), (x2, y, -1, 1),
                               (x, y2, 1, -1), (x2, y2, -1, -1)):
            cv2.line(img, (px, py), (px + dx * tick, py), C_ROI, 4)
            cv2.line(img, (px, py), (px, py + dy * tick), C_ROI, 4)

    def _draw_qr(self, img: np.ndarray, result, matched: Optional[bool]) -> None:
        """画识别到的二维码：匹配=亮绿，不匹配=红"""
        if result is None:
            return
        col = C_QR if matched is None or matched else C_BAD
        poly = getattr(result, "polygon", None)

        # 码内容标签要画在**二维码框上方**（画在中心会挡住码本身）
        if poly is not None:
            try:
                pts = np.asarray(poly, dtype=np.int32)
                cv2.polylines(img, [pts], True, col, 3)
                top = int(np.min(pts[:, 1]))
                cx = int(np.mean(pts[:, 0]))
                left = int(np.min(pts[:, 0]))
            except Exception:
                top, cx, left = -1, -1, -1
        elif getattr(result, "cx", -1) > 0:
            half = int(max(30, result.width / 2))
            left = int(result.cx - half)
            top = int(result.cy - half)
            cx = int(result.cx)
            cv2.rectangle(img, (left, top),
                          (int(result.cx + half), int(result.cy + half)), col, 3)
        else:
            top, cx, left = -1, -1, -1

        label = str(getattr(result, "code", "?"))
        if matched is True:
            label += "  匹配 OK"
        elif matched is False:
            label += "  不是这个"

        if left >= 0 and top >= 0:
            tw, th = self.tr_s.measure(label)
            ty = top - th - 8
            if ty < 4:                      # 上方放不下就贴到框内上沿
                ty = top + 6
            tx = max(6, min(cx - tw // 2, img.shape[1] - tw - 6))
            self.tr_s.draw(img, [label], (tx, ty),
                           color=col, bg=True, bg_color=(25, 25, 30))

    def _bar(self, img: np.ndarray, top: bool, height: int,
             color: Optional[Tuple[int, int, int]] = None) -> None:
        """在顶部/底部压一条半透明深色带，保证文字可读"""
        H, W = img.shape[:2]
        y0, y1 = (0, height) if top else (H - height, H)
        try:
            band = img[y0:y1, 0:W].copy()
            band[:] = (26, 26, 30)
            img[y0:y1, 0:W] = cv2.addWeighted(band, 0.80, img[y0:y1, 0:W], 0.20, 0)
        except Exception:
            return
        if color is not None:
            # 左侧一根竖色条，颜色 = 当前状态
            cv2.rectangle(img, (0, y0), (9, y1), color, -1)

    def _brightness(self, frame: np.ndarray) -> float:
        if frame is None:
            return 0.0
        try:
            if frame.ndim == 2:
                return float(frame.mean())
            return float(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean())
        except Exception:
            return 0.0

    @staticmethod
    def _brightness_tag(b: float) -> str:
        if b < 8:
            return "全黑"
        if b < 45:
            return "偏暗"
        if b > 240:
            return "过曝"
        if b > 205:
            return "偏亮"
        if 90 <= b <= 160:
            return "正常"
        return "可用"

    def _current_exposure(self) -> float:
        cam = self._camera
        exp = getattr(cam, "exposure", None) if cam is not None else None
        return float(exp) if exp is not None else -6.0

    def _set_exposure(self, value: float) -> None:
        """
        运行中改曝光 —— 与 tools/camera_preview.py 的 [ ] 完全同一套逻辑：
        写 CAP_PROP_EXPOSURE → 更新 cam.exposure → 丢 3 帧让参数生效。
        ★ 一旦手动调过，记下 manual_exposure，过检流程不会再被自动曝光覆盖。
        """
        cam = self._camera
        if cam is None:
            self._note = "没有相机，无法调曝光"
            return
        value = float(max(self.exp_min, min(self.exp_max, float(value))))
        try:
            if getattr(cam, "cap", None) is not None:
                cam.cap.set(cv2.CAP_PROP_EXPOSURE, value)
            cam.exposure = value
            for _ in range(3):
                cam.read()
            self.manual_exposure = True
            try:
                cam.auto_exposure = False
            except Exception:
                pass
            self._note = "曝光 -> %g（手动）" % value
        except Exception as e:
            self._note = "调曝光失败: %s" % e

    def _auto_expose(self) -> None:
        """重新跑自适应曝光（换场地/换灯后按 a，不用改代码）"""
        cam = self._camera
        if cam is None or not hasattr(cam, "auto_expose"):
            self._note = "当前相机不支持自适应曝光"
            return
        try:
            res = cam.auto_expose()
            self.manual_exposure = False
            try:
                cam.auto_exposure = True
            except Exception:
                pass
            if res is not None:
                self._note = "自动曝光 -> %g（亮度 %.0f）" % (cam.exposure, res[1])
            else:
                self._note = "自动曝光失败：读不到画面"
        except Exception as e:
            self._note = "自动曝光失败: %s" % e

    def _handle_key(self, key: int) -> Optional[str]:
        """
        处理扫码窗快捷键 —— 与 tools/camera_preview.py 对齐：
          [ ] 手动调暗/调亮   a 重新自动曝光   q/ESC 跳过/关闭
          空格/回车 在 hold 阶段表示“开始识别”（由 hold() 处理）
        返回 'quit' 表示应跳过当前阶段；其它情况返回 None。
        """
        if key is None or key < 0 or key == 255:
            return None
        if key in (ord("q"), 27):
            return "quit"
        if key == ord("["):
            self._set_exposure(self._current_exposure() - 1)
        elif key == ord("]"):
            self._set_exposure(self._current_exposure() + 1)
        elif key == ord("a"):
            self._auto_expose()
        elif key == ord("=") or key == ord("+"):
            self._set_exposure(self._current_exposure() + 1)
        elif key == ord("-"):
            self._set_exposure(self._current_exposure() - 1)
        return None

    def _drain_and_handle(self, wait_ms: int = 20) -> Optional[str]:
        """
        显示一帧后，把键盘缓冲区里的按键全部吃掉并处理。
        避免 on_frame 里 waitKey(1) 漏按键（想调曝光却没反应）。
        """
        while True:
            try:
                key = cv2.waitKey(1) & 0xFF
            except Exception:
                return None
            if key == 255:
                return None
            if key in (ord(" "), 13):
                self._space_pressed = True
                continue
            act = self._handle_key(key)
            if act == "quit":
                return "quit"

    def _compose(self, frame: np.ndarray, status: str,
                 s_color: Tuple[int, int, int],
                 subtitle: str = "", detail: str = "",
                 result=None, matched: Optional[bool] = None,
                 hint: str = "") -> np.ndarray:
        """把一帧画成"带状态条的确认画面" """
        img = frame.copy()
        self._draw_roi(img)
        self._draw_qr(img, result, matched)

        # ---- 左上角 HUD（与 tools/camera_preview.py 同风格）----
        # 数字取整：动态小数会让 TextRenderer 缓存失效，每帧重排 PIL，预览会掉帧
        br = self._brightness(frame)
        btag = self._brightness_tag(br)
        exp = self._current_exposure()
        mode = "手动" if self.manual_exposure else "自动"
        hud_lines = [
            "亮度 %d / 255  [%s]" % (int(br + 0.5), btag),
            "曝光 %g（%s）" % (exp, mode),
        ]
        if br < 45:
            hud_lines.append("偏暗 → 按 ] 调亮 或 a 自动")
        elif br > 240:
            hud_lines.append("过曝 → 按 [ 调暗")
        # 粗算显示帧率（指数平滑），方便对照 camera_preview
        now = time.time()
        if self._last_ui_t:
            dt = now - self._last_ui_t
            if dt > 0:
                inst = 1.0 / dt
                self._ui_fps = inst if self._ui_fps <= 0 else (self._ui_fps * 0.7 + inst * 0.3)
        self._last_ui_t = now
        if self._ui_fps > 0:
            hud_lines.append("预览 %.0f fps" % (self._ui_fps + 0.5))
        y = 12
        for i, t in enumerate(hud_lines):
            col = C_WARN if (i == 2 or btag in ("全黑", "偏暗", "过曝")) else C_DIM
            if i == 0 and btag in ("全黑", "偏暗", "过曝"):
                col = C_WARN
            y = self.tr_s.draw(img, [t], (16, y), color=col, bg=True) + 2

        top_h = self.tr.size + 24
        self._bar(img, True, top_h, s_color)
        self.tr.draw(img, [status], (24, 10), color=s_color)

        lines = [t for t in (subtitle, detail) if t]
        if self._note:
            lines.append(self._note)
        default_hint = "[ 调暗 ] 调亮  a 自动曝光  q 跳过   （与 camera_preview 相同）"
        if hint:
            lines.append(hint)
        else:
            lines.append(default_hint)
        if lines:
            bot_h = self.tr_s.size * len(lines) + self.tr_s.line_gap * len(lines) + 20
            self._bar(img, False, bot_h)
            y = img.shape[0] - bot_h + 10
            for i, t in enumerate(lines):
                col = C_DIM if i < len(lines) - 1 else C_WHITE
                self.tr_s.draw(img, [t], (24, y), color=col)
                y += self.tr_s.size + self.tr_s.line_gap
        return img

    def _imshow(self, img: np.ndarray, wait_ms: int = 1) -> int:
        """显示一帧并处理按键。返回按键值（-1 = 无）。窗口坏了就自动禁用。"""
        if not self._open:
            return -1
        try:
            cv2.imshow(self.TITLE, img)
            return int(cv2.waitKey(max(1, wait_ms)) & 0xFF)
        except Exception:
            self.enabled = False
            self._open = False
            return -1

    # ==================================================================
    # 三个阶段
    # ==================================================================
    def hold(self, camera, seconds: float = 3.0,
             status: str = "请把药品标签举进黄框",
             subtitle: str = "对准黄色框、举稳，准备好后按【空格】开始识别",
             hint: str = "空格=开始识别 · q=立刻扫 · [ ] 调暗 ] 调亮 a 自动曝光",
             wait_space: bool = True) -> bool:
        """
        准备阶段：实时显示画面，等人举好码。

        wait_space=True（推荐，真摄像头人工举码）：
            **一直等到按【空格】才开始识别**，seconds 只是超时上限。
            这样绝不会出现“还没举起手机就扫完了”。
        wait_space=False：
            按 seconds 倒计时结束后自动开始（仿真/自动化用）。

        返回
        ----
        True  = 可以开始识别（按了空格 / 倒计时结束）
        False = 窗口不可用，或按 q 跳过等待（仍会进入识别）
        """
        if not self._open:
            return False
        self._camera = camera
        self._note = ""
        max_wait = max(0.5, float(seconds))

        t_end = time.time() + max_wait
        ok_shown = False
        while time.time() < t_end:
            ok, frame = camera.read()
            if not ok or frame is None:
                time.sleep(0.03)
                continue
            self._last_good = frame
            left = max(0.0, t_end - time.time())
            if wait_space:
                detail = "举稳后按【空格】开始识别（最长再等 %d 秒）" % int(left + 0.999)
            else:
                detail = "倒计时 %d 秒后开始识别" % int(left + 0.999)
            img = self._compose(
                frame, status, C_WARN,
                subtitle=subtitle,
                detail=detail,
                hint=hint)
            key = self._imshow(img, 30)
            act = self._handle_key(key)
            if act == "quit":
                return False
            if wait_space and (key in (ord(" "), 13) or self._space_pressed):
                self._space_pressed = False
                self._note = "开始识别…"
                return True
            act = self._drain_and_handle()
            if act == "quit":
                return False
            if wait_space and self._space_pressed:
                self._space_pressed = False
                self._note = "开始识别…"
                return True
            ok_shown = True
            if not self._open:
                break
        return ok_shown

    def on_frame(self, frame, result, idx: int, total: int) -> None:
        """
        投票阶段回调：每读到一帧就实时显示（含这一帧的识别结果）。
        同样支持 [ ] a 调曝光 —— 识别进行中也能把画面调亮。

        直接传给 qr_detector.robust_scan(on_frame=...)，签名必须一致。
        """
        if not self._open or frame is None:
            return
        self._frame_seen += 1
        self._last_good = frame

        if result is not None:
            self._last_result = result
            status = "已识别: %s" % result.code
            col = C_OK
            sub = "保持不动，继续投票确认（多帧一致才采信）"
        else:
            status = "正在找二维码..."
            col = C_WARN
            sub = "把标签举到黄色框内、正对镜头、举稳；偏暗按 ] 或 a"

        img = self._compose(
            frame, status, col,
            subtitle=sub,
            detail="采样进度 %d / %d 帧" % (idx + 1, total),
            result=result,
            hint="[ 调暗 ] 调亮  a 自动曝光  q 跳过本次扫码",
            )
        key = self._imshow(img, 10)   # 10ms：够收按键，又不至于拖帧
        self._handle_key(key)
        self._drain_and_handle()

    def verdict(self, code: Optional[str], expected: Optional[str] = None,
                seconds: float = 1.8) -> None:
        """
        结论阶段：把"扫到的是什么、对不对"定住几秒，让人看清。
        同时把上一帧识别到的码框也画出来，让人知道"码在哪、对上没有"。
        """
        if not self._open:
            return

        if code is None:
            status, col = "没有识别到二维码", C_BAD
            sub = "标签举进黄框、正对镜头、举稳约 1 秒；光线太暗也会扫不到"
            matched = None
            show_res = None
        elif expected is None or code == expected:
            status, col = "识别成功: %s" % code, C_OK
            sub = "身份确认通过，继续取药" if expected else "已读到: %s" % code
            matched = True
            show_res = self._last_result
        else:
            status, col = "扫到的是 %s（不是 %s）" % (code, expected), C_BAD
            sub = "举错标签了：请换成 %s" % expected
            matched = False
            show_res = self._last_result

        frame = self._last_good
        t_end = time.time() + max(0.3, float(seconds))
        while time.time() < t_end:
            if frame is not None:
                img = self._compose(frame, status, col, subtitle=sub,
                                    detail="窗口 %.0f 秒后自动关闭"
                                           % max(0.0, t_end - time.time()),
                                    result=show_res, matched=matched,
                                    hint="q 立即关闭")
                key = self._imshow(img, 30)
                if key in (ord("q"), 27):
                    return
            else:
                time.sleep(0.05)
            if not self._open:
                return

    # ==================================================================
    def pump_text(self, status: str, s_color=None, subtitle: str = "",
                  seconds: float = 1.0) -> None:
        """
        只显示一段文字（用最后一帧当底图），用于流程中穿插提示，
        比如"抓错药，正在放回"。
        """
        if not self._open or self._last_good is None:
            return
        col = s_color or C_WARN
        t_end = time.time() + max(0.2, float(seconds))
        while time.time() < t_end:
            img = self._compose(self._last_good, status, col, subtitle=subtitle)
            key = self._imshow(img, 30)
            if key in (ord("q"), 27):
                return
            if not self._open:
                return
