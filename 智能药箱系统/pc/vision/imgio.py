# -*- coding: utf-8 -*-
"""
中文路径 cv2 imread/imwrite 兼容层。
Windows 下 cv2.imread 对含中文的路径静默返回 None，
这里用 imencode/imdecode + 二进制读写绕开。
"""
import cv2
import numpy as np


def imread(path, flags=cv2.IMREAD_COLOR):
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
        return cv2.imdecode(data, flags)
    except Exception:
        return None


def imwrite(path, img, params=None):
    try:
        ext = str(path).rsplit(".", 1)[-1]
        ok, buf = cv2.imencode("." + ext, img, params or [])
        if ok:
            buf.tofile(str(path))
            return True
    except Exception:
        pass
    return False


# 接管 cv2 原生函数
_orig_imread = cv2.imread
_orig_imwrite = cv2.imwrite
cv2.imread = imread
cv2.imwrite = imwrite

print("[imgio] 已接管 cv2.imread/imwrite（cv2 %s）："
      "绕开 Windows 中文路径下 fopen 静默失败的问题（改用 imencode/imdecode + 二进制读写）"
      % cv2.__version__)
