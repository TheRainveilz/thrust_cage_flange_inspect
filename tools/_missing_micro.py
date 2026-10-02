# -*- coding: utf-8 -*-
"""微基准：refine_hole 内部的固定开销、HoughCircles 不同分辨率、高光量测的掩膜开销。"""
import os
import sys
import time

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "src", "flange_inspect"))

import logging
logging.getLogger().addHandler(logging.NullHandler())
logging.disable(logging.CRITICAL)

import inspector_pure as ip      # noqa: E402
import inspector_missing as im   # noqa: E402

OUT = []


def P(*a):
    OUT.append(" ".join(str(x) for x in a))


def t(fn, n=30):
    fn()  # warm
    s = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        s.append((time.perf_counter() - t0) * 1000.0)
    return float(np.median(s))


frame = "I:/data.zip/data/missing/Class/正面/OK/20260928.180428.874.000001.origin.png"
bgr = ip.imread_unicode(frame)
_b, gray, work, _cl, _sc = ip.preprocess(bgr)
gray_f = gray.astype(np.float32)
P("帧 %s  work=%s" % (os.path.basename(frame), work.shape))

P("")
P("---- 固定开销 ----")
P("np.uint8(1280x800).astype(float32)  : %6.3f ms" % t(lambda: work.astype(np.float32)))
P("np.float32(1280x800).astype(float32): %6.3f ms" % t(lambda: gray_f.astype(np.float32)))
P("cv2.resize 1/2 INTER_AREA           : %6.3f ms"
  % t(lambda: cv2.resize(work, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)))
P("cv2.resize 1/4 INTER_AREA           : %6.3f ms"
  % t(lambda: cv2.resize(work, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)))

P("")
P("---- refine_hole 一次(现路径 vs 已是 float32 的输入) ----")
ang = np.radians(np.arange(0.0, 360.0, ip.REFINE_ANGLE_STEP_DEG))
cos_t = np.cos(ang).astype(np.float32)
sin_t = np.sin(ang).astype(np.float32)
P("refine_hole(work uint8)   : %6.3f ms"
  % t(lambda: ip.refine_hole(work, 650.0, 402.0, 47.0, cos_t, sin_t)))
P("refine_hole(gray_f float32): %6.3f ms"
  % t(lambda: ip.refine_hole(gray_f, 650.0, 402.0, 47.0, cos_t, sin_t)))
P("  ⇒ 单次省 %.3f ms；×34.6 次/帧 = %.1f ms/帧"
  % (t(lambda: ip.refine_hole(work, 650.0, 402.0, 47.0, cos_t, sin_t))
     - t(lambda: ip.refine_hole(gray_f, 650.0, 402.0, 47.0, cos_t, sin_t)),
     (t(lambda: ip.refine_hole(work, 650.0, 402.0, 47.0, cos_t, sin_t))
      - t(lambda: ip.refine_hole(gray_f, 650.0, 402.0, 47.0, cos_t, sin_t))) * 34.6))

P("")
P("---- 找孔候选 HoughCircles 不同分辨率 ----")
for fx in (1.0, 0.5, 0.25):
    img = work if fx == 1.0 else cv2.resize(work, None, fx=fx, fy=fx, interpolation=cv2.INTER_AREA)
    w = img.shape[1]
    r_lo = max(3, int(ip.HOLE_R_MIN_RATIO * w))
    r_hi = max(r_lo + 2, int(ip.HOLE_R_MAX_RATIO * w))
    md = max(8, int(ip.HOLE_MIN_DIST_RATIO * w))
    P("  fx=%.2f  %s  r=[%d,%d] minDist=%d" % (fx, img.shape, r_lo, r_hi, md))
    for dp in (1.0, 2.0):
        def h():
            cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, dp=dp, minDist=md,
                             param1=ip.HOLE_HOUGH_P1, param2=ip.HOLE_HOUGH_P2,
                             minRadius=r_lo, maxRadius=r_hi)
        c = h()
        P("     dp=%.1f  %6.2f ms   候选 %s" % (dp, t(h, 10), 0 if c is None else len(c[0])))

P("")
P("---- 高光量测的掩膜/分位开销(单槽) ----")
hr = 47.0
half = max(4, int(round(im.HIGHLIGHT_R_RATIO * hr)))
P("  half=%d -> patch %dx%d" % (half, 2 * half + 1, 2 * half + 1))


def mask_old():
    h = w_ = patch.shape[0]          # crop_pad 返回 2*half 见方
    yy, xx = np.mgrid[0:h, 0:w_]
    m = np.hypot(xx - (w_ - 1) / 2.0, yy - (h - 1) / 2.0) <= half * 0.9
    return m


patch, _oof = im.crop_pad(gray_f, 650.0, 402.0, half)
P("  实际 patch=%s" % (patch.shape,))
P("  mgrid+hypot 建掩膜  : %6.4f ms" % t(mask_old, 200))
m = mask_old()
sub = patch[m]
P("  np.percentile(99)   : %6.4f ms" % t(lambda: np.percentile(sub, 99), 200))
P("  np.median           : %6.4f ms" % t(lambda: np.median(sub), 200))
P("  ⇒ 单槽 %.3f ms × 18 = %.2f ms/帧"
  % (t(mask_old, 200) + t(lambda: np.percentile(sub, 99), 200) + t(lambda: np.median(sub), 200),
     (t(mask_old, 200) + t(lambda: np.percentile(sub, 99), 200)
      + t(lambda: np.median(sub), 200)) * 18))

open(os.path.join(HERE, "_micro_missing.txt"), "w", encoding="utf-8").write("\n".join(OUT) + "\n")
print("saved")
