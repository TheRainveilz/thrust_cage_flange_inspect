# -*- coding: utf-8 -*-
"""量两件事：
  ① detect_hole_candidates 里 p2=55 那一遍的命中率(命中就不跑 fallback，也就省一遍 Hough)；
  ② 候选数分布(= 后面要做多少次 refine_hole)。
"""
import os
import sys

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "src", "flange_inspect"))

import logging
logging.getLogger().addHandler(logging.NullHandler())
logging.disable(logging.CRITICAL)

import inspector_pure as ip   # noqa: E402

CLASS = "I:/data.zip/data/missing/Class"
SAMPLE = [("正面/OK", 40), ("正面/NG", 20), ("反面/NG", 20), ("无件", 15)]

rows = []
for sub, n in SAMPLE:
    d = os.path.join(CLASS, sub)
    fs = sorted(f for f in os.listdir(d) if f.lower().endswith(".png"))[:n]
    for f in fs:
        p = os.path.join(d, f)
        bgr = ip.imread_unicode(p)
        if bgr is None:
            continue
        _b, gray, work, _cl, _sc = ip.preprocess(bgr)
        w = work.shape[1]
        r_lo = max(3, int(ip.HOLE_R_MIN_RATIO * w))
        r_hi = max(r_lo + 2, int(ip.HOLE_R_MAX_RATIO * w))
        md = max(8, int(ip.HOLE_MIN_DIST_RATIO * w))
        counts = {}
        for p2 in (ip.HOLE_HOUGH_P2, ip.HOLE_HOUGH_P2_FALLBACK):
            c = cv2.HoughCircles(work, cv2.HOUGH_GRADIENT, dp=ip.HOLE_HOUGH_DP, minDist=md,
                                 param1=ip.HOLE_HOUGH_P1, param2=p2,
                                 minRadius=r_lo, maxRadius=r_hi)
            counts[p2] = 0 if c is None else len(c[0])
        final = ip.detect_hole_candidates(work)
        rows.append((sub, counts[ip.HOLE_HOUGH_P2], counts[ip.HOLE_HOUGH_P2_FALLBACK],
                     len(final)))

out = []
out.append("%-10s %8s %8s %8s" % ("目录", "p2=55", "p2=32", "最终候选"))
for sub, _ in SAMPLE:
    v = [r for r in rows if r[0] == sub]
    if not v:
        continue
    out.append("%-10s %8.1f %8.1f %8.1f   (n=%d)"
               % (sub, np.mean([x[1] for x in v]), np.mean([x[2] for x in v]),
                  np.mean([x[3] for x in v]), len(v)))

hit = sum(1 for r in rows if r[1] >= ip.MIN_HOLE_COUNT)
out.append("")
out.append("总帧 %d；p2=55 一遍就够(>=%d 个候选)的帧：%d (%.1f%%) ⇒ 另 %.1f%% 的帧白跑两遍 Hough"
           % (len(rows), ip.MIN_HOLE_COUNT, hit, 100.0 * hit / len(rows),
              100.0 * (len(rows) - hit) / len(rows)))
fin = np.array([r[3] for r in rows])
out.append("最终候选数：p50=%.0f p90=%.0f max=%.0f  ⇒ 每帧候选精修次数(未截断前)"
           % (np.percentile(fin, 50), np.percentile(fin, 90), fin.max()))
out.append("(detect_hole_candidates 会在 MAX_HOLE_CANDIDATES=%d 处按半径截断)"
           % ip.MAX_HOLE_CANDIDATES)

txt = os.path.join(HERE, "_hough_probe.txt")
with open(txt, "w", encoding="utf-8") as f:
    f.write("\n".join(out) + "\n")
print("saved")
