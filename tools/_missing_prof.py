# -*- coding: utf-8 -*-
"""inspect_missing 耗时分解：把 preprocess / 找孔候选 / 精修候选 / 定位 / 逐槽(精修+高光) 分开计时。

做法：包一层计时器替换模块里的同名函数，再对一批真帧跑完整 inspect_missing —— 不复制判定逻辑，
所以量到的就是产线路径本身。输出中位数/最大值/占比。
"""
import os
import sys
import time
from collections import defaultdict

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "src", "flange_inspect"))

import logging
logging.getLogger().addHandler(logging.NullHandler())
logging.disable(logging.CRITICAL)

import inspector_missing as im   # noqa: E402

CLASS = "I:/data.zip/data/missing/Class"

# ---- 采样：正面OK 是"18 槽全都要量"的最坏情形，多取；无件最轻，少取。
SAMPLE = [("正面/OK", 30), ("正面/NG", 15), ("反面/NG", 15), ("无件", 10)]

ACC = defaultdict(list)   # 名字 -> [每帧耗时]
COUNT = defaultdict(int)
CALLS = []                # refine_hole 逐次 (帧内序号, ms, r0)


def wrap(mod, name, label):
    fn = getattr(mod, name)

    def inner(*a, **kw):
        t = time.perf_counter()
        out = fn(*a, **kw)
        ACC[label].append((time.perf_counter() - t) * 1000.0)
        return out
    setattr(mod, name, inner)


def wrap_refine():
    fn = im.refine_hole
    state = {"n": 0}

    def inner(*a, **kw):
        t = time.perf_counter()
        out = fn(*a, **kw)
        dt = (time.perf_counter() - t) * 1000.0
        ACC["refine_hole"].append(dt)
        CALLS.append((state["n"], dt, float(a[3])))
        state["n"] += 1
        return out
    im.refine_hole = inner


wrap(im, "preprocess", "preprocess")
wrap(im, "detect_hole_candidates", "找孔候选")
wrap(im, "locate_part_fast", "定位")
wrap_refine()

# 逐帧调用顺序：refine 候选(N 次) -> 定位 -> 逐槽 refine(18 次)。用"到定位为止"和"定位之后"切开。
frames = []
for sub, n in SAMPLE:
    d = os.path.join(CLASS, sub)
    fs = sorted(f for f in os.listdir(d) if f.lower().endswith(".png"))[:n]
    frames += [(sub, os.path.join(d, f)) for f in fs]

per_frame = []
per_class = defaultdict(list)
for sub, path in frames:
    bgr = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        continue
    before = len(CALLS)
    t0 = time.perf_counter()
    r = im.inspect_missing(bgr, os.path.basename(path))
    total = (time.perf_counter() - t0) * 1000.0
    per_frame.append(total)
    per_class[sub].append(total)
    n_cand = sum(1 for c in CALLS[before:] if c[2] > 60.0)   # seed r 大的那批 = 候选精修
    per_class[sub + "#ncand"].append(n_cand)
    per_class[sub + "#nslot"].append(len(CALLS) - before - n_cand)

out = []


def P(*a):
    out.append(" ".join(str(x) for x in a))


P("样本 %d 帧" % len(per_frame))
P("")
P("%-12s %8s %8s %8s %8s" % ("阶段", "中位ms", "均值ms", "最大ms", "占比%"))
grand = float(np.sum(per_frame))
for k in ("preprocess", "找孔候选", "定位", "refine_hole"):
    v = np.array(ACC[k])
    P("%-12s %8.2f %8.2f %8.2f %7.1f%%   (共 %d 次调用)"
      % (k, np.median(v), v.mean(), v.max(), 100.0 * v.sum() / grand, len(v)))
other = grand - sum(np.sum(ACC[k]) for k in ("preprocess", "找孔候选", "定位", "refine_hole"))
P("%-12s %8s %8.2f %8s %7.1f%%" % ("其余(高光/判定)", "-", other / len(per_frame),
                                    "-", 100.0 * other / grand))
P("%-12s %8.2f %8.2f %8.2f" % ("整帧", np.median(per_frame), np.mean(per_frame),
                                np.max(per_frame)))
P("")
for sub, _ in SAMPLE:
    v = np.array(per_class[sub])
    P("%-10s 帧%3d  中位 %6.2f  均值 %6.2f  最大 %6.2f | 候选精修 %d 次 / 逐槽 %d 次"
      % (sub, len(v), np.median(v), v.mean(), v.max(),
         np.median(per_class[sub + "#ncand"]), np.median(per_class[sub + "#nslot"])))

# refine_hole 单次耗时分布：候选 vs 逐槽
allc = np.array([c[1] for c in CALLS])
big = np.array([c[1] for c in CALLS if c[2] > 60.0])
sml = np.array([c[1] for c in CALLS if c[2] <= 60.0])
P("")
P("refine_hole 单次 ms: 全部 p50=%.2f p90=%.2f max=%.2f (n=%d)"
  % (np.percentile(allc, 50), np.percentile(allc, 90), allc.max(), len(allc)))
P("  候选精修(r0>60):   p50=%.2f p90=%.2f max=%.2f (n=%d)"
  % (np.percentile(big, 50), np.percentile(big, 90), big.max(), len(big)))
P("  逐槽(r0<=60):     p50=%.2f p90=%.2f max=%.2f (n=%d)"
  % (np.percentile(sml, 50), np.percentile(sml, 90), sml.max(), len(sml)))

txt = os.path.join(HERE, "_timing_missing.txt")
with open(txt, "w", encoding="utf-8") as f:
    f.write("\n".join(out) + "\n")
print("saved", txt)
