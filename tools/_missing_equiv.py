# -*- coding: utf-8 -*-
"""全量逐帧逐槽快照：判定 + 每槽半径比/高光。改优化前后各跑一次，用 diff 证明等价。

用法：
    python tools/_missing_equiv.py before      # 改动前
    python tools/_missing_equiv.py after       # 改动后
    python tools/_missing_equiv.py cmp         # 比对，输出差异报告
"""
import json
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

import inspector_missing as im   # noqa: E402

CLASS = "I:/data.zip/data/missing/Class"
DIRS = ["正面/OK", "正面/NG", "反面/NG", "无件"]
TAG = sys.argv[1] if len(sys.argv) > 1 else "before"
# 可选：临时覆盖两个候选加速开关，用来隔离"降采样"与"跳严阈值"各自的影响。
#   python tools/_missing_equiv.py d1s1 1 0   → CAND_DOWNSCALE=1 / CAND_SKIP_FIRST_PASS=False
if TAG != "cmp" and len(sys.argv) > 3:
    im.CAND_DOWNSCALE = int(sys.argv[2])
    im.CAND_SKIP_FIRST_PASS = bool(int(sys.argv[3]))


def collect():
    data, times = {}, {}
    for sub in DIRS:
        d = os.path.join(CLASS, sub)
        for f in sorted(os.listdir(d)):
            if not f.lower().endswith(".png"):
                continue
            p = os.path.join(d, f)
            bgr = cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                continue
            key = sub + "/" + f
            t0 = time.perf_counter()
            r = im.inspect_missing(bgr, f)
            times[key] = (time.perf_counter() - t0) * 1000.0
            data[key] = {
                "verdict": r.verdict,
                "n_empty": r.n_empty,
                "n_slot_fail": r.n_slot_fail,
                "rr_med": round(r.r_ratio_med, 6),
                "phase_deg": round(r.phase_deg, 6),
                "phase_coh": round(r.phase_coh, 6),
                "method": r.locate_method,
                "cx": round(r.part_cx, 4), "cy": round(r.part_cy, 4),
                "pitch_r": round(r.pitch_r, 4),
                "slots": [[p2.index, round(p2.r_ratio, 6), round(p2.highlight, 6)]
                          for p2 in r.pockets],
            }
    return data, times


if TAG == "cmp":
    TA = sys.argv[2] if len(sys.argv) > 2 else "before"
    TB = sys.argv[3] if len(sys.argv) > 3 else "after"
    a = json.load(open(os.path.join(HERE, "_equiv_%s.json" % TA), encoding="utf-8"))
    b = json.load(open(os.path.join(HERE, "_equiv_%s.json" % TB), encoding="utf-8"))
    ta = json.load(open(os.path.join(HERE, "_equiv_%s_t.json" % TA), encoding="utf-8"))
    tb = json.load(open(os.path.join(HERE, "_equiv_%s_t.json" % TB), encoding="utf-8"))
    out = []

    def P(*x):
        out.append(" ".join(str(i) for i in x))

    keys = sorted(set(a) | set(b))
    P("对比 %s vs %s" % (TA, TB))
    P("帧数 before=%d after=%d" % (len(a), len(b)))
    diff_v = [k for k in keys if a.get(k, {}).get("verdict") != b.get(k, {}).get("verdict")]
    diff_s = []
    maxdr = maxdh = 0.0
    for k in keys:
        if k not in a or k not in b:
            continue
        sa, sb = a[k]["slots"], b[k]["slots"]
        if len(sa) != len(sb) or any(x[0] != y[0] for x, y in zip(sa, sb)):
            diff_s.append((k, "槽数/编号不同 %d vs %d" % (len(sa), len(sb))))
            continue
        for x, y in zip(sa, sb):
            maxdr = max(maxdr, abs(x[1] - y[1]))
            maxdh = max(maxdh, abs(x[2] - y[2]))
            if abs(x[1] - y[1]) > 1e-9 or abs(x[2] - y[2]) > 1e-9:
                diff_s.append((k, "槽#%d rr %.6f->%.6f hi %.6f->%.6f" % (x[0], x[1], y[1], x[2], y[2])))
    P("")
    P("== 判定 ==")
    if diff_v:
        for k in diff_v:
            P("  ✗ %-52s %s -> %s" % (k, a[k]["verdict"], b[k]["verdict"]))
    else:
        P("  ✓ 598 帧判定逐帧完全相同")
    P("")
    P("== 逐槽读数 ==")
    P("  最大 |Δ半径比| = %.10f    最大 |Δ高光| = %.10f" % (maxdr, maxdh))
    P("  不一致槽条目 %d" % len(diff_s))
    for k, m in diff_s[:20]:
        P("    %-52s %s" % (k, m))

    # 逃逸 / 过杀(真值只认目录名)
    def tally(d):
        esc = over = 0
        for k, v in d.items():
            truth_ok = k.startswith("正面/OK")
            truth_ng = k.startswith("正面/NG") or k.startswith("反面/NG") or k.startswith("无件")
            if truth_ng and v["verdict"].startswith("OK"):
                esc += 1
            if truth_ok and not v["verdict"].startswith("OK"):
                over += 1
        return esc, over

    ea, oa = tally(a)
    eb, ob = tally(b)
    P("")
    P("== 安全指标 ==")
    P("  before: 逃逸 %d  过杀(正面OK) %d/%d" % (ea, oa, sum(1 for k in a if k.startswith("正面/OK"))))
    P("  after : 逃逸 %d  过杀(正面OK) %d/%d" % (eb, ob, sum(1 for k in b if k.startswith("正面/OK"))))

    va = np.array(list(ta.values()))
    vb = np.array(list(tb.values()))
    P("")
    P("== 耗时(本机, 单进程) ==")
    P("  before: 中位 %.2f ms  均值 %.2f  最大 %.2f  p90 %.2f"
      % (np.median(va), va.mean(), va.max(), np.percentile(va, 90)))
    P("  after : 中位 %.2f ms  均值 %.2f  最大 %.2f  p90 %.2f"
      % (np.median(vb), vb.mean(), vb.max(), np.percentile(vb, 90)))
    P("  提速 中位 %.2fx  最大 %.2fx" % (np.median(va) / np.median(vb), va.max() / vb.max()))

    txt = os.path.join(HERE, "_equiv_report_%s_vs_%s.txt" % (TA, TB))
    open(txt, "w", encoding="utf-8").write("\n".join(out) + "\n")
    print("saved", txt)
else:
    data, times = collect()
    json.dump(data, open(os.path.join(HERE, "_equiv_%s.json" % TAG), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    json.dump(times, open(os.path.join(HERE, "_equiv_%s_t.json" % TAG), "w", encoding="utf-8"))
    t = np.array(list(times.values()))
    print("%s: %d 帧  中位 %.2f ms  最大 %.2f ms" % (TAG, len(t), np.median(t), t.max()))
