#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""一次性诊断脚本：在 Class 数据集上全量跑 inspector_missing.inspect_missing，
把逐帧判定 + 逐槽量值落成 JSON，供后续分析混淆矩阵与阈值分布。

用法: python tools/_missing_sweep.py "<数据集根>"  [--limit N]
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "flange_inspect"))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import inspector_missing as im  # noqa: E402
import inspector_pure as ip  # noqa: E402

logging.getLogger().addHandler(logging.NullHandler())
logging.disable(logging.CRITICAL)  # 关掉 [SLOT] 明细刷屏


def true_label(path: str) -> str:
    """按目录名给真值：正面/OK -> OK，其余 -> NG。"""
    p = path.replace("\\", "/")
    if "/正面/OK/" in p or p.endswith("/正面/OK"):
        return "OK"
    return "NG"


def imread_u(path: str):
    data = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--out", default="tools/_missing_sweep.json")
    args = ap.parse_args()

    files = []
    for dirpath, _d, fnames in os.walk(args.root):
        for fn in sorted(fnames):
            if fn.lower().endswith((".png", ".jpg", ".jpeg", ".bmp")):
                files.append(os.path.join(dirpath, fn))
    files.sort()
    if args.limit:
        files = files[: args.limit]

    rows = []
    t_start = time.time()
    for i, path in enumerate(files, 1):
        bgr = imread_u(path)
        lab = true_label(path)
        if bgr is None:
            rows.append({"file": path, "true": lab, "verdict": "READ_FAIL", "reason": ""})
            continue
        try:
            r = im.inspect_missing(bgr, os.path.basename(path))
        except Exception as exc:  # noqa: BLE001
            rows.append({"file": path, "true": lab, "verdict": "EXC:%s" % exc, "reason": ""})
            continue
        rows.append({
            "file": path, "true": lab, "verdict": r.verdict, "reason": r.reason,
            "method": r.locate_method, "pitch_r": round(r.pitch_r, 1),
            "phase_deg": round(r.phase_deg, 2), "phase_coh": round(r.phase_coh, 3),
            "n_empty": r.n_empty, "n_slot_fail": r.n_slot_fail,
            "r_ratio_med": round(r.r_ratio_med, 4), "elapsed_ms": round(r.elapsed_ms, 1),
            "pockets": [{"k": p.index, "rr": round(p.r_ratio, 4),
                         "hl": round(p.highlight, 1), "ball": p.has_ball} for p in r.pockets],
        })
        if i % 25 == 0 or i == len(files):
            print("[%d/%d] %.1fs" % (i, len(files), time.time() - t_start), flush=True)

    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(rows, fh, ensure_ascii=False, indent=1)

    # 混淆矩阵
    from collections import Counter
    cm = Counter()
    for r in rows:
        got = "OK" if r["verdict"] == im.OK_PASS else "NG"
        cm[(r["true"], got)] += 1
    print("\n==== 混淆矩阵 (真值 -> 判定) ====")
    for k in sorted(cm):
        print("  %-3s -> %-3s : %d" % (k[0], k[1], cm[k]))

    # 逐目录细分
    bydir = {}
    for r in rows:
        d = os.path.basename(os.path.dirname(r["file"])) + "/" + os.path.basename(r["file"]).split("_")[0]
        bydir.setdefault(os.path.dirname(r["file"]), Counter())[r["verdict"]] += 1
    print("\n==== 逐目录判定 ====")
    for d in sorted(bydir):
        print("  %-60s %s" % (d.replace(args.root, "."), dict(bydir[d])))

    # 失败原因分布
    reasons = Counter()
    for r in rows:
        if r["verdict"] != im.OK_PASS:
            reasons[r["verdict"]] += 1
    print("\n==== NG 原因分布 ====")
    for k, v in reasons.most_common():
        print("  %-24s %d" % (k, v))
    print("\nJSON ->", args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
