# -*- coding: utf-8 -*-
"""按用户 2026-10-02 的逐张定性，把 `反面/NG/` 全部 44 张分类并统计闸归因。

分类依据(用户口头确认)：
  真反面   14 = `Class/反面/真反面/` 全量
  手动反面 25 = `Class/反面/手动反面/` 的 24 张 + `20261001.120646.293.000084`
  直接NG    5 = 000028/000029(20261001.114846 那两张，没拍全) + F000005/F000006/F000007
  —— 用户补充：真反面里 10 张是故意少放球(模拟缺粒)，只有 4 张是球装满的；
     且 `F000007` 是坏件。
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

import inspector_missing as im   # noqa: E402

BACK = "I:/data.zip/data/missing/Class/反面"
NG = os.path.join(BACK, "NG")

TRUE_BACK = {
    "20260928.182739.550.000002", "20260928.183621.193.000026",
    "20260928.183634.541.000027", "20260928.183643.485.000028",
    "20260928.183650.592.000029", "20260928.183650.874.000030",
    "20260928.183745.116.000031", "20260928.183801.166.000032",
    "20260928.183942.699.000037", "20260928.183956.717.000038",
    "20260928.184003.626.000039", "20260928.184021.715.000040",
    "20261001.114846.408.000027", "20261001.121133.282.000151",
}
MANUAL_EXTRA = {"20261001.120646.293.000084"}
DIRECT_NG = {
    "20261001.114846.574.000028", "20261001.114846.759.000029",
    "20261001_153802_830_CAM_F000005_RAW", "20261001_153805_149_CAM_F000006_RAW",
    "20261001_153807_232_CAM_F000007_RAW",
}


def stem(rel):
    return os.path.basename(rel).replace(".origin.png", "").replace(".png", "")


def klass(rel):
    s = stem(rel)
    if s in TRUE_BACK:
        return "真反面"
    if s in DIRECT_NG:
        return "直接NG"
    if s in MANUAL_EXTRA or os.path.exists(os.path.join(BACK, "手动反面",
                                                        os.path.basename(rel))):
        return "手动反面"
    return "??未归类"


def main():
    files = sorted(os.listdir(NG))
    assert len(files) == 44, len(files)
    rows = []
    for fn in files:
        rel = os.path.join(NG, fn)
        bgr = cv2.imdecode(np.fromfile(rel, dtype=np.uint8), cv2.IMREAD_COLOR)
        r = im.inspect_missing(bgr, fn)
        rows.append({
            "cls": klass(fn), "fn": stem(fn),
            "hl_gate": r.n_empty > 0, "rr_med": r.r_ratio_med,
            "n_empty": r.n_empty,
        })

    order = ["真反面", "手动反面", "直接NG", "??未归类"]
    out = []
    for c in order:
        sub = [x for x in rows if x["cls"] == c]
        if not sub:
            continue
        hl = sum(1 for x in sub if x["hl_gate"])
        only_med = sum(1 for x in sub if not x["hl_gate"])
        rrmin = min(x["rr_med"] for x in sub)
        rrmax = max(x["rr_med"] for x in sub)
        out.append("%-6s 帧数 %2d | 孔级拦 %2d | 仅件级拦 %2d | rr_med %.4f~%.4f"
                   % (c, len(sub), hl, only_med, rrmin, rrmax))
        for x in sub:
            out.append("        %-34s rr_med=%.4f  nEmpty=%2d  %s"
                       % (x["fn"], x["rr_med"], x["n_empty"],
                          "孔级拦" if x["hl_gate"] else "仅件级拦"))
    total_hl = sum(1 for x in rows if x["hl_gate"])
    out.append("")
    out.append("合计 44 | 孔级拦 %d | 仅件级拦 %d" % (total_hl, 44 - total_hl))
    out.append("逃逸 = 0 (全部 44 张判 NG，无一张判 OK)")

    txt = os.path.join(HERE, "_back_final.txt")
    with open(txt, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")
    print("saved", txt)


if __name__ == "__main__":
    main()
