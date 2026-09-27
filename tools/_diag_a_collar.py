# -*- coding: utf-8 -*-
"""量"翻边领圈"的深/黑/宽 —— 肉眼可见的真正正反差异, 前几版量错了地方。

看了实图(OK CAM#000003 vs NG CAM#000009)后确认: 正面每孔周围一圈**又黑又粗的深色领圈**
(翻边立体 collar), 反面只有一条**细边线**、没有领圈。之前量的孔径(差 3 丝)、翻边区亮度斜坡
(还反了)都不是这个信号。这里直接量领圈:

  depth  领圈暗程度 = (外侧亮环中位灰 - 领圈带最暗箱灰) / 外侧亮环中位灰
         正面深领圈 -> 大; 反面无领圈 -> 小。对整体曝光免疫(比值)。
  darkfrac 领圈带里"明显暗于外侧"的像素占比 = 领圈有多宽。
  ringmin_ratio 领圈带最暗箱 / 外侧亮环   (越小 = 领圈越黑)

领圈带取 [1.00r, 1.35r](孔壁外沿到翻边顶), 外侧亮环取 [1.40r, 1.65r](平面金属)。
只统计 on_pitch=1 且 feature_a=1 的孔 —— 就是"反面里已经骗过 A 的那 47%", 看能不能靠领圈深度把它们卡掉。
只读。用法: .venv/Scripts/python.exe tools/_diag_a_collar.py
"""
import os
import sys
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from src.flange_inspect import inspector_pure as T

IMG_ROOT = r"I:\zq.zip\CPlusPlus_Vs_PurePy\imageDataClass"
COLLAR = np.arange(1.00, 1.36, 0.05)      # 领圈带分箱
OUT_LO, OUT_HI = 1.40, 1.65               # 外侧亮环(参考)


def truth_of(name):
    for p in [q.lower() for q in name.replace("/", "\\").split("\\")][:-1]:
        if p == "ok":
            return "front"
        if p == "ng":
            return "back"
    return "?"


def collar_metrics(gray, cx, cy, r):
    h, w = gray.shape[:2]
    rr = r * 1.72
    x0, x1 = max(0, int(cx - rr)), min(w, int(cx + rr) + 1)
    y0, y1 = max(0, int(cy - rr)), min(h, int(cy + rr) + 1)
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    ys, xs = np.mgrid[y0:y1, x0:x1]
    d = np.hypot(xs - cx, ys - cy) / max(r, 1e-6)
    patch = gray[y0:y1, x0:x1].astype(np.float64)
    out_m = (d >= OUT_LO) & (d < OUT_HI)
    if out_m.sum() < 20:
        return None
    surround = float(np.median(patch[out_m]))
    if surround < 1.0:
        return None
    bins = []
    for lo, hi in zip(COLLAR[:-1], COLLAR[1:]):
        m = (d >= lo) & (d < hi)
        if m.sum() >= 6:
            bins.append(patch[m].mean())
    coll_m = (d >= COLLAR[0]) & (d < COLLAR[-1])
    if len(bins) < 3 or coll_m.sum() < 20:
        return None
    ringmin = float(min(bins))
    depth = (surround - ringmin) / surround
    darkfrac = float((patch[coll_m] < 0.75 * surround).mean())
    return depth, darkfrac, ringmin / surround


def pct(a, b):
    return "%.1f%%" % (100.0 * a / b) if b else "n/a"


def qtiles(v):
    v = np.sort(np.asarray(v, np.float64))
    if len(v) == 0:
        return "n=0"
    def q(p):
        return float(v[min(len(v) - 1, int(p * len(v)))])
    return "n=%d  p10=%.3f p25=%.3f p50=%.3f p75=%.3f p90=%.3f" % (
        len(v), q(.10), q(.25), q(.50), q(.75), q(.90))


def gate_scan(front, back, name, direction):
    f, b = np.asarray(front), np.asarray(back)
    allv = np.concatenate([f, b])
    lo, hi = np.percentile(allv, 2), np.percentile(allv, 98)
    print("  [%s 当附加闸, direction=%s]  (孔级)" % (name, direction))
    for thr in np.linspace(lo, hi, 9):
        if direction == "high":       # 值大=真正面孔(深领圈)
            fk, bk = int((f >= thr).sum()), int((b >= thr).sum())
        else:                          # 值小=真正面孔
            fk, bk = int((f <= thr).sum()), int((b <= thr).sum())
        print("     %7.3f   正面保留 %6s      反面压下 %6s"
              % (thr, pct(fk, len(f)), pct(len(b) - bk, len(b))))


def main():
    T.HOLE_CHECK_COUNT = 0
    src = T.LocalFolderSource(IMG_ROOT, True)
    print("[样本] %d 张   领圈带=[1.00,1.35]r  外侧亮环=[1.40,1.65]r   只统计 on_pitch & feature_a"
          % len(src))
    d = {"front": {"depth": [], "darkfrac": [], "rmr": []},
         "back": {"depth": [], "darkfrac": [], "rmr": []}}
    for name, bgr in src.frames():
        if bgr is None:
            continue
        truth = truth_of(name)
        if truth == "?":
            continue
        try:
            _b, gray, _w, _c, _s = T.preprocess(bgr)
            res, _ = T.inspect(bgr, os.path.basename(name))
        except Exception as exc:  # noqa: BLE001
            print("[异常] %s: %s" % (name, exc))
            continue
        if res.pitch_r <= 0:
            continue
        checked = set(res.checked)
        for h in res.holes:
            if h.index not in checked or not h.on_pitch or not h.feature_a:
                continue
            m = collar_metrics(gray, h.cx, h.cy, h.r)
            if m is not None:
                d[truth]["depth"].append(m[0])
                d[truth]["darkfrac"].append(m[1])
                d[truth]["rmr"].append(m[2])

    for key, title, direction in (
            ("depth", "领圈暗程度 depth (正面深领圈->大)", "high"),
            ("darkfrac", "领圈暗像素占比 darkfrac (领圈宽->大)", "high"),
            ("rmr", "领圈最暗/外侧 ringmin_ratio (越小领圈越黑)", "low")):
        print("\n" + "=" * 84)
        print("C %s" % title)
        print("=" * 84)
        print("  front ", qtiles(d["front"][key]))
        print("  back  ", qtiles(d["back"][key]))
        gate_scan(d["front"][key], d["back"][key], key, direction)
    print("\n[读法] 找一档: 正面保留>=98%(几乎不砸召回) 且 反面压下尽量高。"
          "\n       若某个量能在保正面 98% 时压下反面 30%+, 它就能当 A 的收紧项(结构上只增过杀、绝不增逃逸)。")


if __name__ == "__main__":
    main()
