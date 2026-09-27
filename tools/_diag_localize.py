# -*- coding: utf-8 -*-
"""量化"检出的孔到底有多少真的落在节圆上"。

动机: locate_part() 用 fit_circle_robust(RANSAC 式, 内点容差 = 6% 节圆半径) 拟合节圆,
返回的 locate_method 里带 n_in = 内点数。之后所有"半径一致"的候选都当孔用 —— 不管它在不在节圆上。
不在节圆上的孔 -> 拐角 ROI 切偏 -> A 的环检测和 B 的压痕检测都在读错的像素。

本脚本直接调主模块, 对每个受检孔算 resid_n = |dist(孔心, 工件中心) - pitch_r| / hole_r,
按真值/A-B 判定分组统计。只读, 不改任何东西。
"""
import os
import sys
from collections import Counter, defaultdict
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from src.flange_inspect import inspector_pure as T

IMG_ROOT = r"I:\zq.zip\CPlusPlus_Vs_PurePy\imageDataClass"
TOL_RATIO = T.PITCH_FIT_TOL_RATIO          # 模块自己的内点容差(6% 节圆半径)


def truth_of(name):
    parts = [p.lower() for p in name.replace("/", "\\").split("\\")]
    for p in parts[:-1]:
        if p == "ok":
            return "front"
        if p == "ng":
            return "back"
    return "?"


def main():
    print("[配置] 模块默认 HOLE_CHECK_COUNT=%s (本脚本强制 0=全部孔, 与生产配置一致)"
          % T.HOLE_CHECK_COUNT)
    T.HOLE_CHECK_COUNT = 0
    src = T.LocalFolderSource(IMG_ROOT, True)
    print("[样本] %d 张" % len(src))
    recs = []
    ring = []          # 每张图: 孔心到"模块自报工件中心"的距离统计 —— 用来分清
                       # "是 pitch_r/中心 定错了" 还是 "孔心本身散得到处都是(伪孔)"
    n_method = Counter()
    for name, bgr in src.frames():
        if bgr is None:
            continue
        res, _ = T.inspect(bgr, name)
        n_method[res.locate_method.split("(")[0]] += 1
        if res.part_cx is None or res.pitch_r <= 0:
            continue
        truth = truth_of(name)
        ds = []
        for h in res.holes[:len(res.checked)]:
            d = float(np.hypot(h.cx - res.part_cx, h.cy - res.part_cy))
            ds.append(d)
            recs.append({
                "img": os.path.basename(name), "truth": truth, "hole": int(h.index),
                "resid_n": abs(d - res.pitch_r) / max(h.r, 1e-6),
                "on_pitch": abs(d - res.pitch_r) <= TOL_RATIO * res.pitch_r,
                "feature_a": bool(h.feature_a), "marks": int(h.valid_marks),
                "passed": bool(h.passed),
                "n_in": int(res.locate_method.split("n=")[1].rstrip(")"))
                        if "n=" in res.locate_method else -1,
                "n_holes": len(res.holes), "n_checked": len(res.checked),
            })
        if ds:
            a = np.array(ds)
            hr = max(float(res.holes[0].r), 1e-6)
            ring.append({"truth": truth, "n": len(a),
                         "bias": (float(np.median(a)) - res.pitch_r) / hr,   # 中位半径 - pitch_r
                         "mad": float(np.median(np.abs(a - np.median(a)))) / hr})  # 散度

    print("[定位方式] %s" % dict(n_method))
    print("\n[机理: 孔心环 相对 模块自报 pitch_r 是'整体偏'还是'散'?] (单位=hole_r)")
    print("   bias = 中位(孔心到模块中心距离) - pitch_r  ->  大负值=孔心整体落在节圆内侧")
    print("   mad  = 该距离的中位绝对偏差            ->  大值=孔心根本不在任何一个同心圆上")
    for t in ("front", "back"):
        v = [r for r in ring if r["truth"] == t]
        if not v:
            continue
        b = np.array([r["bias"] for r in v])
        m = np.array([r["mad"] for r in v])
        print("   %-6s 图数=%-4d bias: p10=%.2f p50=%.2f p90=%.2f | mad: p50=%.2f p90=%.2f"
              % (t, len(v), np.percentile(b, 10), np.percentile(b, 50), np.percentile(b, 90),
                 np.percentile(m, 50), np.percentile(m, 90)))
    rn = np.array([r["resid_n"] for r in recs])
    print("\n[受检孔 resid/hole_r 分布] n=%d" % len(recs))
    for q in (50, 75, 90, 95, 99, 100):
        print("   p%-4s = %.3f" % (q, np.percentile(rn, q)))
    print("   节圆内点占比(容差 %.0f%% pitch_r) = %.1f%%"
          % (TOL_RATIO * 100, 100.0 * sum(r["on_pitch"] for r in recs) / len(recs)))

    print("\n[按真值]")
    for t in ("front", "back"):
        v = [r for r in recs if r["truth"] == t]
        if not v:
            continue
        arr = np.array([r["resid_n"] for r in v])
        print("   %-6s n=%-5d 在节圆上 %.1f%%  p50=%.3f p90=%.3f max=%.3f"
              % (t, len(v), 100.0 * sum(r["on_pitch"] for r in v) / len(v),
                 np.percentile(arr, 50), np.percentile(arr, 90), arr.max()))

    print("\n[关键: 判定 x 是否在节圆上]")
    print("   %-6s %-26s %6s %10s %10s" % ("真值", "分组", "孔数", "在节圆上%", "平均resid"))
    for t in ("front", "back"):
        for lbl, sel in (
            ("A PASS", lambda r: r["truth"] == t and r["feature_a"]),
            ("A FAIL", lambda r: r["truth"] == t and not r["feature_a"]),
            ("有压痕(marks>=1)", lambda r: r["truth"] == t and r["marks"] >= 1),
            ("无压痕", lambda r: r["truth"] == t and r["marks"] == 0),
        ):
            v = [r for r in recs if sel(r)]
            if not v:
                continue
            print("   %-6s %-26s %6d %9.1f%% %10.3f"
                  % (t, lbl, len(v), 100.0 * sum(r["on_pitch"] for r in v) / len(v),
                     float(np.mean([r["resid_n"] for r in v]))))

    print("\n[反面 有压痕(marks>=1) 的孔逐个看 —— 这些是唯一的'差点逃逸'来源]")
    bad = [r for r in recs if r["truth"] == "back" and r["marks"] >= 1]
    bad.sort(key=lambda r: -r["resid_n"])
    onp = sum(r["on_pitch"] for r in bad)
    print("   共 %d 个孔, 其中在节圆上的只有 %d 个 (%.1f%%)"
          % (len(bad), onp, 100.0 * onp / max(len(bad), 1)))
    print("   %-46s %5s %8s %8s %5s %5s" % ("image", "marks", "resid_n", "on_pitch", "n_in", "A"))
    for r in bad[:25]:
        print("   %-46s %5d %8.3f %8s %5d %5s"
              % (r["img"][:46], r["marks"], r["resid_n"], "Y" if r["on_pitch"] else "N",
                 r["n_in"], "P" if r["feature_a"] else "F"))

    print("\n[每张图有几个孔真在节圆上] (n_in = 模块 RANSAC 内点数)")
    hist = Counter(r["n_in"] for r in recs if r["marks"] >= 0)
    hh = defaultdict(Counter)
    for r in recs:
        hh[r["truth"]][r["n_in"]] += 1
    for t in ("front", "back"):
        if hh[t]:
            print("   %-6s 节圆内点数分布: %s"
                  % (t, sorted(hh[t].items())))

    print("\n[11 个反面 marks>=2 的孔(唯一能凑够 tm=2 的那批)]")
    two = [r for r in recs if r["truth"] == "back" and r["marks"] >= 2]
    two.sort(key=lambda r: -r["resid_n"])
    for r in two:
        print("   %-46s hole#%-3s marks=%d resid_n=%6.3f 在节圆上=%s A=%s"
              % (r["img"][:46], r["hole"], r["marks"], r["resid_n"],
                 "Y" if r["on_pitch"] else "N", "P" if r["feature_a"] else "F"))
    print("   -> %d 个里 %d 个在节圆上" % (len(two), sum(r["on_pitch"] for r in two)))


def image_level():
    """把"只认可在节圆上的孔"当成一种更严的判定, 看正面召回/反面逃逸各是多少。"""
    src = T.LocalFolderSource(IMG_ROOT, True)
    T.HOLE_CHECK_COUNT = 0
    stat = defaultdict(lambda: Counter())
    for name, bgr in src.frames():
        if bgr is None:
            continue
        res, _ = T.inspect(bgr, name)
        truth = truth_of(name)
        if res.part_cx is None or res.pitch_r <= 0:
            stat[truth]["无定位"] += 1
            continue
        tol = TOL_RATIO * res.pitch_r
        hs = list(res.holes[:len(res.checked)])
        cur = any(h.passed for h in hs)                       # 现状: 全部受检孔
        onp = [h for h in hs
               if abs(float(np.hypot(h.cx - res.part_cx, h.cy - res.part_cy)) - res.pitch_r) <= tol]
        strict = any(h.passed for h in onp)                   # 只算节圆上的孔
        stat[truth]["张数"] += 1
        stat[truth]["现状OK" if cur else "现状NG"] += 1
        stat[truth]["严OK" if strict else "严NG"] += 1
        stat[truth]["节圆内孔数为0" if not onp else "节圆内孔数>0"] += 1

    print("\n" + "=" * 78)
    print("[真实健康度: 把'孔必须落在节圆上'作为合法性前提后重算]")
    fr = stat["front"]
    bk = stat["back"]
    print("   正面(OK) %d 张:  现状判OK %d (%.1f%%)   ->  只认节圆孔 %d (%.1f%%)"
          % (fr["张数"], fr["现状OK"], 100.0 * fr["现状OK"] / max(fr["张数"], 1),
             fr["严OK"], 100.0 * fr["严OK"] / max(fr["张数"], 1)))
    print("   反面(NG) %d 张:  现状判OK %d (%.1f%%)  <- 逃逸   ->  只认节圆孔 %d (%.1f%%)"
          % (bk["张数"], bk["现状OK"], 100.0 * bk["现状OK"] / max(bk["张数"], 1),
             bk["严OK"], 100.0 * bk["严OK"] / max(bk["张数"], 1)))
    print("   反面里'一个节圆内孔都没有'的图: %d / %d (%.1f%%) —— 这些图根本没有合法 ROI 可用"
          % (bk["节圆内孔数为0"], bk["张数"], 100.0 * bk["节圆内孔数为0"] / max(bk["张数"], 1)))
    print("   正面里'一个节圆内孔都没有'的图: %d / %d (%.1f%%)"
          % (fr["节圆内孔数为0"], fr["张数"], 100.0 * fr["节圆内孔数为0"] / max(fr["张数"], 1)))


if __name__ == "__main__":
    main()
    image_level()
