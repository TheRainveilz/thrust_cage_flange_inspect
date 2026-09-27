# -*- coding: utf-8 -*-
"""特征A 到底是什么在通过? —— 反面的"第 2 圈"是谁。

用户的疑问(2026-09-21): 翻边是深度吗? 从 OK 面看止口是有锥度的, 那 NG(反面)本来就看不到
翻边, 为什么图片级 A 还有 97.6%、孔级 A 还有 72%?

读码结论: **A 根本不测深度**。`feature_a_ring_contours` 只做一件事 —— 在孔心周围
[0.86r, 1.36r] 的环带里, 用 9 个灰度百分位 × 正反两个方向 = 18 张二值图找轮廓, 保留
"够圆(径向std/r<0.16) + 够完整(角度覆盖>0.20) + 平均半径落在环带内"的轮廓, 再按半径聚类,
**被 >=2 个阈值重复命中的簇才算一环**; ring_count >= 2 即 A 通过(或同心 Hough 在
[1.12r,1.36r] 找到圆作为 OR 兜底)。所以 A 判的是"孔周围有没有**同心环状边缘结构**",
不是"有没有翻边"。而同心环结构在正反面都可能存在 —— 这就是反面 A 高的候选解释。

本脚本用**已有的** `HoleResult.ring_radii` / `flange_hough_r` 直接量这件事, 不加载任何新逻辑:
  1) 所有环的半径比分布(按真值分组): 正面是否堆在 >1.1r(真翻边), 反面是否堆在 ~1.0r(孔口本身)?
  2) ring_count 分布 + ring_count>=2 的孔上 "最小圈/最大圈" 半径。
  3) 同心 Hough 命中的半径比分布(front vs back)。
若反面的第 2 圈与正面**同分布**, 说明反面确实有同样的环形结构(那不是"假阳性", 是 A 判据不特异);
若反面堆在 1.0r 附近, 说明 A 在反面数的其实是**孔口边缘自己**被拆成了两圈。

只读, 不改任何东西。
"""
import os
import sys
from collections import Counter, defaultdict
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from src.flange_inspect import inspector_pure as T

IMG_ROOT = r"I:\zq.zip\CPlusPlus_Vs_PurePy\imageDataClass"
BIN = 0.05  # 半径比分箱宽度


def truth_of(name):
    for p in [q.lower() for q in name.replace("/", "\\").split("\\")][:-1]:
        if p == "ok":
            return "front"
        if p == "ng":
            return "back"
    return "?"


def hist(vals):
    """半径比 -> 分箱直方图字符串(只打非空箱, 按箱号排序)。"""
    c = Counter(round(v / BIN) * BIN for v in vals)
    return " ".join("%.2f:%d" % (k, c[k]) for k in sorted(c))


def main():
    T.HOLE_CHECK_COUNT = 0
    print("[配置] FEATURE_A_MODE=%s  RING_COUNT_MIN=%d  RING_BAND=%s  HOUGH_BAND=%s"
          % (T.FEATURE_A_MODE, T.RING_COUNT_MIN, T.RING_BAND, T.FLANGE_HOUGH_BAND))
    src = T.LocalFolderSource(IMG_ROOT, True)

    rings = defaultdict(list)          # 真值 -> 所有环的半径比
    nring = defaultdict(Counter)       # 真值 -> ring_count 分布
    minmax = defaultdict(list)         # 真值 -> (最小圈, 最大圈) for ring_count>=2
    hough = defaultdict(list)          # 真值 -> flange_hough_r
    by_pitch = defaultdict(lambda: defaultdict(list))  # 真值 -> on_pitch -> 环半径比
    a_hole = defaultdict(Counter)      # 真值 -> 孔级 A 通过/不通过
    n_hole = Counter()

    for name, bgr in src.frames():
        if bgr is None:
            continue
        truth = truth_of(name)
        try:
            res, _ = T.inspect(bgr, os.path.basename(name))
        except Exception as exc:  # noqa: BLE001
            print("[异常] %s: %s" % (name, exc))
            continue
        for h in res.holes:
            if h.index not in set(res.checked):
                continue
            n_hole[truth] += 1
            a_hole[truth]["A通过" if h.feature_a else "A不通过"] += 1
            nring[truth][h.ring_count] += 1
            if h.ring_radii:
                rings[truth].extend(h.ring_radii)
                by_pitch[truth][int(bool(h.on_pitch))].extend(h.ring_radii)
            if h.ring_count >= 2 and len(h.ring_radii) >= 2:
                minmax[truth].append((min(h.ring_radii), max(h.ring_radii)))
            if h.flange_hough_r is not None:
                hough[truth].append(h.flange_hough_r)

    print("\n[孔级 A 通过率]")
    for t in ("front", "back"):
        n = n_hole[t]
        if n:
            print("  %-6s %s / %d = %.1f%%"
                  % (t, dict(a_hole[t]), n, 100.0 * a_hole[t]["A通过"] / n))

    print("\n[ring_count 分布]  (0=环带内找不到同心环, >=%d 即 A 的轮廓分支通过)" % T.RING_COUNT_MIN)
    for t in ("front", "back"):
        c = nring[t]
        tot = sum(c.values())
        if not tot:
            continue
        print("  %-6s " % t + " ".join("%d:%d(%.0f%%)" % (k, c[k], 100.0 * c[k] / tot)
                                       for k in sorted(c)))

    print("\n[所有环的半径比分布 / r]  (正面该堆在真翻边处; 反面若堆在 1.0 附近 = 数的是孔口自己)")
    for t in ("front", "back"):
        if rings[t]:
            print("  %-6s n=%d" % (t, len(rings[t])))
            print("        " + hist(rings[t]))

    print("\n[按是否在节圆上拆开]  (只有 on_pitch=1 的孔才是真孔位, 它的环才有资格谈翻边)")
    for t in ("front", "back"):
        for op in (1, 0):
            v = by_pitch[t].get(op)
            if v:
                print("  %-6s on_pitch=%d n=%-5d 中位半径比=%.3f  p10=%.3f p90=%.3f"
                      % (t, op, len(v), np.median(v), np.percentile(v, 10), np.percentile(v, 90)))

    print("\n[ring_count>=2 的孔: 最小圈 / 最大圈 半径比]")
    for t in ("front", "back"):
        v = minmax[t]
        if v:
            a = np.array(v)
            print("  %-6s n=%-5d 最小圈 中位=%.3f  最大圈 中位=%.3f  两者差 中位=%.3f"
                  % (t, len(v), np.median(a[:, 0]), np.median(a[:, 1]),
                     np.median(a[:, 1] - a[:, 0])))

    print("\n[同心 Hough 命中半径比]  (OR 兜底分支, 目标带 %s)" % (T.FLANGE_HOUGH_BAND,))
    for t in ("front", "back"):
        v = hough[t]
        if v:
            print("  %-6s 命中 %d 个孔  中位=%.3f  p10=%.3f p90=%.3f"
                  % (t, len(v), np.median(v), np.percentile(v, 10), np.percentile(v, 90)))
        else:
            print("  %-6s 命中 0 个孔" % t)


if __name__ == "__main__":
    main()
