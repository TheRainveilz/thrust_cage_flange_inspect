# -*- coding: utf-8 -*-
"""CFG5(精定位孔心拟合节圆)的**真实裕度**测量: NG 侧离翻判到底还差几个压痕。

为什么必须量这个: CFG4 的 "0 逃逸" 里, 410 张 NG 是靠"非 pitch_fit -> 前置闸全额否决"这种
**结构性**保护兜住的 —— 那种 0 没有信息量。CFG5 把其中 117 张拉回了 pitch_fit(反面
pitch_fit 213->330), 节圆判据**真的开始生效**了。此时 0 逃逸才是一次有信息量的观测:
它说明"真节圆 + 闸"这套判据在反面上扛住了, 而不是"我们没看"。

本脚本量的是**距离**(还差多少), 不是结果(过没过):

  NG 侧每张图按"最接近翻判的那个在节圆上的孔"归类:
     [闸保护]  受检孔全不在节圆上        -> 根本没验证, fail-safe 否决
     [A否决]   有在节圆上的孔, 但 feature_a 全为 0
     [只差B]   有 on_pitch 且 feature_a=1 的孔, 但压痕 < MIN_VALID_MARKS  -> 真实裕度所在
     [只差1痕] 上述孔还带 >=1 个压痕       -> 再冒一个压痕就是逃逸(最危险的一档)
  OK 侧: 有多少张图受检孔全不在节圆上(= 被闸过杀的)。

只读, 不改任何东西。用法:
    .venv/Scripts/python.exe tools/_diag_margin_cfg5.py
"""
import os
import sys
from collections import Counter, defaultdict
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from src.flange_inspect import inspector_pure as T

IMG_ROOT = r"I:\zq.zip\CPlusPlus_Vs_PurePy\imageDataClass"
BUCKETS = ("闸保护", "A否决", "只差B", "只差1痕")


def truth_of(name):
    for p in [q.lower() for q in name.replace("/", "\\").split("\\")][:-1]:
        if p == "ok":
            return "front"
        if p == "ng":
            return "back"
    return "?"


def main():
    T.HOLE_CHECK_COUNT = 0
    print("[配置] HOLE_PITCH_GATE=%s  容差=%.0f%% pitch_r / %.0fpx  MIN_VALID_MARKS=%d"
          % (T.HOLE_PITCH_GATE, T.HOLE_PITCH_TOL_RATIO * 100, T.PITCH_FIT_TOL_MIN_PX,
             T.MIN_VALID_MARKS))
    src = T.LocalFolderSource(IMG_ROOT, True)
    print("[样本] %d 张" % len(src))

    bucket = defaultdict(Counter)          # 真值 -> 归类 -> 图数
    onpitch_per_img = defaultdict(list)    # 真值 -> 每图 on_pitch 孔数
    near_miss_marks = []                   # [只差B/只差1痕] 图上, 最危险孔压痕数
    loc_split = defaultdict(Counter)       # 真值 -> 定位方法 -> 图数
    n_img = Counter()

    for name, bgr in src.frames():
        if bgr is None:
            continue
        truth = truth_of(name)
        n_img[truth] += 1
        _b, _g, work, _c, _s = T.preprocess(bgr)
        try:
            res, _ = T.inspect(bgr, os.path.basename(name))
        except Exception as exc:  # noqa: BLE001
            bucket[truth]["异常"] += 1
            print("[异常] %s: %s" % (name, exc))
            continue
        loc_split[truth][res.locate_method.split("(")[0]] += 1

        checked = [h for h in res.holes if h.index in set(res.checked)]
        if not checked:
            bucket[truth]["无受检孔"] += 1
            onpitch_per_img[truth].append(0)
            continue

        op = [h for h in checked if h.on_pitch]
        onpitch_per_img[truth].append(len(op))
        if not op:
            bucket[truth]["闸保护"] += 1
            continue
        a_ok = [h for h in op if h.feature_a]
        if not a_ok:
            bucket[truth]["A否决"] += 1
            continue
        # 有 on_pitch 且 feature_a=1 的孔: 现在只差 B 的压痕数
        m = max(h.valid_marks for h in a_ok)
        bucket[truth]["只差1痕" if m >= 1 else "只差B"] += 1
        near_miss_marks.append(m)
        if truth == "back":
            print("[最危险] %-14s %s  on_pitch=%d  a_ok=%d  最危险孔 marks=%d (need>=%d)"
                  % (os.path.basename(name), res.locate_method, len(op), len(a_ok),
                     m, T.MIN_VALID_MARKS))

    print("\n[逐图归类]  (NG 侧每一张离翻判的距离)")
    for t in ("front", "back"):
        tot = sum(bucket[t].values())
        if not tot:
            continue
        print("  %-6s 共 %d 张:" % (t, tot))
        for k in BUCKETS + ("无受检孔", "异常"):
            if bucket[t][k]:
                print("        %-6s %5d (%.1f%%)" % (k, bucket[t][k], 100.0 * bucket[t][k] / tot))

    print("\n[每图 '在节圆上的受检孔' 个数分布]")
    for t in ("front", "back"):
        v = np.array(onpitch_per_img[t])
        if len(v):
            print("  %-6s n=%d  0个: %d (%.1f%%)  中位=%d  p90=%d  max=%d"
                  % (t, len(v), int((v == 0).sum()), 100.0 * (v == 0).mean(),
                     np.median(v), np.percentile(v, 90), v.max()))

    print("\n[定位方法落点]  (CFG5 是否把反面从 boundary_circle 拉回了 pitch_fit)")
    for t in ("front", "back"):
        if loc_split[t]:
            print("  %-6s %s" % (t, dict(loc_split[t])))

    if near_miss_marks:
        v = np.array(near_miss_marks)
        print("\n[最危险孔压痕数分布] (feature_a=1 且在节圆上, 距离逃逸只差 %d 个压痕)"
              % T.MIN_VALID_MARKS)
        print("  n=%d  0个: %d  1个: %d  >=2(应恒为0, 否则已逃逸): %d"
              % (len(v), int((v == 0).sum()), int((v == 1).sum()), int((v >= 2).sum())))


if __name__ == "__main__":
    main()
