# -*- coding: utf-8 -*-
"""特征A 分支四档扫描 —— FEATURE_A_MODE ∈ {contour, hough, contour_or_hough, contour_and_hough}

动机(2026-09-21 实测, `tools/_diag_feature_a_rings.py`, 975 张):
  A 的判别力**几乎全部来自 hough 分支**: 同心 Hough(1.12~1.36r) 正面命中 96.4% 的孔、
  反面只有 47.0%; 而轮廓分支(ring_count>=2)正反面几乎一样(56.7% vs 50.9%, 且反面在
  1.20~1.25r 的环占比 37.5% 比正面 26.7% 还高)。当前 FEATURE_A_MODE="contour_or_hough"
  是**或**逻辑 —— 判别力弱的那个分支把强的淹掉了, 所以孔级 A 只有 98.1%(正) vs 72.0%(反)。
  把模式换成只认 hough, A 才可能真变成一道反面的闸。

本脚本对四档各跑一遍全量, 每档报同一组数(重点是**NG 裕度**, 不是召回):
  正面: 判OK 图数 / A 图级 / A 孔级        —— 保召回的代价
  反面: 逃逸(必须恒 0) / 裕度归类 / **真孔位(on_pitch=1)上的 A 通过率** / Hough 命中率
裕度归类(同 `_diag_margin_cfg5.py`): 闸保护 / A否决 / 只差B / 只差1痕
  "只差B"+"只差1痕" = 节圆上已有 A 孔、只等压痕的图数 —— 这个数越小, 裕度越厚。

只读, 不改 inspector_pure.py 的任何常量(只改运行时全局 T.FEATURE_A_MODE)。
用法: .venv/Scripts/python.exe tools/_diag_feature_a_sweep.py
"""
import os
import sys
from collections import Counter
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from src.flange_inspect import inspector_pure as T

IMG_ROOT = r"I:\zq.zip\CPlusPlus_Vs_PurePy\imageDataClass"
MODES = ("contour_or_hough", "hough", "contour", "contour_and_hough")


def truth_of(name):
    for p in [q.lower() for q in name.replace("/", "\\").split("\\")][:-1]:
        if p == "ok":
            return "front"
        if p == "ng":
            return "back"
    return "?"


def one_pass(src, mode):
    T.FEATURE_A_MODE = mode
    st = {
        "img": Counter(), "ok_img": Counter(), "a_img": Counter(),
        "holes": Counter(), "a_holes": Counter(),
        "onpitch_holes": Counter(), "onpitch_a": Counter(),
        "hough_hit": Counter(), "escape": Counter(), "bucket": Counter(),
    }
    for name, bgr in src.frames():
        if bgr is None:
            continue
        truth = truth_of(name)
        st["img"][truth] += 1
        try:
            res, _ = T.inspect(bgr, os.path.basename(name))
        except Exception as exc:  # noqa: BLE001
            print("[异常] %s: %s" % (name, exc))
            continue
        if res.is_ok:
            st["ok_img"][truth] += 1
        checked = [h for h in res.holes if h.index in set(res.checked)]
        if not checked:
            st["bucket"][truth + "/无受检孔"] += 1
            continue
        if any(h.feature_a for h in checked):
            st["a_img"][truth] += 1
        for h in checked:
            st["holes"][truth] += 1
            if h.feature_a:
                st["a_holes"][truth] += 1
            if h.flange_hough_r is not None:
                st["hough_hit"][truth] += 1
            if h.on_pitch:
                st["onpitch_holes"][truth] += 1
                if h.feature_a:
                    st["onpitch_a"][truth] += 1
            # 硬逃逸: 三个判据同时成立(闸/排序无关, 只看这个孔够不够翻判)
            if h.on_pitch and h.feature_a and h.valid_marks >= T.MIN_VALID_MARKS:
                st["escape"][truth] += 1
        op = [h for h in checked if h.on_pitch]
        if not op:
            st["bucket"][truth + "/闸保护"] += 1
            continue
        a_ok = [h for h in op if h.feature_a]
        if not a_ok:
            st["bucket"][truth + "/A否决"] += 1
            continue
        m = max(h.valid_marks for h in a_ok)
        st["bucket"][truth + "/只差1痕" if m >= 1 else truth + "/只差B"] += 1
    return st


def pct(a, b):
    return "%.1f%%" % (100.0 * a / b) if b else "n/a"


def main():
    T.HOLE_CHECK_COUNT = 0
    src = T.LocalFolderSource(IMG_ROOT, True)
    print("[样本] %d 张   HOLE_CHECK_COUNT=%d   MIN_VALID_MARKS=%d   节圆闸=%s"
          % (len(src), T.HOLE_CHECK_COUNT, T.MIN_VALID_MARKS, T.HOLE_PITCH_GATE))

    for mode in MODES:
        st = one_pass(src, mode)
        nf, nb = st["img"]["front"], st["img"]["back"]
        print("\n" + "=" * 76)
        print("FEATURE_A_MODE = %s" % mode)
        print("=" * 76)
        print("  正面 %d 张: 判OK=%d (%s)   A图级=%d (%s)   A孔级=%d/%d (%s)   Hough命中=%d (%s)"
              % (nf, st["ok_img"]["front"], pct(st["ok_img"]["front"], nf),
                 st["a_img"]["front"], pct(st["a_img"]["front"], nf),
                 st["a_holes"]["front"], st["holes"]["front"],
                 pct(st["a_holes"]["front"], st["holes"]["front"]),
                 st["hough_hit"]["front"], pct(st["hough_hit"]["front"], st["holes"]["front"])))
        print("  反面 %d 张: 判OK=%d   <-- 逃逸, 必须 0" % (nb, st["ok_img"]["back"]))
        print("        硬逃逸孔(on_pitch&A&marks>=%d)=%d   <-- 必须 0" % (T.MIN_VALID_MARKS, st["escape"]["back"]))
        print("        A图级=%d (%s)   A孔级=%d/%d (%s)   Hough命中=%d (%s)"
              % (st["a_img"]["back"], pct(st["a_img"]["back"], nb),
                 st["a_holes"]["back"], st["holes"]["back"],
                 pct(st["a_holes"]["back"], st["holes"]["back"]),
                 st["hough_hit"]["back"], pct(st["hough_hit"]["back"], st["holes"]["back"])))
        print("        真孔位(on_pitch=1)上的 A 通过率 = %d/%d (%s)   <-- 越低 = A 在反面越能拦"
              % (st["onpitch_a"]["back"], st["onpitch_holes"]["back"],
                 pct(st["onpitch_a"]["back"], st["onpitch_holes"]["back"])))
        print("        裕度归类(623 张):")
        for k in ("闸保护", "A否决", "只差B", "只差1痕", "无受检孔"):
            v = st["bucket"]["back/" + k]
            if v:
                print("             %-6s %4d (%.1f%%)" % (k, v, 100.0 * v / nb))
        only_b = st["bucket"]["back/只差B"] + st["bucket"]["back/只差1痕"]
        print("        >>> 只等压痕的图(只差B + 只差1痕) = %d (%.1f%%)  <-- 越小裕度越厚"
              % (only_b, 100.0 * only_b / nb))
        sys.stdout.flush()


if __name__ == "__main__":
    main()
