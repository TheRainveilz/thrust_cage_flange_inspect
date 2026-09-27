# -*- coding: utf-8 -*-
"""验证"整件压痕广度"能否当第二道**件级**闸(用户 2026-09-22 提议)。
现在判 OK 只看"有没有一个孔 on_pitch&A&marks>=2"。提议再 AND 一条件级要求:
整件必须有足够多的孔/拐角见到压痕, 让某孔侥幸凑 2 痕也翻不了盘。
本脚本**只量分布**, 先看 OK/NG 分不分得开, 再决定要不要加。

对每张图, 只统计 on_pitch 的受检孔(其余本就被节圆闸否决):
  sum_marks   整件总压痕数 = Σ valid_marks
  n_ge1       有 >=1 痕的孔数        n_ge2  有 >=2 痕的孔数(=当前 B 通过的孔数)
另给一个件级闸扫描: 在"当前已判 OK"的基础上再要求 n_ge1>=K, 看 OK 召回掉多少、
NG 里"只差压痕"的薄图有多少张连这条广度都不满足(= 侥幸凑 2 痕也翻不了 = 加厚的裕度)。
只读。用法: .venv/Scripts/python.exe tools/_diag_breadth.py
"""
import os
import sys
import numpy as np
from collections import defaultdict

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
from src.flange_inspect import inspector_pure as T

IMG_ROOT = r"I:\zq.zip\CPlusPlus_Vs_PurePy\imageDataClass"


def truth_of(name):
    for p in [q.lower() for q in name.replace("/", "\\").split("\\")][:-1]:
        if p == "ok":
            return "front"
        if p == "ng":
            return "back"
    return "?"


def qt(v):
    v = np.sort(np.asarray(v, float))
    if len(v) == 0:
        return "n=0"
    def q(p):
        return v[min(len(v) - 1, int(p * len(v)))]
    return "n=%d  p10=%.1f p25=%.1f p50=%.1f p75=%.1f p90=%.1f max=%.0f" % (
        len(v), q(.10), q(.25), q(.50), q(.75), q(.90), v[-1])


def main():
    T.HOLE_CHECK_COUNT = 0
    src = T.LocalFolderSource(IMG_ROOT, True)
    print("[配置] A=%s COLLAR=%s(%.2f) tm=%d  样本=%d  (只统计 on_pitch 孔)"
          % (T.FEATURE_A_MODE, T.COLLAR_GATE, T.COLLAR_DARKFRAC_MIN, T.MIN_VALID_MARKS, len(src)))
    # per image: (sum_marks, n_ge1, n_ge2, currently_ok, is_only_missing_B)
    rec = {"front": [], "back": []}
    for name, bgr in src.frames():
        if bgr is None:
            continue
        truth = truth_of(name)
        if truth == "?":
            continue
        res, _ = T.inspect(bgr, os.path.basename(name))
        checked = set(res.checked)
        op = [h for h in res.holes if h.index in checked and h.on_pitch]
        sum_marks = sum(h.valid_marks for h in op)
        n_ge1 = sum(h.valid_marks >= 1 for h in op)
        n_ge2 = sum(h.valid_marks >= 2 for h in op)
        cur_ok = any(h.feature_a and h.valid_marks >= T.MIN_VALID_MARKS for h in op)
        # "只差压痕": 有 on_pitch&A 的孔, 但没有一个到 tm(即当前判 NG 却卡在 B)
        a_holes = [h for h in op if h.feature_a]
        only_missing_b = (not cur_ok) and len(a_holes) > 0
        rec[truth].append((sum_marks, n_ge1, n_ge2, cur_ok, only_missing_b))

    for key, lab in (("front", "正面 OK"), ("back", "反面 NG")):
        R = rec[key]
        print("\n== %s (%d 张) ==" % (lab, len(R)))
        print("  整件总压痕数 sum_marks : ", qt([r[0] for r in R]))
        print("  有>=1痕的孔数 n_ge1     : ", qt([r[1] for r in R]))
        print("  有>=2痕的孔数 n_ge2     : ", qt([r[2] for r in R]))

    # 件级广度闸扫描: 在"当前判定"上再 AND (n_ge1 >= K)
    front = rec["front"]
    back = rec["back"]
    nf, nb = len(front), len(back)
    cur_ok_f = sum(r[3] for r in front)
    onlyB_back = [r for r in back if r[4]]   # 反面"只差压痕"的薄图
    print("\n[基线] 正面判OK=%d/%d  反面'只差压痕'薄图=%d 张" % (cur_ok_f, nf, len(onlyB_back)))
    print("\n件级广度闸: 判 OK 额外要求 '有>=1痕的孔数 n_ge1 >= K'")
    print("%-4s | %-18s | %-28s" % ("K", "正面召回(代价)", "反面薄图里被广度挡下的(加厚裕度)"))
    print("-" * 68)
    for K in (1, 2, 3, 4, 5, 6):
        f_ok = sum(1 for r in front if r[3] and r[1] >= K)
        # 反面薄图: 即便某孔侥幸凑到 tm, 若 n_ge1<K 仍会被广度闸挡下
        blocked = sum(1 for r in onlyB_back if r[1] < K)
        print("%-4d | %4d/%d (%.1f%%)     | %3d/%d 张 (%.0f%%)"
              % (K, f_ok, nf, 100.0 * f_ok / nf,
                 blocked, len(onlyB_back), 100.0 * blocked / max(len(onlyB_back), 1)))
    print("\n[读法] K 从 1 往上: 正面召回是代价(掉多少张真OK), '被广度挡下'是收益"
          "\n       (那些薄反面图即便 B 侥幸凑够、也会因整件压痕太稀被这道闸再挡一层)。"
          "\n       若某个 K 能挡下大部分薄图而正面几乎不掉, 这道闸就值得加; 若正面同步大掉, 说明"
          "\n       OK 件本身压痕就稀(85%漏检), 广度分不开两者 —— 那就别加, 老实说是成像问题。")


if __name__ == "__main__":
    main()
