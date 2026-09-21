#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""批量统计视觉算法的特征 A / 特征 B 通过率（纯 Py 与 C++ 加速版合一）。

用 --impl 选择实现，两版算法共用同一套统计逻辑：
    python tools/analyze_feature_ab.py --impl pure --dir "D:\\zq\\imageData" --holes 0
    python tools/analyze_feature_ab.py --impl cpp  --dir "D:\\zq\\imageData" --holes 0

CSV 输出：
    不传 --csv 时，自动写到 --outdir(默认 artifacts/reports/)，
    文件名为 ab_<impl>_holes<N>_<YYYYmmdd>.csv；
    传 --csv 则完全用它作为落盘路径(忽略 --outdir 自动命名)。

目录可以是 imageData/OK/*.png、imageData/NG/*.png，或直接含图片的目录。
标签无法从父目录推断时记为 UNKNOWN，不影响整体统计。
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Iterable, List

import cv2

# 允许从任意工作目录运行：把仓库根(tools/ 的上一级)加入 sys.path，
# 以便 import src.flange_inspect。
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
# 默认输出锚定到项目根(而非运行时 cwd)：无论从哪个目录调用，
# 不传 --csv 时都会落到 <项目根>/artifacts/reports/。
DEFAULT_OUTDIR = os.path.join(_REPO_ROOT, "artifacts", "reports")


def load_impl(impl: str):
    """按 --impl 返回对应算法模块(纯 Py 或 C++ 加速版)。"""
    if impl == "cpp":
        from src.flange_inspect import inspector_cpp as T
    elif impl == "pure":
        from src.flange_inspect import inspector_pure as T
    else:
        raise ValueError("--impl 只能是 pure 或 cpp: %r" % impl)
    return T


@dataclass
class ImageStat:
    path: str
    label: str
    verdict: str
    reason: str
    locate_method: str
    holes: int = 0
    checked: int = 0
    a_pass: int = 0
    a_fail: int = 0
    b_pass: int = 0
    b_fail: int = 0
    pair_pass: int = 0
    max_marks: int = 0
    ring_counts: str = ""
    marks: str = ""
    elapsed_ms: float = 0.0
    error: str = ""


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="统计视觉检测特征 A/B 通过率")
    ap.add_argument("--impl", choices=("pure", "cpp"), default="pure",
                    help="算法实现: pure=纯 Python, cpp=C++ 加速版(默认 pure)")
    ap.add_argument("--dir", required=True, help="图片目录，支持 OK/NG 子目录")
    ap.add_argument("--csv", default=None,
                    help="输出逐图 CSV 完整路径；不传则按 --outdir 自动命名")
    ap.add_argument("--outdir", default=DEFAULT_OUTDIR,
                    help="不传 --csv 时的 CSV 输出目录(默认 artifacts/reports/)")
    ap.add_argument("--holes", type=int, default=None,
                    help="参与统计的孔数；0=全部孔；默认使用主程序当前配置")
    ap.add_argument("--limit", type=int, default=0, help="最多处理多少张；0=全部")
    ap.add_argument("--no-pitch-gate", action="store_true",
                    help="关掉节圆前置闸(HOLE_PITCH_GATE=False)，用于复现闸前基线做对照；"
                         "仅 pure 实现有该开关")
    ap.add_argument("--feature-a-mode", default=None,
                    choices=("contour", "hough", "contour_or_hough", "contour_and_hough"),
                    help="覆盖 FEATURE_A_MODE，决定特征A用哪个分支；默认用模块当前值")
    return ap.parse_args()


def resolve_csv_path(args: argparse.Namespace, holes: int) -> str:
    """确定 CSV 落盘路径：优先 --csv，否则 outdir/ab_<impl>_holes<N>_<date>.csv。"""
    if args.csv:
        return args.csv
    stamp = datetime.now().strftime("%Y%m%d")
    name = "ab_%s_holes%d_%s.csv" % (args.impl, holes, stamp)
    return os.path.join(args.outdir, name)


def iter_images(root: str) -> Iterable[str]:
    if os.path.isfile(root):
        if os.path.splitext(root)[1].lower() in IMAGE_EXTS:
            yield root
        return
    for base, _dirs, files in os.walk(root):
        for name in sorted(files):
            if os.path.splitext(name)[1].lower() in IMAGE_EXTS:
                yield os.path.join(base, name)


def infer_label(path: str) -> str:
    parts = [p.lower() for p in os.path.normpath(path).split(os.sep)]
    if "ok" in parts:
        return "OK"
    if "ng" in parts:
        return "NG"
    name = os.path.basename(path).lower()
    if "_ok" in name or "ok_" in name:
        return "OK"
    if "_ng" in name or "ng_" in name:
        return "NG"
    return "UNKNOWN"


def inspect_one(T, path: str) -> ImageStat:
    label = infer_label(path)
    bgr = T.imread_unicode(path, cv2.IMREAD_COLOR)
    if bgr is None:
        return ImageStat(path, label, "READ_ERROR", "", "", error="read failed")

    try:
        result, _ = T.inspect(bgr, os.path.basename(path))
    except Exception as exc:  # noqa: BLE001
        import traceback
        print(f"\n[TRACEBACK] {path}")
        traceback.print_exc()
        return ImageStat(path, label, "EXCEPTION", "", "", error=str(exc))

    checked = [h for h in result.holes if h.index in set(result.checked)]
    # 兼容异常情况下 checked 为空但 holes 有内容。
    if not checked and result.checked:
        checked = result.holes[:len(result.checked)]

    a_pass = sum(bool(h.feature_a) for h in checked)
    b_pass = sum(bool(h.feature_b) for h in checked)
    pair_pass = sum(bool(h.passed) for h in checked)
    ring_counts = "|".join(str(h.ring_count) for h in checked)
    marks = "|".join(str(h.valid_marks) for h in checked)

    return ImageStat(
        path=path,
        label=label,
        verdict="OK" if result.is_ok else result.verdict,
        reason=result.reason,
        locate_method=result.locate_method,
        holes=len(result.holes),
        checked=len(checked),
        a_pass=a_pass,
        a_fail=len(checked) - a_pass,
        b_pass=b_pass,
        b_fail=len(checked) - b_pass,
        pair_pass=pair_pass,
        max_marks=max((h.valid_marks for h in checked), default=0),
        ring_counts=ring_counts,
        marks=marks,
        elapsed_ms=float(result.elapsed_ms),
    )


def pct(num: int, den: int) -> str:
    return "%.1f%%" % (100.0 * num / den) if den else "n/a"


def print_group(title: str, rows: List[ImageStat]) -> None:
    valid = [r for r in rows if r.error == ""]
    # 图片级分母 = 成功分析的全部图片。定位失败/无受检孔的图必须计入分母:
    # 它们不可能贡献任何通过孔, 剔掉会把 NG 侧的"100%"报成假象(623 张里 15 张定位失败)。
    denom = len(valid)
    a_images = sum(r.a_pass > 0 for r in valid)
    b_images = sum(r.b_pass > 0 for r in valid)
    pair_images = sum(r.pair_pass > 0 for r in valid)
    holes = sum(r.checked for r in valid)
    a_holes = sum(r.a_pass for r in valid)
    b_holes = sum(r.b_pass for r in valid)
    pair_holes = sum(r.pair_pass for r in valid)
    times = [r.elapsed_ms for r in valid]

    print(f"\n[{title}] 图片 {len(rows)} 张，成功分析 {len(valid)} 张")
    print("  图片级: A通过=%d/%d (%s) | B通过=%d/%d (%s) | A+B同时通过=%d/%d (%s)"
          % (a_images, denom, pct(a_images, denom),
             b_images, denom, pct(b_images, denom),
             pair_images, denom, pct(pair_images, denom)))
    print("  孔级  : A通过=%d/%d (%s) | B通过=%d/%d (%s) | A+B同时通过=%d/%d (%s)"
          % (a_holes, holes, pct(a_holes, holes),
             b_holes, holes, pct(b_holes, holes),
             pair_holes, holes, pct(pair_holes, holes)))

    if times:
        print("  elapsed_ms: avg=%.1f min=%.1f max=%.1f"
              % (sum(times) / len(times), min(times), max(times)))

    combos = Counter()
    for row in valid:
        if row.checked == 0:
            combos["无受检孔/定位失败"] += 1
        elif row.a_pass > 0 and row.b_pass > 0:
            combos["A+B均有通过孔"] += 1
        elif row.a_pass > 0:
            combos["仅A有通过孔"] += 1
        elif row.b_pass > 0:
            combos["仅B有通过孔"] += 1
        else:
            combos["A/B均无通过孔"] += 1
    print("  失败组合:", " | ".join("%s=%d" % item for item in combos.items()))


def write_csv(path: str, rows: List[ImageStat]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    fields = list(ImageStat.__dataclass_fields__.keys())
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: getattr(row, field) for field in fields})


def main() -> int:
    args = parse_args()
    T = load_impl(args.impl)
    if args.holes is not None:
        T.HOLE_CHECK_COUNT = args.holes

    paths = list(iter_images(args.dir))
    if args.limit > 0:
        paths = paths[:args.limit]
    if not paths:
        print("[ERROR] 未找到图片:", args.dir)
        return 2

    print("[INFO] 实现:", args.impl)
    print("[INFO] 图片数:", len(paths))
    print("[INFO] HOLE_CHECK_COUNT:", T.HOLE_CHECK_COUNT)
    # 节圆前置闸(见 inspector_pure.HOLE_PITCH_GATE): 默认开。关掉即复现闸前行为,
    # 用来产出对照基线(如 ab_pure_holes0_<date>_pregate.csv)。
    if hasattr(T, "HOLE_PITCH_GATE"):
        if args.no_pitch_gate:
            T.HOLE_PITCH_GATE = False
        print("[INFO] 节圆前置闸:", T.HOLE_PITCH_GATE)
    elif args.no_pitch_gate:
        print("[WARN] %s 无 HOLE_PITCH_GATE, 忽略 --no-pitch-gate" % args.impl)
    # 特征A 分支选择(见 inspector_pure.FEATURE_A_MODE)。实测(2026-09-21, 975 张):
    # 判别力几乎全在 hough 分支(正面命中 96.4% vs 反面 47.0%), contour 分支正反面几乎一样
    # (ring_count>=2: 56.7% vs 50.9%) —— "or" 把判别力弱的那个淹掉了。用本开关扫单分支。
    if args.feature_a_mode is not None:
        if hasattr(T, "FEATURE_A_MODE"):
            T.FEATURE_A_MODE = args.feature_a_mode
        else:
            print("[WARN] %s 无 FEATURE_A_MODE, 忽略 --feature-a-mode" % args.impl)
    if hasattr(T, "FEATURE_A_MODE"):
        print("[INFO] FEATURE_A_MODE:", T.FEATURE_A_MODE)
    rows = []
    for index, path in enumerate(paths, 1):
        row = inspect_one(T, path)
        rows.append(row)
        if row.error:
            print("[WARN] %d/%d %s: %s" % (index, len(paths), path, row.error))
        else:
            print("[%d/%d] %-7s %-3s checked=%d A=%d/%d B=%d/%d pair=%d/%d %.1fms"
                  % (index, len(paths), row.label, row.verdict, row.checked,
                      row.a_pass, row.checked, row.b_pass, row.checked,
                      row.pair_pass, row.checked, row.elapsed_ms))

    groups: Dict[str, List[ImageStat]] = defaultdict(list)
    for row in rows:
        groups[row.label].append(row)
    for label in ("OK", "NG", "UNKNOWN"):
        if groups.get(label):
            print_group(label, groups[label])
    print_group("全部", rows)

    csv_path = resolve_csv_path(args, T.HOLE_CHECK_COUNT)
    write_csv(csv_path, rows)
    print("\n[INFO] 明细 CSV:", os.path.abspath(csv_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
