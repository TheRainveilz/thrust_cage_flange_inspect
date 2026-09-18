#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""批量统计现有视觉算法的特征 A / 特征 B 通过率。

示例:
    python analyze_feature_ab.py --dir "D:\\zq\\imageData"
    python analyze_feature_ab.py --dir "D:\\zq\\imageData" --holes 0 --csv "D:\\zq\\ab.csv"

目录可以是:
    imageData/OK/*.png
    imageData/NG/*.png

也支持直接传入包含图片的目录。标签无法从父目录推断时记为 UNKNOWN，
不影响整体统计。
"""

from __future__ import annotations

import argparse
import csv
import os
from collections import Counter, defaultdict
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import cv2

import thrust_cage_flange_inspect as T


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


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
    error: str = ""


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="统计视觉检测特征 A/B 通过率")
    ap.add_argument("--dir", required=True, help="图片目录，支持 OK/NG 子目录")
    ap.add_argument("--csv", default=None, help="输出逐图 CSV 路径")
    ap.add_argument("--holes", type=int, default=None,
                    help="参与统计的孔数；0=全部孔；默认使用主程序当前配置")
    ap.add_argument("--limit", type=int, default=0, help="最多处理多少张；0=全部")
    return ap.parse_args()


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


def inspect_one(path: str) -> ImageStat:
    label = infer_label(path)
    bgr = T.imread_unicode(path, cv2.IMREAD_COLOR)
    if bgr is None:
        return ImageStat(path, label, "READ_ERROR", "", "", error="read failed")

    try:
        result, _ = T.inspect(bgr, os.path.basename(path))
    except Exception as exc:  # noqa: BLE001
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
    )


def pct(num: int, den: int) -> str:
    return "%.1f%%" % (100.0 * num / den) if den else "n/a"


def print_group(title: str, rows: List[ImageStat]) -> None:
    valid = [r for r in rows if r.error == ""]
    images_with_holes = [r for r in valid if r.checked > 0]
    a_images = sum(r.a_pass > 0 for r in images_with_holes)
    b_images = sum(r.b_pass > 0 for r in images_with_holes)
    pair_images = sum(r.pair_pass > 0 for r in images_with_holes)
    holes = sum(r.checked for r in valid)
    a_holes = sum(r.a_pass for r in valid)
    b_holes = sum(r.b_pass for r in valid)
    pair_holes = sum(r.pair_pass for r in valid)

    print(f"\n[{title}] 图片 {len(rows)} 张，成功分析 {len(valid)} 张")
    print("  图片级: A通过=%d/%d (%s) | B通过=%d/%d (%s) | A+B同时通过=%d/%d (%s)"
          % (a_images, len(images_with_holes), pct(a_images, len(images_with_holes)),
             b_images, len(images_with_holes), pct(b_images, len(images_with_holes)),
             pair_images, len(images_with_holes), pct(pair_images, len(images_with_holes))))
    print("  孔级  : A通过=%d/%d (%s) | B通过=%d/%d (%s) | A+B同时通过=%d/%d (%s)"
          % (a_holes, holes, pct(a_holes, holes),
             b_holes, holes, pct(b_holes, holes),
             pair_holes, holes, pct(pair_holes, holes)))

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
    if args.holes is not None:
        T.HOLE_CHECK_COUNT = args.holes

    paths = list(iter_images(args.dir))
    if args.limit > 0:
        paths = paths[:args.limit]
    if not paths:
        print("[ERROR] 未找到图片:", args.dir)
        return 2

    print("[INFO] 图片数:", len(paths))
    print("[INFO] HOLE_CHECK_COUNT:", T.HOLE_CHECK_COUNT)
    rows = []
    for index, path in enumerate(paths, 1):
        row = inspect_one(path)
        rows.append(row)
        if row.error:
            print("[WARN] %d/%d %s: %s" % (index, len(paths), path, row.error))
        else:
            print("[%d/%d] %-7s %-3s checked=%d A=%d/%d B=%d/%d pair=%d/%d"
                  % (index, len(paths), row.label, row.verdict, row.checked,
                     row.a_pass, row.checked, row.b_pass, row.checked,
                     row.pair_pass, row.checked))

    groups: Dict[str, List[ImageStat]] = defaultdict(list)
    for row in rows:
        groups[row.label].append(row)
    for label in ("OK", "NG", "UNKNOWN"):
        if groups.get(label):
            print_group(label, groups[label])
    print_group("全部", rows)

    if args.csv:
        write_csv(args.csv, rows)
        print("\n[INFO] 明细 CSV:", os.path.abspath(args.csv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
