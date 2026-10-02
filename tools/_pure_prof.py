# -*- coding: utf-8 -*-
"""第一站(inspector_pure)整帧耗时：跑一批真帧，统计 inspect() 总耗时 + 各阶段占比。

与 `tools/_missing_prof.py` 同姿态、同产物风格（那边 → `_timing_missing.txt`，这边 → `_timing_pure.txt`），
用来给**产线机选型**提供第一站的实测数字（第二站的数字由 `_missing_prof.py` 出）。

**为什么是 inspect() 的耗时而不是"从取图到吹气"的墙钟**：产线节拍 = 相机传输(~100ms，百兆网固定开销)
+ 算法。选型要回答的是"换个 CPU 算法那部分要多久"，所以量的是算法本身；取图/解码两站口径一致
（`_missing_prof` 量的也是算法，不含 imdecode）。相机模式下的 acquisition/queue 在本地样本里没有意义。

只做两件"从外部覆盖模块全局"的事（**一行不改主算法**，本项目的既定扩展手法）：
  1. `ENABLE_UNO = False` —— 离线跑样本绝不能碰串口。开发机可能插着 CH340，不关掉的话
     每个 NG 帧都会真发一次脉冲，既污染计时又没意义。
  2. 包一层 `log_frame_timing` 把每帧耗时记下来（本文件靠它取数）。

⚠ 本文件与 `_missing_prof.py` 一样 `logging.disable(CRITICAL)`：**运行日志不会写文件**（也就不会
   因为 2000+ 帧把 .log 撑大）。计时全靠上面那个 hook 直接取，不依赖日志。

用法（项目根目录，用装了 cv2 的解释器）：
    python tools/_pure_prof.py
换样本目录/限量改下面的常量（`_` 前缀 = 一次性探针，不当稳定接口）。
"""
import os
import sys
import time
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "src", "flange_inspect"))

import logging
logging.getLogger().addHandler(logging.NullHandler())
logging.disable(logging.CRITICAL)

import inspector_pure as ip   # noqa: E402

# ---- 改成你要跑的样本目录（默认是 2026-10-01 那批 2051 张 1280x800 产线 RAW，与直连帧同尺寸）
SAMPLE_DIR = r"I:\data.zip\data\pure\RAW"
LIMIT = 0            # 0 = 全跑；先摸数时可以填 300
OUT_TXT = os.path.join(HERE, "_timing_pure.txt")

# 离线跑：关 UNO、关逐孔明细刷屏。存图默认就是关的（SAVE_*_IMAGE=False），不用动。
ip.ENABLE_UNO = False
ip.PRINT_DEBUG = False

REC = []   # [(帧名, 总ms, {阶段: ms}, 判定)]


def _hook(name, timing, res):
    """包 log_frame_timing：原样输出，外加记账。"""
    REC.append((os.path.basename(name), float(res.elapsed_ms), dict(res.timings_ms),
                "OK" if res.is_ok else "NG"))
    return _orig(name, timing, res)


_orig = ip.log_frame_timing
ip.log_frame_timing = _hook

argv = ["--mode", "local", "--dir", SAMPLE_DIR, "--holes", "0", "--timing", "--quiet"]
if LIMIT > 0:
    argv += ["--limit", str(LIMIT)]

t0 = time.time()
rc = ip.main(argv)
wall = time.time() - t0

if not REC:
    print("[FATAL] 一帧都没跑到，检查 SAMPLE_DIR（当前 %s）" % SAMPLE_DIR)
    sys.exit(2)

tot = np.array([r[1] for r in REC])
stage = defaultdict(list)
for _n, _ms, st, _v in REC:
    for k, v in st.items():
        if k in ("inspect_total", "inspect_ms"):
            continue
        stage[k].append(float(v))
ng = sum(1 for r in REC if r[3] == "NG")

out = []


def P(*a):
    out.append(" ".join(str(x) for x in a))


P("样本 %s" % SAMPLE_DIR)
P("帧数 %d   NG %d (%.1f%%)   退出码 %d   墙钟 %.1fs(含 imdecode)"
  % (len(REC), ng, 100.0 * ng / len(REC), rc, wall))
P("")
P("%-10s %9s %9s %9s %9s %9s" % ("整帧ms", "均值", "中位", "p90", "p99", "最大"))
P("%-10s %9.2f %9.2f %9.2f %9.2f %9.2f"
  % ("inspect", tot.mean(), np.median(tot), np.percentile(tot, 90),
     np.percentile(tot, 99), tot.max()))
P("")
P("阶段占比（各阶段中位 ms，占比按总和算）")
P("%-16s %9s %8s" % ("阶段", "中位ms", "占比%"))
gsum = float(np.sum([sum(stage[k]) for k in stage]))
for k in sorted(stage, key=lambda x: -sum(stage[x])):
    v = np.array(stage[k])
    P("%-16s %9.2f %7.1f%%" % (ip._STAGE_CN.get(k, k), np.median(v), 100.0 * v.sum() / gsum))

txt = "\n".join(out) + "\n"
with open(OUT_TXT, "w", encoding="utf-8") as f:
    f.write(txt)
print(txt)
print("saved", OUT_TXT)
