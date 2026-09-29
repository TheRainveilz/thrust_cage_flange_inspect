#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""L1 验证：缺粒站**单跑**（不连 IPC、不碰串口、不经主程序）。

这一层只回答一个问题：**算法本身在样本上判得对不对**。链路（IPC/UNO/主程序）留给 L2，
所以在 L1 里任何链路噪音都会掩盖算法问题 —— 故本层显式 `--no-uno`，且断言"一个 NG 都没触发出去"。

用法（在项目根目录跑）：
    python tools/verify_l1_missing.py
    python tools/verify_l1_missing.py --dir datasets/缺粒样本 --repeat 3

硬闸（不过就是 FAIL）：
  1. `inspector_pure.py` 无未提交改动 —— 它是黄金基线，一行都不许动；
  2. 检测器退出码 0，逐帧行数 == 处理帧数（没有静默丢帧）；
  3. **零逃逸**：真值 NG 判 OK 的帧数必须为 0（唯一不可接受的方向）；
  4. `no_actuator == NG`：证明这一层确实没去碰串口，且每个 NG 都记了账（没被悄悄吞掉）；
  5. 可重复：跑 repeat 次，逐帧判定序列完全一致（RANSAC 已固定种子）。

过杀（真值 OK 判 NG）只报 WARN：它是可接受的方向，但在样本上偏高时值得看一眼。
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_common import (DEFAULT_MISSING_DIR, PROJECT_ROOT, Report, count_escapes,  # noqa: E402
                           find_python, parse_frames, parse_summary, pure_is_clean, run)

DETECTOR = os.path.join(PROJECT_ROOT, "src", "flange_inspect", "inspector_missing.py")


def run_once(py: str, sample_dir: str):
    """跑一次缺粒站（local 模式、--no-uno），返回 (rc, 逐帧判定, [SUMMARY], 原始输出行)。

    L1 刻意**不加** `--quiet`：这样逐帧判定和 [SUMMARY] 都直接落在 stdout 上，一次读完，
    不用碰日志文件（L2 那层因为继承了 STATIONS 的 --quiet，只能去读日志文件）。
    """
    rc, text = run([py, DETECTOR, "--mode", "local", "--dir", sample_dir, "--no-uno"])
    lines = text.splitlines()
    return rc, parse_frames(lines), parse_summary(lines), lines


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="L1：缺粒站单跑验证(不连 IPC、不碰串口)")
    ap.add_argument("--dir", default=DEFAULT_MISSING_DIR, help="样本目录(默认 %(default)s)")
    ap.add_argument("--repeat", type=int, default=2, help="重复跑几次做确定性比对(默认 %(default)s)")
    ap.add_argument("--python", default=None, help="解释器(默认自动找项目 venv)")
    args = ap.parse_args(argv)

    py = args.python or find_python()
    rep = Report("L1  缺粒站单跑验证(不连 IPC、不碰串口)")
    rep.info("解释器", py)
    rep.info("样本目录", args.dir)

    # ---- ① 黄金基线不许动 ----
    rep.section("① 前置：inspector_pure.py 不得有改动(黄金基线铁律)")
    clean = pure_is_clean()
    if clean is None:
        rep.skip("inspector_pure.py 未提交改动检查", "不是 git 仓库或 git 不可用")
    else:
        rep.check(clean, "inspector_pure.py 无未提交改动",
                  "" if clean else "它被改过了 —— 这个站的算法结论就不可信了，先还原")

    if not os.path.isdir(args.dir):
        rep.check(False, "样本目录存在", args.dir)
        return rep.finish()

    # ---- ② 跑 ----
    rep.section("② 跑检测器")
    runs = []
    for i in range(max(1, args.repeat)):
        rc, frames, summary, lines = run_once(py, args.dir)
        runs.append((rc, frames, summary, lines))
        rep.info("第 %d 次" % (i + 1),
                 "rc=%d 逐帧行=%d" % (rc, len(frames)))
    rc, frames, summary, lines = runs[0]

    rep.check(rc == 0, "退出码 = 0", "rc=%d" % rc)
    if summary is None:
        rep.check(False, "拿到 [SUMMARY] 汇总行", "一行都没有 —— 检测器没跑到收尾")
        return rep.finish()

    n_proc = int(summary.get("processed", -1))
    n_ok = int(summary.get("OK", -1))
    n_ng = int(summary.get("NG", -1))
    n_noact = int(summary.get("no_actuator", -1))
    rep.info("[SUMMARY]",
             "processed=%d OK=%d NG=%d invalid=%d timeout_ng=%d"
             % (n_proc, n_ok, n_ng, int(summary.get("invalid", -1)),
                int(summary.get("timeout_ng", -1))))

    rep.check(len(frames) == n_proc, "逐帧行数 == processed(没有静默丢帧)",
              "%d vs %d" % (len(frames), n_proc))

    # ---- ③ 没碰串口 ----
    rep.section("③ 不得碰串口(本层显式 --no-uno)")
    # no_actuator 是"判了 NG 但 uno 为 None"的计数：它等于 NG 总数，说明这层每个 NG 都走了
    # "无执行器"分支 —— 既证明没开串口，也证明 NG 一个没漏记(否则就是静默吞判定)。
    rep.check(n_noact == n_ng, "no_actuator == NG(未碰串口，且 NG 全部记账)",
              "no_actuator=%d NG=%d" % (n_noact, n_ng))

    # ---- ④ 判得对不对 ----
    rep.section("④ 判定质量(真值只认目录名，不认文件名)")
    esc, over, labeled = count_escapes(frames)
    rep.check(esc == 0, "**零逃逸**(真值 NG 判 OK 的帧数 = 0)",
              "逃逸 %d 帧%s" % (esc, "" if esc == 0 else " ← 唯一不可接受的方向，先别上线"))
    rep.check(labeled > 0, "样本带真值(路径里有 OK/NG 目录层级)",
              "有真值的帧 %d / %d" % (labeled, len(frames)))
    if over:
        rep.warn("过杀 %d 帧(真值 OK 判 NG)" % over, "方向可接受，但样本上偏高值得看一眼")
    else:
        rep.info("过杀", "0 帧")
    if n_ng:
        rep.info("判 NG 合计", "%d 帧(含过杀 %d)" % (n_ng, over))

    # 逐目录明细：一眼看清是哪套样本被过杀/被逃逸
    rep.section("按目录")
    per_dir = {}
    for _seq, name, verdict in frames:
        key = os.path.dirname(os.path.relpath(name, args.dir))
        slot = per_dir.setdefault(key, [0, 0, 0])
        slot[0] += 1
        slot[1 if verdict == "OK" else 2] += 1
    for key in sorted(per_dir):
        n, ok, ng = per_dir[key]
        rep.info("%-24s" % key, "共 %3d 张 -> 判 OK %3d / 判 NG %3d" % (n, ok, ng))

    # 兜孔统计：判据的余量就藏在这两行里，改动阈值后先看这里有没有贴到判据线上
    rep.section("兜孔统计(判据余量)")
    for ln in lines:
        if "[SUMMARY-SLOT]" in ln:
            rep.info(ln.split("] ", 1)[-1])

    # ---- ⑤ 可重复 ----
    rep.section("⑤ 可重复性(算法必须确定性，否则现场无法复现问题)")
    base = list(frames)
    for i, (rc_i, frames_i, _s, _l) in enumerate(runs[1:], start=2):
        same = list(frames_i) == base
        rep.check(same, "第 %d 次与第 1 次逐帧判定完全一致" % i,
                  "" if same else "判定序列不同 —— 有非确定性来源(随机种子/线程竞态)")

    return rep.finish()


if __name__ == "__main__":
    sys.exit(main())
