#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""L2 验证：主程序独占串口 + IPC 转发这条链，**不改变任何检测器的判定**。

要回答的问题只有一个：**把「谁驱动执行器」从检测器搬到主程序之后，判定结果还是原来那个吗？**
所以本层的核心不是"跑通了"，而是**等价性** —— 同一个样本目录：
  A) 检测器独立跑（今天的生产形态）
  B) 经主程序跑（新架构：--child + IPC + 主程序代为触发执行器）
两次的**逐帧判定序列必须逐条相同**，且 B 里每一次 NG 都恰好变成一次对应引脚的脉冲。

**为什么这是关键闸门**：算法（缺粒二期判据、正反面黄金基线）都已经各自验证过了。本层要能把锅
**精确定位到"新架构"还是"新算法"**：判定一致 + 脉冲击数对齐 ⇒ 新架构是透明的；不一致 ⇒ 是新
架构的锅，跟算法无关，别去改阈值。

五段：
  ① 前置：inspector_pure.py 不得有改动（黄金基线铁律）
  ② 无 UNO 时必须**拒绝启动**（fail-safe：绝不跑一个触发不了执行器的检测器）
  ③ front 站：逐目录 独立跑 vs 经主程序跑（+ 黄金基线 318/352、623/623 零逃逸）
  ④ missing 站：同上（+ 缺粒二期判据 40 张零逃逸）
  ⑤ 两站同跑：互不串路（front 只触发 D8=吹气、missing 只触发 D9=开闸）、互不干扰（判定不受另一站影响）

**全程 `--dry-uno`**：不碰真串口，只把"将要触发哪个引脚"打出来 —— 产线机上也能安全跑；而且
"触发了哪一路、触发了几次"正是本层唯一能对账的证据。真硬件那段（两相机 + 真 UNO + LED）是 L3。

**逐帧判定只能从 .log 读**：主程序给子进程的 argv 里带 `--quiet`（见 main_pipeline.STATIONS），
它把控制台抬到 WARNING，逐帧 `[INSPECT-DONE]` 只进 .log。所以"独立跑"那一侧也必须加 `--quiet`
并从 .log 读 —— 两边同一把尺子，等价性才有意义。

用法（在项目根目录跑）：
    python tools/verify_l2_pipeline.py
    python tools/verify_l2_pipeline.py --skip-both      # 跳过第⑤段(两站同跑，最慢)
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_common import (DEFAULT_FRONT_DIR, DEFAULT_MISSING_DIR, FRONT_LOG_ROOT,  # noqa: E402
                           FRONT_PIPELINE_LOG_ROOT,
                           GOLDEN_FRONT, MISSING_LOG_ROOT, PROJECT_ROOT, Report, count_escapes,
                           count_in, find_python, parse_frames, parse_summary, pure_is_clean,
                           read_new_lines, run, snapshot_sizes)

SRC = os.path.join(PROJECT_ROOT, "src", "flange_inspect")
MAIN = os.path.join(SRC, "main_pipeline.py")
PURE = os.path.join(SRC, "inspector_pure.py")
MISSING = os.path.join(SRC, "inspector_missing.py")

# 缺粒站已标定的期望值（二期判据 / datasets/缺粒样本）
GOLDEN_MISSING = {"processed": 40, "n_ok": 3, "n_ng": 37}


def log_root_for(station: str) -> str:
    """该站**独立跑**的运行日志根目录（两站各写各的，绝不互踩）。"""
    return FRONT_LOG_ROOT if station == "front" else MISSING_LOG_ROOT


def pipeline_log_root_for(station: str) -> str:
    """该站**经主程序跑**的日志根。missing 不论怎么跑都写 data/missing/logs；
    front 经主程序时被 main_pipeline 收拢到 data/pure/logs（独立跑仍是 data/logs）——
    等价性比对因此各读各的根，比的是判定内容而非落盘位置。"""
    return FRONT_PIPELINE_LOG_ROOT if station == "front" else MISSING_LOG_ROOT


# ============================  两种跑法  ============================
def detector_run(py: str, script: str, sample_dir: str, station: str,
                 extra: Tuple[str, ...] = ()) -> Tuple[int, List, Optional[Dict], List[str]]:
    """A) 检测器**独立**跑（不经主程序）。返回 (rc, 逐帧, [SUMMARY], 日志新增行)。

    与经主程序跑的那一侧对称：一样加 `--quiet`，一样从 .log 读逐帧 —— 差别只剩"谁触发执行器"。
    """
    argv = [py, script, "--mode", "local", "--dir", sample_dir, "--quiet"] + list(extra)
    root = log_root_for(station)
    before = snapshot_sizes(root)
    rc, _text = run(argv)
    new = read_new_lines(before, root)
    return rc, parse_frames(new), parse_summary(new), new


def pipeline_run(py: str, stations: List[str], sample_dir: str,
                 dry_uno: bool = True) -> Tuple[int, List[str], Dict[str, Tuple]]:
    """B) 经**主程序**跑（可多站同跑）。返回 (rc, 主程序输出行, {站: (逐帧, [SUMMARY], 日志行)})。"""
    before = {s: snapshot_sizes(pipeline_log_root_for(s)) for s in stations}
    argv = [py, MAIN, "--exit-when-done", "--status-interval", "0"]
    if dry_uno:
        argv.append("--dry-uno")
    for s in stations:
        argv += ["--only", s]
    argv += ["--mode", "local", "--dir", sample_dir]
    rc, text = run(argv)
    lines = text.splitlines()
    out: Dict[str, Tuple] = {}
    for s in stations:
        root = pipeline_log_root_for(s)
        new = read_new_lines(before[s], root)
        out[s] = (parse_frames(new), parse_summary(new), new)
    return rc, lines, out


# ============================  比对与断言  ============================
def compare_verdicts(rep: Report, tag: str, a_frames: List, b_frames: List) -> bool:
    """逐帧判定序列必须**逐条相同**（顺序也一致 —— 同一目录的遍历顺序是确定的）。"""
    a = [(n, v) for _s, n, v in a_frames]
    b = [(n, v) for _s, n, v in b_frames]
    if rep.check(len(a) == len(b) and a == b, "%s 逐帧判定与独立跑完全一致" % tag,
                 "%d vs %d 帧" % (len(a), len(b))):
        return True
    # 不一致时把前几处差异打出来，别只说一句"不一致"
    diff = 0
    for i, (x, y) in enumerate(zip(a, b)):
        if x != y:
            diff += 1
            if diff <= 5:
                rep.info("差异 #%d" % (i + 1), "独立跑 %s -> 经主程序 %s" % (x, y))
    if len(a) != len(b):
        rep.info("帧数不同", "多出来的部分无法比对（截断处之后全部跳过）")
    rep.info("差异合计", "%d 处" % diff)
    return False


def count_cmd(lines: List[str], cmd: str) -> int:
    """数 `[UNO][DRY-RUN] would send: <cmd>` 的行数。

    **必须整词匹配**：`NG` 是 `NG2` 的前缀，用子串数会把 D8/D9 两路混成一堆
    (两站同跑时表现为"front 触发了 65 次"，其实就是 28 次 NG + 37 次 NG2)。
    """
    needle = "would send: " + cmd
    return sum(1 for ln in lines if ln.rstrip().endswith(needle))


def check_actuation(rep: Report, tag: str, lines: List[str], station: str,
                    n_ng: int, pin: int, other_pin: int) -> None:
    """B 侧的执行核对：每一次 NG 恰好一次脉冲，且**只**打自己那一路。

    两条断言都按**站名**取行(`[PIPE] 站 <station> NG -> 触发 D<pin>`)。
    不能用裸的 `触发 D9` —— 两站同跑时那会把另一站名下的脉冲也算进来，误报串路。
    """
    fired = count_in(lines, "[PIPE] 站 %s NG -> 触发 D%d" % (station, pin))
    rep.check(fired == n_ng, "%s 代触发 D%d 次数 == 判 NG 数" % (tag, pin),
              "%d vs %d" % (fired, n_ng))
    wrong = count_in(lines, "[PIPE] 站 %s NG -> 触发 D%d" % (station, other_pin))
    rep.check(wrong == 0, "%s 未误触发 D%d（不串路）" % (tag, other_pin), "%d 次" % wrong)
    # 固件命令对账：D8 发 "NG"、D9 发 "NG2"。这一层直接查 _DryRelay 打出来的命令，
    # 与上面的 [PIPE] 行相互印证 —— 引脚->命令这条映射错了，这里会立刻暴露。
    cmd = "NG2" if pin == 9 else "NG"
    sent = count_cmd(lines, cmd)
    rep.check(sent == n_ng, "%s 串口命令 %s 次数 == 判 NG 数" % (tag, cmd),
              "%d vs %d" % (sent, n_ng))


def check_pipeline_health(rep: Report, tag: str, rc: int, lines: List[str], station: str) -> None:
    """B 侧的健康度：建链成功、没重启、没致命故障。"""
    rep.check(rc == 0, "%s 主程序退出码 = 0" % tag, "rc=%d" % rc)
    rep.check(count_in(lines, "[IPC] 站 %s 已接入" % station) == 1,
              "%s 子进程经 IPC 接入主程序（没有静默降级）" % tag)
    restarts = count_in(lines, "[PIPE] 重启站 %s" % station)
    rep.check(restarts == 0, "%s 子进程未触发重启" % tag, "%d 次" % restarts)
    fatal = count_in(lines, "[PIPE][FATAL]")
    rep.check(fatal == 0, "%s 无致命故障" % tag, "%d 条" % fatal)


def check_golden(rep: Report, tag: str, sub_dir: str, summary: Optional[Dict]) -> None:
    """默认数据集上的黄金基线数字（换数据集时自动跳过）。"""
    key = os.path.basename(os.path.normpath(sub_dir)).upper()
    same_dir = (os.path.normcase(os.path.abspath(os.path.dirname(sub_dir)))
                == os.path.normcase(os.path.abspath(DEFAULT_FRONT_DIR)))
    if key not in GOLDEN_FRONT or not same_dir:
        rep.skip("%s 黄金基线对照" % tag, "非默认数据集(%s)，无对照数字" % sub_dir)
        return
    want = GOLDEN_FRONT[key]
    got = None if summary is None else {
        "processed": int(summary.get("processed", -1)),
        "n_ok": int(summary.get("OK", -1)),
        "n_ng": int(summary.get("NG", -1)),
    }
    rep.check(got == want, "%s 复现黄金基线 %d/%d（判 OK %d / 判 NG %d）"
              % (tag, want["n_ok"], want["processed"], want["n_ok"], want["n_ng"]),
              "实测 %s" % (got,))


def escapes_in(frames: List) -> int:
    """真值 NG 判 OK 的帧数（唯一不可接受的方向）。真值只认目录名（见 verify_common）。"""
    return count_escapes(frames)[0]


# ============================  主流程  ============================
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="L2：主程序代管串口 + IPC 转发的等价性与安全验证")
    ap.add_argument("--front-dir", default=DEFAULT_FRONT_DIR, help="正反面站样本根目录(含 OK/NG 子目录)")
    ap.add_argument("--missing-dir", default=DEFAULT_MISSING_DIR, help="缺粒站样本目录")
    ap.add_argument("--python", default=None, help="解释器(默认自动找项目 venv)")
    ap.add_argument("--skip-front", action="store_true", help="跳过第③段")
    ap.add_argument("--skip-missing", action="store_true", help="跳过第④段")
    ap.add_argument("--skip-both", action="store_true", help="跳过第⑤段(两站同跑)")
    args = ap.parse_args(argv)

    py = args.python or find_python()
    rep = Report("L2  主程序代管串口 + IPC 转发：等价性与安全验证")
    rep.info("解释器", py)
    rep.info("正反面样本", args.front_dir)
    rep.info("缺粒样本", args.missing_dir)
    rep.info("全程 --dry-uno", "不碰真串口；只对账'触发了哪一路、触发了几次'")

    # ---- ① 黄金基线不许动 ----
    rep.section("① 前置：inspector_pure.py 不得有改动（黄金基线铁律）")
    clean = pure_is_clean()
    if clean is None:
        rep.skip("inspector_pure.py 未提交改动检查", "不是 git 仓库或 git 不可用")
    else:
        rep.check(clean, "inspector_pure.py 无未提交改动",
                  "" if clean else "它被改过了 —— 本层的'判定一致'就不再是黄金基线的结论，先还原")

    # ---- ② 无 UNO 必须拒绝启动 ----
    rep.section("② fail-safe：无 UNO 时主程序必须拒绝启动任何检测站")
    try:
        # 必须先让 src/flange_inspect 上 sys.path：本站脚本在 tools/ 下，uno_relay 在 src/ 里。
        # 探不到就会退化成"假定无硬件"，然后在真有 UNO 的机器上去开真串口测第②段 —— 那是错的。
        if SRC not in sys.path:
            sys.path.insert(0, SRC)
        from uno_relay import find_ch340_ports
        ports = find_ch340_ports()
    except Exception as exc:  # noqa: BLE001
        ports = []
        rep.warn("无法探测 CH340（按'无硬件'继续测）", str(exc))
    if ports:
        rep.skip("拒绝启动检查", "本机探测到 CH340 %s，有硬件时这条测不了" % ports)
    else:
        # 不传 --dry-uno：应当连不上 UNO 并在起任何子进程之前退出(rc=2)
        rc, text = run([py, MAIN, "--only", "front", "--status-interval", "0",
                        "--exit-when-done", "--mode", "local",
                        "--dir", os.path.join(args.front_dir, "NG")])
        rep.check(rc == 2, "无 UNO 时退出码 = 2", "rc=%d" % rc)
        rep.check("拒绝启动任何检测站" in text, "日志明说拒绝启动（不是悄悄跑起来）")
        rep.check("已拉起" not in text, "**一个子进程都没拉起**（绝无'触发不了执行器还在判'）")

    # ---- ③ front 站 ----
    if not args.skip_front:
        rep.section("③ front 站（正反面 / D8）：独立跑 vs 经主程序跑")
        for sub in ("OK", "NG"):
            sub_dir = os.path.join(args.front_dir, sub)
            if not os.path.isdir(sub_dir):
                rep.skip("子目录 %s" % sub, sub_dir)
                continue
            tag = "[front/%s]" % sub
            rc_a, fa, sa, _la = detector_run(py, PURE, sub_dir, "front",
                                             extra=("--holes", "0"))
            rc_b, lb, out_b = pipeline_run(py, ["front"], sub_dir)
            fb, sb, _lgb = out_b["front"]
            rep.check(rc_a == 0, "%s 独立跑退出码 = 0" % tag, "rc=%d" % rc_a)
            rep.info("%s 帧数" % tag, "独立 %d / 经主程序 %d" % (len(fa), len(fb)))
            compare_verdicts(rep, tag, fa, fb)
            check_pipeline_health(rep, tag, rc_b, lb, "front")
            check_golden(rep, tag + "(独立)", sub_dir, sa)
            check_golden(rep, tag + "(经主程序)", sub_dir, sb)
            if sb is not None:
                check_actuation(rep, tag, lb, "front", int(sb.get("NG", 0)), 8, 9)
            rep.check(escapes_in(fa) == 0, "%s 独立跑零逃逸" % tag)
            rep.check(escapes_in(fb) == 0, "%s 经主程序零逃逸" % tag)

    # ---- ④ missing 站 ----
    if not args.skip_missing:
        rep.section("④ missing 站（兜孔缺粒 / D9）：独立跑 vs 经主程序跑")
        # ⚠ 这里**绝不能**把 --no-uno 透传给子进程：子进程带着 --no-uno 时 IPC 链路照样建得起来
        #   (main_pipeline._run_child 先握手)，但检测器此后永不调 pulse() —— 主程序一条 NG 都收不到，
        #   判定照跑、执行器一次没动 = 逃逸。下面的 check_actuation 就是钉死这件事的那颗钉子。
        tag = "[missing]"
        rc_a, fa, sa, _la = detector_run(py, MISSING, args.missing_dir, "missing",
                                         extra=("--no-uno",))
        rc_b, lb, out_b = pipeline_run(py, ["missing"], args.missing_dir)
        fb, sb, _lgb = out_b["missing"]
        rep.check(rc_a == 0, "%s 独立跑退出码 = 0" % tag, "rc=%d" % rc_a)
        rep.info("%s 帧数" % tag, "独立 %d / 经主程序 %d" % (len(fa), len(fb)))
        compare_verdicts(rep, tag, fa, fb)
        check_pipeline_health(rep, tag, rc_b, lb, "missing")
        if sa is not None:
            got = {"processed": int(sa.get("processed", -1)), "n_ok": int(sa.get("OK", -1)),
                   "n_ng": int(sa.get("NG", -1))}
            rep.check(got == GOLDEN_MISSING,
                      "%s 复现缺粒标定 %d/%d（判 OK %d / 判 NG %d）"
                      % (tag, GOLDEN_MISSING["n_ok"], GOLDEN_MISSING["processed"],
                         GOLDEN_MISSING["n_ok"], GOLDEN_MISSING["n_ng"]), "实测 %s" % (got,))
        if sb is not None:
            # 子进程带 --no-uno 的破法在这里被抓：NG 数 > 0 但 D9 脉冲数 == 0
            check_actuation(rep, tag, lb, "missing", int(sb.get("NG", 0)), 9, 8)
        rep.check(escapes_in(fa) == 0, "%s 独立跑零逃逸" % tag)
        rep.check(escapes_in(fb) == 0, "%s 经主程序零逃逸" % tag)

    # ---- ⑤ 两站同跑 ----
    if not args.skip_both:
        rep.section("⑤ 两站同跑（离线）：互不串路、互不干扰")
        # 两站拿同一个 --dir(主程序的透传参数只有一份)，所以挑一个两边都跑得动的目录：
        # 缺粒样本 40 张 —— 缺粒站有确定的期望值(37 NG)，正反面站在它上面也有确定判定。
        both_dir = args.missing_dir
        rep.info("目录", "%s（两站共用；正反面站对它是'判一堆 NG'，判定值不重要，一致性才重要）" % both_dir)
        _rc0, fa_alone, sa_alone, _l0 = detector_run(py, PURE, both_dir, "front",
                                                     extra=("--holes", "0"))
        front_ng_alone = int(sa_alone.get("NG", -1)) if sa_alone else -1
        rep.info("[front] 单独跑该目录", "判 NG %d 帧（作为同跑时的对照）" % front_ng_alone)

        rc, lines, out = pipeline_run(py, ["front", "missing"], both_dir)
        ff, sf, _lgf = out["front"]
        fm, sm, _lgm = out["missing"]
        rep.check(rc == 0, "两站同跑主程序退出码 = 0", "rc=%d" % rc)
        rep.check(count_in(lines, "[IPC] 站 front 已接入") == 1
                  and count_in(lines, "[IPC] 站 missing 已接入") == 1,
                  "两站都经 IPC 接入")
        rep.check(count_in(lines, "[PIPE] 重启站") == 0, "两站都没有触发重启")
        # 互不干扰：front 的判定不能因为 missing 在同时跑而变
        compare_verdicts(rep, "[front 同跑]", fa_alone, ff)
        # 互不串路：各自 NG 只变成自己那一路的脉冲
        if sf is not None:
            check_actuation(rep, "[front 同跑]", lines, "front", int(sf.get("NG", 0)), 8, 9)
        if sm is not None:
            check_actuation(rep, "[missing 同跑]", lines, "missing", int(sm.get("NG", 0)), 9, 8)
            got = {"processed": int(sm.get("processed", -1)), "n_ok": int(sm.get("OK", -1)),
                   "n_ng": int(sm.get("NG", -1))}
            rep.check(got == GOLDEN_MISSING,
                      "[missing 同跑] 仍复现缺粒标定（不受正反面站干扰）", "实测 %s" % (got,))
        rep.check(escapes_in(ff) == 0, "[front 同跑] 零逃逸")
        rep.check(escapes_in(fm) == 0, "[missing 同跑] 零逃逸")

    return rep.finish()


if __name__ == "__main__":
    sys.exit(main())
