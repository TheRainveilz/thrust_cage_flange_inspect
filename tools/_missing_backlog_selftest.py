"""第二站「判定时限」自检 —— 证明**两条超时分支**真的开闸、真的不丢帧、且方向只增 NG。

## 为什么必须单独有这个脚本

`inspector_missing.py` 的两条超时分支都靠 `current_frame_enqueued_at`(由 `Vn2000Source`
采集线程入队时打，`inspector_pure.py:1637`)，而 `--mode local` 下这个属性恒为 `None` ⇒
**L1 / L2 / `_missing_equiv.py` 598 帧对拍一条都碰不到它们**：
它们证明的是"没改坏既有判定"，**证明不了"新分支本身是对的"**。
本脚本用假图源把这个时间戳摆成想要的样子，直接把两条分支逼出来。

两条分支：

| 分支 | 触发条件 | 行为 |
|---|---|---|
| ① 出队过期预筛 | **出队时**就已 `age > RESULT_DEADLINE_MS` | 跳过算法(省 CPU)，直接判 NG + 开闸 |
| ② 主判定窗口超时 | 算完了，但 `入队→判定`(`window_ms`) `> RESULT_DEADLINE_MS` | 算法照算，判定改 NG + 开闸 |

## 自检的是四条安全不变量（不是"它会不会跳过"——那显然）

1. **判 NG 必开闸**：脉冲数 == 判 NG 帧数。判了不开闸 = 逃逸，这是唯一不能错的一条。
2. **一帧不丢**：逐帧 `[INSPECT-DONE]` 行数 == 喂进去的帧数。跳过的帧也得留下判定行，
   否则 `verify_l1_missing.py:94`(`逐帧行数 == processed`) 与
   `verify_l2_pipeline.check_actuation`(D9 脉冲数 == `[SUMMARY] NG=`) 都会对不上。
3. **触发停线的那一帧也已经开闸**：`AcquisitionError` 必须在 `actuate()` 之后抛
   —— 否则"前 14 帧都开闸，偏偏触发停线的第 15 帧没开"。
4. **没被判过期的帧照常算**：`耗时 > 0`，证明预筛没误伤正常帧。

## 场景 D/C：把"窗口口径"和"纯计算口径"切开（主判定的关键证据）

主判定从 `res.elapsed_ms` 改成 `window_ms` 后，方向必须是**只增 NG**（过杀方向，安全）。
要证明它真的生效、而不是"反正两个数差不多"，就得构造一个
**"算法算得完(不超预算)，但排队等过头(总延迟超预算)"** 的帧 —— 这种帧旧口径必放行、
新口径必拦下。做法是把 `RESULT_DEADLINE_MS` 临时放大到 2000ms 再拿捏排队时长：

- **D 组**（对照，`current_frame_enqueued_at = None`，模拟 watch/http/文件夹源）：
  退回纯计算口径 ⇒ 只有算法自己判的 NG，`cause=algo_slow` 必须 **0 条**。
  顺带在这次运行里量出每帧真实的 `耗时`。
- **C 组**（窗口）：`排队 = 预算 − 5ms`。出队时 `age≈1995 < 2000` ⇒ **预筛不触发**；
  算完 `window = 1995 + 耗时 > 2000` ⇒ **主判定触发**。
  而 `inspect = 耗时 ≈ 几十 ms ≪ 2000` ⇒ **旧口径绝不会触发**，差别只可能来自窗口。
- **D 组量出的最"快"一帧耗时必须 > 5ms**（前提自证）：否则 `window = 1995 + 耗时` 可能
  够不到 2000，C 组断言就不成立。实测最慢/最快都在几十 ms，这条前提有数量级余量。

于是 C 组每一条 `cause=algo_slow` 日志都必须满足 `inspect < budget`（旧口径不可能拦它）
且 `window > budget`（新口径拦下了），**这就是这次改动生效的充要证据**。

**翻转证据（最强的一条）**：C/D 都跑**整份样本**（而不是前 12 张 —— 那批算法本来就全判 NG，
拿"NG 变多"根本证不了什么），并临时把 `TIMEOUT_ESCALATE_N` 置 0 免得连续超时把站停了。
D 组里算法判 **OK** 的那几帧，在 C 组必须被判 **NG** —— 判决真的被窗口口径翻过来了，
这是"多判 NG"唯一无歧义的观察方式。

## 怎么跑

    python tools/_missing_backlog_selftest.py        # 要 cv2；不碰串口、不碰 IPC、不写真目录

假图源 + 假执行器 + 临时日志/存图目录，跑完即删，对仓库无副作用。
"""

from __future__ import annotations

import os
import re
import shutil
import sys
import tempfile
import time
import types
from typing import List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)                                   # verify_common
sys.path.insert(0, os.path.join(ROOT, "src", "flange_inspect"))  # inspector_missing / uno_relay

import inspector_missing as im  # noqa: E402
import inspector_pure as ip  # noqa: E402
from verify_common import INSPECT_RE, parse_frames, parse_summary  # noqa: E402

SAMPLE_DIR = os.path.join(ROOT, "datasets", "缺粒样本")
IMG_EXT = (".png", ".jpg", ".jpeg", ".bmp")
STALE_MS = 5000.0  # 假装这些帧在队列里堆了 5 秒（>> RESULT_DEADLINE_MS，必然过期）

# 场景： (积压帧数, 总帧数, 期望退出码, 说明)
#   场景 B 的 15 == inspector_missing.TIMEOUT_ESCALATE_N，用 im 里的值兜底免得两边漂。
SCENARIOS = [
    (3, 12, 0, "少量积压：跳过 3 帧后正常算完，不该停线"),
    (im.TIMEOUT_ESCALATE_N, 20, 3, "连续积压到停线阈值：第 N 帧开闸之后才停线"),
]

# ---- 场景 C/D：切开"窗口口径"与"纯计算口径" ----
WINDOW_BUDGET_MS = 2000.0  # 临时放大时限：让"算得慢"完全不可能触发，只剩"排队"能触发
WINDOW_GAP_MS = 5.0        # 排队 = 预算 − 5ms ⇒ 算完必然越线，而出队时必然不越线

# 主判定超时行的三个量：排队 / 纯计算 / 窗口(=前两者之和)。与 inspector_missing 的日志格式
# **逐字对应**，改那边格式串就得改这里（和 verify_common.INSPECT_RE 一样的耦合）。
ALGO_SLOW_RE = re.compile(
    r"queue_wait=([0-9.]+)ms inspect=([0-9.]+)ms window=([0-9.]+)ms budget=([0-9.]+)ms")
DUR_RE = re.compile(r"\[INSPECT-DONE\].*?耗时 ([0-9.]+)ms")

_results: List[Tuple[bool, str, str]] = []


def check(ok: bool, what: str, detail: str = "") -> None:
    _results.append((bool(ok), what, detail))


# --------------------------------------------------------------------------
class _FakeSource:
    """假图源：`waits[i]` = 第 i 帧"入队 → 被取走"的毫秒数；`None` = **该帧不暴露入队时间戳**。

    `None` 用来模拟 watch/http/本地文件夹源 —— 真实代码里那条路径上
    `current_frame_enqueued_at` 就是 `None`，两条超时分支都不该触发。

    只实现主循环真正用到的两样：`frames()` 与 `current_frame_enqueued_at`。
    刻意**不**实现 `current_frame_timing` / `current_frame_started_at` —— 第二站主线本来
    就没读它们(`getattr(..., None)` 兜底)，这也顺带证明这两条分支不依赖第一站那套计时结构。
    """

    def __init__(self, paths: List[str], waits: List[Optional[float]]) -> None:
        self._paths = paths
        self._waits = waits
        self.current_frame_enqueued_at = None

    def frames(self):
        for i, path in enumerate(self._paths):
            bgr = ip.imread_unicode(path)
            # 关键：在 yield 之前把"入队时刻"设成当前帧该有的值；主循环下一轮 getattr 读到的
            # 就是它（读的时候 age 就恰好是 waits[i]，因为 yield 与 getattr 之间只有循环开销）。
            w = self._waits[i]
            self.current_frame_enqueued_at = (
                None if w is None else time.perf_counter() - float(w) / 1000.0)
            yield path, bgr

    def close(self) -> None:
        pass


class _DryRelay:
    """替掉 `uno_relay.UnoRelayController`：只数脉冲，绝不碰串口。"""

    pulses: List = []

    def connect(self) -> bool:
        return True

    def pulse(self, pin=None) -> bool:
        _DryRelay.pulses.append(pin)
        return True

    def close(self) -> None:
        pass


def _install_dry_relay() -> None:
    """`main()` 里是局部 `from uno_relay import UnoRelayController` ⇒ 塞个桩模块进 sys.modules 即可。"""
    stub = types.ModuleType("uno_relay")
    stub.UnoRelayController = _DryRelay
    stub.UNO_PIN, stub.UNO_PIN2 = 8, 9
    sys.modules["uno_relay"] = stub


def _collect(n: int) -> List[str]:
    out: List[str] = []
    for base, _dirs, files in os.walk(SAMPLE_DIR):
        for fn in sorted(files):
            if fn.lower().endswith(IMG_EXT):
                out.append(os.path.join(base, fn))
    out.sort()
    if len(out) < n:
        raise SystemExit("样本不足：%s 只有 %d 张，需要 %d 张" % (SAMPLE_DIR, len(out), n))
    return out[:n]


def _count_samples() -> int:
    """样本总数（不改 `_collect` 的"不够就退出"语义，另开一个只数数的）。"""
    n = 0
    for _base, _dirs, files in os.walk(SAMPLE_DIR):
        n += sum(1 for fn in files if fn.lower().endswith(IMG_EXT))
    return n


def _run(waits: List[Optional[float]], budget_ms: Optional[float] = None,
         escalate_n: Optional[int] = None) -> Tuple[int, List[str], int, List]:
    """跑一次 main()，返回 (退出码, 本次日志行, 实际喂进去的帧数, 本次开闸脉冲)。

    脉冲**必须在这次调用里取走**：`_DryRelay.pulses` 是类属性，下一次 `_run` 会清空它，
    所以"跑完 D 再跑 C、最后一起 report"的写法若在 `_report` 里读全局就会读到 C 的次数。
    """
    tmp = tempfile.mkdtemp(prefix="missing_backlog_")
    paths = _collect(len(waits))

    saved = {k: getattr(im, k) for k in
             ("RUNTIME_LOG_DIR", "OK_SAVE_DIR", "NG_SAVE_DIR", "RAW_SAVE_DIR")}
    old_build = im.build_source
    old_budget = im.RESULT_DEADLINE_MS
    old_escalate = im.TIMEOUT_ESCALATE_N
    for k in saved:
        setattr(im, k, os.path.join(tmp, k.lower()))
    im.build_source = lambda args: _FakeSource(paths, waits)
    if budget_ms is not None:
        im.RESULT_DEADLINE_MS = budget_ms
    if escalate_n is not None:
        im.TIMEOUT_ESCALATE_N = escalate_n
    _DryRelay.pulses = []
    _install_dry_relay()

    try:
        # --debug 一起开：顺带覆盖"跳过帧也要能画/存叠加图"这条路(标注信息全空，别炸就行)。
        rc = im.main(["--mode", "camera", "--ip", "127.0.0.1", "--limit", str(len(paths)),
                      "--quiet", "--debug"])
    finally:
        im.build_source = old_build
        im.RESULT_DEADLINE_MS = old_budget
        im.TIMEOUT_ESCALATE_N = old_escalate
        for k, v in saved.items():
            setattr(im, k, v)

    lines: List[str] = []
    for base, _dirs, files in os.walk(tmp):
        for fn in files:
            if fn.endswith(".log"):
                with open(os.path.join(base, fn), "r", encoding="utf-8", errors="replace") as fh:
                    lines.extend(fh.read().splitlines())
    pulses = list(_DryRelay.pulses)   # 见 docstring：立刻取走，别留到下一次 _run 之后
    shutil.rmtree(tmp, ignore_errors=True)
    return rc, lines, len(paths), pulses


# ---- 日志取数 -------------------------------------------------------------
def _durations(lines: List[str]) -> List[float]:
    """逐帧 `耗时 X ms`（算法自报，不含排队）。"""
    return [float(m.group(1)) for m in (DUR_RE.search(x) for x in lines) if m]


def _algo_slow(lines: List[str]) -> List[Tuple[float, float, float, float]]:
    """主判定超时行取 (排队, 纯计算, 窗口, 预算)。"""
    out = []
    for x in lines:
        if "cause=algo_slow" not in x:
            continue
        m = ALGO_SLOW_RE.search(x)
        if m:
            out.append(tuple(float(v) for v in m.groups()))
    return out


def _verdicts(lines: List[str]) -> dict:
    """逐帧判决 {序号: OK/NG}，用真正那道闸的正则。"""
    return {int(m.group(1)): m.group(3) for m in (INSPECT_RE.search(x) for x in lines) if m}


def _report(tag: str, rc: int, lines: List[str], n_frames: int, expect_processed: int,
            expect_backlog: int, expect_zero_ms: int, pulses: int) -> None:
    frames = parse_frames(lines)          # 用**真正那道闸**的正则解析，不是自己写一个
    summary = parse_summary(lines) or {}
    backlog = sum(1 for ln in lines if "cause=queue_backlog" in ln)
    zero_ms = sum(1 for x in lines if INSPECT_RE.search(x) and "耗时 0.0ms" in x)
    n_ng = sum(1 for _s, _n, v in frames if v == "NG")

    print("\n---- %s ----" % tag)
    print("   退出码=%d  喂帧=%d  被处理=%d  逐帧行=%d  判NG=%d  开闸脉冲=%d  积压日志=%d  "
          "[SUMMARY] NG=%s timeout_ng=%s timeout_backlog=%s"
          % (rc, n_frames, expect_processed, len(frames), n_ng, len(pulses), backlog,
             summary.get("NG"), summary.get("timeout_ng"), summary.get("timeout_backlog")))

    check(len(frames) == expect_processed, "%s 一帧不丢(逐帧行数==被处理帧数)" % tag,
          "%d vs %d" % (len(frames), expect_processed))
    check(len(pulses) == n_ng, "%s 判NG必开闸(脉冲数==判NG数)" % tag, "%d vs %d" % (len(pulses), n_ng))
    check(backlog == expect_backlog, "%s 积压分支恰好触发 %d 次" % (tag, expect_backlog),
          "%d 次" % backlog)
    check(summary.get("timeout_backlog") == expect_backlog, "%s [SUMMARY] timeout_backlog=%d"
          % (tag, expect_backlog), "%s" % summary.get("timeout_backlog"))
    check(summary.get("NG") == n_ng, "%s [SUMMARY] NG= 与逐帧行一致" % tag,
          "%s vs %d" % (summary.get("NG"), n_ng))
    check(zero_ms == expect_zero_ms, "%s 跳过的帧耗时如实记 0.0ms" % tag, "%d 条" % zero_ms)


def _report_window(note_d: str, rc_d: int, lines_d: List[str], n_d: int,
                   note_c: str, rc_c: int, lines_c: List[str], n_c: int) -> None:
    """场景 D(对照) / C(窗口) 的差分断言。"""
    dur_d = _durations(lines_d)
    slow_d = _algo_slow(lines_d)
    slow_c = _algo_slow(lines_c)
    vd, vc = _verdicts(lines_d), _verdicts(lines_c)
    ng_d = sum(1 for v in vd.values() if v == "NG")
    ng_c = sum(1 for v in vc.values() if v == "NG")

    tag_d = "场景D[无时间戳/预算%.0fms] %s" % (WINDOW_BUDGET_MS, note_d)
    tag_c = "场景C[排队%.0fms/预算%.0fms] %s" % (WINDOW_BUDGET_MS - WINDOW_GAP_MS,
                                                 WINDOW_BUDGET_MS, note_c)
    print("\n---- %s ----" % tag_d)
    print("   退出码=%d  帧数=%d  判NG=%d  cause=algo_slow=%d  逐帧耗时 min=%.1f max=%.1f ms"
          % (rc_d, n_d, ng_d, len(slow_d), min(dur_d) if dur_d else -1,
             max(dur_d) if dur_d else -1))
    print("\n---- %s ----" % tag_c)
    print("   退出码=%d  帧数=%d  判NG=%d  cause=algo_slow=%d"
          % (rc_c, n_c, ng_c, len(slow_c)))
    for qw, ins, win, bud in slow_c[:3]:
        print("     排队=%.1f  计算=%.1f  窗口=%.1f  预算=%.1f ms" % (qw, ins, win, bud))
    if len(slow_c) > 3:
        print("     …其余 %d 条同形" % (len(slow_c) - 3))

    # D 组：没有入队时间戳 ⇒ 退回纯计算口径 ⇒ 一条窗口超时都不该有。
    check(len(slow_d) == 0, "%s 无时间戳时退回纯计算口径，0 条窗口超时" % tag_d,
          "%d 条" % len(slow_d))
    # 前提自证：D 组量出的最"快"一帧也必须比 WINDOW_GAP_MS 慢，否则 C 组的窗口够不到预算、
    # 构造不成立（断言会以"没触发"的形式失败，而不是假绿）。
    min_dur = min(dur_d) if dur_d else -1.0
    check(min_dur > WINDOW_GAP_MS, "%s 前提：最慢帧耗时 %.1fms > 间隙 %.1fms(构造成立)"
          % (tag_d, min_dur, WINDOW_GAP_MS), "min=%.1fms" % min_dur)
    # C 组：出队时 age<预算(预筛不触发)、算完窗口>预算(主判定触发) ⇒ 每帧都该被拦下。
    check(len(slow_c) == n_c, "%s 每帧都被窗口口径拦下" % tag_c, "%d vs %d" % (len(slow_c), n_c))
    check(ng_c == n_c, "%s 窗口超时的帧全部判 NG" % tag_c, "%d vs %d" % (ng_c, n_c))
    # ★ 这次改动的充要证据：拦下的每一帧，"纯计算"都远在预算之内 ⇒ **旧口径绝不会拦它**。
    #   所以 C 组比 D 组多出来的那些 NG，只可能来自窗口(入队→判定)这一条钟。
    bad = [(ins, bud) for _qw, ins, _win, bud in slow_c if ins >= bud]
    check(not bad, "%s 每条拦下的帧 纯计算<预算(旧口径不可能拦它 ⇒ 差别只来自窗口)" % tag_c,
          "%d 条反例" % len(bad))
    # ★★ 翻转证据：D 组算法判 OK 的帧，C 组必须变成 NG —— 判决真的被窗口翻过来了。
    #    （只能拿"OK→NG"当证据：本样本集里算法判 NG 的帧占多数，光看"NG 变多"分不清
    #      是窗口拦的还是它本来就 NG。）
    ok_d = sorted(s for s, v in vd.items() if v == "OK")
    not_flipped = [s for s in ok_d if vc.get(s) != "NG"]
    check(len(ok_d) >= 1, "%s 样本里存在算法判 OK 的帧(否则证不了翻转)" % tag_d,
          "%d 帧 OK" % len(ok_d))
    check(not not_flipped, "%s 算法判 OK 的 %d 帧被窗口口径强制成 NG(翻转证据)"
          % (tag_c, len(ok_d)), "未翻转 %d 帧: %s" % (len(not_flipped), not_flipped[:5]))
    # 窗口 = 排队 + 计算（同一条钟，三个量必须自洽，不然日志会误导事后复盘）
    incons = [(qw, ins, win) for qw, ins, win, _b in slow_c if abs((qw + ins) - win) > 2.0]
    check(not incons, "%s 日志自洽(queue_wait + inspect ≈ window)" % tag_c,
          "%d 条不自洽" % len(incons))


def main() -> int:
    print("第二站「判定时限」自检  (RESULT_DEADLINE_MS=%.1fms  "
          "TIMEOUT_ESCALATE_N=%d  STALE_MS=%.0f)" %
          (im.RESULT_DEADLINE_MS, im.TIMEOUT_ESCALATE_N, STALE_MS))
    print("样本 %s" % SAMPLE_DIR)

    # ---- 分支①：出队过期预筛 ----
    for stale_n, total, want_rc, note in SCENARIOS:
        waits: List[Optional[float]] = [STALE_MS if i < stale_n else 0.0 for i in range(total)]
        rc, lines, n_frames, pulses = _run(waits)
        # rc=3 是**故意停线**：第 stale_n 帧开闸之后抛 AcquisitionError，后面的帧根本没被取，
        # 所以"被处理的帧数"就是 stale_n —— 而**触发停线的那一帧也必须留下判定行**
        # (它已经开闸了，少一行就对不上开闸次数)。这就是下面这条断言的语义。
        expect_lines = stale_n if rc == 3 else n_frames
        tag = "场景[%d积压/%d帧] %s" % (stale_n, total, note)
        _report(tag, rc, lines, n_frames, expect_lines, stale_n, stale_n, pulses)
        check(rc == want_rc, "%s 退出码 = %d" % (tag, want_rc), "rc=%d" % rc)

    # ---- 分支②：主判定窗口超时（先跑 D 对照，再跑 C 窗口）----
    # 跑**整份样本**：需要 D 组里确实有"算法判 OK"的帧，才证得了"被窗口翻成 NG"。
    # `escalate_n=0` = 永不自动停线 —— 否则 C 组连续超时到第 15 帧就把站停了，走不完全程。
    n_win = _count_samples()
    rc_d, lines_d, n_d, pulses_d = _run([None] * n_win, budget_ms=WINDOW_BUDGET_MS, escalate_n=0)
    rc_c, lines_c, n_c, pulses_c = _run([WINDOW_BUDGET_MS - WINDOW_GAP_MS] * n_win,
                                        budget_ms=WINDOW_BUDGET_MS, escalate_n=0)
    for tag, rc, lines, n, pulses in (("D", rc_d, lines_d, n_d, pulses_d),
                                      ("C", rc_c, lines_c, n_c, pulses_c)):
        _report("场景%s[预算%.0fms]" % (tag, WINDOW_BUDGET_MS), rc, lines, n, n, 0, 0, pulses)
    check(rc_d == 0, "场景D 退出码 = 0", "rc=%d" % rc_d)
    check(rc_c == 0, "场景C 退出码 = 0", "rc=%d" % rc_c)
    _report_window("watch/http/文件夹源应无窗口超时", rc_d, lines_d, n_d,
                   "排队越线但算法没超时应被拦下", rc_c, lines_c, n_c)

    print("\n" + "=" * 78)
    bad = [(w, d) for ok, w, d in _results if not ok]
    for ok, what, detail in _results:
        print("  [%s] %s%s" % (" OK " if ok else "FAIL", what,
                               ("  (%s)" % detail) if detail else ""))
    print("-" * 78)
    print("通过 %d / 失败 %d" % (len(_results) - len(bad), len(bad)))
    print("结论：%s" % ("两条超时分支都确实开闸且不丢帧" if not bad else "有硬闸未过，别信这两条分支"))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
