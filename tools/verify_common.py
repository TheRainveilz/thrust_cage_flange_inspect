# -*- coding: utf-8 -*-
"""L1/L2 验证脚本的公共工具：跑检测器、取"本次运行新增的日志"、解析逐帧判定。

**为什么"独立跑"和"经主程序跑"都从 .log 取逐帧判定，而不是 stdout**：
产线配置带 `--quiet`(见 main_pipeline.STATIONS 的 argv)，它把控制台 handler 抬到 WARNING，
逐帧 `[INSPECT-DONE]` 只进 .log。主程序跑的那条链必然是 --quiet，所以两边必须用同一把尺子
—— 都从 .log 取。否则"经主程序跑"这一侧根本读不到逐帧行，等价性就成了空话。

**为什么按"文件大小增量"取日志**：运行日志是**追加**写的(`open(path, "a")`)、按天分片，
同一天里多个进程/多次运行共用同一个文件。所以"跑之前记下每个文件的字节数、跑完只读那之后的
字节"是唯一不会串到别的运行上去的办法。日志按天分片且单片超 RUNTIME_LOG_MAX_BYTES 会切到
`_01` 后缀，故逐文件分别记增量，而不是只盯一个文件。
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
from typing import Dict, List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(HERE)
SRC_DIR = os.path.join(PROJECT_ROOT, "src", "flange_inspect")

# 控制台编码兜底：这些脚本会把检测器日志(含 U+2212 "−" 等非 GBK 字符，如"99分位−中位数")原样
# 转印到 stdout。中文 Windows 默认 GBK 控制台会在 print 时抛 UnicodeEncodeError —— 而且是在
# **所有断言都跑完、正要收尾**时崩，白白让一道硬闸死在终点线。这里把 stdout/stderr 强制转成
# UTF-8(errors="replace" 再兜一层，实在编不出的字符降级成 ? 也绝不崩)。Py3.7+ 才有 reconfigure。
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[union-attr]
    except (AttributeError, ValueError):
        pass  # 老解释器或已被重定向的流：保持原样，大不了回到旧的偶发崩溃

# 两站的运行日志根目录(与 inspector_pure / inspector_missing 的常量保持一致)
FRONT_LOG_ROOT = os.path.join(PROJECT_ROOT, "data", "logs")
MISSING_LOG_ROOT = os.path.join(PROJECT_ROOT, "data", "missing", "logs")
# front 经主程序跑时, main_pipeline 把它的日志收拢到 data/pure/logs(与 data/missing/* 对称);
# 独立跑 inspector_pure 仍写 data/logs(基线一行不改, 无处覆盖模块级常量)。两条链因此各读各的根。
FRONT_PIPELINE_LOG_ROOT = os.path.join(PROJECT_ROOT, "data", "pure", "logs")

# 默认样本目录
DEFAULT_FRONT_DIR = os.path.join(PROJECT_ROOT, "datasets", "flange")
DEFAULT_MISSING_DIR = os.path.join(PROJECT_ROOT, "datasets", "缺粒样本")

# 交叉验证过的黄金基线(`datasets/flange` 的 {OK,NG} 两个子目录，`--holes 0`)。
# 只在跑到这个数据集时才断言 —— 换数据集这些数字没有意义。
GOLDEN_FRONT = {
    "OK": {"processed": 352, "n_ok": 318, "n_ng": 34},   # 正面：318/352，34 过杀(可接受)
    "NG": {"processed": 623, "n_ok": 0, "n_ng": 623},    # 反面：623/623 零逃逸
}


def _detail(text: str) -> str:
    """把补充说明拼成 "  (说明)"；空则不拼。"""
    return ("  (%s)" % text) if text else ""


class Report:
    """很小的检查清单：硬闸(check)不过 -> 退出码 1；其余只记录不阻塞。"""

    def __init__(self, title: str) -> None:
        self.title = title
        self.n_ok = self.n_fail = self.n_warn = self.n_skip = 0
        print("=" * 78)
        print(title)
        print("=" * 78)

    def section(self, text: str) -> None:
        print("\n---- %s ----" % text)

    def check(self, ok: bool, label: str, detail: str = "") -> bool:
        """硬闸：安全不变量/等价性。不通过则整体 FAIL。"""
        if ok:
            self.n_ok += 1
            print("  [ OK ] %s%s" % (label, _detail(detail)))
        else:
            self.n_fail += 1
            print("  [FAIL] %s%s" % (label, _detail(detail)))
        return bool(ok)

    def warn(self, label: str, detail: str = "") -> None:
        """软闸：过杀之类"可接受但要看见"的事。"""
        self.n_warn += 1
        print("  [WARN] %s%s" % (label, _detail(detail)))

    def skip(self, label: str, detail: str = "") -> None:
        self.n_skip += 1
        print("  [SKIP] %s%s" % (label, _detail(detail)))

    def info(self, label: str, detail: str = "") -> None:
        print("  [info] %s%s" % (label, _detail(detail)))

    def finish(self) -> int:
        print("\n" + "-" * 78)
        print("通过 %d / 失败 %d / 警告 %d / 跳过 %d"
              % (self.n_ok, self.n_fail, self.n_warn, self.n_skip))
        if self.n_fail:
            print("结论：**不通过** —— 有 %d 项硬闸未过，先别上线" % self.n_fail)
            return 1
        print("结论：全部硬闸通过")
        return 0


# ============================  跑子进程  ============================
def find_python() -> str:
    """解析项目 venv 的解释器(与 run_inspector.sh 同一套候选顺序)。

    产线 Ubuntu 用 venv/(无点)，Windows 开发机用 .venv/Scripts/。都不在才回退 sys.executable。
    """
    for cand in (os.path.join(PROJECT_ROOT, "venv", "bin", "python"),
                 os.path.join(PROJECT_ROOT, ".venv", "bin", "python"),
                 os.path.join(PROJECT_ROOT, ".venv", "Scripts", "python.exe")):
        if os.path.isfile(cand):
            return cand
    return sys.executable


def run(argv: List[str], timeout: float = 3600.0) -> Tuple[int, str]:
    """跑一个子进程并合并捕获 stdout+stderr。返回 (退出码, 文本)。

    强制 PYTHONUTF8/PYTHONIOENCODING：否则 Windows 上按 GBK 输出中文日志，解码成乱码后
    逐帧判定行会解析不出来(表现为"一帧都没读到"，很容易误判成检测器没跑)。
    """
    env = dict(os.environ, PYTHONUTF8="1", PYTHONIOENCODING="utf-8")
    try:
        p = subprocess.run(argv, cwd=PROJECT_ROOT, env=env, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, timeout=timeout)
    except subprocess.TimeoutExpired:
        return -9, "[verify] 子进程超时 %.0fs，已放弃" % timeout
    return p.returncode, p.stdout.decode("utf-8", errors="replace")


def pure_is_clean() -> Optional[bool]:
    """inspector_pure.py 是否**没有**未提交改动(黄金基线铁律)。

    返回 True/False；不是 git 仓库或 git 不可用则返回 None(调用方应记 SKIP 而非 FAIL)。
    """
    try:
        p = subprocess.run(["git", "status", "--porcelain", "--",
                            "src/flange_inspect/inspector_pure.py"],
                           cwd=PROJECT_ROOT, stdout=subprocess.PIPE,
                           stderr=subprocess.DEVNULL, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if p.returncode != 0:
        return None
    return p.stdout.decode("utf-8", "replace").strip() == ""


# ============================  日志增量  ============================
def snapshot_sizes(root: str) -> Dict[str, int]:
    """记下 root 树下每个文件的字节数(跑之前调)。目录不存在则返回空。"""
    out: Dict[str, int] = {}
    for dirpath, _dirs, files in os.walk(root):
        for f in files:
            p = os.path.join(dirpath, f)
            try:
                out[p] = os.path.getsize(p)
            except OSError:
                pass
    return out


def read_new_lines(before: Dict[str, int], root: str) -> List[str]:
    """读出 root 树下"相对 before 新增"的那些行(跑之后调)。

    老文件从旧字节数处续读，本次新建的文件从头读。按路径排序只是为了让输出稳定 ——
    调用方一律按帧号重新排序，不依赖文件顺序。
    """
    lines: List[str] = []
    paths: List[str] = []  # 日志按 YYYY/MM/DD 分层，得递归收
    for dirpath, _dirs, files in os.walk(root):
        for f in files:
            paths.append(os.path.join(dirpath, f))
    for p in sorted(paths):
        start = before.get(p, 0)
        try:
            size = os.path.getsize(p)
        except OSError:
            continue
        if p in before and size <= start:
            continue
        try:
            with open(p, "r", encoding="utf-8", errors="replace") as fh:
                fh.seek(start)
                lines.extend(fh.read().splitlines())
        except OSError:
            pass
    return lines


# ============================  解析  ============================
# 两站的逐帧行格式一致：`[INSPECT-DONE] #<帧号> <名字>  判定 OK|NG  ...`
# 名字用非贪婪匹配到"判定"之前 —— 缺粒站打的是**完整路径**(可能带空格)，正反面站打的是
# _short_name(形如 CAM#001310)，两种都得吃下。
INSPECT_RE = re.compile(r"\[INSPECT-DONE\]\s+#(\d+)\s+(.+?)\s+判定\s+(OK|NG)")


def parse_frames(lines: List[str]) -> List[Tuple[int, str, str]]:
    """解析逐帧判定，返回 [(帧号, 名字, "OK"/"NG")]，按帧号升序。"""
    recs = []
    for ln in lines:
        m = INSPECT_RE.search(ln)
        if m:
            recs.append((int(m.group(1)), m.group(2).strip(), m.group(3)))
    recs.sort(key=lambda r: r[0])
    return recs


def verdict_by_name(frames: List[Tuple[int, str, str]]) -> Dict[str, str]:
    """把逐帧列表折成 {名字: 判定}，用来跨运行比对(不依赖帧号对齐)。"""
    return {name: verdict for _seq, name, verdict in frames}


def parse_summary(lines: List[str]) -> Optional[Dict[str, float]]:
    """取最后一条 `[SUMMARY] processed=` 行里的所有 key=value。

    两站的 [SUMMARY] 字段不一样(缺粒站没有 raw_saved/received 那一串)，所以按 key 抽，
    不按位置抽 —— 少一个字段不该让整行解析失败。
    """
    tail = None
    for ln in lines:
        i = ln.find("[SUMMARY] processed=")
        if i >= 0:
            tail = ln[i:]
    if tail is None:
        return None
    out: Dict[str, float] = {}
    # 键名大小写都要收：[SUMMARY] 里既有 processed/no_actuator 这类小写，也有 OK=/NG= 这类大写。
    for key, val in re.findall(r"([A-Za-z_]+)=([0-9]+(?:\.[0-9]+)?)", tail):
        out[key] = float(val)
    return out


def truth_from_path(path: str) -> Optional[bool]:
    """真值只认**目录名**(OK/正 -> True；NG/反 -> False)，不看文件名。

    文件名在这个项目里是不可信的：`datasets/flange/OK/` 里的图也叫
    `..._CAM#001310_NG_NO_FEATURE.jpg` —— 按文件名猜会把整批正面样本反成反面
    (inspector_pure 的 `[SELF-CHECK]` 正是这么猜的，所以那一行在这里是错的，别无脑引用)。
    """
    parts = os.path.normpath(path).split(os.sep)[:-1]  # 去掉文件名，只看目录层
    for part in reversed(parts):
        if part.upper() == "OK" or "正" in part:
            return True
        if part.upper() == "NG" or "反" in part:
            return False
    return None


def count_escapes(frames: List[Tuple[int, str, str]]) -> Tuple[int, int, int]:
    """统计 (逃逸数, 过杀数, 有真值的帧数)。

    逃逸 = 真值 NG 却判 OK(唯一不可接受的方向)；过杀 = 真值 OK 却判 NG(可接受，但要看得见)。
    """
    esc = over = labeled = 0
    for _seq, name, verdict in frames:
        truth = truth_from_path(name)
        if truth is None:
            continue
        labeled += 1
        if truth is False and verdict == "OK":
            esc += 1
        elif truth is True and verdict == "NG":
            over += 1
    return esc, over, labeled


def count_in(lines: List[str], needle: str) -> int:
    """出现次数(逐帧计数用)。"""
    return sum(1 for ln in lines if needle in ln)
