#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""产线主程序(监督者)：独占 UNO 串口 + 拉起/看护两个检测子进程。

**为什么要有这一层**：一块 UNO 只有一条串口，而"正反面翻边止口"和"兜孔缺粒"两个检测站
都要在判 NG 时驱动各自的执行器(D8=吹气把件从料道吹掉；D9=开闸门放件掉进回收盒)。
串口只能有一个主人，所以：

    本站(main_pipeline.py)  ──独占串口──►  UNO D8(正反面/吹气) / D9(缺粒/开闸)
        ▲  ▲
        │  └── 子进程 B: inspector_missing.py  (缺粒站, 上报 NG → 本站开闸 D9)
        └───── 子进程 A: inspector_pure.py     (正反面站, 上报 NG → 本站吹气 D8)

两个子进程只**判定**，判定结果经 localhost TCP 报给本站(见 station_link.py)，本站再触发对应引脚。
两个子进程各自是一个独立进程(独立 GIL、独立相机 RX 线程) —— 同进程多线程会把 RX 线程
饿死出残帧，这是本项目已经付过代价的教训。

**inspector_pure.py 一行不改**：它 `main()` 里是局部 `from uno_relay import UnoRelayController`
+ 无参构造，所以本站把 `uno_relay.UnoRelayController` 换成 `IpcRelay` 就能拦下它唯一的执行点
(`emit_control` 里的 `uno.pulse()`)，判定逻辑与黄金基线完全一致。

**失败语义(安全铁律)**：真实 NG 判 OK(逃逸)绝不可接受，过杀可接受。因此
  · 任一子进程挂掉 → 本站重启它(退避很短：停得越久，没判的件越多)，**不影响另一站**；
  · 本站自己的执行链路(串口写失败/UNO 掉线)故障 → **两站一起停**。因为 UNO 是两条链共用的，
    触发不了执行器(吹气/开闸都发不出脉冲)时继续判就是在制造逃逸，没有"只停一站"这个选项。
"""
from __future__ import annotations

import argparse
import importlib
import logging
import os
import signal
import subprocess
import sys
import threading
import time
from logging.handlers import TimedRotatingFileHandler
from typing import Dict, List, Optional

import uno_relay  # 只为借用 UNO_PIN/UNO_PIN2(引脚的唯一定义源)

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(HERE))  # src/flange_inspect -> 项目根
SELF = os.path.abspath(__file__)

# ============================  两站的唯一配置源  ============================
# id      站点名(同时是 IPC 握手里的 station 名, 子进程经 STATION_ID 环境变量收到)
# script  检测器脚本(与本站同目录)
# ip      相机 IP。**必须显式给**：同网段现在有两台相机, 而 inspector_pure 的自动探测
#         只认"10001 段内唯一应答者", 两台时必然返回 None 并回退到硬编码默认值 —— 会指错相机。
# pin     本站收到该站 NG 时该触发的 UNO 引脚(D8=吹气/D9=开闸, 见 uno_relay.py)
# argv    传给检测器的固定参数(**参数不在别处再抄一份**：阈值等一律留在检测器自己的参数区)
STATIONS: List[Dict] = [
    {
        "id": "front",
        "script": "inspector_pure.py",
        "ip": "169.254.44.201",
        "pin": uno_relay.UNO_PIN,  # D8
        # --holes 0 = 判全部孔。这是黄金基线(318/352 & 反面 623/623 零逃逸)验证时的配置，
        # 值上等于 inspector_pure 的模块默认，但**写出来**才是对生产配置的忠实镜像：
        # 哪天默认值被改动，这里仍能保证跑的是已验证配置。
        # "argv": ["--mode", "camera", "--holes", "0", "--quiet", "--timing"], # 测试环境带--timing用于查看判定耗时
        "argv": ["--mode", "camera", "--holes", "0", "--quiet"], # 生产环境不带--timing
    },
    {
        "id": "missing",
        "script": "inspector_missing.py",
        "ip": "169.254.44.202",
        "pin": uno_relay.UNO_PIN2,  # D9
        # "argv": ["--mode", "camera", "--quiet", "--timing"],# 测试环境带--timing用于查看判定耗时
        "argv": ["--mode", "camera", "--quiet"], # 生产环境不带--timing
    },
]

IPC_HOST, IPC_PORT = "127.0.0.1", 47653

MONITOR_INTERVAL_S = 0.5  # 子进程存活巡检周期
RESTART_BACKOFF_S = 1.5  # 首次重启退避：停得越久漏检越多，必须短
RESTART_BACKOFF_MAX_S = 12.0
RESTART_ESCALATE_N = 5  # 连续失败这么多次 -> 升级 CRITICAL 提示人工介入
HEALTHY_RUN_S = 60.0  # 子进程活过这么久就认为"跑稳了"，失败计数清零
CHILD_TERM_GRACE_S = 5.0  # 停机时先 terminate，这么久还没退再 kill
UNO_DRAIN_S = 0.3  # 停机时等最后一个脉冲走完(固件 PULSE_MS=50ms)再 close 串口
STATUS_INTERVAL_S = 60.0  # 定期打印监督摘要(0=关)

# 常驻语义: 相机没上电**不是故障**。  子进程在这么长的启动建链窗口内一直重试连接而不退出,
# 所以"先开的那台相机那一路先工作", 后开的那路等相机上电自然接上, 主程序绝不会因"连不上相机"
# 反复重启子进程(churn)/误报连续失败。用有限大值(≈10 年)即可: 单调钟永远到不了这个 deadline,
# 语义比 float('inf') 清晰。仅在 _run_child 里覆盖 inspector_pure 的模块级窗口, 基线一行不改。
CAM_RESIDENT_WAIT_S = 10 * 365 * 24 * 3600  # ≈10 年(s): 子进程等相机上电的窗口
# 向 UNO 双色 LED 刷健康态的周期(s)。**必须 < 固件 LED_WATCHDOG_MS(3s)**, 否则看门狗会把
# "主程序没按时喂指令"误判成失联, 自动把绿灯打成红闪。1s 每次都发, 兼作"主程序还活着"的心跳。
LED_REFRESH_S = 1.0

SUP_LOG_DIR = os.path.join(PROJECT_ROOT, "data", "supervisor")
SUP_LOG_BASENAME = "supervisor.log"
SUP_LOG_RETAIN_DAYS = 14
SUP_LOG_MAX_BYTES = 20 * 1024 * 1024


# ============================  日志  ============================
def setup_supervisor_logging() -> logging.Logger:
    """本站自己的日志：控制台 + data/supervisor/ 下按天滚动(留 SUP_LOG_RETAIN_DAYS 天)。

    刻意**不**复用 inspector_pure 的 DailyDirRotatingHandler —— 那会把 cv2/numpy 拖进
    监督者进程。监督者要尽量轻：它得在检测器出任何问题时都还能活着重启子进程。
    """
    log = logging.getLogger("pipeline")
    if getattr(log, "_configured", False):
        return log
    log.setLevel(logging.DEBUG)
    log.propagate = False
    fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s", "%H:%M:%S")

    sh = logging.StreamHandler(sys.stdout)
    sh.setLevel(logging.INFO)
    sh.setFormatter(fmt)
    log.addHandler(sh)

    try:
        os.makedirs(SUP_LOG_DIR, exist_ok=True)
        fh = TimedRotatingFileHandler(os.path.join(SUP_LOG_DIR, SUP_LOG_BASENAME),
                                      when="midnight", backupCount=SUP_LOG_RETAIN_DAYS,
                                      encoding="utf-8")
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        log.addHandler(fh)
    except OSError as exc:  # 磁盘满/无权限：监督者不能因此不干活
        log.error("[PIPE] 无法建立监督日志文件(%s)，只写控制台", exc)
    log._configured = True  # type: ignore[attr-defined]
    return log


class Station:
    """一个检测子进程的运行时状态。"""

    def __init__(self, cfg: Dict) -> None:
        self.id: str = cfg["id"]
        self.cfg = cfg
        self.proc: Optional[subprocess.Popen] = None
        self.started_at = 0.0
        self.restarts = 0  # 累计重启次数
        self.consec_fail = 0  # 连续启动即失败次数
        self.ng_relayed = 0  # 本站代它触发执行器的次数
        self.ng_failed = 0  # 执行器触发失败次数(致命)
        self.link_up = False  # 上一轮巡检时 IPC 是否已建链
        self.exited_ok = False  # 本站在本轮是否以 rc=0 正常收工(供 --exit-when-done 判断)


# ============================  监督者  ============================
class Supervisor:
    """拉起并看护两个检测子进程；独占 UNO；把上报的 NG 变成对应引脚的脉冲。"""

    def __init__(self, stations: List[Station], passthrough: List[str],
                 dry_uno: bool = False, status_interval: float = STATUS_INTERVAL_S,
                 exit_when_done: bool = False) -> None:
        self.stations = stations
        self.passthrough = list(passthrough)
        self.dry_uno = dry_uno
        self.status_interval = status_interval
        self.exit_when_done = exit_when_done
        self.log = setup_supervisor_logging()
        self._by_id = {s.id: s for s in stations}
        self._stop = threading.Event()
        self._uno: Optional[uno_relay.UnoRelayController] = None
        self._server = None  # station_link.VerdictServer
        self._drainers: List[threading.Thread] = []
        self._fatal_reason = ""
        # 健康态 LED 的上一次状态与发送时刻(供 _drive_led 做"跳变即发 / 否则定期刷")
        self._last_led: Optional[bool] = None
        self._last_led_t = 0.0

    # ---------- 启动 ----------
    def start(self) -> int:
        """按"先通道、再执行器、最后检测器"的顺序启动 —— 绝不先跑一个触发不了执行器的检测器。"""
        import station_link

        # ① 上报通道先就位(子进程一起来就要能握手)
        self._server = station_link.VerdictServer(
            on_ng=self.on_ng, host=IPC_HOST, port=IPC_PORT, logger=self.log,
            on_link_lost=self._on_link_lost)
        try:
            self._server.listen()
        except OSError as exc:
            self.log.critical("[PIPE][FATAL] 上报通道监听 %s:%d 失败: %s"
                              "(端口被占? 已有实例在跑?)", IPC_HOST, IPC_PORT, exc)
            return 4

        # ② 再拿执行器。连不上就没有任何理由让检测器上线 —— 那等于造逃逸。
        if self.dry_uno:
            self._uno = uno_relay._DryRelay()
            self.log.warning("[PIPE] --dry-uno: 不连串口, 只打印将要触发哪个引脚")
            self._uno.connect()
        else:
            self._uno = uno_relay.UnoRelayController()
            if not self._uno.connect():
                self.log.critical("[PIPE][FATAL] UNO 未连接, 拒绝启动任何检测站"
                                  "(fail-safe: 触发不了执行器的检测器绝不上线)")
                self._server.close()
                return 2

        # ③ 最后起检测器
        for st in self.stations:
            self._spawn(st)
        self.log.info("[PIPE] 已启动 %d 个检测站: %s",
                      len(self.stations), ", ".join(s.id for s in self.stations))
        return 0

    def _spawn(self, st: Station) -> None:
        """起一个检测子进程(env 注入站点身份 + IPC 端口)。"""
        script = os.path.join(HERE, st.cfg["script"])
        if not os.path.exists(script):
            self.log.critical("[PIPE][FATAL] 站 %s 的检测器脚本不存在: %s", st.id, script)
            self._fatal_reason = "缺少检测器脚本 %s" % st.cfg["script"]
            return
        argv = [sys.executable, SELF, "--child", st.id] + list(st.cfg["argv"]) + \
               ["--ip", st.cfg["ip"]] + self.passthrough
        env = dict(os.environ)
        env["STATION_ID"] = st.id
        env["IPC_PORT"] = str(self._server.port if self._server else IPC_PORT)
        env["PYTHONUTF8"] = "1"  # 子进程 stdout 统一 UTF-8, 本站按 UTF-8 解码
        env["PYTHONIOENCODING"] = "utf-8"
        st.started_at = time.time()
        st.link_up = False
        st.exited_ok = False
        try:
            # stdout 走管道 + 专用排空线程：既汇聚进监督日志，又绝不会因管道写满而
            # 阻塞子进程的检测主线程(300ms 预算下，阻塞等于假 NG)。
            st.proc = subprocess.Popen(argv, cwd=PROJECT_ROOT, env=env,
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       bufsize=1, text=True, encoding="utf-8",
                                       errors="replace")
        except OSError as exc:
            self.log.critical("[PIPE][FATAL] 站 %s 启动失败: %s", st.id, exc)
            st.proc = None
            return
        t = threading.Thread(target=self._drain, args=(st,), name="drain-%s" % st.id,
                             daemon=True)
        self._drainers.append(t)
        t.start()
        self.log.info("[PIPE] 站 %s 已拉起 (pid=%s, pin=D%d, cam=%s)",
                      st.id, st.proc.pid, st.cfg["pin"], st.cfg["ip"])

    def _drain(self, st: Station) -> None:
        """把子进程 stdout 逐行转发进监督日志(带站点前缀)，供事后对账。"""
        proc = st.proc
        if proc is None or proc.stdout is None:
            return
        for line in proc.stdout:
            line = line.rstrip("\n")
            if line:
                self.log.info("[%s] %s", st.id, line)

    # ---------- 执行 ----------
    def on_ng(self, station_id: str) -> None:
        """收到某站上报的 NG -> 立刻触发该站对应的引脚(关键路径；D8 吹气 / D9 开闸)。"""
        st = self._by_id.get(station_id)
        if st is None:
            # 身份不明的上报一律不触发：宁可不动作(过杀/停线)也不能触发错一路
            self.log.critical("[PIPE][FATAL] 收到未知站 %r 的 NG, 拒绝触发", station_id)
            self._fatal_reason = "未知站 %r 上报 NG" % station_id
            self._stop.set()
            return
        if self._uno is None or not self._uno.pulse(st.cfg["pin"]):
            st.ng_failed += 1
            self.log.critical("[PIPE][FATAL] 站 %s 的 NG 未能驱动 D%d —— 执行链路故障",
                              st.id, st.cfg["pin"])
            self._fatal_reason = "站 %s 执行器触发失败(UNO 掉线?)" % st.id
            self._stop.set()  # UNO 是两站共用的：触发不了就两站一起停，见模块文档
            return
        st.ng_relayed += 1
        # 执行留痕：uno_relay 自己只用 print() 报引脚(那走 stdout、不进日志文件)，
        # 而"到底触发了哪一路、触发了几次"是事后唯一能对账的证据，必须进监督日志。
        self.log.info("[PIPE] 站 %s NG -> 触发 D%d (本站累计代触发 %d 次)",
                      st.id, st.cfg["pin"], st.ng_relayed)

    def _on_link_lost(self, station_id: str) -> None:
        """某站上报链路断开(子进程会自己停线；这里的重启交给巡检循环)。"""
        st = self._by_id.get(station_id)
        if st is not None:
            st.link_up = False
        self.log.warning("[PIPE] 站 %s 上报链路断开(该站将自行停线, 等待重启)", station_id)

    # ---------- 健康态 LED(绿=全部正常 / 红=有问题) ----------
    def _compute_healthy(self) -> bool:
        """绿灯的充要条件: UNO 在线, 且**每个站**都(进程活着 + IPC 建链 + 相机已就绪在判定)。

        任一条不满足即红灯 —— 相机没上电、子进程挂了、IPC 断了都算"有问题"。刻意用
        is_inspecting(该站发过 UP)而非仅 is_connected: 常驻等相机上电期间 IPC 早握好了, 但相机
        还没通, 这时必须红灯。绿灯只能在"两台相机 + UNO 全部正常、正在判定"时亮 —— 绿灯不说谎。
        """
        if self._uno is None or not self._uno.connected or self._server is None:
            return False
        for st in self.stations:
            alive = st.proc is not None and st.proc.poll() is None
            if not (alive and self._server.is_connected(st.id)
                    and self._server.is_inspecting(st.id)):
                return False
        return True

    def _drive_led(self, now: float) -> None:
        """把健康态刷给 UNO 双色 LED：状态跳变立即发, 否则每 LED_REFRESH_S 发一次
        (既反映状态, 又持续喂固件看门狗证明主程序活着)。

        最佳努力：发失败只是 LED 不准, 真正的执行安全由 pulse() 那条路把关, 且固件看门狗会在
        失联 3s 后自动转红。不连真串口(--dry-uno)时经 _DryRelay.send 打印, 不影响判定/对账。
        """
        if self._uno is None:
            return
        healthy = self._compute_healthy()
        if healthy == self._last_led and (now - self._last_led_t) < LED_REFRESH_S:
            return
        self._uno.set_led(healthy)
        self._last_led = healthy
        self._last_led_t = now

    # ---------- 巡检/重启 ----------
    def monitor_loop(self) -> None:
        """存活巡检 + 重启 + 定期摘要。"""
        next_status = time.time() + self.status_interval if self.status_interval else 0.0
        while not self._stop.wait(MONITOR_INTERVAL_S):
            now = time.time()
            for st in self.stations:
                self._check_link(st)
                if st.proc is None:
                    continue
                rc = st.proc.poll()
                if rc is None:
                    continue
                self._on_child_exit(st, rc)
            self._drive_led(now)  # 每巡检周期刷一次健康态 LED(顺带喂固件看门狗)
            if next_status and now >= next_status:
                next_status = now + self.status_interval
                self.log_status()
            # 离线跑(local/watch 等有限样本源)时，所有站都正常收工就该退，别一直守着；
            # 产线相机模式下子进程不会 rc=0 收工，所以这条对产线无效。
            if self.exit_when_done and all(s.proc is None and s.exited_ok
                                           for s in self.stations):
                self.log.info("[PIPE] 所有站已正常收工，主程序退出")
                self._stop.set()

    def _check_link(self, st: Station) -> None:
        """记录 IPC 建链/断链的跳变(仅供可见性，动作仍由子进程自己 fail-safe 停线)。"""
        if self._server is None:
            return
        up = self._server.is_connected(st.id)
        if up != st.link_up:
            st.link_up = up
            if up:
                self.log.info("[PIPE] 站 %s 上报链路已建立", st.id)
            else:
                self.log.warning("[PIPE] 站 %s 上报链路未建立/已断开", st.id)

    def _on_child_exit(self, st: Station, rc: int) -> None:
        """子进程退出 -> 决定要不要重启它(只影响这一站)。"""
        lived = time.time() - st.started_at
        st.proc = None
        if rc == 0:
            # 正常收工(取图源耗尽等)。产线相机模式下不该出现，所以留一条 WARNING。
            st.exited_ok = True
            self.log.warning("[PIPE] 站 %s 正常退出(rc=0, 存活 %.0fs), 不重启", st.id, lived)
            return
        if lived >= HEALTHY_RUN_S:
            st.consec_fail = 0  # 跑了很久才挂 = 单次故障，不计入"连续失败"
        st.consec_fail += 1
        st.restarts += 1
        if rc == 3:
            self.log.critical("[PIPE] 站 %s 停线(rc=3, 采集/链路故障)，存活 %.0fs；重启",
                              st.id, lived)
        elif st.consec_fail >= RESTART_ESCALATE_N:
            self.log.critical("[PIPE] ⚠ 站 %s 连续 %d 次启动失败(rc=%d)，需人工介入"
                              "(检查相机 IP/网线/脚本参数)", st.id, st.consec_fail, rc)
        else:
            self.log.error("[PIPE] 站 %s 退出(rc=%d, 存活 %.0fs)，重启", st.id, rc, lived)

        backoff = min(RESTART_BACKOFF_S * (2 ** (st.consec_fail - 1)), RESTART_BACKOFF_MAX_S)
        if self._stop.wait(backoff):  # 退避期间被要求停机就不再重启
            return
        self.log.info("[PIPE] 重启站 %s(第 %d 次, 退避 %.1fs)", st.id, st.restarts, backoff)
        self._spawn(st)

    def log_status(self) -> None:
        """一行式监督摘要(产线巡检用)。"""
        parts = []
        for st in self.stations:
            alive = "运行中" if (st.proc is not None and st.proc.poll() is None) else "已停止"
            parts.append("%s=%s(pid=%s,链路=%s,重启=%d,代触发NG=%d)"
                         % (st.id, alive,
                            st.proc.pid if st.proc is not None else "-",
                            "通" if st.link_up else "断", st.restarts, st.ng_relayed))
        self.log.info("[PIPE-STATUS] %s", " | ".join(parts))

    # ---------- 停机 ----------
    def stop(self) -> None:
        """停机：先把子进程收掉，再关串口(留足最后一个脉冲的时间)。"""
        self.log.info("[PIPE] 正在停机 ...")
        if self._server is not None:
            # BYE 会让各站的保活线程直接退出进程 —— 比等它们跑到帧边界快且确定
            self._server.close()
        for st in self.stations:
            self._terminate(st)
        if self._uno is not None:
            if self._uno.connected:
                self._uno.set_led(False)  # 停机=产线未运行: 立刻显式红灯(不等固件看门狗那 3s)
            time.sleep(UNO_DRAIN_S)  # 别把刚发出去的 50ms 脉冲掐断
            self._uno.close()
        if self._server is not None:
            self._server.close()
        self.log_status()
        self.log.info("[PIPE] 已停机")

    def _terminate(self, st: Station) -> None:
        """先 terminate，宽限期内不退再 kill(两站互不影响地各自收掉)。"""
        proc = st.proc
        if proc is None or proc.poll() is not None:
            return
        self.log.info("[PIPE] 停止站 %s (pid=%s)", st.id, proc.pid)
        try:
            proc.terminate()
            proc.wait(timeout=CHILD_TERM_GRACE_S)
        except subprocess.TimeoutExpired:
            self.log.warning("[PIPE] 站 %s 未在 %.0fs 内退出，强杀", st.id, CHILD_TERM_GRACE_S)
            proc.kill()
        except OSError as exc:
            self.log.warning("[PIPE] 停止站 %s 出错: %s", st.id, exc)
        st.proc = None

    @property
    def fatal_reason(self) -> str:
        return self._fatal_reason


# ============================  子进程入口  ============================
def _run_child(station_id: str, child_argv: List[str]) -> int:
    """以某个站的身份运行检测器：把它的 `uno.pulse()` 转发给主程序。

    **顺序很重要**：必须先自己握手成功，再进检测循环。因为 inspector_pure 里
    `uno.connect()` 失败只是打印一行 WARN 然后把 uno 置 None，检测照跑 —— 那就会出现
    "判了 NG 却触发不了执行器" = 逃逸。所以握不上就在这里直接非零退出，让主程序重启本站。
    """
    import station_link

    os.environ.setdefault(station_link.STATION_ENV, station_id)
    os.environ.setdefault(station_link.IPC_PORT_ENV, str(IPC_PORT))

    relay = station_link.IpcRelay()  # 进程内单例：与检测器随后那次构造共用同一条链路
    if not relay.connect():
        print("[PIPE][FATAL] 与主程序的上报链路建立失败，本站拒绝启动"
              "(fail-safe: 触发不了执行器的检测器绝不上线)", flush=True)
        return station_link.EXIT_LINK_LOST

    # 唯一执行点被转发：main() 里的 `from uno_relay import UnoRelayController` 是局部 import，
    # 所以在这里改模块属性就能让它拿到 IpcRelay —— inspector_pure.py 一行不用改。
    uno_relay.UnoRelayController = station_link.IpcRelay  # type: ignore[misc]

    # 常驻语义：相机没上电不是故障 —— 把 inspector_pure 的启动建链窗口撑到 ≈10 年, 子进程就会在
    # 窗口内一直重试连接而不退出。于是"先开的那台相机那一路先工作", 后开的那路等相机上电自然接上,
    # 主程序永不因"连不上相机"反复重启(churn)。中途掉线仍走有限次 CAM_RECONNECT_TRY 重连 -> 耗尽
    # 抛 AcquisitionError(rc=3) -> 主程序重启该站(此时 IPC 断, LED 如实转红, 绝不绿灯说谎)。
    # 只覆盖模块级常量, 基线一行不改；inspector_missing 复用同一个 Vn2000Source, 一并生效。
    import inspector_pure as _ip
    _ip.CAM_STARTUP_WAIT_S = CAM_RESIDENT_WAIT_S  # type: ignore[attr-defined]

    cfg = next((s for s in STATIONS if s["id"] == station_id), None)
    if cfg is None:
        print("[PIPE][FATAL] 未知站点 %r" % station_id, flush=True)
        return 2

    # 目录归拢: 与 inspector_missing 的 data/missing/* 对称, 把正反面站(inspector_pure)的
    # 日志/结果图也收进 data/pure/{logs,OK,NG,RAW}, 不再散在 data/ 根下。手法同 CAM 窗口:
    # 只覆盖 inspector_pure 的模块级目录常量(它们都是运行时才被 setup_runtime_logging/存图读取),
    # 基线一行不改。inspector_pure.main() 不会重设 RUNTIME_LOG_DIR, OK/NG/RAW 也只有传 --save-dir
    # 时才改(生产 argv 不传), 故这里的覆盖如实生效。inspector_missing 自己在 main() 里设 data/missing/*,
    # 故只对 inspector_pure 这一站做, 免得互相顶。
    if cfg["script"] == "inspector_pure.py":
        _pure_root = os.path.join(_ip.DEFAULT_DATA_DIR, "pure")
        _ip.RUNTIME_LOG_DIR = os.path.join(_pure_root, "logs")  # type: ignore[attr-defined]
        _ip.OK_SAVE_DIR = os.path.join(_pure_root, "OK")  # type: ignore[attr-defined]
        _ip.NG_SAVE_DIR = os.path.join(_pure_root, "NG")  # type: ignore[attr-defined]
        _ip.RAW_SAVE_DIR = os.path.join(_pure_root, "RAW")  # type: ignore[attr-defined]

    module = os.path.splitext(cfg["script"])[0]
    print("[PIPE] 本站=%s, 检测器=%s, 判定将经主程序触发执行器(引脚由主程序按站点决定)"
          % (station_id, module), flush=True)
    detector = importlib.import_module(module)
    return int(detector.main(child_argv))


# ============================  命令行  ============================
def parse_args(argv: Optional[List[str]] = None):
    """解析本站参数；**无法识别的参数原样透传**给各检测子进程(便于 L2 用 local 模式跑)。"""
    ap = argparse.ArgumentParser(
        description="产线主程序：独占 UNO + 看护两个检测站",
        epilog="未知参数会原样透传给各检测器，例如: %(prog)s --only front --mode local --dir samples --limit 20")
    ap.add_argument("--only", action="append", default=[], metavar="站名",
                    help="只跑指定站(可多次；默认全跑)")
    ap.add_argument("--dry-uno", action="store_true",
                    help="不连真串口，只打印该触发哪个引脚(无硬件时验链路)")
    ap.add_argument("--status-interval", type=float, default=STATUS_INTERVAL_S,
                    help="定期打印监督摘要的秒数(0=关，默认 %(default)s)")
    ap.add_argument("--exit-when-done", action="store_true",
                    help="所有站都 rc=0 正常收工时主程序即退出(离线跑样本用)")
    args, passthrough = ap.parse_known_args(argv)
    return args, passthrough


def main(argv: Optional[List[str]] = None) -> int:
    """入口。`--child <站名> [检测器参数...]` 是主程序自己 spawn 子进程用的内部模式。

    `--child` 必须在 argparse **之前**拦下：它后面跟的是检测器自己的参数，
    不能被本站的参数解析吃掉。
    """
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == "--child":
        if len(argv) < 2:
            print("[PIPE][FATAL] 用法: --child <站名> [检测器参数...]", flush=True)
            return 2
        return _run_child(argv[1], argv[2:])

    args, passthrough = parse_args(argv)
    log = setup_supervisor_logging()
    picked = [s for s in STATIONS if not args.only or s["id"] in args.only]
    unknown = [i for i in args.only if i not in {s["id"] for s in STATIONS}]
    if unknown:
        log.critical("[PIPE][FATAL] --only 里有未知站名: %s (可选: %s)",
                     unknown, [s["id"] for s in STATIONS])
        return 2
    if not picked:
        log.critical("[PIPE][FATAL] 没有可跑的站")
        return 2

    sup = Supervisor([Station(c) for c in picked], passthrough, dry_uno=args.dry_uno,
                     status_interval=args.status_interval,
                     exit_when_done=args.exit_when_done)
    rc = sup.start()
    if rc != 0:
        sup.stop()
        return rc

    stopping = threading.Event()

    def _on_signal(signum, _frame):
        if not stopping.is_set():
            stopping.set()
            log.info("[PIPE] 收到信号 %s，准备停机", signum)
            sup._stop.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _on_signal)
        except (ValueError, OSError, AttributeError):  # Windows 上部分信号不可用
            pass

    try:
        sup.monitor_loop()
    except KeyboardInterrupt:  # 兜底(信号处理器没装上时)
        log.info("[PIPE] 键盘中断，准备停机")
        sup._stop.set()
    finally:
        sup.stop()

    if sup.fatal_reason:
        log.critical("[PIPE] 因致命故障停机: %s", sup.fatal_reason)
        return 5
    return 0


if __name__ == "__main__":
    sys.exit(main())
