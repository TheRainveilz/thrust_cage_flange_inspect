# -*- coding: utf-8 -*-
"""检测站 → 主程序的判定上报通道(localhost TCP + 行协议)。

**为什么需要它**：一块 UNO 只有一条串口，而两个检测进程都要发 NG。串口只能有一个"主人"，
所以产线上由主程序(main_pipeline.py)独占串口并负责触发执行器(D8 吹气/D9 开闸)，两个检测进程只判定、把 NG 上报过来
(背景见 uno_relay.py 顶部注释)。

协议(一行一条, '\\n' 结尾, ASCII)：

    子→主   HELLO <station>    建链握手；主程序据此把这条连接绑定到某一站/某个引脚
    子→主   NG                 这一件判 NG，请触发该站对应的引脚(吹气或开闸)
    子→主   PING               存活(每 PING_INTERVAL_S 一次)
    主→子   READY <station>    握手确认
    主→子   PONG               存活应答

**失败语义(安全设计的核心)**：判定→执行这条链路一旦不可用，绝不能让检测继续跑下去 ——
那样 NG 件会直接流过去(逃逸，唯一不可接受的方向)。所以：

  · `IpcRelay.pulse()` 发送失败     → 抛 inspector_pure.AcquisitionError，
    主循环的 `except AcquisitionError` 接住并干净停线(exit 3)；
  · 保活线程 PONG 超时 / 发送失败   → 打 CRITICAL 日志后 `os._exit(3)` 立即停该站
    (不能从子线程 raise 到主线程，故直接退出进程；日志 handler 每行都 flush，不会丢)。

两种结局都是 **exit 3**，主程序据此重启该站，且另一站完全不受影响。
"""
from __future__ import annotations

import logging
import os
import socket
import sys
import threading
import time
from typing import Callable, Optional

# ============================  参 数 区  ============================
IPC_HOST = "127.0.0.1"
IPC_PORT = 47653  # 主程序监听端口；改端口只改这里(主程序与子进程共用本模块)
IPC_PORT_ENV = "IPC_PORT"  # 子进程侧覆盖端口(主程序 spawn 时注入环境变量)
STATION_ENV = "STATION_ID"  # 子进程侧站点 id(主程序 spawn 时注入)
IPC_CONNECT_TIMEOUT_S = 5.0  # 子进程连主程序的上限
IPC_READY_TIMEOUT_S = 5.0  # 子进程等 READY 的上限(握手不成功=链路不可信, 不进入检测)
IPC_BACKLOG = 8
PING_INTERVAL_S = 1.0  # 子进程发 PING 的周期
PONG_TIMEOUT_S = 3.0  # 超过该时间没等到 PONG -> 认定链路已死 -> 停该站
READ_CHUNK = 4096
EXIT_LINK_LOST = 3  # 与 inspector_pure 的 AcquisitionError 出口码一致(主程序按"停线"处理)
RUNTIME_LOGGER_NAME = "thrust_cage_inspect"  # 按名字取, 拿到 inspector_pure 配好的同一个 logger
#   子进程里 inspector_pure.setup_runtime_logging() 已给这个名字挂好文件 handler,
#   故本模块无需 import inspector_pure 就能把日志写进同一个 .log；主程序侧则传入自己的 logger。


class LinkLost(RuntimeError):
    """上报链路不可用(兜底异常)。

    子进程里实际抛的是 inspector_pure.AcquisitionError(见 _fatal_error_class)，
    这样 inspector_pure.main() 的 `except AcquisitionError` 才能接住并干净停线。
    """


def _fatal_error_class() -> type:
    """返回能让 inspector_pure.main() 转成"停线(exit 3)"的异常类。

    IpcRelay 只在子进程里用，那时 inspector_pure 必然已经 import 过(sys.modules 里有)，
    直接取它的 AcquisitionError 即可。主程序(监督者)不构造 IpcRelay，取不到就退回 LinkLost。
    """
    ip = sys.modules.get("inspector_pure")
    return getattr(ip, "AcquisitionError", LinkLost) if ip is not None else LinkLost


def _runtime_logger() -> logging.Logger:
    """按名字取运行日志(子进程里 = inspector_pure 配好的那个；取不到则是个静默 logger)。"""
    return logging.getLogger(RUNTIME_LOGGER_NAME)


def _env_int(name: str, default: int) -> int:
    """读整型环境变量；缺失或非法则用默认值。"""
    raw = os.environ.get(name)
    if not raw:
        return default
    try:
        return int(raw)
    except ValueError:
        return default


class _LineConn:
    """把一条 TCP 连接包成"按行收发"，收发各自加锁(收发可能在不同线程)。"""

    def __init__(self, sock: socket.socket, peer: str) -> None:
        self.sock = sock
        self.peer = peer
        self._buf = bytearray()
        self._send_lock = threading.Lock()

    def send_line(self, text: str) -> None:
        """发一行(自动补 '\\n')；失败抛 OSError。"""
        with self._send_lock:
            self.sock.sendall((text + "\n").encode("ascii"))

    def read_line(self, timeout: float) -> Optional[str]:
        """读一行(不含 '\\n')；超时返回 None；对端关闭抛 ConnectionError。"""
        deadline = time.monotonic() + timeout
        while True:
            nl = self._buf.find(b"\n")
            if nl >= 0:
                line = bytes(self._buf[:nl])
                del self._buf[:nl + 1]
                return line.decode("ascii", errors="replace").strip()
            remain = deadline - time.monotonic()
            if remain <= 0:
                return None
            self.sock.settimeout(max(0.05, remain))
            try:
                chunk = self.sock.recv(READ_CHUNK)
            except (TimeoutError, socket.timeout):
                # recv 超时抛的是 OSError 子类，必须在这里吃掉：否则上层会把"暂时没数据"
                # 误判成"链路断开"，触发 fail-safe 误停站。
                continue
            if not chunk:
                raise ConnectionError("对端已关闭连接")
            self._buf.extend(chunk)

    def close(self) -> None:
        """静默关闭(不抛异常)。"""
        try:
            self.sock.close()
        except OSError:
            pass


class VerdictServer:
    """主程序侧：监听 localhost，收各站上报的 NG，回调 on_ng(station)。

    只做"收 + 回调"，不认识引脚 —— 站点→引脚的映射由主程序自己决定(见 main_pipeline.STATIONS)。
    """

    def __init__(self, on_ng: Callable[[str], None], host: str = IPC_HOST,
                 port: int = IPC_PORT, logger: Optional[logging.Logger] = None,
                 on_link_lost: Optional[Callable[[str], None]] = None) -> None:
        self.on_ng = on_ng
        self.host = host
        self.port = port
        self.log = logger or _runtime_logger()
        self.on_link_lost = on_link_lost
        self._srv: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._stations: dict[str, _LineConn] = {}
        self._conns: list[socket.socket] = []

    # ---------- 生命周期 ----------
    def listen(self) -> int:
        """开始监听并起 accept 线程；返回实际端口(port=0 时由内核分配, 便于自测)。"""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(IPC_BACKLOG)
        srv.settimeout(0.5)
        self.port = int(srv.getsockname()[1])
        self._srv = srv
        self._thread = threading.Thread(target=self._accept_loop, name="verdict-server",
                                        daemon=True)
        self._thread.start()
        self.log.info("[IPC] 判定上报通道已监听 %s:%d", self.host, self.port)
        return self.port

    def close(self) -> None:
        """停机：通知各站、关所有连接与监听口。"""
        self._stop.set()
        with self._lock:
            conns = list(self._stations.values())
            self._stations.clear()
        for c in conns:
            try:
                c.send_line("BYE")
            except OSError:
                pass
            c.close()
        for s in self._conns:
            try:
                s.close()
            except OSError:
                pass
        self._conns.clear()
        if self._srv is not None:
            try:
                self._srv.close()
            except OSError:
                pass
            self._srv = None

    # ---------- 内部 ----------
    def _accept_loop(self) -> None:
        """accept 循环：每条连接起一个 handler 线程。"""
        while not self._stop.is_set():
            assert self._srv is not None
            try:
                sock, addr = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break  # 监听口被 close()
            sock.settimeout(None)
            self._conns.append(sock)
            threading.Thread(target=self._handle, args=(sock, addr), name="verdict-conn",
                             daemon=True).start()

    def _handle(self, sock: socket.socket, addr) -> None:
        """处理一条站连接：HELLO 握手 -> 循环收 NG/PING。"""
        conn = _LineConn(sock, "%s:%d" % addr)
        station = self._handshake(conn)
        if station is None:
            conn.close()
            return
        try:
            graceful = False
            while not self._stop.is_set():
                line = conn.read_line(0.5)
                if line is None or line == "":
                    continue
                if line == "NG":
                    self.on_ng(station)
                elif line == "PING":
                    conn.send_line("PONG")
                elif line == "BYE":
                    graceful = True  # 子进程正常收工退出, 不是链路故障
                    break
                else:
                    self.log.warning("[IPC] 站 %s 收到未知消息: %r", station, line)
        except (ConnectionError, OSError) as exc:
            graceful = False
            if not self._stop.is_set():
                self.log.warning("[IPC] 站 %s 连接断开: %s", station, exc)
        finally:
            conn.close()
            with self._lock:
                if self._stations.get(station) is conn:
                    self._stations.pop(station, None)
            # 只有"非正常断开且不是我们自己停机"才算链路丢失(主程序据此判断要不要重启该站)
            if not graceful and not self._stop.is_set() and self.on_link_lost is not None:
                self.on_link_lost(station)

    def _handshake(self, conn: _LineConn) -> Optional[str]:
        """等 HELLO <station>，登记并回 READY <station>；超时/非法返回 None。"""
        try:
            line = conn.read_line(IPC_READY_TIMEOUT_S)
        except (ConnectionError, OSError):
            return None
        if not line or not line.startswith("HELLO "):
            self.log.warning("[IPC] 来自 %s 的连接握手失败(首行=%r)", conn.peer, line)
            return None
        station = line.split(None, 1)[1].strip()
        with self._lock:
            old = self._stations.get(station)
            self._stations[station] = conn
        if old is not None:
            self.log.warning("[IPC] 站 %s 重复建链，旧连接作废", station)
            old.close()
        try:
            conn.send_line("READY %s" % station)
        except OSError as exc:
            self.log.warning("[IPC] 站 %s 回 READY 失败: %s", station, exc)
            return None
        self.log.info("[IPC] 站 %s 已接入(来自 %s)", station, conn.peer)
        return station

    def is_connected(self, station: str) -> bool:
        """该站当前是否有活连接(主程序用来判断"绝不能起一个触发不了执行器的检测器")。"""
        with self._lock:
            return station in self._stations


class IpcRelay:
    """子进程侧：鸭子类型完全对齐 uno_relay.UnoRelayController，由 main() **无参构造**。

    站点 id 与端口来自环境变量(STATION_ID / IPC_PORT，主程序 spawn 时注入) ——
    inspector_pure.main() 里是 `UnoRelayController()` 无参构造，没有地方传参。

    与真串口 UNO 的唯一区别：`pulse()` 不驱动引脚，而是把 "NG" 发给主程序，
    由主程序在自己那一侧的串口上 pulse() 对应引脚(站点→引脚由主程序决定)。

    **单例**：一个子进程 = 一个站 = 一条链路，所以子进程内 `IpcRelay()` 永远返回同一个实例。
    这一点是必需的：子进程要在进检测循环**之前**先握手(握不上就拒绝启动，见 main_pipeline
    的 `_run_child`)，而 `inspector_pure.main()` 随后还会自己 `UnoRelayController()` 一次；
    若不是同一个实例，那第二次就会再发一次 HELLO，把刚握好的连接顶掉。
    """

    _shared: Optional["IpcRelay"] = None

    def __new__(cls) -> "IpcRelay":
        """返回进程内唯一实例(见类文档的"单例")。"""
        if cls._shared is None:
            inst = super().__new__(cls)
            inst._ready = False  # type: ignore[attr-defined]
            cls._shared = inst
        return cls._shared

    def __init__(self) -> None:
        if getattr(self, "_ready", False):
            return  # 单例: 重复构造不得重置已有连接
        self._ready = True
        self.station = (os.environ.get(STATION_ENV) or "").strip()
        self.port = _env_int(IPC_PORT_ENV, IPC_PORT)
        self.host = IPC_HOST
        self.log = _runtime_logger()
        self._conn: Optional[_LineConn] = None
        self._ping_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._pong_at = 0.0
        self.ng_sent = 0  # 本进程已上报的 NG 数(停机对账用)

    # ---------- 与 UnoRelayController 对齐的接口 ----------
    @property
    def connected(self) -> bool:
        return self._conn is not None

    def connect(self) -> bool:
        """连主程序并完成 HELLO/READY 握手；起保活线程。失败返回 False(调用方必须当致命处理)。"""
        if self.connected:
            return True
        if not self.station:
            self.log.error("[IPC] 未设置环境变量 %s，无法确定本站身份", STATION_ENV)
            return False
        try:
            sock = socket.create_connection((self.host, self.port),
                                            timeout=IPC_CONNECT_TIMEOUT_S)
        except OSError as exc:
            self.log.error("[IPC] 连不上主程序 %s:%d: %s", self.host, self.port, exc)
            return False
        conn = _LineConn(sock, "%s:%d" % (self.host, self.port))
        try:
            conn.send_line("HELLO %s" % self.station)
            ready = conn.read_line(IPC_READY_TIMEOUT_S)
        except (ConnectionError, OSError) as exc:
            conn.close()
            self.log.error("[IPC] 与主程序握手失败: %s", exc)
            return False
        if ready != "READY %s" % self.station:
            conn.close()
            self.log.error("[IPC] 主程序握手应答异常(收到 %r, 期望 'READY %s')",
                           ready, self.station)
            return False
        self._conn = conn
        self._pong_at = time.monotonic()
        self._stop.clear()
        self._ping_thread = threading.Thread(target=self._ping_loop, name="ipc-ping",
                                             daemon=True)
        self._ping_thread.start()
        self.log.info("[IPC] 已接上主程序 %s:%d，本站=%s(判定经主程序触发执行器)",
                      self.host, self.port, self.station)
        return True

    def pulse(self, pin: Optional[int] = None) -> bool:
        """上报一次 NG。pin 参数只为对齐 UnoRelayController 的签名，实际引脚由主程序按站点决定。

        发送失败 -> 抛 AcquisitionError 让主循环干净停线(绝不能继续判定却触发不了执行器)。
        """
        conn = self._conn
        if conn is None:
            self._fatal("上报链路未建立，无法触发执行器")
        try:
            conn.send_line("NG")  # type: ignore[union-attr]
        except OSError as exc:
            self._conn = None
            self._fatal("上报 NG 失败(主程序可能已退出): %s" % exc)
        self.ng_sent += 1
        return True

    def off(self) -> bool:
        """无引脚可复位(主程序侧自管脉冲)，恒 True。"""
        return True

    def status(self) -> bool:
        """无状态可查，恒 True。"""
        return True

    def close(self) -> None:
        """停机：停保活线程并关连接(静默，不抛异常)。"""
        self._stop.set()
        conn, self._conn = self._conn, None
        if conn is not None:
            try:
                conn.send_line("BYE")
            except OSError:
                pass
            conn.close()

    # ---------- 内部 ----------
    def _fatal(self, why: str) -> None:
        """链路不可用 -> 抛致命异常停线(main() 的 except 会接住并 exit 3)。"""
        self.log.critical("[IPC-FATAL] %s；本站停线(fail-safe: 触发不了执行器就不许继续判)", why)
        raise _fatal_error_class()(why)

    def _ping_loop(self) -> None:
        """保活：定期发 PING 并要求 PONG。超时即认定链路死 -> 立即停该站。"""
        while not self._stop.wait(PING_INTERVAL_S):
            conn = self._conn
            if conn is None:
                return
            try:
                conn.send_line("PING")
                deadline = time.monotonic() + PONG_TIMEOUT_S
                while True:
                    line = conn.read_line(max(0.05, deadline - time.monotonic()))
                    if line is None:
                        break  # 超时
                    if line == "PONG":
                        self._pong_at = time.monotonic()
                        break
                    if line == "BYE":
                        self._stop_station("主程序已停机(BYE)", fatal=False)
                        return
            except (ConnectionError, OSError) as exc:
                self._stop_station("保活链路断开: %s" % exc)
                return
            if time.monotonic() - self._pong_at > PONG_TIMEOUT_S:
                self._stop_station("%.1fs 内未收到 PONG" % PONG_TIMEOUT_S)
                return

    def _stop_station(self, why: str, fatal: bool = True) -> None:
        """从子线程停站：不能 raise 到主线程，故打日志后直接退出进程(exit 3)。

        日志 handler 每行都 flush，所以 os._exit 不会丢掉已写内容。
        fatal=False 用于主程序主动停机(BYE)：一样退，但日志措辞不诬告成链路故障。
        """
        if fatal:
            self.log.critical("[IPC-FATAL] %s；本站停线(fail-safe: 判定无法送达主程序)", why)
        else:
            self.log.critical("[IPC] %s，本站随之退出", why)
        try:
            sys.stderr.flush()
        except Exception:  # noqa: BLE001
            pass
        os._exit(EXIT_LINK_LOST)


# ============================  自测  ============================
def _selftest() -> int:
    """L0 验证：起一个 VerdictServer，再用两个子进程各发几次 NG，检查回调收到的站点与次数。

    用法: python station_link.py            # 打印替身, 不碰硬件
          python station_link.py --real-uno # 真 UNO, 收到哪个站的 NG 就触发哪个引脚
    """
    import argparse
    import subprocess

    ap = argparse.ArgumentParser(description="station_link L0 自测")
    ap.add_argument("--real-uno", action="store_true",
                    help="用真 UNO 触发执行器(默认只打印替身, 产线机上也能安全跑)")
    ap.add_argument("--rounds", type=int, default=3, help="每站发几次 NG")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    log = logging.getLogger("station_link-selftest")

    uno = None
    if args.real_uno:
        from uno_relay import UNO_PIN, UNO_PIN2, UnoRelayController
        uno = UnoRelayController()
        if not uno.connect():
            print("[FATAL] UNO 未连接")
            return 2
        pins = {"front": UNO_PIN, "missing": UNO_PIN2}
    else:
        pins = {"front": 8, "missing": 9}

    got: dict[str, int] = {}

    def on_ng(station: str) -> None:
        got[station] = got.get(station, 0) + 1
        if uno is not None:
            ok = uno.pulse(pins[station])
            print("[SELFTEST] 站 %s -> D%d pulse=%s" % (station, pins[station], ok))
        else:
            print("[SELFTEST] 站 %s -> D%d (打印替身, 未接硬件)" % (station, pins[station]))

    srv = VerdictServer(on_ng, port=0, logger=log)
    port = srv.listen()
    print("[SELFTEST] 端口 %d，起两个假子进程..." % port)

    child = (
        "import os,sys,time;"
        "sys.path.insert(0, os.path.dirname(os.path.abspath(%r)));"
        "from station_link import IpcRelay;"
        "r=IpcRelay();"
        "assert r.connect(), 'IPC 连接失败';"
        "[r.pulse() for _ in range(%d)];"
        "time.sleep(1.5);"
        "r.close()"
    )
    procs = []
    for station in ("front", "missing"):
        env = dict(os.environ, **{STATION_ENV: station, IPC_PORT_ENV: str(port)})
        procs.append(subprocess.Popen([sys.executable, "-c", child % (os.path.dirname(
            os.path.abspath(__file__)), args.rounds)], env=env))
    for p in procs:
        p.wait(timeout=30)
    srv.close()
    if uno is not None:
        uno.close()

    ok = all(got.get(s) == args.rounds for s in ("front", "missing"))
    print("[SELFTEST] 收到: %s  期望每站 %d 次 -> %s"
          % (got, args.rounds, "通过" if ok else "失败"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(_selftest())
