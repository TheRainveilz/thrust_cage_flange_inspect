#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Arduino UNO 自动控制模块。

与 uno_plc_trigger.ino 使用同一协议（固件为非阻塞短脉冲版）。**两路独立输出**：
    NG\n      D8 输出一个短触发脉冲(固件 PULSE_MS，默认 50ms)后自动回 LOW；固件立刻回 "NG" ACK
    NG2\n     D9 同上(第二站/缺粒站)；固件回 "NG2"
    OK\n      确保 D8/D9 都为 LOW，不触发 PLC；固件回 "OK"
    OK2\n     只把 D9 拉回 LOW；固件回 "OK2"
    STATUS\n  查询两路状态

D8 = 第一站(正反面翻边止口, 吹气把件从料道吹掉)，D9 = 第二站(兜孔缺粒, 开闸门放件掉进回收盒)。
两路各自计时、互不阻塞、互不影响。执行器动作(吹气/开闸)的延迟与时长由 PLC 自管，UNO 只负责给
PLC 一个干净的触发脉冲。
固件对 NG/OK 立即 ACK，send() 收到即返回，不再空等 SERIAL_TIMEOUT。
主程序只在最终判定 NG 时调用 pulse(pin)；OK 不发送指令。

⚠ 一条串口只能有一个主人：两个检测进程不能各自开同一个串口。产线上由主程序
(main_pipeline.py) 独占本模块并代为触发执行器，检测进程只上报判定(见 station_link.py)。
"""
from __future__ import annotations

import sys
import threading
import time
from typing import Optional

import serial
from serial.tools import list_ports


# ★ UNO 串口/引脚/波特率的唯一定义源(single source)。inspector_pure / inspector_cpp /
#   tools 都引用这里，别在别处再抄一份——换机器改 COM 口只改这三行。
#   注意: 固件 uno_plc_trigger.ino 里的 Serial.begin(115200) 是 Arduino C，无法 import，
#   BAUD_RATE 若改了，固件那行必须手动跟着改，否则串口乱码。
# COM_PORT 填具体串口(如 "COM7") = 显式优先，连不上会自动回退探测 CH340(自愈)；
#   填 "AUTO" = 直接自动探测 CH340 挂在哪个 COM。逻辑见 find_ch340_ports() / connect()。
COM_PORT = "Auto"
UNO_PIN = 8   # 第一站: 正反面翻边止口(保持原值, 向后兼容)
UNO_PIN2 = 9  # 第二站: 兜孔缺粒
BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.20
UNO_RESET_WAIT = 1.0
# 注意: 触发脉冲的实际宽度只由固件 uno_plc_trigger.ino 的 PULSE_MS(默认 50ms)决定，
# 主机侧无法调脉宽。故这里不再保留任何 pulse_seconds 参数, 免得误以为改它能改脉宽。

# CH340 自动探测: WCH CH340 的标准 USB VID:PID，及设备管理器/描述里的关键词。
CH340_VID_PID = (0x1A86, 0x7523)
CH340_DESC_KEY = "CH340"


def find_ch340_ports() -> list[str]:
    """扫描系统所有串口，返回像 CH340 的端口名列表(按 COM 号排序、去重)。
    匹配依据: USB VID:PID=1A86:7523，或 description/hwid 里含 'CH340'(大小写不敏感)。
    正常只插一块 UNO 时返回单元素列表；返回空表示没插或驱动未装。"""
    hits = []
    for p in list_ports.comports():
        blob = ((p.description or "") + " " + (p.hwid or "")).upper()
        if (p.vid == CH340_VID_PID[0] and p.pid == CH340_VID_PID[1]) \
                or CH340_DESC_KEY in blob:
            hits.append(p.device)
    return sorted(set(hits))


def _explain_serial_error(exc: Exception, port: str) -> str:
    """把 pyserial 的英文异常翻成现场能看懂的中文诊断+处置建议。"""
    text = str(exc)
    low = text.lower()
    # 端口被别的程序占用: Windows PermissionError / "拒绝访问" / Access is denied
    if "permission" in low or "access is denied" in low or "拒绝访问" in text:
        return (f"串口 {port} 被占用，已被其它程序打开。常见元凶: Arduino IDE 的串口监视器、"
                f"另一个本程序实例、或别的串口调试工具——把它们关掉再重试。")
    # 端口不存在: FileNotFoundError / "系统找不到指定的文件" / cannot find
    if "filenotfound" in low or "cannot find" in low or "no such" in low or "系统找不到" in text:
        detected = find_ch340_ports()
        if detected:
            hint = (f"但检测到 CH340 实际在 {detected}——把 uno_relay.py 的 COM_PORT 改成它，"
                    f"或设成 'AUTO' 让它自动连。")
        else:
            hint = ("也没探测到任何 CH340——检查 USB 是否插好、CH340 驱动是否已装、"
                    "或数据线是否只供电不传数据(换一根)。")
        return f"找不到串口 {port}: 该 COM 口不存在(没插好/COM 号变了/驱动未装)。{hint}"
    # 参数不被驱动接受(少见)
    if "could not configure" in low or "配置" in text:
        return f"串口 {port} 参数被驱动拒绝——核对 BAUD_RATE(应为 115200)。原始: {text}"
    return f"打开串口 {port} 失败。原始错误: {text}"


class UnoRelayController:
    """保持 UNO 串口连接，按 NG/OK 协议控制 D8。"""

    def __init__(
        self,
        port: str = COM_PORT,
        pin: int = UNO_PIN,
        baudrate: int = BAUD_RATE,
        timeout: float = SERIAL_TIMEOUT,
        reset_wait: float = UNO_RESET_WAIT,
    ) -> None:
        self.port = port
        self.pin = pin
        self.baudrate = baudrate
        self.timeout = timeout
        self.reset_wait = reset_wait
        self.ser: Optional[serial.Serial] = None
        # 串口是单一物理资源，但现在有多个线程会碰它：主程序的 verdict-conn 线程(两站各一个)
        # 调 pulse()，监控线程调 set_led() 喂 LED/看门狗。没有锁时它们的
        # reset_input_buffer()/write()/flush() 会交错，把两条命令搅在一起。用可重入锁把所有
        # 串口写串行化(RLock: send() 内部会再调 connect() 自愈重连，需可重入)。
        self._io_lock = threading.RLock()

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def _resolve_candidates(self) -> list[str]:
        """决定按什么顺序、试哪些串口:
        - port=="AUTO": 只用自动探测到的 CH340;
        - 否则: 显式端口优先，连不上再回退探测到的 CH340(自愈)。
        探测到多个 CH340 时全部列出并把第一个排进候选(告警)。"""
        detected = find_ch340_ports()
        if len(detected) > 1:
            print(f"[UNO][WARN] 探测到多个 CH340: {detected}; 将优先用第一个 {detected[0]}")
        explicit = None if (self.port or "").strip().upper() == "AUTO" else self.port
        order: list[str] = []
        if explicit:
            order.append(explicit)
        for dev in detected:               # 显式端口之后追加探测结果做兜底/自愈
            if dev not in order:
                order.append(dev)
        return order

    def connect(self) -> bool:
        with self._io_lock:
            if self.connected:
                return True
            candidates = self._resolve_candidates()
            if not candidates:
                print("[UNO][ERROR] 无可用串口: 未指定有效端口且未探测到 CH340(检查接线/驱动)")
                return False
            last_exc: Optional[Exception] = None
            for i, port in enumerate(candidates):
                try:
                    self.ser = serial.Serial(
                        port=port,
                        baudrate=self.baudrate,
                        timeout=self.timeout,
                        write_timeout=self.timeout,
                    )
                    time.sleep(self.reset_wait)
                    self.port = port           # 记住实际连上的端口
                    self._read_available()
                    if i > 0:
                        print(f"[UNO] 已自愈切换到 {port}(前一候选连不上)")
                    print(f"[UNO] connected: {port} @ {self.baudrate}, D{self.pin}")
                    return True
                except (serial.SerialException, OSError) as exc:
                    self.ser = None
                    last_exc = exc
                    if i + 1 < len(candidates):
                        print(f"[UNO][WARN] {port} 连不上，尝试下一候选 {candidates[i + 1]}")
            print(f"[UNO][ERROR] UNO 未连接: {_explain_serial_error(last_exc, port)}")
            return False

    def _read_available(self) -> None:
        if not self.connected:
            return
        while self.ser.in_waiting:
            line = self.ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(f"[UNO] {line}")

    def send(self, command: str) -> bool:
        """发送命令。写入成功即返回 True，不要求 UNO 必须返回日志。

        全程持 _io_lock：pulse()(多个 verdict 线程) 与 set_led()(监控线程) 会并发调这里，
        锁保证任一条命令的 reset/write/flush 三步不被另一条打断。
        """
        with self._io_lock:
            if not self.connected and not self.connect():
                return False
            command = command.strip().upper()
            if command not in {"NG", "NG2", "OK", "OK2", "STATUS", "HEALTHY", "FAULT"}:
                print(f"[UNO][ERROR] unsupported command: {command}")
                return False
            try:
                # 关键路径不等 ACK：脉冲在 write()+flush() 即发出，固件收到 NG 会立刻拉高 D8。
                # 读回执只是确认日志，且旧代码无论收没收到都 return True——空等最多 SERIAL_TIMEOUT
                # (0.20s) 却不改任何行为，反而把执行触发堵在单消费者主线程上，连累下一帧排队超时假 NG。
                # 故此处不再自旋等回执；上一条命令的 ACK 字节由下次调用开头的 reset_input_buffer() 清掉。
                # 真正的串口故障(端口断/写不进)仍由 write()/flush() 抛异常被下面 except 捕获 -> return False。
                self.ser.reset_input_buffer()
                self.ser.write((command + "\n").encode("ascii"))
                self.ser.flush()
                return True
            except (serial.SerialException, OSError) as exc:
                print(f"[UNO][ERROR] command {command} failed: {exc}")
                return False

    def pulse(self, pin: int = UNO_PIN) -> bool:
        """给指定引脚发一次 NG 脉冲；由 Arduino 输出短触发脉冲(固件 PULSE_MS)并自动回 LOW。

        pin=UNO_PIN(D8) -> 发 "NG"；pin=UNO_PIN2(D9) -> 发 "NG2"。
        默认 UNO_PIN, 老调用方(tools/uno_manual_console.py)行为完全不变。
        传入其它引脚值一律拒绝(fail-safe)：绝不把判定悄悄触发到错的那一路。
        """
        if int(pin) == UNO_PIN:
            cmd, pin_used = "NG", UNO_PIN
        elif int(pin) == UNO_PIN2:
            cmd, pin_used = "NG2", UNO_PIN2
        else:
            print(f"[UNO][ERROR] 未知引脚 {pin}: 只有 D{UNO_PIN} / D{UNO_PIN2} 两路输出，拒绝发送")
            return False
        print(f"[UNO] {cmd} -> D{pin_used} 触发脉冲(脉宽由固件 PULSE_MS 控制)")
        return self.send(cmd)

    def off(self) -> bool:
        """用 OK 命令确保 D8/D9 两路都回到 LOW。"""
        return self.send("OK")

    def set_led(self, healthy: bool) -> bool:
        """驱动状态 LED：healthy=True 发 "HEALTHY"(绿灯常亮)，False 发 "FAULT"(红灯闪烁)。

        由主程序(main_pipeline)的监控线程每 <=1s 调一次：既刷新固件看门狗(证明主程序活着)，
        又如实反映"两台相机 + UNO 是否全部正常"。发送失败(串口断)返回 False，调用方按最佳努力处理
        —— LED 只是指示，真正的执行安全由 pulse() 那条路把关；且固件看门狗会在 3s 无指令时自动闪红。
        """
        return self.send("HEALTHY" if healthy else "FAULT")

    def status(self) -> bool:
        return self.send("STATUS")

    def close(self) -> None:
        with self._io_lock:
            if self.ser is None:
                return
            try:
                if self.ser.is_open:
                    self.off()
                    self.ser.close()
                    print("[UNO] serial closed")
            except (serial.SerialException, OSError) as exc:
                print(f"[UNO][WARN] serial close failed: {exc}")
            finally:
                self.ser = None


class _DryRelay(UnoRelayController):
    """不开串口的替身: 只把 send 换成打印, 其余走真实现。

    用途: 无硬件时验证「引脚 -> 命令」的拼装是否正确(pulse(UNO_PIN) 必须发 "NG",
    pulse(UNO_PIN2) 必须发 "NG2")。不碰串口, 产线机上也能安全跑。
    """

    @property
    def connected(self) -> bool:
        return True

    def connect(self) -> bool:
        return True

    def send(self, command: str) -> bool:
        print(f"[UNO][DRY-RUN] would send: {command}")
        return True

    def close(self) -> None:
        print("[UNO][DRY-RUN] closed")


def main() -> None:
    """独立检查(也是 L0' 两引脚 + 双色 LED 验证)：
        STATUS -> NG(D8) -> STATUS -> NG2(D9) -> STATUS -> LED 三段测试。

    现场接 LED/万用表时，应看到 D8 先闪一下、再 D9 闪一下 —— 证明两路互不干扰；
    随后的 LED 三段应看到：绿灯常亮 -> (停发指令约 3s)看门狗自动转红闪 -> 显式红闪。
    加 --dry-run 则不碰串口，只打印将要发送的命令(无硬件时验证拼装)。

    ⚠ LED 段需要先用 Arduino IDE 烧录新版 uno_plc_trigger.ino(含 D10/D11)，否则旧固件会把
      HEALTHY/FAULT 当未知命令回 "ERROR"，D10/D11 不会亮。
    """
    if "--dry-run" in sys.argv[1:]:
        print("[UNO][DRY-RUN] 不实际开串口, 只打印将要发送的命令")
        controller: UnoRelayController = _DryRelay()
    else:
        controller = UnoRelayController()
        if not controller.connect():
            return
    dry = isinstance(controller, _DryRelay)
    try:
        controller.status()
        for pin, label in ((UNO_PIN, "D8 第一站/正反面"), (UNO_PIN2, "D9 第二站/缺粒")):
            print(f"[UNO] 测试 {label} ...")
            controller.pulse(pin)
            if not dry:
                time.sleep(0.2)  # 等固件 50ms 脉冲结束(留足余量)后再查 STATUS, 应见回到 LOW
                controller.status()

        # ---- 双色状态 LED 三段测试(D10 绿 / D11 红) ----
        print("\n[UNO] === LED 测试(请盯着 D10 绿灯 / D11 红灯) ===")
        print("[UNO] ① HEALTHY -> 绿灯应【常亮】约 2s ...")
        controller.set_led(True)
        if not dry:
            time.sleep(2.0)
        print("[UNO] ② 看门狗：现在停发任何指令约 4s，绿灯应在 ~3s 后【自动转红灯闪烁】")
        print("       (这一段证明：主程序若崩溃/串口被拔，UNO 会自己亮红，绿灯永不说谎)")
        if not dry:
            time.sleep(4.0)
            controller.status()  # 此时应回 LED=RED
        print("[UNO] ③ FAULT -> 红灯应【闪烁】约 3s ...")
        controller.set_led(False)
        if not dry:
            time.sleep(3.0)
        print("[UNO] === LED 测试结束；关串口后固件会因看门狗保持红闪(=产线未运行) ===\n")
    finally:
        controller.close()


if __name__ == "__main__":
    main()
