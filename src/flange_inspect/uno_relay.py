#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Arduino UNO 自动控制模块。

与 uno_plc_trigger.ino 使用同一协议（固件为非阻塞短脉冲版）：
    NG\n      D8 输出一个短触发脉冲(固件 PULSE_MS，默认 50ms)后自动回 LOW；固件立刻回 "NG" ACK
    OK\n      确保 D8 为 LOW，不触发 PLC；固件回 "OK"
    STATUS\n  查询 D8 状态

电磁阀吹气的延迟/踢除时间由 PLC 自管，UNO 只负责给 PLC 一个干净的触发脉冲。
固件对 NG/OK 立即 ACK，send() 收到即返回，不再空等 SERIAL_TIMEOUT。
主程序只在最终判定 NG 时调用 pulse()；OK 不发送指令。
"""
from __future__ import annotations

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
COM_PORT = "COM7"
UNO_PIN = 8
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
        """发送命令。写入成功即返回 True，不要求 UNO 必须返回日志。"""
        if not self.connected and not self.connect():
            return False
        command = command.strip().upper()
        if command not in {"NG", "OK", "STATUS"}:
            print(f"[UNO][ERROR] unsupported command: {command}")
            return False
        try:
            # 关键路径不等 ACK：脉冲在 write()+flush() 即发出，固件收到 NG 会立刻拉高 D8。
            # 读回执只是确认日志，且旧代码无论收没收到都 return True——空等最多 SERIAL_TIMEOUT
            # (0.20s) 却不改任何行为，反而把吹气堵在单消费者主线程上，连累下一帧排队超时假 NG。
            # 故此处不再自旋等回执；上一条命令的 ACK 字节由下次调用开头的 reset_input_buffer() 清掉。
            # 真正的串口故障(端口断/写不进)仍由 write()/flush() 抛异常被下面 except 捕获 -> return False。
            self.ser.reset_input_buffer()
            self.ser.write((command + "\n").encode("ascii"))
            self.ser.flush()
            return True
        except (serial.SerialException, OSError) as exc:
            print(f"[UNO][ERROR] command {command} failed: {exc}")
            return False

    def pulse(self) -> bool:
        """发送一次 NG 指令；由 Arduino 输出短触发脉冲(固件 PULSE_MS)并自动回 LOW。"""
        print(f"[UNO] NG -> D{self.pin} 触发脉冲(脉宽由固件 PULSE_MS 控制)")
        return self.send("NG")

    def off(self) -> bool:
        """用 OK 命令确保 D8 回到 LOW。"""
        return self.send("OK")

    def status(self) -> bool:
        return self.send("STATUS")

    def close(self) -> None:
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


def main() -> None:
    """独立检查：STATUS -> NG -> STATUS。"""
    controller = UnoRelayController()
    if not controller.connect():
        return
    try:
        controller.status()
        controller.pulse()
        time.sleep(0.2)  # 等固件 50ms 脉冲结束(留足余量)后再查 STATUS，应见回到 LOW
        controller.status()
    finally:
        controller.close()


if __name__ == "__main__":
    main()
