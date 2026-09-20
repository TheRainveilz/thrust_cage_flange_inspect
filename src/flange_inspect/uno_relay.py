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


COM_PORT = "COM7"
UNO_PIN = 8
BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.20
UNO_RESET_WAIT = 1.0
NG_PULSE_SECONDS = 0.50


class UnoRelayController:
    """保持 UNO 串口连接，按 NG/OK 协议控制 D8。"""

    def __init__(
        self,
        port: str = COM_PORT,
        pin: int = UNO_PIN,
        baudrate: int = BAUD_RATE,
        timeout: float = SERIAL_TIMEOUT,
        reset_wait: float = UNO_RESET_WAIT,
        pulse_seconds: float = NG_PULSE_SECONDS,
    ) -> None:
        self.port = port
        self.pin = pin
        self.baudrate = baudrate
        self.timeout = timeout
        self.reset_wait = reset_wait
        self.pulse_seconds = pulse_seconds
        self.ser: Optional[serial.Serial] = None

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def connect(self) -> bool:
        if self.connected:
            return True
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout,
            )
            time.sleep(self.reset_wait)
            self._read_available()
            print(f"[UNO] connected: {self.port} @ {self.baudrate}, D{self.pin}")
            return True
        except (serial.SerialException, OSError) as exc:
            self.ser = None
            print(f"[UNO][ERROR] serial open failed: {exc}")
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
            self.ser.reset_input_buffer()
            self.ser.write((command + "\n").encode("ascii"))
            self.ser.flush()
            deadline = time.monotonic() + self.timeout
            got_response = False
            while time.monotonic() < deadline:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        print(f"[UNO] {line}")
                        got_response = True
                        break
                else:
                    time.sleep(0.005)
            if not got_response:
                print(f"[UNO][WARN] no response for {command}")
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
        time.sleep(NG_PULSE_SECONDS + 0.1)
        controller.status()
    finally:
        controller.close()


if __name__ == "__main__":
    main()
