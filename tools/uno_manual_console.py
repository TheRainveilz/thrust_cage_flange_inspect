# -*- coding: utf-8 -*-
"""UNO 手动控制台(调试用): 手动发 OK/NG/NG2/STATUS 给 Arduino UNO。

NG -> D8(第一站 吹气)、NG2 -> D9(第二站 开闸落料)，两路各自独立触发，用来现场逐路核对
光耦/PLC 接线是否对号入座(发 NG 只该第一站动、发 NG2 只该第二站动)。

动作全部委托给 uno_relay.UnoRelayController, 因此自动继承它的自愈能力:
CH340 自动探测、显式端口连不上时回退到探测口、send 时断线自动重连、中文错误诊断,
以及 COM_PORT='AUTO' 的自动选口。不再自己裸开 serial.Serial —— 那样 COM 口一变或被
占用就直接失败, 且 'AUTO' 会被当成真串口名报错。串口/波特率仍以 uno_relay.py 为唯一定义源。
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "flange_inspect"))
from uno_relay import UNO_PIN, UNO_PIN2, UnoRelayController

ctrl = UnoRelayController()
if not ctrl.connect():
    # connect() 已打印具体中文诊断(端口占用/不存在/探测到的 CH340 等), 这里不再重复
    input("按回车退出...")
    sys.exit(1)

print("=" * 40)
print("Arduino UNO 手动控制台")
print(f"串口：{ctrl.port}")        # connect 后是实际连上的端口(可能已自愈切换)
print(f"波特率：{ctrl.baudrate}")
print("=" * 40)

try:
    while True:
        print()
        print("请选择测试功能：")
        print("1. OK      (D8/D9 都保持 LOW, 不触发 PLC)")
        print("2. NG      (D8 输出触发脉冲 -> 第一站 吹气；脉宽由固件 PULSE_MS 决定)")
        print("3. NG2     (D9 输出触发脉冲 -> 第二站 开闸落料；脉宽由固件 PULSE_MS 决定)")
        print("4. STATUS  (查询 D8/D9 状态)")
        print("5. 退出")

        choice = input("请输入：").strip()

        if choice == "1":
            print("已发送：OK" if ctrl.off() else "发送失败：OK(串口异常, 见上方日志)")

        elif choice == "2":
            ok = ctrl.pulse(UNO_PIN)   # -> NG，D8 一个短脉冲
            print("已发送：NG (D8)" if ok else "发送失败：NG(串口异常, 见上方日志)")
            time.sleep(0.2)  # 等固件脉冲(PULSE_MS)结束再回菜单

        elif choice == "3":
            ok = ctrl.pulse(UNO_PIN2)  # -> NG2，D9 一个短脉冲
            print("已发送：NG2 (D9)" if ok else "发送失败：NG2(串口异常, 见上方日志)")
            time.sleep(0.2)

        elif choice == "4":
            if ctrl.status():
                time.sleep(0.15)  # 给固件回执到达的时间
                resp = (ctrl.ser.readline().decode("utf-8", errors="ignore").strip()
                        if ctrl.ser else "")
                print(f"Arduino返回：{resp}" if resp else "Arduino没有返回数据")
            else:
                print("发送失败：STATUS(串口异常, 见上方日志)")

        elif choice == "5":
            print("退出测试程序")
            break

        else:
            print("输入错误，请输入 1、2、3、4 或 5")
finally:
    ctrl.close()  # 内部会先发 OK 确保 D8/D9 回 LOW 再关串口
    print("串口已关闭")
