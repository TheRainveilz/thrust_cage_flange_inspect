
import serial
import time

# =========================
# Arduino 串口配置
# =========================
PORT = "COM6"
BAUDRATE = 9600

try:
    arduino = serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        timeout=1
    )

    # Arduino打开串口后通常会自动复位
    time.sleep(2)

    print("=" * 40)
    print("Arduino 串口测试程序")
    print(f"串口：{PORT}")
    print(f"波特率：{BAUDRATE}")
    print("=" * 40)

except Exception as e:
    print(f"打开串口失败：{e}")
    print("请检查：")
    print("1. Arduino是否连接")
    print("2. 是否确实是 COM6")
    print("3. Arduino IDE 的串口监视器是否已经关闭")
    input("按回车退出...")
    exit()

while True:
    print()
    print("请选择测试功能：")
    print("1. OK")
    print("2. NG")
    print("3. STATUS")
    print("4. 退出")

    choice = input("请输入：").strip()

    if choice == "1":
        arduino.write(b"OK\n")
        print("已发送：OK")
        print("Arduino：OUT_PIN = LOW，不触发 PLC")

    elif choice == "2":
        arduino.write(b"NG\n")
        print("已发送：NG")
        print("Arduino：OUT_PIN = HIGH，持续500ms后自动关闭")

        # 等待Arduino执行完500ms
        time.sleep(0.6)

    elif choice == "3":
        arduino.write(b"STATUS\n")

        response = arduino.readline().decode("utf-8", errors="ignore").strip()

        if response:
            print(f"Arduino返回：{response}")
        else:
            print("Arduino没有返回数据")

    elif choice == "4":
        print("退出测试程序")
        break

    else:
        print("输入错误，请输入 1、2、3 或 4")

arduino.close()
print("串口已关闭")
