// uno_plc_trigger.ino —— 高速非阻塞版
//
// 接线不变：UNO D8 -> 光耦隔离器 -> 给 PLC 一个触发信号；
//   电磁阀吹气的“延迟(0.01s)/踢除(0.09s)”由 PLC 自己控制。
//   所以 UNO 只需给 PLC 一个短促、干净的“触发脉冲”，绝不能用 delay() 阻塞串口，
//   否则脉冲保持期间收不到下一件的 NG，会错吹到后件。
//
// 极性沿用原版：D8 = HIGH 触发，LOW 空闲(OK)。上线前请在现场核实：
//   - 光耦这端到底是 HIGH 还是 LOW 才让 PLC 侧得到 0V；若相反，只改 setup/触发的电平。
//   - PULSE_MS 只要 > PLC 扫描周期能被稳定锁存即可；PLC 自管 90ms 踢除，无需 UNO 长保持。

const int OUT_PIN = 8;
const unsigned long PULSE_MS = 50;  // 触发脉冲宽度(ms)：够 PLC 锁存即可，按现场实测可调小/调大

bool pulsing = false;              // 当前是否在输出触发脉冲
unsigned long pulseStart = 0;      // 本次脉冲起点(millis)
bool ngState = false;              // 供 STATUS 查询

void setup() {
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);      // 默认 OK：不触发
  Serial.begin(115200);            // 与上位机 测试2.py BAUD_RATE 必须一致，否则串口乱码
  Serial.setTimeout(20);           // 命令都带 '\n'，限制 readStringUntil 最坏等待
}

void loop() {
  // 1) 非阻塞收命令：任何时刻都能立即响应，不会被脉冲保持挡住
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "NG") {
      digitalWrite(OUT_PIN, HIGH);   // 触发；若脉冲期间又来 NG，则重新计时(重新武装)
      pulseStart = millis();
      pulsing = true;
      ngState = true;
      Serial.println("NG");          // 立刻回 ACK：上位机 send() 无需空等 200ms 超时
    }
    else if (cmd == "OK") {
      digitalWrite(OUT_PIN, LOW);
      pulsing = false;
      ngState = false;
      Serial.println("OK");
    }
    else if (cmd == "STATUS") {
      Serial.println(ngState ? "NG" : "OK");
    }
    else {
      Serial.println("ERROR");
    }
  }

  // 2) 非阻塞收回脉冲：到时自动回 LOW，期间串口照常响应
  if (pulsing && (millis() - pulseStart >= PULSE_MS)) {
    digitalWrite(OUT_PIN, LOW);
    pulsing = false;
    ngState = false;
  }
}
