// uno_plc_trigger.ino —— 高速非阻塞版 / 双路输出
//
// 接线：UNO D8 -> 光耦隔离器 -> PLC(第一站：正反面翻边止口)
//       UNO D9 -> 光耦隔离器 -> PLC(第二站：兜孔缺粒)
//   D8/D9 各自独立，互不阻塞、互不影响；两站可以同时吹、也可以只吹一路。
//   电磁阀吹气的“延迟(0.01s)/踢除(0.09s)”由 PLC 自己控制。
//   所以 UNO 只需给 PLC 一个短促、干净的“触发脉冲”，绝不能用 delay() 阻塞串口，
//   否则脉冲保持期间收不到下一件的 NG，会错吹到后件。
//
// 串口命令(与上位机 uno_relay.py 严格一致，全是大写、'\n' 结尾)：
//   NG        D8 拉高一个 PULSE_MS 脉冲后自动回 LOW；立刻回 "NG"
//   NG2       D9 同上；立刻回 "NG2"
//   OK        两路都拉回 LOW；回 "OK"
//   OK2       只把 D9 拉回 LOW；回 "OK2"
//   STATUS    回 "D8=<NG|LOW> D9=<NG|LOW>"
//   其它      回 "ERROR"
//
// 极性沿用原版：HIGH 触发，LOW 空闲(OK)。上线前请在现场核实：
//   - 光耦这端到底是 HIGH 还是 LOW 才让 PLC 侧得到 0V；若相反，只改 setup/触发的电平。
//   - PULSE_MS 只要 > PLC 扫描周期能被稳定锁存即可；PLC 自管 90ms 踢除，无需 UNO 长保持。
//
// ⚠ 改了这个固件必须重新烧录 UNO，上位机 uno_relay.py 才能用 NG2/OK2。

const int OUT_PIN  = 8;   // 第一站：正反面翻边止口
const int OUT_PIN2 = 9;   // 第二站：兜孔缺粒
const unsigned long PULSE_MS = 50;  // 触发脉冲宽度(ms)：够 PLC 锁存即可，按现场实测可调小/调大

bool pulsing  = false;             // D8 当前是否在输出触发脉冲
bool pulsing2 = false;             // D9 当前是否在输出触发脉冲
unsigned long pulseStart  = 0;     // D8 本次脉冲起点(millis)
unsigned long pulseStart2 = 0;     // D9 本次脉冲起点(millis)
bool ngState  = false;             // D8 状态，供 STATUS 查询
bool ngState2 = false;             // D9 状态，供 STATUS 查询

void setup() {
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUT_PIN2, OUTPUT);
  digitalWrite(OUT_PIN, LOW);      // 默认 OK：不触发
  digitalWrite(OUT_PIN2, LOW);
  Serial.begin(115200);            // 与上位机 uno_relay.py 的 BAUD_RATE 必须一致，否则串口乱码
  Serial.setTimeout(20);           // 命令都带 '\n'，限制 readStringUntil 最坏等待
}

void loop() {
  // 1) 非阻塞收命令：任何时刻都能立即响应，不会被脉冲保持挡住
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "NG") {
      digitalWrite(OUT_PIN, HIGH);  // 触发；若脉冲期间又来 NG，则重新计时(重新武装)
      pulseStart = millis();
      pulsing = true;
      ngState = true;
      Serial.println("NG");         // 立刻回 ACK：上位机 send() 无需空等 200ms 超时
    }
    else if (cmd == "NG2") {
      digitalWrite(OUT_PIN2, HIGH);
      pulseStart2 = millis();
      pulsing2 = true;
      ngState2 = true;
      Serial.println("NG2");
    }
    else if (cmd == "OK") {         // 两路都回 LOW(停机/关串口时用)
      digitalWrite(OUT_PIN, LOW);
      digitalWrite(OUT_PIN2, LOW);
      pulsing = false;
      pulsing2 = false;
      ngState = false;
      ngState2 = false;
      Serial.println("OK");
    }
    else if (cmd == "OK2") {        // 只回 D9
      digitalWrite(OUT_PIN2, LOW);
      pulsing2 = false;
      ngState2 = false;
      Serial.println("OK2");
    }
    else if (cmd == "STATUS") {
      Serial.print("D8=");
      Serial.print(ngState ? "NG" : "LOW");
      Serial.print(" D9=");
      Serial.println(ngState2 ? "NG" : "LOW");
    }
    else {
      Serial.println("ERROR");
    }
  }

  // 2) 非阻塞收回脉冲：各自到时自动回 LOW，期间串口照常响应
  if (pulsing && (millis() - pulseStart >= PULSE_MS)) {
    digitalWrite(OUT_PIN, LOW);
    pulsing = false;
    ngState = false;
  }
  if (pulsing2 && (millis() - pulseStart2 >= PULSE_MS)) {
    digitalWrite(OUT_PIN2, LOW);
    pulsing2 = false;
    ngState2 = false;
  }
}
