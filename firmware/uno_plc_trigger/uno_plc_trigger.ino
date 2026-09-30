// uno_plc_trigger.ino —— 高速非阻塞版 / 双路触发 + 双色状态 LED
//
// 接线：
//   D8  -> 光耦隔离器 -> PLC(第一站：正反面翻边止口, 吹气把 NG 件从料道吹掉)
//   D9  -> 光耦隔离器 -> PLC(第二站：兜孔缺粒, 开闸门放 NG 件掉进回收盒)
//   D10 -> 绿色 LED(串限流电阻)：常亮 = 两台相机 + UNO 全部正常接入、产线可正常工作
//   D11 -> 红色 LED(串限流电阻)：闪烁 = 有问题(相机没通 / 主程序未汇报健康 / 串口失联)
//   D8/D9 各自独立、互不阻塞；执行器动作(吹气/开闸)的延迟与时长由 PLC 自管，UNO 只给触发脉冲。
//
// 串口命令(与上位机 uno_relay.py 严格一致，全大写、'\n' 结尾)：
//   NG        D8 拉高一个 PULSE_MS 脉冲后自动回 LOW；回 "NG"
//   NG2       D9 同上(第二站/缺粒站)；回 "NG2"
//   OK        两路都拉回 LOW；回 "OK"
//   OK2       只把 D9 拉回 LOW；回 "OK2"
//   HEALTHY   健康：绿灯常亮、红灯灭；回 "HEALTHY"
//   FAULT     故障：绿灯灭、红灯闪烁；回 "FAULT"
//   STATUS    回 "D8=<NG|LOW> D9=<NG|LOW> LED=<GREEN|RED>"
//   其它      回 "ERROR"
//
// ★ 状态 LED 设计说明(为什么红灯"闪烁"而非"常亮")：
//   1) 产线嘈杂，闪烁远比常亮抓眼球，一眼就知道要处理；
//   2) 常亮红容易被误当成"通电指示"，闪烁明确表示"正在故障、需人工介入"；
//   3) 关键 —— 看门狗：主程序每 <=1s 必须发一次 HEALTHY/FAULT。若 LED_WATCHDOG_MS(3s)内
//      一条指令都没收到(主程序崩了 / 主机死机 / 串口被拔)，UNO 自己就把 LED 打成"红灯闪烁"。
//      所以绿灯只可能在"有一个活着的主程序、刚确认过两台相机与自身都正常"时亮 —— 绿灯永不说谎；
//      闪烁的红灯同时证明"UNO 这块板自己是活的(loop 在跑)，是上游报故障或上游失联"。
//   开机默认 = 红灯闪烁(还没有任何主程序确认过健康)，同样是 fail-safe：绝不开机就误亮绿。
//
// 极性：D8/D9 HIGH 触发、LOW 空闲；LED 按共阴接法 HIGH 点亮。若 LED 为共阳(HIGH 灭)，
//   把 ledWrite() 里的两行电平取反即可。
//
// ⚠ 改了这个固件必须用 Arduino IDE 重新烧录 UNO，上位机才能用 HEALTHY/FAULT 并驱动 D10/D11。

const int OUT_PIN   = 8;    // 第一站：正反面翻边止口(吹气)
const int OUT_PIN2  = 9;    // 第二站：兜孔缺粒(开闸)
const int LED_GREEN = 10;   // 绿灯：一切正常(常亮)
const int LED_RED   = 11;   // 红灯：故障(闪烁)

const unsigned long PULSE_MS        = 50;    // 触发脉冲宽度(ms)：够 PLC 锁存即可
const unsigned long LED_WATCHDOG_MS = 3000;  // 超过这么久没收到任何有效指令 -> 强制红灯(主程序失联)
const unsigned long RED_BLINK_MS    = 400;   // 红灯闪烁半周期(ms)

bool pulsing   = false;            // D8 是否在输出脉冲
bool pulsing2  = false;            // D9 是否在输出脉冲
unsigned long pulseStart  = 0;
unsigned long pulseStart2 = 0;
bool ngState   = false;            // 供 STATUS 查询
bool ngState2  = false;

bool healthy   = false;            // 主程序最近汇报的健康态(开机默认故障)
unsigned long lastCmdMs = 0;       // 最近一次收到有效指令的时刻(看门狗)
bool redOn     = false;            // 红灯当前亮灭(闪烁状态机)
unsigned long redToggleMs = 0;

void ledWrite(bool green, bool red) {
  // 共阴接法：HIGH 点亮。若 LED 为共阳(HIGH 灭)，把下面两行取反。
  digitalWrite(LED_GREEN, green ? HIGH : LOW);
  digitalWrite(LED_RED,   red   ? HIGH : LOW);
}

bool healthyEffective() {
  // 绿灯的充要条件：主程序最近汇报健康，且看门狗未超时(主程序仍在持续汇报)。
  return healthy && (millis() - lastCmdMs <= LED_WATCHDOG_MS);
}

void updateLed() {
  // 非阻塞：绿灯常亮 / 红灯按 RED_BLINK_MS 半周期闪烁，绝不 delay()。
  if (healthyEffective()) {
    redOn = false;
    ledWrite(true, false);           // 绿灯常亮
  } else {
    if (millis() - redToggleMs >= RED_BLINK_MS) {
      redToggleMs = millis();
      redOn = !redOn;
    }
    ledWrite(false, redOn);          // 红灯闪烁, 绿灯灭
  }
}

void setup() {
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUT_PIN2, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(OUT_PIN, LOW);        // 默认 OK：不触发
  digitalWrite(OUT_PIN2, LOW);
  ledWrite(false, true);             // 开机默认：红灯(还没有主程序确认过健康)
  Serial.begin(115200);              // 与上位机 uno_relay.py 的 BAUD_RATE 必须一致
  Serial.setTimeout(20);             // 命令都带 '\n'，限制 readStringUntil 最坏等待
}

void loop() {
  // 1) 非阻塞收命令：任何时刻都能立即响应，不会被脉冲保持挡住
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    bool known = true;

    if (cmd == "NG") {
      digitalWrite(OUT_PIN, HIGH);   // 触发；若脉冲期间又来 NG，则重新计时(重新武装)
      pulseStart = millis();
      pulsing = true;
      ngState = true;
      Serial.println("NG");          // 立刻回 ACK：上位机 send() 无需空等超时
    }
    else if (cmd == "NG2") {
      digitalWrite(OUT_PIN2, HIGH);
      pulseStart2 = millis();
      pulsing2 = true;
      ngState2 = true;
      Serial.println("NG2");
    }
    else if (cmd == "OK") {          // 两路都回 LOW(停机/关串口时用)
      digitalWrite(OUT_PIN, LOW);
      digitalWrite(OUT_PIN2, LOW);
      pulsing = false;
      pulsing2 = false;
      ngState = false;
      ngState2 = false;
      Serial.println("OK");
    }
    else if (cmd == "OK2") {         // 只回 D9
      digitalWrite(OUT_PIN2, LOW);
      pulsing2 = false;
      ngState2 = false;
      Serial.println("OK2");
    }
    else if (cmd == "HEALTHY") {     // 主程序汇报：一切正常 -> 绿灯常亮
      healthy = true;
      Serial.println("HEALTHY");
    }
    else if (cmd == "FAULT") {       // 主程序汇报：有故障 -> 红灯闪烁
      healthy = false;
      Serial.println("FAULT");
    }
    else if (cmd == "STATUS") {
      Serial.print("D8=");
      Serial.print(ngState ? "NG" : "LOW");
      Serial.print(" D9=");
      Serial.print(ngState2 ? "NG" : "LOW");
      Serial.print(" LED=");
      Serial.println(healthyEffective() ? "GREEN" : "RED");
    }
    else {
      known = false;
      Serial.println("ERROR");
    }

    if (known) lastCmdMs = millis();  // 收到任何有效指令都喂看门狗(证明主程序活着)
  }

  // 2) 非阻塞回脉冲：各自到时自动回 LOW，期间串口照常响应
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

  // 3) 状态 LED：绿灯常亮 / 红灯闪烁 / 看门狗超时强制红，全程非阻塞
  updateLed();
}
