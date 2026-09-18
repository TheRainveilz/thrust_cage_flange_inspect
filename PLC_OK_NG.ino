const int OUT_PIN = 8;
bool ngState = false;
void setup() {
  pinMode(OUT_PIN, OUTPUT);
  // 默认OK不触发
  digitalWrite(OUT_PIN, LOW);

  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "NG") {
      ngState = true;
      digitalWrite(OUT_PIN, HIGH);  // 触发
      delay(500);                   // 保持500ms
      digitalWrite(OUT_PIN, LOW);   // 自动关闭
      ngState = false;
    }

    else if (cmd == "OK") {
      ngState = false;
      digitalWrite(OUT_PIN, LOW);   // OK不触发
    }

    else if (cmd == "STATUS") {
      if (ngState) {
        Serial.println("NG");
      } else {
        Serial.println("OK");
      }
    }

    else {
      Serial.println("ERROR");
    }
  }
}
