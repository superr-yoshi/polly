// 집게 서보(DS3218) 단독 시험·보정 스케치 (Mega D9)
// 본 펌웨어(mega_sensor_controller) 대신 잠깐 올려서 배선을 확인하고 열림/닫힘 펄스 값을 찾는다.
// 끝나도 옛 본 펌웨어(mega_sensor_controller)는 다시 올리지 않는다: MDD10A 모터 핀을 안 잡아 바퀴가 돌 수 있다.
//
// 시리얼 모니터 115200 bps, 한 줄씩 입력:
//   1500  -> 1500 us로 이동 (500~2500). 펄스가 없을 때 처음 넣은 값은 그 위치로 바로 간다.
//            그 뒤 값은 천천히 이동한다.
//   + / - -> 20 us씩 이동
//   x     -> 펄스 끊기 (서보 힘 빠짐). 웅 소리가 계속 나거나 기구에 걸리면 바로 입력
//
// 전원: 서보 +/-는 별도 5V 컨버터에서 받고, 컨버터 -와 Mega GND를 잇는다. Mega 5V 핀은 쓰지 않는다.
#include <Servo.h>

const uint8_t SERVO_PIN = 9;
const uint8_t MOTOR_PIN[4] = {6, 7, 30, 31};  // MDD10A PWM1·PWM2·DIR1·DIR2 (motor_test와 같은 배선)
const int MIN_US = 500;               // DS3218: 500~2500 us = 0~180도
const int MAX_US = 2500;
const int STEP_US = 10;               // 20 ms마다 10 us (약 45도/초)
const unsigned long STEP_MS = 20;
const int NUDGE_US = 20;

Servo servo;
int curUs = 1500;
int targetUs = 1500;
unsigned long tStep = 0;

void report() {
  Serial.print(F("현재 "));
  Serial.print(curUs);
  Serial.print(F(" us (약 "));
  Serial.print((long)(curUs - MIN_US) * 180 / (MAX_US - MIN_US));
  Serial.println(F("도)"));
}

void moveTo(int us) {
  us = constrain(us, MIN_US, MAX_US);
  if (!servo.attached()) {
    curUs = us;
    servo.writeMicroseconds(curUs);   // attach 직후 첫 펄스가 기본값 1500 us로 나가지 않게
    servo.attach(SERVO_PIN, MIN_US, MAX_US);
    servo.writeMicroseconds(curUs);
    Serial.println(F("펄스 시작 (서보 힘 들어감)"));
    report();
  }
  targetUs = us;
  Serial.print(F("목표 "));
  Serial.print(targetUs);
  Serial.println(F(" us"));
}

void handle(String s) {
  s.trim();
  s.toLowerCase();
  if (s.length() == 0) return;
  if (s == "x") {
    servo.detach();
    Serial.println(F("펄스 끊음 (서보 힘 빠짐)"));
    return;
  }
  if (s == "+" || s == "-") {
    if (!servo.attached()) {
      Serial.println(F("먼저 숫자로 시작 (예: 1500)"));
      return;
    }
    moveTo(targetUs + (s == "+" ? NUDGE_US : -NUDGE_US));
    return;
  }
  long v = s.toInt();
  if (v < MIN_US || v > MAX_US) {
    Serial.println(F("500~2500 숫자, +, -, x 중 하나"));
    return;
  }
  moveTo((int)v);
}

void setup() {
  // 규정 3.5.6: 켜자마자 모터 핀부터 꺼짐으로 둔다 (MDD10A 입력이 떠 있지 않게)
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(MOTOR_PIN[i], OUTPUT);
    digitalWrite(MOTOR_PIN[i], LOW);
  }
  Serial.begin(115200);
  Serial.setTimeout(50);              // 줄 끝 문자 없이 보내도 50 ms 뒤 처리
  Serial.println(F("servo_test: 펄스 없음 (서보 힘 빠짐). 숫자를 넣으면 시작"));
}

void loop() {
  if (Serial.available()) handle(Serial.readStringUntil('\n'));

  if (servo.attached() && curUs != targetUs && millis() - tStep >= STEP_MS) {
    tStep = millis();
    curUs += constrain(targetUs - curUs, -STEP_US, STEP_US);
    servo.writeMicroseconds(curUs);
    if (curUs == targetUs) report();
  }
}
