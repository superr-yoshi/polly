// 주행 모터(MDD10A)·집게 서보 시험 스케치 (Mega)
// 본 펌웨어 대신 잠깐 올려서 MDD10A·모터·엔코더·집게를 확인한다. 끝나도 옛 본 펌웨어는 다시 올리지 않는다 (MDD10A 모터 핀을 안 잡아 바퀴가 돌 수 있음).
//
// 시리얼 모니터 115200 bps, 한 줄씩 입력:
//   왼쪽% 오른쪽% ms   예) 20 0 1000 -> 왼쪽 바퀴만 20%로 1초 (-는 반대 방향)
//   0 0 10000           -> 모터는 멈춘 채 10초 동안 엔코더만 센다 (손으로 바퀴를 1바퀴 돌려 보기)
//   o / g               -> 집게 열기 (1500 us) / 닫아 잡기 (2000 us). 천천히 움직인다
//   p 1800              -> 집게를 1800 us로 (잡는 값 맞출 때)
//   s                   -> 모터 즉시 정지 (천천히 서지 않고 바로 끊는다)
//   x                   -> 모터 즉시 정지 + 집게 힘 빼기 (콘솔을 끝낼 때 자동으로 보낸다)
// 출발·정지는 10 ms마다 1%씩 천천히 바꾼다 (0 -> 100%에 1초). 급출발·급정지로 바퀴·기어가 튀지 않게.
// 정해진 시간이 지나면 천천히 멈춘 뒤 엔코더 수를 보고한다. 최대 100%, 3초로 제한한다 (0 0은 10초).
//
// PWM·DIR은 실제 배선 (2026-10-30 팀 확인), 엔코더는 사양서_E 기준 (TODO_MEASURE: 실제 배선 확인)
#include <Servo.h>

const uint8_t PWM_PIN[2] = {6, 7};    // MDD10A PWM1(왼쪽), PWM2(오른쪽)
const uint8_t DIR_PIN[2] = {30, 31};  // MDD10A DIR1, DIR2
const uint8_t ENC_A[2] = {2, 18};     // 엔코더 A (인터럽트 핀)
const uint8_t ENC_B[2] = {3, 19};     // 엔코더 B
const int MAX_PCT = 100;              // MDD10A 전원 12.3 V 실측 (2026-10-30) = 모터 정격. 전원이 12 V보다 높으면 낮춘다
const long MAX_MS = 3000;
const long COUNT_ONLY_MS = 10000;     // 0 0 (모터 정지)일 때만 허용
const int COUNTS_PER_REV = 660;       // A상 CHANGE만 셈: 11 PPR x 30 x 2
const int RAMP_STEP_PCT = 1;          // RAMP_MS마다 바꾸는 양 (클수록 빨리 출발·정지)
const unsigned long RAMP_MS = 10;
const uint8_t SERVO_PIN = 9;          // 집게 DS3218
const int GRIP_OPEN_US = 1500;        // 실측 2026-10-30 (servo_test)
const int GRIP_CLOSE_US = 2000;       // 빈 집게가 끝까지 닫히는 값. 대상을 물면 서보가 계속 밀 수 있다 (TODO_MEASURE)
const int SERVO_STEP_US = 10;         // 20 ms마다 10 us (약 45도/초), servo_test와 같음
const unsigned long SERVO_STEP_MS = 20;

volatile long enc[2] = {0, 0};
bool running = false;
unsigned long tStart = 0;
long runMs = 0;
int curPct[2] = {0, 0};               // 지금 내보내는 힘
int targetPct[2] = {0, 0};            // 가려는 힘 (loop에서 RAMP_STEP_PCT씩 따라간다)
bool stopping = false;                // 시간이 다 되어 천천히 서는 중
unsigned long tRamp = 0;
Servo grip;
int gripCur = GRIP_OPEN_US;
int gripTarget = GRIP_OPEN_US;
unsigned long tServo = 0;

void encLeft() { enc[0] += (digitalRead(ENC_A[0]) == digitalRead(ENC_B[0])) ? 1 : -1; }
void encRight() { enc[1] += (digitalRead(ENC_A[1]) == digitalRead(ENC_B[1])) ? 1 : -1; }

void setMotor(uint8_t i, int pct) {
  pct = constrain(pct, -MAX_PCT, MAX_PCT);
  digitalWrite(DIR_PIN[i], pct < 0 ? HIGH : LOW);
  analogWrite(PWM_PIN[i], (long)abs(pct) * 255 / 100);
}

void stopAll() {
  for (uint8_t i = 0; i < 2; i++) {
    analogWrite(PWM_PIN[i], 0);
    digitalWrite(DIR_PIN[i], LOW);
  }
}

// 규정 3.5.6: 첫 집게 명령 전에는 펄스를 내지 않는다 (서보 힘 빠진 상태). 첫 명령은 그 위치로 바로 간다
void gripTo(int us) {
  if (!grip.attached()) {
    gripCur = us;
    grip.writeMicroseconds(gripCur);  // attach 직후 첫 펄스가 기본값 1500 us로 나가지 않게
    grip.attach(SERVO_PIN, 500, 2500);
    grip.writeMicroseconds(gripCur);
  }
  gripTarget = us;
  Serial.print(F("집게 목표 "));
  Serial.print(us);
  Serial.println(F(" us"));
}

void report() {
  noInterrupts();
  long l = enc[0], r = enc[1];
  interrupts();
  Serial.print(F("정지. "));
  Serial.print(millis() - tStart);
  Serial.print(F(" ms, 엔코더 왼쪽 "));
  Serial.print(l);
  Serial.print(F(" ("));
  Serial.print((float)l / COUNTS_PER_REV, 2);
  Serial.print(F("바퀴), 오른쪽 "));
  Serial.print(r);
  Serial.print(F(" ("));
  Serial.print((float)r / COUNTS_PER_REV, 2);
  Serial.println(F("바퀴)"));
}

void handle(String s) {
  s.trim();
  s.toLowerCase();
  if (s.length() == 0) return;
  if (s == "s" || s == "x") {
    stopAll();
    curPct[0] = curPct[1] = targetPct[0] = targetPct[1] = 0;
    if (running || stopping) report();
    running = stopping = false;
    Serial.println(F("정지 명령"));
    if (s == "x" && grip.attached()) {
      grip.detach();
      Serial.println(F("집게 힘 뺌"));
    }
    return;
  }
  if (s == "o" || s == "g") {
    gripTo(s == "o" ? GRIP_OPEN_US : GRIP_CLOSE_US);
    return;
  }
  if (s.startsWith("p ")) {
    long v = s.substring(2).toInt();
    if (v < 500 || v > 2500) {
      Serial.println(F("p 500~2500 (예: p 1800)"));
      return;
    }
    gripTo((int)v);
    return;
  }
  int l, r;
  long ms;
  if (sscanf(s.c_str(), "%d %d %ld", &l, &r, &ms) != 3 || ms <= 0) {
    Serial.println(F("형식: 왼쪽% 오른쪽% ms (예: 20 0 1000), 집게 o / g / p 1800, 정지 s, 전부 끄기 x"));
    return;
  }
  runMs = min(ms, (l == 0 && r == 0) ? COUNT_ONLY_MS : MAX_MS);
  noInterrupts();
  enc[0] = enc[1] = 0;
  interrupts();
  tStart = millis();
  running = true;
  stopping = false;
  targetPct[0] = constrain(l, -MAX_PCT, MAX_PCT);  // 실제 출력은 loop에서 천천히 따라간다
  targetPct[1] = constrain(r, -MAX_PCT, MAX_PCT);
  Serial.print(F("돌림: 왼쪽 "));
  Serial.print(constrain(l, -MAX_PCT, MAX_PCT));
  Serial.print(F("%, 오른쪽 "));
  Serial.print(constrain(r, -MAX_PCT, MAX_PCT));
  Serial.print(F("%, "));
  Serial.print(runMs);
  Serial.println(F(" ms"));
}

void setup() {
  // 규정 3.5.6: 켜자마자 모터 핀부터 꺼짐으로 둔다
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(PWM_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
  }
  stopAll();
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), encLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), encRight, CHANGE);
  Serial.begin(115200);
  Serial.setTimeout(50);
  Serial.println(F("motor_test: 모터 정지, 집게 힘 빠진 상태. 예) 20 0 1000, 집게 o / g"));
}

void loop() {
  if (Serial.available()) handle(Serial.readStringUntil('\n'));
  if (running && (long)(millis() - tStart) >= runMs) {  // 시간이 다 되면 천천히 세운다
    running = false;
    stopping = true;
    targetPct[0] = targetPct[1] = 0;
  }
  if (millis() - tRamp >= RAMP_MS) {
    tRamp = millis();
    for (uint8_t i = 0; i < 2; i++) {
      if (curPct[i] != targetPct[i]) {
        curPct[i] += constrain(targetPct[i] - curPct[i], -RAMP_STEP_PCT, RAMP_STEP_PCT);
        setMotor(i, curPct[i]);
      }
    }
    if (stopping && curPct[0] == 0 && curPct[1] == 0) {
      stopping = false;
      report();
    }
  }
  if (grip.attached() && gripCur != gripTarget && millis() - tServo >= SERVO_STEP_MS) {
    tServo = millis();
    gripCur += constrain(gripTarget - gripCur, -SERVO_STEP_US, SERVO_STEP_US);
    grip.writeMicroseconds(gripCur);
    if (gripCur == gripTarget) {
      Serial.print(F("집게 도착 "));
      Serial.print(gripCur);
      Serial.println(F(" us"));
    }
  }
}
