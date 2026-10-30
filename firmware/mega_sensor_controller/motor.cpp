#include <Arduino.h>
#include "config.h"
#include "encoder.h"
#include "motor.h"

// 안전 (AGENTS.md 4장):
// - 첫 $DRV/$PWM 전에는 모터를 돌리지 않는다 (규정 3.5.6). Pi는 시작 전 0 속도만 보낸다.
// - 명령이 CMD_TIMEOUT_MS 넘게 끊기면 스스로 멈춘다 (Pi 멈춤, USB 빠짐). RRC Lite와 달리 펌웨어가 직접 보장한다.
// - 목표 0이면 PI를 쓰지 않고 바로 duty 0 (정지 명령이 PI 누적 때문에 안 먹히는 일이 없게).
// - 방향 반대·신호 없음을 감지하면 모터를 끄고 리셋 전까지 다시 돌리지 않는다.

struct Wheel {
  uint8_t pwmPin, dirPin;
  bool invert;
  long target;            // tick/s
  long lastTicks;
  float integ;            // 오차 적분 (tick)
  int duty;
  unsigned int revMs, stallMs;
};

static Wheel g_w[2] = {
  {MOTOR_L_PWM, MOTOR_L_DIR, MOTOR_L_INVERT != 0, 0, 0, 0.0f, 0, 0, 0},
  {MOTOR_R_PWM, MOTOR_R_DIR, MOTOR_R_INVERT != 0, 0, 0, 0.0f, 0, 0, 0},
};
static MotorStatus g_status = MOTOR_IDLE;
static unsigned long g_tCmd = 0, g_tCtrl = 0;

static void writeWheel(Wheel& w, int duty) {
  w.duty = duty;
  if (w.invert) duty = -duty;
  digitalWrite(w.dirPin, duty < 0 ? HIGH : LOW);   // MDD10: DIR LOW/HIGH = 정/역회전
  analogWrite(w.pwmPin, abs(duty));                 // 0이면 LOW 고정
}

static void stopAll(MotorStatus status) {
  g_status = status;
  for (Wheel& w : g_w) {
    w.target = 0;
    w.integ = 0.0f;
    w.revMs = w.stallMs = 0;
    writeWheel(w, 0);
  }
}

static bool isFault() {
  return g_status == MOTOR_FAULT_REVERSE || g_status == MOTOR_FAULT_STALL;
}

void motorBegin() {
  for (Wheel& w : g_w) {
    pinMode(w.pwmPin, OUTPUT);
    pinMode(w.dirPin, OUTPUT);
  }
  stopAll(MOTOR_IDLE);
  long l, r;
  encoderRead(&l, &r);
  g_w[0].lastTicks = l;
  g_w[1].lastTicks = r;
  g_tCtrl = millis();
}

void motorDrive(long leftTps, long rightTps) {
  if (isFault()) return;
  long t[2] = {leftTps, rightTps};
  for (uint8_t i = 0; i < 2; i++) {
    if ((t[i] > 0) != (g_w[i].target > 0) || t[i] == 0) g_w[i].integ = 0.0f;  // 방향 바뀜 → 적분 초기화
    g_w[i].target = t[i];
  }
  g_status = MOTOR_DRIVE;
  g_tCmd = millis();
}

void motorPwm(int leftDuty, int rightDuty) {
  if (isFault()) return;
  g_status = MOTOR_PWM;
  g_w[0].target = g_w[1].target = 0;
  writeWheel(g_w[0], constrain(leftDuty, -MAX_DUTY, MAX_DUTY));
  writeWheel(g_w[1], constrain(rightDuty, -MAX_DUTY, MAX_DUTY));
  g_tCmd = millis();
}

static void controlWheel(Wheel& w, long measured, unsigned int dtMs) {
  if (w.target == 0) {
    w.integ = 0.0f;
    w.revMs = w.stallMs = 0;
    writeWheel(w, 0);
    return;
  }
  float err = (float)(w.target - measured);
  w.integ += err * dtMs / 1000.0f;
  float iLimit = MAX_DUTY / SPEED_KI;                  // 적분 항이 MAX_DUTY를 넘지 않게 (anti-windup)
  w.integ = constrain(w.integ, -iLimit, iLimit);
  float out = SPEED_FF * w.target + SPEED_KP * err + SPEED_KI * w.integ;
  writeWheel(w, constrain((int)out, -MAX_DUTY, MAX_DUTY));

  bool reversed = (measured > 0) != (w.target > 0) && labs(measured) >= FAULT_REV_TPS;
  w.revMs = reversed ? w.revMs + dtMs : 0;
  bool stalled = abs(w.duty) >= FAULT_DUTY && labs(measured) < FAULT_STALL_TPS;
  w.stallMs = stalled ? w.stallMs + dtMs : 0;
}

void motorUpdate() {
  unsigned long now = millis();
  if ((g_status == MOTOR_DRIVE || g_status == MOTOR_PWM) && now - g_tCmd > CMD_TIMEOUT_MS) {
    stopAll(MOTOR_IDLE);
  }
  if (now - g_tCtrl < CONTROL_MS) return;
  unsigned int dtMs = (unsigned int)(now - g_tCtrl);
  g_tCtrl = now;

  long ticks[2];
  encoderRead(&ticks[0], &ticks[1]);
  for (uint8_t i = 0; i < 2; i++) {
    Wheel& w = g_w[i];
    long measured = (ticks[i] - w.lastTicks) * 1000L / (long)dtMs;
    w.lastTicks = ticks[i];
    if (g_status != MOTOR_DRIVE) continue;
    controlWheel(w, measured, dtMs);
    if (w.revMs > FAULT_REV_MS) stopAll(MOTOR_FAULT_REVERSE);
    else if (w.stallMs > FAULT_STALL_MS) stopAll(MOTOR_FAULT_STALL);
  }
}

MotorStatus motorStatus() { return g_status; }
