#include <Arduino.h>
#include "config.h"
#include "gripper.h"

#if !SIM_GRIPPER
#include <Servo.h>
static Servo g_servo;
#endif

// 부팅 시 '열림'으로 가정한다. 실제 집게 위치와 다를 수 있음 (TODO_MEASURE)
static int g_angle  = GRIP_OPEN_DEG;
static int g_target = GRIP_OPEN_DEG;
static unsigned long g_tStep = 0;
static bool g_attached = false;

// 대회 규정 3.5.6: 심판의 시작 선언 전에 로봇이 기동하면 실격.
// 문의 17번 답변대로 시작 전에 부팅·초기화를 해 두므로, 부팅(또는 Pi가 포트를 열 때의
// 자동 리셋) 때 서보가 움직이면 안 된다. Servo.attach()는 즉시 펄스를 내보내 서보를
// 움직이므로, 첫 집게 명령($GRIP)을 받을 때까지 attach하지 않는다 (그동안 서보는 힘이 빠진 상태).
void gripperBegin() {
}

void gripperCommand(uint8_t action) {
#if !SIM_GRIPPER
  if (!g_attached) {
    g_servo.attach(GRIPPER_PIN, SERVO_MIN_US, SERVO_MAX_US);
    g_servo.write(g_angle);
    g_attached = true;
  }
#endif
  g_attached = true;
  g_target = (action == 1) ? GRIP_CLOSE_DEG : GRIP_OPEN_DEG;
}

void gripperUpdate() {
  if (!g_attached || g_angle == g_target) return;
  unsigned long now = millis();
  if (now - g_tStep < GRIP_STEP_MS) return;
  g_tStep = now;
  g_angle += (g_target > g_angle) ? 1 : -1;
#if !SIM_GRIPPER
  g_servo.write(g_angle);
#endif
}

GripState gripperState() {
  if (g_angle != g_target) return GRIP_MOVING;
  return (g_target == GRIP_CLOSE_DEG) ? GRIP_CLOSED : GRIP_OPEN;
}

int gripperAngle() { return g_angle; }
