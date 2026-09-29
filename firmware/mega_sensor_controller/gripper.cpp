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

void gripperBegin() {
#if !SIM_GRIPPER
  g_servo.attach(GRIPPER_PIN);
  g_servo.write(g_angle);
#endif
}

void gripperCommand(uint8_t action) {
  g_target = (action == 1) ? GRIP_CLOSE_DEG : GRIP_OPEN_DEG;
}

void gripperUpdate() {
  if (g_angle == g_target) return;
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
