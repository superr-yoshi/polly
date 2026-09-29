#pragma once
#include <stdint.h>

// ---- SIM / REAL 스위치 -------------------------------------------------
// 1 = 모의값 (부품 없이 테스트), 0 = 실제 하드웨어 사용
#define SIM_ULTRASONIC 1   // HC-SR04 4개 도착 + 배선 후 0
#define SIM_GRIPPER    1   // 서보(DS3218) + 별도 전원 배선 후 0

// ---- 초음파 핀 (TODO_MEASURE: 실제 배선 후 확정) -------------------------
// 순서: 0=front_left, 1=front_right, 2=rear_left, 3=rear_right
constexpr uint8_t US_TRIG[4] = {22, 24, 26, 28};
constexpr uint8_t US_ECHO[4] = {23, 25, 27, 29};

// ---- 집게 (TODO_MEASURE: 실제 집게를 달고 안전한 각도로 보정) ----------------
#define GRIPPER_PIN     9      // PWM 신호선. 전원은 Mega 5V가 아니라 별도 DC-DC + GND 공통
#define GRIP_OPEN_DEG   40     // SIM_ONLY
#define GRIP_CLOSE_DEG  120    // SIM_ONLY
#define GRIP_STEP_MS    15     // 1도 이동 간격 (작을수록 빠름)
