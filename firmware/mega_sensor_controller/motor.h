#pragma once
#include <Arduino.h>

// $ENC의 status 값 (docs/serial_protocol.md)
enum MotorStatus : uint8_t {
  MOTOR_IDLE = 0,         // 정지: 첫 명령 전 또는 명령 끊김(CMD_TIMEOUT_MS)
  MOTOR_DRIVE = 1,        // $DRV 속도 제어 중
  MOTOR_PWM = 2,          // $PWM 직접 출력 중 (배선 확인용)
  MOTOR_FAULT_REVERSE = 3,  // 목표와 반대로 돎 → 정지 (Mega 리셋 전까지 유지)
  MOTOR_FAULT_STALL = 4,    // 큰 duty인데 안 돎 (엔코더 신호 없음·바퀴 막힘) → 정지 (리셋 전까지 유지)
};

void motorBegin();
void motorDrive(long leftTps, long rightTps);   // 목표 속도 tick/s, 전진 = +
void motorPwm(int leftDuty, int rightDuty);     // -255~255, 엔코더 없이 직접
void motorUpdate();                             // loop에서 매번 호출 (제어·timeout)
MotorStatus motorStatus();
