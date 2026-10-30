#pragma once
#include <stdint.h>

// ---- SIM / REAL 스위치 -------------------------------------------------
// 1 = 모의값 (부품 없이 테스트), 0 = 실제 하드웨어 사용
#define SIM_ULTRASONIC 0   // 2026-09-29 실측 확인 (전방 1개). 센서 없는 자리는 0(실패)으로 전송
#define NO_ULTRASONIC  1   // 2026-10-30 초음파 센서 없음 (팀 확인). 재지 않고 0(실패)만 보낸다.
                           //   없는 센서를 24 ms씩 기다리면 50 Hz 모터 제어가 밀린다. 센서를 달면 0
#define SIM_GRIPPER    0   // 2026-10-29 집게 서보를 다시 Mega D9로 (MDD10 전환). 서보 없이 시험하려면 1
#define SIM_IMU        1   // BNO085 배선 후 0. 0이면 SparkFun BNO080 라이브러리 필요. 1이면 $IMU를 보내지 않음

// ---- 초음파 핀 (TODO_MEASURE: 실제 배선 후 확정) -------------------------
// 순서: 0=전방(front), 1=좌측(left), 2=우측(right), 3=후방(rear) -> ROS /range/<이름>
constexpr uint8_t US_TRIG[4] = {22, 24, 26, 28};
constexpr uint8_t US_ECHO[4] = {23, 25, 27, 29};

// ---- 집게 (TODO_MEASURE: 실제 집게를 달고 안전한 각도로 보정) ----------------
#define GRIPPER_PIN     9      // PWM 신호선(노랑). 전원(빨강·갈색)은 5V 10A 컨버터 + GND 공통 (Mega 5V 금지)
// DS3218 사양 (docs/hardware_reference.md): PWM 500~2500 us = 0~180도, 중립 1500 us,
// 사용 전압 4.8~6.8 V (5V 10A 컨버터 5 V). Arduino Servo 기본값(544~2400 us)을 쓰면 각도가 어긋난다.
#define SERVO_MIN_US    500
#define SERVO_MAX_US    2500
#define GRIP_OPEN_DEG   90     // 1500 us = 열림 (2026-10-30 firmware/servo_test 실측)
#define GRIP_CLOSE_DEG  135    // 2000 us = 빈 집게가 끝까지 닫히는 값 (2026-10-30 실측). 50 mm 대상을 물면
                               //   서보가 계속 민다 -> 웅 소리·발열을 보고 낮춘다 (TODO_MEASURE: 대상 잡는 값)
#define GRIP_STEP_MS    15     // 1도 이동 간격 (작을수록 빠름)

// ---- 주행 모터: Cytron MDD10 (docs/pin_map.md) ------------------------------
// D6·D7은 Timer4 PWM (약 490 Hz). Servo 라이브러리는 Mega에서 Timer5(D44~46)를 쓰므로 겹치지 않는다.
#define MOTOR_L_PWM     6      // MDD10 PWM1 (왼쪽 모터 M1A/M1B)
#define MOTOR_L_DIR     30     // MDD10 DIR1
#define MOTOR_R_PWM     7      // MDD10 PWM2 (오른쪽 모터 M2A/M2B)
#define MOTOR_R_DIR     31     // MDD10 DIR2
// 전진 명령(+)에 바퀴가 뒤로 돌면 1로 바꾼다
// 2026-10-30 실측 (firmware/motor_test, 바닥 주행): 전진 = 왼쪽 DIR HIGH, 오른쪽 DIR LOW -> 왼쪽만 반대
#define MOTOR_L_INVERT  1
#define MOTOR_R_INVERT  0

// ---- 엔코더: JGB37-520 C1/C2 (A상/B상) --------------------------------------
// 4개 모두 외부 인터럽트 핀. 20·21은 I2C(BNO085)라 쓰지 않는다.
// 왼쪽 C1 = D2 (PE4), C2 = D3 (PE5) / 오른쪽 C1 = D18 (PD3), C2 = D19 (PD2)
// 핀을 바꾸면 encoder.cpp의 포트 읽기도 함께 고친다.
#define ENC_L_A         2
#define ENC_L_B         3
#define ENC_R_A         18
#define ENC_R_B         19
// 바퀴가 앞으로 돌 때 tick이 줄어들면 1로 바꾼다 (이 펌웨어는 B상이 앞서면 -로 센다)
// 왼쪽: 2026-10-30 motor_test에서 왼쪽 후진 = A상 앞섬 (왼쪽 C1·C2를 바꾼 뒤) -> 전진 = B상 앞섬 = - -> 1
#define ENC_L_INVERT    1
// 오른쪽: 2026-10-30 실측 (바퀴 띄우고 /cmd_vel 전진): INVERT 1이면 오른쪽이 -0.48 m/s로 세져
//   fault_reverse로 멈춤 (왼쪽은 +0.21로 맞음) -> 0
#define ENC_R_INVERT    0

// ---- 속도 제어 (Mega 안의 PI + 앞먹임) ----------------------------------------
// 단위: 목표·측정 속도 = 엔코더 tick/s (4체배, 바퀴 1바퀴 약 1320 tick), 출력 = PWM duty -255~255
#define CONTROL_MS      20     // 50 Hz
#define CMD_TIMEOUT_MS  300    // $DRV/$PWM이 이 시간 넘게 안 오면 모터 정지 (Pi가 멈추거나 USB가 빠져도)
#define MAX_DUTY        255    // 12 V 배터리 = 모터 정격 12 V. 배터리 전압이 더 높으면 낮춘다
#define SPEED_FF        0.035f // duty / (tick/s). 무부하 330 RPM = 약 7260 tick/s에서 255 (SIM_ONLY)
#define SPEED_KP        0.02f  // SIM_ONLY: 실제 바퀴로 보정
#define SPEED_KI        0.05f  // SIM_ONLY: 실제 바퀴로 보정
// 폭주 방지 (RRC 때 엔코더 신호 없음·방향 반대로 폭주한 경험, 2026-10-08)
// 걸리면 모터를 끄고 Mega가 리셋될 때까지 다시 돌리지 않는다 (Pi 노드 재시작 = 포트 재오픈 = 리셋)
#define FAULT_REV_TPS   300    // 목표와 반대 방향으로 이 속도 이상 돌면
#define FAULT_REV_MS    100    //   이 시간 넘게 계속될 때 → 방향 반대 (엔코더 C1·C2 또는 모터선 반대)
#define FAULT_DUTY      100    // duty가 이 값 이상인데 (무부하면 약 2800 tick/s로 돌 세기)
#define FAULT_STALL_TPS 100    //   측정 속도가 이보다 느린 상태가
#define FAULT_STALL_MS  300    //   이 시간 넘게 계속되면 → 엔코더 신호 없음 또는 바퀴 막힘
#define ENC_REPORT_MS   20     // $ENC 50 Hz

// ---- IMU: BNO085 (I2C, SDA = D20, SCL = D21) -------------------------------
// ⚠ Mega는 SDA/SCL이 보드 안에서 5 V로 풀업되어 있다. 3.3 V 전용 BNO085 보드는 레벨 시프터 필요
#define IMU_I2C_ADDR    0x4A   // TODO_MEASURE: Adafruit 보드 0x4A, SparkFun 보드 0x4B
#define IMU_REPORT_MS   40     // $IMU 25 Hz (115200 baud 여유를 위해 ENC보다 느리게)
