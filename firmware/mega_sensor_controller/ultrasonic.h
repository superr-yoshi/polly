// ultrasonic.h
// HC-SR04 초음파 센서 x4 인터페이스
//
// 관련 문서:
//   - docs/pin_map.md (핀 번호)
//   - docs/serial_protocol.md (RNG packet: fl_mm, fr_mm, rl_mm, rr_mm)

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

// ---------------------------------------------------------------
// 센서 위치
// RNG packet의 필드 순서(fl, fr, rl, rr)와 같은 순서로 둔다.
// ---------------------------------------------------------------
enum UltrasonicPosition {
  ULTRASONIC_FRONT_LEFT = 0,   // fl_mm
  ULTRASONIC_FRONT_RIGHT = 1,  // fr_mm
  ULTRASONIC_REAR_LEFT = 2,    // rl_mm
  ULTRASONIC_REAR_RIGHT = 3,   // rr_mm
  ULTRASONIC_COUNT = 4
};

// ---------------------------------------------------------------
// 핀 번호 (docs/pin_map.md 확정 후 입력)
// ---------------------------------------------------------------

// TODO: Front Left
// #define ULTRASONIC_FL_TRIG_PIN ...
// #define ULTRASONIC_FL_ECHO_PIN ...

// TODO: Front Right
// #define ULTRASONIC_FR_TRIG_PIN ...
// #define ULTRASONIC_FR_ECHO_PIN ...

// TODO: Rear Left
// #define ULTRASONIC_RL_TRIG_PIN ...
// #define ULTRASONIC_RL_ECHO_PIN ...

// TODO: Rear Right
// #define ULTRASONIC_RR_TRIG_PIN ...
// #define ULTRASONIC_RR_ECHO_PIN ...

// ---------------------------------------------------------------
// 측정 실패 표시값
// TODO: docs/serial_protocol.md의 "Timeout / invalid range rule" 확정 후 수정한다.
//       현재 0은 코드가 컴파일되도록 둔 임시값이며 확정된 규칙이 아니다.
// ---------------------------------------------------------------
#define ULTRASONIC_INVALID_MM 0

// ---------------------------------------------------------------
// 함수 선언 (구현은 ultrasonic.cpp)
// ---------------------------------------------------------------

// 초음파 센서 핀 초기화. setup()에서 한 번 호출한다.
void ultrasonicInit();

// 지정한 위치의 센서 거리를 mm 단위로 반환한다.
// TODO: 측정 실패(타임아웃) 시 반환값은
//       docs/serial_protocol.md의 "Timeout / invalid range rule" 확정 후 결정한다.
uint16_t ultrasonicReadMm(UltrasonicPosition position);

#endif  // ULTRASONIC_H
