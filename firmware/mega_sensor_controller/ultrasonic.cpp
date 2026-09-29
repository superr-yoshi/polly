// ultrasonic.cpp
// HC-SR04 초음파 센서 x4 구현
//
// 핀 번호가 docs/pin_map.md에서 확정되기 전까지는
// 실제 핀 동작(pinMode, digitalWrite, pulseIn)을 넣지 않는다.

#include "ultrasonic.h"

void ultrasonicInit() {
  // TODO: 각 센서의 TRIG 핀을 OUTPUT, ECHO 핀을 INPUT으로 설정한다.
  //       (핀 번호 확정 후)
}

uint16_t ultrasonicReadMm(UltrasonicPosition position) {
  // TODO: position에 해당하는 센서로 거리를 측정한다.
  //   1. TRIG 핀에 펄스 출력
  //   2. ECHO 핀의 HIGH 시간 측정 (timeout 값 미정)
  //   3. 측정 시간을 mm로 변환
  //   4. 측정 실패 시 ULTRASONIC_INVALID_MM 반환

  (void)position;  // 구현 전까지 사용하지 않는 인자 경고 방지
  return ULTRASONIC_INVALID_MM;
}
