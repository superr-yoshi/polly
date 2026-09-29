// battery_monitor.h
// 배터리 전압 측정 인터페이스
//
// 관련 문서:
//   - docs/hardware_plan.md (Battery: 6S LiPo)
//   - docs/serial_protocol.md (BAT packet: millivolt)

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

// ---------------------------------------------------------------
// 하드웨어 설정 (실제 회로 확정 후 입력)
// ---------------------------------------------------------------

// TODO: 배터리 전압을 읽을 아날로그 핀 (docs/pin_map.md에 아직 없음)
// #define BATTERY_ADC_PIN ...

// TODO: 전압 분배 회로 비율
//       6S 배터리 전압은 Mega 아날로그 입력 한계(5V)보다 높으므로
//       분배 회로가 필요하다. 저항 값 확정 후 입력한다.
// #define BATTERY_DIVIDER_RATIO ...

// ---------------------------------------------------------------
// 측정 실패 표시값
// TODO: 규칙 확정 후 수정한다. 현재 0은 컴파일용 임시값이다.
// ---------------------------------------------------------------
#define BATTERY_INVALID_MV 0

// ---------------------------------------------------------------
// 함수 선언 (구현은 battery_monitor.cpp)
// ---------------------------------------------------------------

// 배터리 측정 초기화. setup()에서 한 번 호출한다.
void batteryMonitorInit();

// 배터리 전압을 mV 단위로 반환한다.
uint16_t batteryReadMillivolt();

#endif  // BATTERY_MONITOR_H
