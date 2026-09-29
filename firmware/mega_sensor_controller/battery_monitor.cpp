// battery_monitor.cpp
// 배터리 전압 측정 구현
//
// 아날로그 핀과 분배 회로가 확정되기 전까지는
// 실제 측정 코드(analogRead)를 넣지 않는다.

#include "battery_monitor.h"

void batteryMonitorInit() {
  // TODO: 필요 시 아날로그 기준 전압 등 초기 설정 (회로 확정 후)
}

uint16_t batteryReadMillivolt() {
  // TODO: 배터리 전압을 측정한다.
  //   1. BATTERY_ADC_PIN에서 analogRead
  //   2. ADC 값을 핀 전압(mV)으로 변환
  //   3. BATTERY_DIVIDER_RATIO를 곱해 실제 배터리 전압(mV)으로 변환

  return BATTERY_INVALID_MV;
}
