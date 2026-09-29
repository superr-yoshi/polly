// mega_sensor_controller.ino
// Arduino Mega 2560 센서 컨트롤러
//
// 역할:
//   - HC-SR04 초음파 센서 x4 거리 측정
//   - 배터리 전압 측정
//   - 측정값을 USB Serial로 Raspberry Pi 5에 전송
//
// 관련 문서:
//   - docs/serial_protocol.md
//   - docs/pin_map.md

#include "serial_protocol.h"
#include "ultrasonic.h"
#include "battery_monitor.h"

void setup() {
  // TODO: Serial.begin(PROTOCOL_BAUDRATE);
  //       Baudrate가 docs/serial_protocol.md에서 확정되면 활성화한다.

  ultrasonicInit();
  batteryMonitorInit();
}

void loop() {
  // TODO: RNG packet 전송 (주기 미정)
  //       ultrasonicReadMm()으로 4개 센서를 읽어
  //       $RNG,seq,millis,fl_mm,fr_mm,rl_mm,rr_mm*CS 형식으로 전송

  // TODO: BAT packet 전송 (주기 미정)
  //       batteryReadMillivolt()로 읽어
  //       $BAT,seq,millis,millivolt*CS 형식으로 전송
}
