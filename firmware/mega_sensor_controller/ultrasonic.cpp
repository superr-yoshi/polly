#include <Arduino.h>
#include "config.h"
#include "ultrasonic.h"

static long g_mm[4] = {0, 0, 0, 0};

#if SIM_ULTRASONIC
// SIM_ONLY: 센서마다 다른 모의값 (Pi 쪽 파서 테스트용)
void ultrasonicBegin() {}
void ultrasonicUpdate(uint8_t idx) {
  unsigned long t = millis();
  switch (idx) {
    case 0:  g_mm[0] = 1000 + lround(700 * sin(t / 2000.0f)); break;  // 300~1700 왕복
    case 1:  g_mm[1] = 1500; break;
    case 2:  g_mm[2] = 2000; break;
    default: g_mm[3] = ((t / 3000) % 2) ? 0 : 800; break;             // 3초마다 실패(0)
  }
}
#else
void ultrasonicBegin() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(US_TRIG[i], OUTPUT);
    digitalWrite(US_TRIG[i], LOW);
    pinMode(US_ECHO[i], INPUT);
  }
}
void ultrasonicUpdate(uint8_t idx) {
  digitalWrite(US_TRIG[idx], LOW);  delayMicroseconds(2);
  digitalWrite(US_TRIG[idx], HIGH); delayMicroseconds(10);
  digitalWrite(US_TRIG[idx], LOW);
  unsigned long us = pulseIn(US_ECHO[idx], HIGH, 24000UL);  // 최대 약 4 m
  long mm = (long)(us * 0.343f / 2.0f);                     // 음속 343 m/s
  g_mm[idx] = (us == 0 || mm < 20 || mm > 4000) ? 0 : mm;
}
#endif

long ultrasonicGetMm(uint8_t idx) { return g_mm[idx & 3]; }
