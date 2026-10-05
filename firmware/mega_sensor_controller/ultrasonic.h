#pragma once
#include <Arduino.h>

void ultrasonicBegin();
void ultrasonicUpdate(uint8_t idx);   // 센서 idx(0~3) 한 개만 측정해 값 갱신
long ultrasonicGetMm(uint8_t idx);    // 마지막 측정값 (mm), 0 = 실패/범위 밖
