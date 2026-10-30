#pragma once
#include <Arduino.h>

void encoderBegin();
// 누적 tick (4체배). config.h의 ENC_*_INVERT를 적용한 값: 로봇 전진 방향 = +
void encoderRead(long* left, long* right);
