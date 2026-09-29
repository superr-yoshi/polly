#pragma once
#include <Arduino.h>

enum GripState : uint8_t { GRIP_OPEN = 0, GRIP_CLOSED = 1, GRIP_MOVING = 2 };

void gripperBegin();
void gripperCommand(uint8_t action);  // 0 = 열기, 1 = 닫기
void gripperUpdate();                 // loop에서 매번 호출 (1도씩 서서히 이동)
GripState gripperState();
int gripperAngle();
