#include <Arduino.h>
#include "config.h"
#include "encoder.h"

// A상·B상의 모든 변화(CHANGE)를 세는 4체배. 바퀴 1바퀴 = 11 PPR x 4 x 30:1 = 약 1320 tick.
// 최고 속도에서 모터당 초당 약 5천~7천 번 인터럽트가 걸린다 → ISR은 포트를 직접 읽어 짧게 끝낸다.

// (이전 상태 << 2 | 현재 상태) -> 증감. 상태 = (B << 1) | A. 건너뛴 변화(잡음)는 0
static const int8_t QDEC[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

static volatile long g_left = 0, g_right = 0;
static volatile uint8_t g_lPrev = 0, g_rPrev = 0;

// 왼쪽: A = D2 (PE4), B = D3 (PE5)
static inline uint8_t readLeft() {
  return (PINE >> 4) & 0x03;
}
// 오른쪽: A = D18 (PD3), B = D19 (PD2)
static inline uint8_t readRight() {
  uint8_t p = PIND;
  return ((p >> 3) & 0x01) | (((p >> 2) & 0x01) << 1);
}

static void isrLeft() {
  uint8_t s = readLeft();
  g_left += QDEC[(g_lPrev << 2) | s];
  g_lPrev = s;
}

static void isrRight() {
  uint8_t s = readRight();
  g_right += QDEC[(g_rPrev << 2) | s];
  g_rPrev = s;
}

void encoderBegin() {
  // 엔코더 출력은 보통 오픈 컬렉터라 내부 풀업을 켠다 (모듈에 풀업이 있어도 문제없음)
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  g_lPrev = readLeft();
  g_rPrev = readRight();
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), isrRight, CHANGE);
}

void encoderRead(long* left, long* right) {
  noInterrupts();          // 4바이트 값을 읽는 도중에 ISR이 바꾸지 않게
  long l = g_left, r = g_right;
  interrupts();
  *left = ENC_L_INVERT ? -l : l;
  *right = ENC_R_INVERT ? -r : r;
}
