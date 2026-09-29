#pragma once
#include <Arduino.h>

// $ 와 * 사이 모든 문자의 XOR
inline uint8_t nmeaChecksum(const char* s) {
  uint8_t c = 0;
  while (*s) c ^= (uint8_t)*s++;
  return c;
}

// body = "RNG,..." ($, * 제외). 한 줄(\r\n)로 전송
inline void sendPacket(const char* body) {
  char cs[3];
  snprintf(cs, sizeof(cs), "%02X", nmeaChecksum(body));
  Serial.print('$'); Serial.print(body); Serial.print('*'); Serial.println(cs);
}
