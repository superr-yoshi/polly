// Mega 센서/집게 컨트롤러 (조원 A)
// 담당: 초음파 4개 + 집게 서보 1개 + Pi와 시리얼 통신
// IMU / 모터 / 엔코더는 RRC Lite 담당이라 여기서 다루지 않는다.
#include "config.h"
#include "serial_protocol.h"
#include "ultrasonic.h"
#include "gripper.h"

static unsigned long seqRng = 0, seqGst = 0;
static unsigned long tUs = 0, tRng = 0, tGst = 0, tHb = 0;
static uint8_t usIdx = 0;
static long lastCmdId = 0;          // 마지막으로 처리한 $GRIP 명령 id
static char buf[96];
static char rx[64];
static uint8_t rxLen = 0;

static void sendGst() {
  snprintf(buf, sizeof(buf), "GST,%lu,%lu,%ld,%d,%d",
           seqGst++, millis(), lastCmdId, (int)gripperState(), gripperAngle());
  sendPacket(buf);
}

// 한 줄 예: "$GRIP,7,1*0C"  (\r\n은 제거된 상태)
static void handleLine(char* line) {
  if (line[0] != '$') return;
  char* star = strrchr(line, '*');
  if (!star || strlen(star) != 3) return;
  *star = 0;
  char* body = line + 1;
  char* end;
  long cs = strtol(star + 1, &end, 16);
  if (*end != 0 || cs != nmeaChecksum(body)) return;      // 체크섬 오류 → 폐기

  if (strncmp(body, "GRIP,", 5) != 0) return;             // 모르는 명령 → 무시
  char* p = body + 5;
  long id = strtol(p, &end, 10);
  if (end == p || *end != ',') return;
  p = end + 1;
  long action = strtol(p, &end, 10);
  if (end == p || *end != 0 || (action != 0 && action != 1)) return;

  gripperCommand((uint8_t)action);
  lastCmdId = id;
  sendGst();                                              // 즉시 응답
}

static void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      rx[rxLen] = 0;
      if (rxLen > 0) handleLine(rx);
      rxLen = 0;
    } else if (c != '\r') {
      if (rxLen < sizeof(rx) - 1) rx[rxLen++] = c;
      else rxLen = 0;                                     // 너무 긴 줄 → 폐기
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  ultrasonicBegin();
  gripperBegin();
}

void loop() {
  unsigned long now = millis();

  pollSerial();
  gripperUpdate();

  if (now - tUs >= 30) {                 // 센서 1개씩 순환 측정 → 4개가 약 120 ms에 한 바퀴
    tUs += 30;
    ultrasonicUpdate(usIdx);
    usIdx = (usIdx + 1) & 3;
  }
  if (now - tRng >= 125) {               // 8 Hz
    tRng += 125;
    snprintf(buf, sizeof(buf), "RNG,%lu,%lu,%ld,%ld,%ld,%ld",
             seqRng++, now, ultrasonicGetMm(0), ultrasonicGetMm(1),
             ultrasonicGetMm(2), ultrasonicGetMm(3));
    sendPacket(buf);
  }
  if (now - tGst >= 500) {               // 2 Hz
    tGst += 500;
    sendGst();
  }
  if (now - tHb >= 1000) {               // 1초마다 LED 토글 (동작 표시)
    tHb += 1000;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
