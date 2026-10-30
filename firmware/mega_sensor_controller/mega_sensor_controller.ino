// Mega 센서·주행 컨트롤러 (조원 A)
// 담당: 초음파 4개 + 집게 서보 1개 + 주행 모터(MDD10) 속도 제어 + 엔코더 2개 + IMU(BNO085) + Pi와 시리얼 통신
// 2026-10-29: RRC Lite를 쓰지 않고 MDD10으로 바꾸면서 모터·엔코더·IMU를 Mega가 맡는다 (docs/serial_protocol.md v2.0).
#include "config.h"
#include "serial_protocol.h"
#include "ultrasonic.h"
#include "gripper.h"
#include "encoder.h"
#include "motor.h"
#include "imu.h"

static unsigned long seqRng = 0, seqGst = 0, seqEnc = 0, seqImu = 0;
static unsigned long tUs = 0, tRng = 0, tGst = 0, tHb = 0, tEnc = 0, tImu = 0;
static uint8_t usIdx = 0;
static long lastCmdId = 0;          // 마지막으로 처리한 $GRIP 명령 id
static char buf[128];
static char rx[64];
static uint8_t rxLen = 0;

static void sendGst() {
  snprintf(buf, sizeof(buf), "GST,%lu,%lu,%ld,%d,%d",
           seqGst++, millis(), lastCmdId, (int)gripperState(), gripperAngle());
  sendPacket(buf);
}

static void sendEnc() {
  long l, r;
  encoderRead(&l, &r);
  snprintf(buf, sizeof(buf), "ENC,%lu,%lu,%ld,%ld,%d",
           seqEnc++, millis(), l, r, (int)motorStatus());
  sendPacket(buf);
}

static void sendImu() {
  const ImuSample& s = imuSample();
  snprintf(buf, sizeof(buf), "IMU,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
           seqImu++, millis(), s.q[0], s.q[1], s.q[2], s.q[3],
           s.gyro[0], s.gyro[1], s.gyro[2], s.accel[0], s.accel[1], s.accel[2]);
  sendPacket(buf);
}

// "a,b" 정수 두 개 (부호 허용). 성공하면 true
static bool parseTwo(char* p, long* a, long* b) {
  char* end;
  *a = strtol(p, &end, 10);
  if (end == p || *end != ',') return false;
  p = end + 1;
  *b = strtol(p, &end, 10);
  return end != p && *end == 0;
}

// 한 줄 예: "$GRIP,7,1*0C", "$DRV,1616,-1616*CS"  (\r\n은 제거된 상태)
static void handleLine(char* line) {
  if (line[0] != '$') return;
  char* star = strrchr(line, '*');
  if (!star || strlen(star) != 3) return;
  *star = 0;
  char* body = line + 1;
  char* end;
  long cs = strtol(star + 1, &end, 16);
  if (*end != 0 || cs != nmeaChecksum(body)) return;      // 체크섬 오류 → 폐기

  long a, b;
  if (strncmp(body, "DRV,", 4) == 0) {                    // 목표 바퀴 속도 tick/s (Pi 25 Hz)
    if (parseTwo(body + 4, &a, &b)) motorDrive(a, b);
  } else if (strncmp(body, "PWM,", 4) == 0) {             // 직접 duty (배선·방향 확인용)
    if (parseTwo(body + 4, &a, &b)) motorPwm((int)constrain(a, -255L, 255L),
                                             (int)constrain(b, -255L, 255L));
  } else if (strncmp(body, "GRIP,", 5) == 0) {
    if (!parseTwo(body + 5, &a, &b) || (b != 0 && b != 1)) return;
    gripperCommand((uint8_t)b);
    lastCmdId = a;
    sendGst();                                            // 즉시 응답
  }                                                       // 모르는 명령 → 무시
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
  encoderBegin();
  motorBegin();                          // 가장 먼저 모터 출력을 0으로
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  ultrasonicBegin();
  gripperBegin();
  imuBegin();
}

void loop() {
  unsigned long now = millis();

  pollSerial();
  motorUpdate();
  gripperUpdate();
  imuUpdate();

  if (now - tUs >= 30) {                 // 센서 1개씩 순환 측정 → 4개가 약 120 ms에 한 바퀴
    tUs += 30;
    ultrasonicUpdate(usIdx);
    usIdx = (usIdx + 1) & 3;
    now = millis();                      // 초음파 측정이 최대 24 ms 걸린다
  }
  if (now - tEnc >= ENC_REPORT_MS) {     // 50 Hz. 밀린 만큼 몰아 보내지 않는다 (누적 tick이라 손실 없음)
    tEnc = now;
    sendEnc();
  }
  if (imuReady() && now - tImu >= IMU_REPORT_MS) {   // 25 Hz, BNO085가 있을 때만
    tImu = now;
    sendImu();
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
