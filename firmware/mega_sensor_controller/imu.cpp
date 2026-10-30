#include <Arduino.h>
#include "config.h"
#include "imu.h"

// MPU-6050 (GY-521 등). 2026-10-30 BNO085 대신 사용.
// 자이로·가속도만 있고 방향(쿼터니언)은 계산하지 않는다 -> q는 0,0,0,0 ("방향 없음", docs/serial_protocol.md)
static ImuSample g_s = {{0, 0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

#if SIM_IMU
// IMU가 없으면 아무것도 보내지 않는다 (가짜 IMU 값을 진짜처럼 보내지 않음)
void imuBegin() {}
bool imuUpdate() { return false; }
bool imuReady() { return false; }
#else
#include <Wire.h>

static bool g_found = false, g_ready = false;

// MPU-6050 레지스터
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B   // 여기서부터 14바이트: 가속도 xyz, 온도, 자이로 xyz (big endian)
#define REG_PWR_MGMT_1   0x6B

// 자이로 ±500 dps = 65.5 LSB/(deg/s) -> mrad/s, 가속도 ±4 g = 8192 LSB/g -> mm/s^2
static const float GYRO_MRAD_PER_LSB = 1000.0f * (PI / 180.0f) / 65.5f;
static const float ACCEL_MM_PER_LSB = 9806.65f / 8192.0f;

static float g_bias[3] = {0, 0, 0};   // 자이로 영점 (raw), imuBegin에서 정지 상태로 잰다
static unsigned long g_tRead = 0, g_tOk = 0;

static bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool readRaw(int16_t raw[7]) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)IMU_I2C_ADDR, (uint8_t)14) != 14) return false;
  for (uint8_t i = 0; i < 7; i++) {
    uint8_t hi = Wire.read(), lo = Wire.read();
    raw[i] = (int16_t)((hi << 8) | lo);
  }
  return true;
}

void imuBegin() {
  Wire.begin();
  Wire.setClock(400000);
  // I2C가 멈춰도(선 빠짐·잡음) loop 전체가 멈추지 않게 3 ms timeout. loop가 멈추면 모터 timeout도 못 지킨다
  Wire.setWireTimeout(3000, true);
  g_found = writeReg(REG_PWR_MGMT_1, 0x01)     // 잠에서 깨우고 자이로 X 클록 사용
            && writeReg(REG_SMPLRT_DIV, 9)     // 1 kHz / (1 + 9) = 100 Hz
            && writeReg(REG_CONFIG, 3)         // 저역 필터 약 44 Hz
            && writeReg(REG_GYRO_CONFIG, 0x08) // ±500 dps
            && writeReg(REG_ACCEL_CONFIG, 0x08);  // ±4 g
  if (!g_found) return;
  delay(100);
  // 자이로 영점: 켤 때 로봇이 가만히 있다고 보고 약 1초 평균 (센서 초기화, 바퀴·집게는 움직이지 않음)
  long sum[3] = {0, 0, 0};
  int n = 0;
  int16_t raw[7];
  for (int i = 0; i < IMU_BIAS_SAMPLES; i++) {
    if (readRaw(raw)) {
      for (uint8_t k = 0; k < 3; k++) sum[k] += raw[4 + k];
      n++;
    }
    delay(5);
  }
  if (n == 0) { g_found = false; return; }
  for (uint8_t k = 0; k < 3; k++) g_bias[k] = (float)sum[k] / n;
}

bool imuUpdate() {
  if (!g_found) return false;
  unsigned long now = millis();
  if (now - g_tRead < IMU_REPORT_MS) return false;
  g_tRead = now;
  int16_t raw[7];
  if (!readRaw(raw)) return false;
  for (uint8_t k = 0; k < 3; k++) {
    g_s.accel[k] = lround(raw[k] * ACCEL_MM_PER_LSB);
    g_s.gyro[k] = lround((raw[4 + k] - g_bias[k]) * GYRO_MRAD_PER_LSB);
  }
  g_tOk = now;
  g_ready = true;
  return true;
}

// 읽기가 0.2초 넘게 실패하면(선 빠짐) $IMU를 멈춘다 -> Pi는 휠 각속도로 대신함. 옛 값을 계속 보내지 않는다
bool imuReady() { return g_found && g_ready && millis() - g_tOk < 200; }
#endif

const ImuSample& imuSample() { return g_s; }
