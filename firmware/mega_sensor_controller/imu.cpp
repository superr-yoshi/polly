#include <Arduino.h>
#include "config.h"
#include "imu.h"

static ImuSample g_s = {{0, 0, 0, 10000}, {0, 0, 0}, {0, 0, 0}};
static bool g_found = false, g_ready = false;

#if SIM_IMU
// BNO085가 없으면 아무것도 보내지 않는다 (가짜 IMU 값을 진짜처럼 보내지 않음)
void imuBegin() {}
bool imuUpdate() { return false; }
#else
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>   // arduino-cli lib install "SparkFun BNO080 Cortex Based IMU"

static BNO080 g_bno;

void imuBegin() {
  Wire.begin();
  Wire.setClock(400000);
  // I2C가 멈춰도(선 빠짐·잡음) loop 전체가 멈추지 않게 3 ms timeout. loop가 멈추면 모터 timeout도 못 지킨다
  Wire.setWireTimeout(3000, true);
  g_found = g_bno.begin(IMU_I2C_ADDR, Wire);
  if (!g_found) return;
  g_bno.enableGameRotationVector(IMU_REPORT_MS);
  g_bno.enableGyro(IMU_REPORT_MS);
  g_bno.enableAccelerometer(IMU_REPORT_MS);
}

bool imuUpdate() {
  if (!g_found) return false;
  bool fresh = false;
  for (uint8_t i = 0; i < 3 && g_bno.dataAvailable(); i++) fresh = true;  // 쌓인 보고 3개까지 읽기
  if (!fresh) return false;
  g_s.q[0] = lround(g_bno.getQuatI() * 10000.0f);
  g_s.q[1] = lround(g_bno.getQuatJ() * 10000.0f);
  g_s.q[2] = lround(g_bno.getQuatK() * 10000.0f);
  g_s.q[3] = lround(g_bno.getQuatReal() * 10000.0f);
  g_s.gyro[0] = lround(g_bno.getGyroX() * 1000.0f);
  g_s.gyro[1] = lround(g_bno.getGyroY() * 1000.0f);
  g_s.gyro[2] = lround(g_bno.getGyroZ() * 1000.0f);
  g_s.accel[0] = lround(g_bno.getAccelX() * 1000.0f);
  g_s.accel[1] = lround(g_bno.getAccelY() * 1000.0f);
  g_s.accel[2] = lround(g_bno.getAccelZ() * 1000.0f);
  g_ready = true;
  return true;
}
#endif

bool imuReady() { return g_found && g_ready; }
const ImuSample& imuSample() { return g_s; }
