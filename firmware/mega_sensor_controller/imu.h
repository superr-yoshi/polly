#pragma once
#include <Arduino.h>

struct ImuSample {
  long q[4];      // 방향 쿼터니언 x, y, z, w x 10000. MPU-6050은 방향을 계산하지 않으므로 항상 0 (= 방향 없음)
  long gyro[3];   // 각속도 x, y, z (mrad/s)
  long accel[3];  // 가속도 x, y, z (mm/s^2, 중력 포함)
};

void imuBegin();            // MPU-6050을 찾고 자이로 영점을 잰다 (약 1초, 가만히). 못 찾으면 $IMU를 보내지 않는다
bool imuUpdate();           // loop에서 매번 호출. 새 값이 있었으면 true
bool imuReady();            // 센서를 찾았고 최근 0.2초 안에 값을 읽었으면 true
const ImuSample& imuSample();
