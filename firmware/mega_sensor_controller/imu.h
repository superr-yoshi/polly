#pragma once
#include <Arduino.h>

struct ImuSample {
  long q[4];      // 방향 쿼터니언 x, y, z, w x 10000 (BNO085 Game Rotation Vector: 자기장 안 씀, yaw는 켤 때 0)
  long gyro[3];   // 각속도 x, y, z (mrad/s)
  long accel[3];  // 가속도 x, y, z (mm/s^2, 중력 포함)
};

void imuBegin();            // BNO085를 찾는다. 못 찾으면 $IMU를 보내지 않는다 (Pi는 휠 각속도로 대신함)
bool imuUpdate();           // loop에서 매번 호출. 새 값이 있었으면 true
bool imuReady();            // 센서를 찾았고 값을 한 번 이상 받았으면 true
const ImuSample& imuSample();
