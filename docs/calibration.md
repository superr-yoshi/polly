# 주행 보정 기록

`drive_test`(poli_hardware) 결과와 반영한 값을 날짜와 함께 남긴다. 값은 `config/hardware.yaml`에 반영한다.

## 순서 (반드시 이 순서로)
1. **바퀴 띄우고** `ros2 run poli_hardware drive_test wheel --revs 10` → 센 바퀴 수 입력 → `motor_ticks_per_rev`
2. 바닥에서 `drive_test straight --distance 1.0` → 줄자로 잰 거리 입력 → `wheel_radius`
3. 바닥에서 `drive_test rotate --angle 360` → Enter(IMU 값 사용) 또는 실측 각도 → `wheel_separation`
4. 각 시험 3회 반복, 평균을 반영. 반영 후 다시 1회 돌려 오차 확인.

- 시험 전 `ros2 launch poli_hardware hardware.launch.py use_fake_hardware:=false` 실행
- 현재 값은 설치된 hardware.yaml에서 자동으로 읽는다. yaml을 고친 뒤에는 `colcon build` 후 다시 실행.
- Ctrl+C로 언제든 정지. 물리 E-stop을 손 닿는 곳에 둔다.
- `min_wheel_speed`(0.25 m/s) 때문에 느린 명령은 빨라진다. drive_test는 노드와 같은 계산으로 실제 속도·시간을 정한다
  (제자리 회전은 약 2.8 rad/s). 기본 속도: 직진 0.25 m/s, 바퀴 약 1.22 rps.
- 바퀴 수는 손으로 세기 빠르다 → 바퀴에 표시하고 **영상으로 세기**, 20 s 이상 돌리기.

## 기록
| 날짜 | 시험 | 명령 | odom | 실측 / IMU | 계산값 | 반영 |
|---|---|---|---|---|---|---|
| 2026-10-08 | wheel (바퀴 띄움, 두 모터, 12 V, 노트북) | 0.25 m/s × 20 s = 24.49 rev | 24.49 | 손으로 셈 24 | 1320 × 24.49 / 24 ≈ 1347 (셈 오차 범위) | **1320 유지** (10 s 시험 10.5는 셈 오차로 판단) |
| | wheel | 10 rev | | | motor_ticks_per_rev = | |
| | straight | 1.0 m | | | wheel_radius = | |
| | rotate | 360° | | | wheel_separation = | |
