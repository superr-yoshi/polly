# RRC Lite 어댑터 — 보드 도착 후 확인 순서

통신 코드(`RrcTransport`)는 제조사 펌웨어 분석(`docs/rrc_protocol.md`)을 바탕으로 구현되어 있다.
보드가 오면 **실측값만 채우고 동작을 확인**한다.

## 실측해서 `config/hardware.yaml`에 넣을 값
| 파라미터 | 확인 방법 |
|---|---|
| `motor_ticks_per_rev` | **1320 입력됨** (JGB37-520 12V 330RPM = 30:1 × 11 PPR × 4). 5번 속도 시험으로 확인. PPR 12 변형이면 1440 |
| `left_sign`, `right_sign` | 바퀴 띄우고 전진 명령 → 둘 다 앞으로 돌아야 함. 반대면 부호 반전 |
| `left_motor_id`, `right_motor_id` | 실제 M1/M2에 어느 바퀴를 꽂았는지 |
| `wheel_radius`, `wheel_separation` | 직선 1~2 m, 제자리 360° 시험 |

## 순서
0. RRC Lite는 **공장 펌웨어 그대로** 쓴다 (2026-10-01 결정). 패치 펌웨어(`firmware/rrc_lite_patch/`)는 보류.
1. Pi에 연결 후 udev 별칭 `/dev/robot_rrc` 설정 (`docs/udev_template.md`)
2. **바퀴를 띄운 상태**에서 실행:
   ```bash
   ros2 launch poli_hardware hardware.launch.py use_fake_hardware:=false
   ros2 topic hz /imu/data           # 약 50 Hz면 통신 정상 (IMU 수신 확인)
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
3. 전진/후진/좌회전/우회전/정지 확인. 방향이 틀리면 sign·motor_id만 고친다 (배선 변경 X, 기록 O).
4. teleop을 끄고 0.3초 안에 바퀴가 서는지 확인 (Pi 쪽 watchdog).
5. 바퀴 1회전 속도 확인: `ros2 run poli_hardware drive_test wheel --revs 10` → 센 바퀴 수 입력
   → `motor_ticks_per_rev` 계산값 반영.
6. IMU 축 확인: 정지 시 az ≈ +9.8, 좌회전 시 gz > 0. 다르면 URDF의 imu_link 회전으로 맞춘다.
7. `drive_test straight --distance 1.0`, `drive_test rotate --angle 360`으로 `wheel_radius`,
   `wheel_separation` 보정 → `docs/calibration.md`에 날짜와 기록.

## 지켜야 할 것
- Topic 이름·타입·frame_id·parameter 이름을 바꾸지 않는다.
- STM32는 명령이 끊겨도 스스로 멈추지 않는다 → **물리 E-stop 필수**, 시험은 바퀴를 띄우고 시작.
- `/odom_raw`는 엔코더가 아니라 명령 기반 추정이다 (펌웨어 한계). EKF 설정 때 통합 담당에게 알린다.
