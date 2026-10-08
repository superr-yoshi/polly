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
   → 2026-10-06 보드 단독으로 확인 완료 (아래 기록). 로봇에 장착한 뒤 보드 방향이 바뀌면 다시 확인한다.
7. `drive_test straight --distance 1.0`, `drive_test rotate --angle 360`으로 `wheel_radius`,
   `wheel_separation` 보정 → `docs/calibration.md`에 날짜와 기록.

## 지켜야 할 것
- Topic 이름·타입·frame_id·parameter 이름을 바꾸지 않는다.
- STM32는 명령이 끊겨도 스스로 멈추지 않는다 → **물리 E-stop 필수**, 시험은 바퀴를 띄우고 시작.
- `/odom_raw`는 엔코더가 아니라 명령 기반 추정이다 (펌웨어 한계). EKF 설정 때 통합 담당에게 알린다.

## 실기 확인 기록
| 날짜 | 내용 | 결과 |
|---|---|---|
| 2026-10-06 | RRC Lite 단독 (모터 미연결, Windows COM8 수신만) | 프레임 152개 / 3 s, CRC 오류 0, IMU 50.0 Hz, 전압 보고 약 1 Hz → **프로토콜 분석 일치** |
| 2026-10-06 | WSL `hardware.launch.py use_fake_hardware:=false` (RRC `/dev/ttyACM1` + Mega `/dev/ttyACM0`) | `/imu/data` 50 Hz, `/odom_raw` 25 Hz, `/range/*` 8.8 Hz, `/gripper/state` 2 Hz, `/battery_state` 1 Hz. cmd_vel 0.1 → odom 0.100, 끊으면 0. 종료 정상 |
| 2026-10-06 | IMU 정지 상태 | 가속도 z = +10.06 m/s² (위쪽 +, ROS 축과 일치). 자이로 정지 오차 약 1~2 °/s |

| 2026-10-06 | 입력 전압 (서플라이 12.0 V) | 11.98 V 보고 → `/battery_state` 정상 |
| 2026-10-06 | IMU 회전 방향 (보드를 손에 들고 위에서 봤을 때 반시계 90° → 정지 → 시계 90°) | 반시계 **+82.7°**, 시계 **−93.6°** → z축 반시계 = + (ROS 규칙과 일치). URDF `imu_yaw` 등 변경 불필요. 정지 오차 약 −0.1 °/s, 15 s 누적 1.8° |

| 2026-10-08 | 모터 폭주 원인 찾기 (모터 1개, 바퀴 띄움, 서플라이 12 V) | ① 모터 쪽 흰 플러그 **C1·C2 접점 불량**으로 엔코더 신호가 RRC 핀(PB3/PA15)에 안 들어감 → 어느 설정·방향이든 폭주. 플러그 교체 후 RRC 핀에서 0 ↔ 4 V 확인 ② 신호가 들어간 뒤 일자 케이블 + JGB37 = 폭주, JGA27 = 진동 후 정지 → **엔코더 방향 반대**. 한쪽 끝에서 **가운데 두 선(C1·C2)을 바꿔** 해결 ③ JGB37 + ROS 노드: 0.1·0.2 m/s 울컥, **전진·후진 0.25 m/s 부드러움(15 s 연속), 정지 정상** → 주행 속도 0.25 m/s로 결정 |

- 주의: **RRC 전원 스위치가 꺼져 있어도 USB가 꽂혀 있으면 보드가 USB 전원으로 켜진다** (IMU·통신은 되지만 모터는 안 돎).
  이때 입력 전압 보고가 약 4.1 V로 나온다. 모터 시험 전 `/battery_state`가 약 12 V인지 확인한다.
- ⚠ **USB 전원만으로도 모터가 (느리게) 돈다** (2026-10-08 확인). 폭주 중 서플라이·스위치를 꺼도 USB가 꽂혀 있으면 계속 돈다.
  노트북 시험의 비상정지 = **USB 뽑기 + 서플라이 OFF**. 실차는 배터리 뒤 비상정지가 Pi·RRC를 함께 끊는다.
- ⚠ **엔코더 신호가 없거나 방향이 반대면 정지 명령으로 못 멈춘다** (공장 펌웨어 PID가 PWM을 누적하는 방식,
  정지 = 목표 0일 뿐). 새 모터·케이블은 반드시 바퀴를 띄우고 0.3 s 이하로 먼저 시험한다.
- 엔코더 케이블 확인법: USB만 꽂고 RRC 뒷면 포트 글씨(M1: PA1/PA0, M2: PB3/PA15)와 GND 사이 전압을 재면서
  모터 뒤 자석 원판을 손으로 조금씩 돌린다 → 두 핀이 0 ↔ 약 4 V로, 서로 어긋나게(00 → 01 → 11 → 10) 바뀌어야 정상.
