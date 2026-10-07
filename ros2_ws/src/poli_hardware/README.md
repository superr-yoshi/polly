# poli_hardware — 하드웨어 계층 (조원 A)

"앞으로 가", "멈춰", "집게 닫아" 같은 명령이 실제 장치를 움직이게 하는 계층이다.
상위(Nav2·미션)는 fake인지 real인지 몰라도 되도록 **Topic 계약을 고정**한다.

## 노드
| 노드 | 상태 | 역할 |
|---|---|---|
| `fake_rrc_node` | 사용 가능 | 부품 없이 /cmd_vel → /odom_raw, /imu/data, 집게 (SIM_ONLY) |
| `rrc_adapter_node` | 구현됨, 실기 미검증 | RRC Lite 실제 주행 + 집게 서보(PWM 포트) (`docs/rrc_protocol.md`, `docs/rrc_adapter_plan.md`) |
| `fake_mega_node` | 사용 가능 | 부품 없이 /range/* (SIM_ONLY) |
| `mega_bridge_node` | 사용 가능 (실물 Mega 확인) | Arduino Mega 시리얼 v1.1 ↔ ROS |
| `drive_test` | 사용 가능 | 보정용 주행 시험 (직진/회전/바퀴 N바퀴) → 새 파라미터 계산. `docs/calibration.md` |

## 인터페이스 계약 (바꾸지 말 것)
| 이름 | 종류 | 타입 | 방향 | frame_id |
|---|---|---|---|---|
| `/cmd_vel` | Topic | geometry_msgs/Twist | 구독 | — |
| `/odom_raw` | Topic | nav_msgs/Odometry | 발행 25 Hz | `odom` → child `base_link` |
| `/imu/data` | Topic | sensor_msgs/Imu | 발행 50 Hz | `imu_link` |
| `/range/front` `/range/left` `/range/right` `/range/rear` | Topic | sensor_msgs/Range | 발행 8 Hz | `ultrasonic_front_link` 등 |
| `/gripper/state` | Topic | std_msgs/String | 발행 2 Hz | `open` / `closed` / `moving` |
| `/gripper/command` | Topic | std_msgs/String | 구독 | `"open"` 열기 / `"grab"` 닫아 잡기 (**들어 올리기 없음**) |
| `/battery_state` | Topic | sensor_msgs/BatteryState | 발행 1 Hz | RRC 입력 전압 (LiPo 잔량 아님) |

- odom → base_link **TF는 발행하지 않는다** (robot_localization EKF 소유).
- `/cmd_vel`이 `cmd_timeout`(0.3 s) 이상 끊기면 0 속도.
- `/imu/data`: orientation 미제공(`orientation_covariance[0] = -1`). real = RRC 내장 IMU 가속도·자이로 3축, fake = z축 각속도만(휠 각속도 복사).
- `/odom_raw`: RRC 공장 펌웨어라 **명령 기반 추정**이다 (엔코더 값이 안 옴). 보류된 패치 펌웨어(`firmware/rrc_lite_patch/`)를 구우면 자동으로 엔코더 기반이 된다.
- `/battery_state`: RRC Lite가 보고하는 자기 입력 전압. RRC는 12V 컨버터 뒤에 있어서 LiPo 잔량이 아니다. fake는 12.0 V.
- `/gripper/*`: 2026-10-07부터 RRC Lite PWM 서보 포트(`rrc_adapter_node`). 서보 포트 전원 점퍼 **5V 필수**.
  첫 명령 전에는 서보 신호를 보내지 않는다 (RRC가 전원 켤 때 내는 1500 µs = 열림).
- `/gripper/holding`: 없음. 잡힘 감지 센서 추가 후 제공 예정 (TODO).
- 기준: `docs/interfaces.md` (담당 1과 약속한 토픽). `/cmd_vel` timeout 0.3 s (약속: 0.5 s 이내).

## 실행 (Ubuntu 24.04 + ROS 2 Jazzy)
```bash
cd ~/polly/ros2_ws
colcon build --symlink-install --packages-select poli_hardware
source install/setup.bash

ros2 launch poli_hardware hardware.launch.py                          # fake
ros2 launch poli_hardware hardware.launch.py use_fake_hardware:=false  # 실제 RRC + Mega
```

## 검증 (통과 기준: 매뉴얼 12장)
```bash
ros2 topic list                       # /odom_raw /imu/data /range/* /gripper/state
ros2 topic hz /imu/data               # 약 50 Hz
ros2 topic hz /range/front            # 약 8 Hz
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
ros2 topic echo /odom_raw --once      # twist.linear.x = 0.2 (0.3 s 뒤에는 0)
ros2 topic pub --once /gripper/command std_msgs/msg/String "{data: grab}"
ros2 topic echo /gripper/state        # moving → closed
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 키보드 주행
```

## 단위 테스트 (ROS 없이도 가능)
```bash
pip install pytest
cd ros2_ws/src/poli_hardware && python -m pytest test -q   # 프로토콜·차동구동·보정 계산 (스타일 검사는 ROS에서만)
```

## 설정
`config/hardware.yaml` — `TODO_MEASURE`(실측 필수) / `SIM_ONLY`(임시값) 표시를 지우기 전에 실측한다.
