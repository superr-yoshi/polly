# poli_hardware — 하드웨어 계층 (조원 A)

"앞으로 가", "멈춰", "집게 닫아" 같은 명령이 실제 장치를 움직이게 하는 계층이다.
상위(Nav2·미션)는 fake인지 real인지 몰라도 되도록 **Topic 계약을 고정**한다.

## 노드
| 노드 | 상태 | 역할 |
|---|---|---|
| `fake_rrc_node` | 사용 가능 | 부품 없이 /cmd_vel → /odom_raw, /imu/data (SIM_ONLY) |
| `rrc_adapter_node` | 구현됨, 실기 미검증 | RRC Lite 실제 주행 (`docs/rrc_protocol.md`, `docs/rrc_adapter_plan.md`) |
| `fake_mega_node` | 사용 가능 | 부품 없이 /range/*, 집게 서비스 (SIM_ONLY) |
| `mega_bridge_node` | 사용 가능 | Arduino Mega 시리얼 v1.1 ↔ ROS |

## 인터페이스 계약 (바꾸지 말 것)
| 이름 | 종류 | 타입 | 방향 | frame_id |
|---|---|---|---|---|
| `/cmd_vel` | Topic | geometry_msgs/Twist | 구독 | — |
| `/odom_raw` | Topic | nav_msgs/Odometry | 발행 25 Hz | `odom` → child `base_link` |
| `/imu/data` | Topic | sensor_msgs/Imu | 발행 50 Hz | `imu_link` |
| `/range/front_left` `/range/front_right` `/range/rear_left` `/range/rear_right` | Topic | sensor_msgs/Range | 발행 8 Hz | `ultrasonic_fl_link` 등 |
| `/gripper/state` | Topic | std_msgs/String | 발행 2 Hz | `open` / `closed` / `moving` |
| `/gripper/set` | Service | std_srvs/SetBool | 제공 | `true` = 닫기 |

- odom → base_link **TF는 발행하지 않는다** (robot_localization EKF 소유).
- `/cmd_vel`이 `cmd_timeout`(0.3 s) 이상 끊기면 0 속도.
- `/imu/data`: orientation 미제공(`orientation_covariance[0] = -1`). real = RRC 내장 IMU 가속도·자이로 3축, fake = z축 각속도만(휠 각속도 복사).
- `/odom_raw`: real도 엔코더가 아니라 **명령 기반 추정**이다 (RRC 제조사 펌웨어가 엔코더 값을 보내지 않음).
- `/battery_state`는 발행하지 않는다 (전압 측정 회로 없음, 프로토콜 v1.1에서 `$BAT` 삭제).

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
ros2 topic hz /range/front_left       # 약 8 Hz
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
ros2 topic echo /odom_raw --once      # twist.linear.x = 0.2 (0.3 s 뒤에는 0)
ros2 service call /gripper/set std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /gripper/state        # moving → closed
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 키보드 주행
```

## 단위 테스트 (ROS 없이도 가능)
```bash
pip install pytest
python -m pytest ros2_ws/src/poli_hardware/test -q     # Mega·RRC 프로토콜 + 차동구동 계산
```

## 설정
`config/hardware.yaml` — `TODO_MEASURE`(실측 필수) / `SIM_ONLY`(임시값) 표시를 지우기 전에 실측한다.
