# poli_hardware — 하드웨어 계층 (조원 A)

"앞으로 가", "멈춰", "집게 닫아" 같은 명령이 실제 장치를 움직이게 하는 계층이다.
상위(Nav2·미션)는 fake인지 real인지 몰라도 되도록 **Topic 계약을 고정**한다.

## 노드
| 노드 | 상태 | 역할 |
|---|---|---|
| `fake_mega_node` | 사용 가능 | 부품 없이 /cmd_vel → /odom_raw, /imu/data, /range/*, 집게 (SIM_ONLY) |
| `mega_bridge_node` | 구현됨, MDD10 실기 미검증 | Arduino Mega 시리얼 v2.0 ↔ ROS. 주행(MDD10)·엔코더·IMU(BNO085)·초음파·집게를 **한 노드**가 맡는다 |
| `drive_test` | 사용 가능 | 보정용 주행 시험 (직진/회전/바퀴 N바퀴) → 새 파라미터 계산. `docs/calibration.md` |

2026-10-29: RRC Lite를 쓰지 않는다 (모터 드라이버 Cytron MDD10 + Mega). `fake_rrc_node`, `rrc_adapter_node`는 삭제했다.
Mega USB 포트 하나를 한 노드만 열 수 있어서 초음파·집게도 `mega_bridge_node`가 함께 맡는다.

## 인터페이스 계약 (바꾸지 말 것)
| 이름 | 종류 | 타입 | 방향 | frame_id |
|---|---|---|---|---|
| `/cmd_vel` | Topic | geometry_msgs/Twist | 구독 | — |
| `/odom_raw` | Topic | nav_msgs/Odometry | 발행 25 Hz | `odom` → child `base_link` |
| `/imu/data` | Topic | sensor_msgs/Imu | 발행 25 Hz | `imu_link` |
| `/range/front` `/range/left` `/range/right` `/range/rear` | Topic | sensor_msgs/Range | 발행 8 Hz | `ultrasonic_front_link` 등 |
| `/gripper/state` | Topic | std_msgs/String | 발행 2 Hz + 바뀔 때 | `open` / `closed` / `moving` |
| `/gripper/command` | Topic | std_msgs/String | 구독 | `"open"` 열기 / `"grab"` 닫아 잡기 (**들어 올리기 없음**) |

- odom → base_link **TF는 발행하지 않는다** (robot_localization EKF 소유).
- `/cmd_vel`이 `cmd_timeout`(0.3 s) 이상 끊기면 0 속도. 노드가 죽거나 USB가 빠져도 **Mega 펌웨어가 0.3 s 뒤 스스로 멈춘다**.
- `/odom_raw`: **엔코더 기반** (Mega가 4체배로 센 tick, `$ENC` 50 Hz). fake는 명령 그대로.
- `/imu/data`: BNO085가 연결되어 있으면 방향(쿼터니언, 자기장 안 씀)·각속도·가속도 3축.
  BNO085가 없거나(`SIM_IMU 1`) 끊기면 `orientation_covariance[0] = -1`, z축 각속도만 휠 값으로 채운다 (fake도 같음).
- `/battery_state`: **없음** (2026-10-29). 전압을 재던 RRC Lite를 뺐고 Mega에 전압 측정 회로가 없다. 필요하면 분압 회로 + `$BAT`를 추가한다 (TODO).
- `/gripper/*`: 집게 서보 DS3218은 다시 **Mega D9** (전원 5V 10A 컨버터, GND 공통). 첫 명령 전에는 서보 신호를 보내지 않는다 (규정 3.5.6).
- `/gripper/holding`: 없음. 잡힘 감지 센서 추가 후 제공 예정 (TODO).
- 모터 고장 정지: Mega가 엔코더 방향 반대·신호 없음(또는 바퀴 막힘)을 감지하면 모터를 끄고, Mega가 리셋될 때까지(= 이 노드 재시작) 다시 돌리지 않는다. 로그에 `Mega 모터 정지: fault_...`가 나온다.
- 기준: `docs/interfaces.md` (담당 1과 약속한 토픽). `/cmd_vel` timeout 0.3 s (약속: 0.5 s 이내).

## 실행 (Ubuntu 24.04 + ROS 2 Jazzy)
```bash
cd ~/polly/ros2_ws
colcon build --symlink-install --packages-select poli_hardware
source install/setup.bash

ros2 launch poli_hardware hardware.launch.py                          # fake
ros2 launch poli_hardware hardware.launch.py use_fake_hardware:=false  # 실제 Mega (MDD10)
```

## 검증 (통과 기준: 매뉴얼 12장)
```bash
ros2 topic list                       # /odom_raw /imu/data /range/* /gripper/state
ros2 topic hz /imu/data               # 약 25 Hz
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
모터·엔코더 방향, 속도 제어 이득, 폭주 감지 기준은 Mega 펌웨어 `firmware/mega_sensor_controller/config.h`에 있다.
