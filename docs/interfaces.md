# POLI ROS2 Interfaces (초안)

> 세 파트가 주고받는 토픽 약속이다.
> **초안**이므로 의견이 있으면 말해주세요. 확정되면 "(초안)"을 뗀다.

## 한눈에 보기

```
조원 B (카메라)  ── /vision/target ──▶  담당 1 (Mission)
                                         │
조원 A (구동)    ◀── /cmd_vel ─────────── │
                 ◀── /gripper/command ─── │
                 ── /gripper/holding ──▶  │
                 ── /odom_raw, /imu/data, /range/*, /battery_state ──▶
LiDAR 드라이버   ── /scan ─────────────▶  │
```

## 1. 새로 정할 토픽 (임무 2에 필요)

### 1-1. `/vision/target` — 빨간 대상 탐지 결과
- 보내는 쪽: 조원 B / 받는 쪽: 담당 1
- 주기: 10Hz 이상 (TODO: 조원 B 확인)
- **대상을 못 찾았을 때도 `detected = false`로 계속 보낸다.**
  (안 보내면 "못 찾은 건지, 카메라가 멈춘 건지" 구분할 수 없다.)

| 필드 | 형식 | 뜻 |
|---|---|---|
| `detected` | bool | 빨간 대상을 찾았는지 |
| `x_offset` | float32 | 화면 가운데 기준 좌우 위치, -1.0 ~ 1.0. **음수 = 화면 왼쪽**, 양수 = 오른쪽 |
| `area` | float32 | 대상이 화면에서 차지하는 넓이 (픽셀). 가까울수록 커진다 |

- 메시지 형식: 전용 메시지 `poli_interfaces/msg/TargetDetection`
  (`ros2_ws/src/poli_interfaces/msg/TargetDetection.msg`)
  - 필드 이름을 코드에서 바로 알 수 있고, 세 값이 항상 한 묶음으로 온다.
  - 사용하려면 `poli_interfaces`를 먼저 빌드한다:
    `colcon build --packages-select poli_interfaces`
  - Python: `from poli_interfaces.msg import TargetDetection`
  - 문제가 있으면 말해주세요. 필드 추가/변경 가능.
- TODO_MEASURE: 집게로 집을 수 있는 거리일 때의 `area` 값

### 1-2. `/gripper/command` — 집게 명령
- 보내는 쪽: 담당 1 / 받는 쪽: 조원 A
- 형식: `std_msgs/msg/String`

| 값 | 동작 |
|---|---|
| `"open"` | 집게 열기 |
| `"grab"` | 집게를 닫아 잡기 (**들어 올리지 않는다**) |

- **조원 A 결정: 집게는 어떤 경우에도 대상을 들어 올리지 않고, 잡은 채로 끌고 간다.**
  들어 올리기 기능은 만들지 않는다. (`"open"`, `"grab"` 외의 값은 경고 후 무시)
- 규정 3.4.4: **심판이** 로봇을 들어 올렸을 때 대상이 같이 들려야 확보로 인정된다.
  → 집게가 스스로 들 필요는 없고, 로봇이 들려도 빠지지 않을 만큼 꽉 잡아야 한다.
- "grab" 후 완전히 닫히기까지 약 1.2초 (서보 40→120도, 1도/15ms). 각도는 실측 전 임시값 (TODO_MEASURE)

### 1-3. `/gripper/holding` — 집게 상태
- 보내는 쪽: 조원 A / 받는 쪽: 담당 1
- 형식: `std_msgs/msg/Bool` (true = 물체를 잡고 있음)
- **현재 없음.** DS3218 서보(1개)는 위치·힘 피드백이 없어서 잡았는지 알 수 없다.
  조원 A가 잡힘 감지 센서를 추가할 예정 (방법 미정). 그때까지는 잡았다고 가정한다.

## 2. 이미 정해진 토픽 (docs/hardware_plan.md)

| 토픽 | 형식 | 보내는 쪽 → 받는 쪽 | 비고 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 담당 1 → 조원 A | `linear.x` 전진 m/s, `angular.z` 회전 rad/s (**양수 = 왼쪽 회전**) |
| `/odom_raw` | `nav_msgs/msg/Odometry` | 조원 A → 담당 1 | 엔코더 odometry |
| `/imu/data` | `sensor_msgs/msg/Imu` | 조원 A → 담당 1 | RRC Lite 내장 IMU |
| `/range/front`, `/range/left`, `/range/right`, `/range/rear` | `sensor_msgs/msg/Range` | 조원 A → 담당 1 | 초음파, 단위 m. **2026-09-30 이름 변경** (예전 front_left 등, 조원 A 결정: 실제 장착이 전방·좌측·우측·후방). 좌측만 낮게, 나머지 지면 약 14 cm. 측정 실패는 `inf` |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 조원 A → 담당 1 | **RRC Lite 입력 전압** (12V 컨버터 출력, 약 1 Hz). LiPo 잔량 아님, percentage = NaN |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR 드라이버 → 담당 1 | RPLIDAR C1 |

## 3. 확인 필요 (TODO)

- [x] 조원 B: `/vision/target` 필드와 `x_offset` 부호(음수 = 왼쪽) 괜찮은지 → `camera_vision`이 그대로 구현 (2026-10-05). 실제 카메라 좌우 반전은 실측 필요 (`CAMERA_HFLIP`)
- [x] 조원 B: 보낼 수 있는 주기 → 20Hz
- [ ] 조원 B: 집게로 집을 수 있는 거리일 때의 `area` (640×480 기준). 지금 `mission2_logic.GRAB_AREA = 15000`은 임시값
- [x] 조원 A: `/gripper/command` 값 (`"open"`, `"grab"`) 괜찮은지 → 그대로 사용. 단 grab = 잡기만 (들어 올리지 않음)
- [ ] 조원 A: 집게가 잡았는지 알 수 있는지 (`/gripper/holding`) → 센서 추가 예정, 지금은 없음
- [x] 팀: 전용 메시지 패키지 `poli_interfaces` 만들지 → 만들었음 (의견 있으면 알려주세요)

## 4. 조원 A 답변 (2026-09-30) — 담당 1 확인 요청

> 조원 A 하드웨어 노드가 완성되어 위 토픽을 모두 제공합니다 (`/gripper/holding` 제외).
> 코드는 `main`에 합쳐져 있습니다 (2026-09-30). `git pull` 후 `colcon build` 하면 됩니다.

### 4-1. 집게 (`/gripper/command`)
- `"open"`, `"grab"` 그대로 사용합니다. 다른 값은 경고 후 무시합니다.
- **`"grab"`은 집게를 닫아 잡기만 합니다. 어떤 경우에도 들어 올리지 않고, 잡은 채로 끌고 갑니다.**
  들어 올리기 기능은 만들지 않기로 결정했습니다. 요청하셔도 추가하지 않습니다.
- 규정 3.4.4는 **심판이** 로봇을 들었을 때 대상이 같이 들리면 확보입니다. 집게는 빠지지 않게 꽉 잡기만 하면 됩니다.
- 서보는 DS3218 **1개**입니다.

### 4-2. "grab" 후 걸리는 시간 → **`GRASP_WAIT_S`를 1.5초로 늘려 주세요**
- 완전히 닫히기까지 약 **1.2초** (서보 40→120도, 1도당 15ms). 각도는 실측 전 임시값이라 바뀔 수 있습니다.
- 지금 `mission2_logic.GRASP_WAIT_S = 1.0`이면 닫히는 도중에 다음 단계로 넘어갑니다.
- 진행 상태는 `/gripper/state` (std_msgs/String: `open` / `closed` / `moving`)로 볼 수 있습니다.
  `closed`가 될 때까지 기다리는 방식도 가능합니다.

### 4-3. 잡았는지 확인 (`/gripper/holding`)
- **지금은 없습니다.** 서보가 위치·힘 피드백을 주지 않아서 잡았는지 알 수 없습니다.
- 조원 A가 잡힘 감지 센서를 추가할 예정입니다 (방법 미정). 그때까지는 잡았다고 가정해 주세요.

### 4-4. 노드 이름과 실행 방법 (대회 launch에 넣을 것)
```bash
ros2 launch poli_hardware hardware.launch.py use_fake_hardware:=false   # 실제 로봇
ros2 launch poli_hardware hardware.launch.py                            # fake (테스트용, 기본)
```
- 실제: `rrc_adapter_node` (`/cmd_vel` → 모터, `/odom_raw`, `/imu/data`, `/battery_state`),
  `/gripper/command`, `/gripper/state` (2026-10-07 Mega에서 이전),
  `mega_bridge_node` (`/range/*`)
- fake: `fake_rrc_node`, `fake_mega_node` — 토픽은 실제와 같습니다.
- 대회 launch에서는 `IncludeLaunchDescription`으로 `poli_hardware/launch/hardware.launch.py`를 포함하면 됩니다.
- `/cmd_vel`이 0.3초 넘게 안 오면 정지합니다 (약속 0.5초 이내). mission2의 20 Hz 발행이면 문제없습니다.

### 4-5. 좌표계 (TF) — 2026-09-30 추가
- `poli_description` 패키지(URDF)가 `base_link` → `laser`, `imu_link`, `camera_link`, `ultrasonic_front/left/right/rear_link`, `gripper_link`를 발행합니다.
  `hardware.launch.py`에 기본 포함 (`use_description:=false`로 끔). robot_state_publisher를 따로 켜지 마세요.
- LiDAR frame 이름은 `laser` (sllidar_ros2 기본값, fake_scan과 같음). `scan_to_grid.py`의 `LASER_OFFSET_X/Y_M` 대신 TF(`base_link` → `laser`)를 쓰면
  실측값을 URDF 한 곳에서만 고치면 됩니다. 지금 URDF 값은 둘 다 0 (TODO_MEASURE).

### 4-6. 주의할 점
- **`/odom_raw`는 엔코더 값이 아닙니다.** RRC Lite는 공장 펌웨어를 그대로 쓰기로 했고(2026-10-01, 조원 A 결정),
  공장 펌웨어는 엔코더 값을 Pi로 보내지 않아서 보낸 속도 명령으로 추정한 값입니다. 바퀴가 미끄러지거나 막혀도 모릅니다.
  대각선 1697mm 주행 거리는 IMU·LiDAR 벽 거리 등으로 보정하는 것을 권장합니다. (자세한 내용: `docs/rrc_protocol.md`)
- RRC Lite는 명령이 끊겨도 스스로 멈추지 않습니다. 임무 노드가 `/cmd_vel`을 계속(20 Hz) 보내는 지금 방식을 유지해 주세요.
- **`/battery_state`는 LiPo 잔량이 아닙니다** (RRC 입력 = 12V 컨버터 출력). 저전압 판단에 쓰면 안 됩니다.
- `poli_navigation/fake_odom.py`도 `/odom_raw`를 발행하므로 `hardware.launch.py`와 **동시에 실행하면 안 됩니다.**
  fake 테스트는 `hardware.launch.py`(기본 fake 모드)만 켜도 조원 A 토픽이 전부 나옵니다.
- mission2는 `"open"`을 시작할 때 한 번만 보냅니다. 집게는 켜질 때 열림 상태라 괜찮지만,
  조원 A 노드가 mission2보다 늦게 켜지면 그 메시지는 받지 못합니다.
- `docs/mission_strategy.md`의 "잡고 들어 올림" 문구를 위 결정에 맞게 "잡음 (들어 올리지 않음)"으로 고쳤습니다.

## 5. 담당 1 답변 (2026-10-05) — 조원 A 답변 반영

- **4-2 집게 대기:** `mission2_logic.GRASP_WAIT_S`를 1.5초로 늘렸습니다.
- **4-6 집게 명령:** mission2(임무 1도)는 마지막 집게 명령(`"open"`/`"grab"`)을 **1초마다 다시 보냅니다.**
  조원 A 노드가 늦게 켜져도 받을 수 있습니다. 같은 명령이 1초마다 들어오니 Mega 쪽에서 문제없는지만 봐 주세요.
- **4-5 TF:** `scan_to_grid.py`의 `LASER_OFFSET_X/Y_M` 대신 TF(`base_link` → `laser`)를 읽습니다 (`poli_navigation/laser_tf.py`).
  위치(x, y)와 방향(yaw, URDF `lidar_yaw`)을 모두 씁니다. 실측값은 **URDF에서만** 고치면 됩니다.
  TF가 없으면(시뮬레이션) 기본값 0을 씁니다.
- **4-4 통합 launch:** `ros2 launch poli_navigation mission2.launch.py` 추가
  - 기본(fake): `hardware.launch.py`(fake) + mission2. 토픽·TF 연결 확인용
  - `use_fake_hardware:=false`: 실제 하드웨어 + LiDAR(`sllidar_ros2`, `/dev/robot_lidar`, 460800) + 카메라(`camera_vision`) + mission2
  - `mission2_sim.launch.py`(fake_robot)와 동시에 켜지 마세요 (`/odom_raw` 겹침).

### 5-1. 조원 A 확인 (2026-10-06) — 집게 명령 1초마다 재전송
- **실제 Mega: 문제없습니다.** 펌웨어는 같은 목표 각도면 그대로 두므로, 움직이던 중이면 계속 움직이고 다 닫혔으면 가만히 있습니다.
  Mega가 리셋됐을 때도 다음 재전송으로 복구되니 오히려 좋습니다.
- **fake Mega(`fake_mega_node`) 버그를 고쳤습니다.** 같은 명령이 올 때마다 이동 타이머(1.2초)를 다시 시작해서,
  1초마다 재전송하면 `/gripper/state`가 `moving`에서 `closed`로 넘어가지 않았습니다. 지금은 같은 명령이면 무시합니다.
  (확인: 1초마다 `grab` → 2초째 `closed`)
- `mega_bridge_node` 로그는 명령이 바뀔 때만 찍습니다 (1초마다 같은 로그가 쌓이지 않게).

### 5-2. 조원 A 변경 (2026-10-07) — 집게 서보를 RRC Lite로 이전
- 제품 사양서_E 연결 계획에 따라 집게 서보(DS3218)를 Mega D9에서 **RRC Lite PWM 서보 포트**로 옮겼습니다.
- **토픽 약속은 그대로입니다.** `/gripper/command` (`"open"` / `"grab"`), `/gripper/state` (`open` / `closed` / `moving`, 2 Hz).
  발행 노드만 `mega_bridge_node` → `rrc_adapter_node`(fake는 `fake_rrc_node`)로 바뀌었습니다. 담당 1 코드 수정 필요 없음.
- 같은 명령 1초마다 재전송: 그대로 괜찮습니다 (같은 목표면 무시).
- 이동 시간 `gripper_move_ms` 1.0 s → `GRASP_WAIT_S` 1.5 s 안에 `closed`가 됩니다.

### 5-3. 조원 A 실측 (2026-10-08) — 저속 주행 주의
- RRC 공장 펌웨어는 모터 세기 25 % 미만을 꺼 버려서, **느린 속도에서 바퀴가 울컥거립니다** (0.1 m/s 확인).
  실측 (모터 1개, 바퀴 띄움, 12 V): 0.10·0.20 m/s 울컥, **전진·후진 0.25 m/s 부드러움, 정지 정상**.
- **조원 A 결정: 주행 속도는 전진·후진 모두 0.25 m/s로 해 주세요.** 더 느린 명령은 울컥거릴 수 있습니다
  (정렬처럼 아주 느린 움직임은 짧게 끊어 가는 것을 감안).
  `max_linear`(지금 0.25 m/s)를 올려야 하면 조원 A와 상의해 주세요.
- **2026-10-08 추가: 하드웨어 노드가 최소 속도를 자동 보정합니다** (`min_wheel_speed: 0.25`).
  바퀴 속도가 0.25 m/s보다 느린 명령은 두 바퀴를 같은 비율로 키워 0.25 m/s로 냅니다 (곡률은 유지, 0은 정지).
  그래서 **실제 속도가 명령보다 빠를 수 있고, 제자리 회전은 약 2.8 rad/s**가 됩니다 (1 rad/s 명령도 동일).
  회전을 IMU 각도로 멈추는 로직은 지나침(overshoot)을 감안해 주세요. 끄려면 `min_wheel_speed: 0.0`.
