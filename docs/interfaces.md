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

- 메시지 형식 제안: 전용 메시지 `poli_interfaces/msg/TargetDetection`
  (새 패키지가 필요하므로 팀 동의 후 만든다. TODO)
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

- [ ] 조원 B: `/vision/target` 필드와 `x_offset` 부호(음수 = 왼쪽) 괜찮은지
- [ ] 조원 B: 보낼 수 있는 주기
- [x] 조원 A: `/gripper/command` 값 (`"open"`, `"grab"`) 괜찮은지 → 그대로 사용. 단 grab = 잡기만 (들어 올리지 않음)
- [ ] 조원 A: 집게가 잡았는지 알 수 있는지 (`/gripper/holding`) → 센서 추가 예정, 지금은 없음
- [ ] 팀: 전용 메시지 패키지 `poli_interfaces` 만들지

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
  `mega_bridge_node` (`/range/*`, `/gripper/command`, `/gripper/state`)
- fake: `fake_rrc_node`, `fake_mega_node` — 토픽은 실제와 같습니다.
- 대회 launch에서는 `IncludeLaunchDescription`으로 `poli_hardware/launch/hardware.launch.py`를 포함하면 됩니다.
- `/cmd_vel`이 0.3초 넘게 안 오면 정지합니다 (약속 0.5초 이내). mission2의 20 Hz 발행이면 문제없습니다.

### 4-5. 좌표계 (TF) — 2026-09-30 추가
- `poli_description` 패키지(URDF)가 `base_link` → `laser`, `imu_link`, `camera_link`, `ultrasonic_front/left/right/rear_link`, `gripper_link`를 발행합니다.
  `hardware.launch.py`에 기본 포함 (`use_description:=false`로 끔). robot_state_publisher를 따로 켜지 마세요.
- LiDAR frame 이름은 `laser` (sllidar_ros2 기본값, fake_scan과 같음). `scan_to_grid.py`의 `LASER_OFFSET_X/Y_M` 대신 TF(`base_link` → `laser`)를 쓰면
  실측값을 URDF 한 곳에서만 고치면 됩니다. 지금 URDF 값은 둘 다 0 (TODO_MEASURE).

### 4-6. 주의할 점
- **`/odom_raw`는 RRC 펌웨어에 따라 다릅니다.** 공장 펌웨어는 엔코더 값을 Pi로 보내지 않아 보낸 속도 명령으로 추정한 값이고
  (바퀴가 미끄러지거나 막혀도 모름), 조원 A의 **POLI 패치 펌웨어**(`firmware/rrc_lite_patch/`, 실기 시험 전)를 구우면 엔코더 실측 기반이 됩니다.
  토픽·형식은 같습니다. 어느 쪽인지는 노드 로그("엔코더 기반" / "명령 기반")로 보입니다. (자세한 내용: `docs/rrc_protocol.md`)
- **`/battery_state`는 LiPo 잔량이 아닙니다** (RRC 입력 = 12V 컨버터 출력). 저전압 판단에 쓰면 안 됩니다.
- `poli_navigation/fake_odom.py`도 `/odom_raw`를 발행하므로 `hardware.launch.py`와 **동시에 실행하면 안 됩니다.**
  fake 테스트는 `hardware.launch.py`(기본 fake 모드)만 켜도 조원 A 토픽이 전부 나옵니다.
- mission2는 `"open"`을 시작할 때 한 번만 보냅니다. Mega는 켜질 때 열림 상태라 괜찮지만,
  조원 A 노드가 mission2보다 늦게 켜지면 그 메시지는 받지 못합니다.
- `docs/mission_strategy.md`의 "잡고 들어 올림" 문구를 위 결정에 맞게 "잡음 (들어 올리지 않음)"으로 고쳤습니다.
