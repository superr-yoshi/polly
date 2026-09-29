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
| `"grab"` | 잡고 들어 올리기 |

- 규정 3.4.4: 로봇을 들어 올렸을 때 대상이 같이 들려야 확보로 인정된다.

### 1-3. `/gripper/holding` — 집게 상태
- 보내는 쪽: 조원 A / 받는 쪽: 담당 1
- 형식: `std_msgs/msg/Bool` (true = 물체를 잡고 있음)
- TODO: 잡았는지 알 수 있는 방법이 있는지 조원 A 확인.
  알 수 없으면 이 토픽은 없어도 된다. (그 경우 잡았다고 가정한다.)

## 2. 이미 정해진 토픽 (docs/hardware_plan.md)

| 토픽 | 형식 | 보내는 쪽 → 받는 쪽 | 비고 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 담당 1 → 조원 A | `linear.x` 전진 m/s, `angular.z` 회전 rad/s (**양수 = 왼쪽 회전**) |
| `/odom_raw` | `nav_msgs/msg/Odometry` | 조원 A → 담당 1 | 엔코더 odometry |
| `/imu/data` | `sensor_msgs/msg/Imu` | 조원 A → 담당 1 | RRC Lite 내장 IMU |
| `/range/front_left` 등 4개 | `sensor_msgs/msg/Range` | 조원 A → 담당 1 | 초음파, 단위 m |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 조원 A → 담당 1 | |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR 드라이버 → 담당 1 | RPLIDAR C1 |

## 3. 확인 필요 (TODO)

- [ ] 조원 B: `/vision/target` 필드와 `x_offset` 부호(음수 = 왼쪽) 괜찮은지
- [ ] 조원 B: 보낼 수 있는 주기
- [ ] 조원 A: `/gripper/command` 값 (`"open"`, `"grab"`) 괜찮은지
- [ ] 조원 A: 집게가 잡았는지 알 수 있는지 (`/gripper/holding`)
- [x] 팀: 전용 메시지 패키지 `poli_interfaces` 만들지 → 만들었음 (의견 있으면 알려주세요)
