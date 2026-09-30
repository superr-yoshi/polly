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
- **현재 없음.** DS3218 서보는 위치·힘 피드백이 없어서 잡았는지 알 수 없다.
  조원 A가 잡힘 감지 센서를 추가할 예정 (방법 미정). 그때까지는 잡았다고 가정한다.

## 2. 이미 정해진 토픽 (docs/hardware_plan.md)

| 토픽 | 형식 | 보내는 쪽 → 받는 쪽 | 비고 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 담당 1 → 조원 A | `linear.x` 전진 m/s, `angular.z` 회전 rad/s (**양수 = 왼쪽 회전**) |
| `/odom_raw` | `nav_msgs/msg/Odometry` | 조원 A → 담당 1 | 엔코더 odometry |
| `/imu/data` | `sensor_msgs/msg/Imu` | 조원 A → 담당 1 | RRC Lite 내장 IMU |
| `/range/front_left` 등 4개 | `sensor_msgs/msg/Range` | 조원 A → 담당 1 | 초음파, 단위 m |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 조원 A → 담당 1 | **RRC Lite 입력 전압** (12V 컨버터 출력, 약 1 Hz). LiPo 잔량 아님, percentage = NaN |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR 드라이버 → 담당 1 | RPLIDAR C1 |

## 3. 확인 필요 (TODO)

- [ ] 조원 B: `/vision/target` 필드와 `x_offset` 부호(음수 = 왼쪽) 괜찮은지
- [ ] 조원 B: 보낼 수 있는 주기
- [x] 조원 A: `/gripper/command` 값 (`"open"`, `"grab"`) 괜찮은지 → 그대로 사용. 단 grab = 잡기만 (들어 올리지 않음)
- [ ] 조원 A: 집게가 잡았는지 알 수 있는지 (`/gripper/holding`) → 센서 추가 예정, 지금은 없음
- [ ] 팀: 전용 메시지 패키지 `poli_interfaces` 만들지
