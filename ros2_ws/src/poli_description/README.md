# poli_description — 로봇 위치 모델 (조원 A)

센서가 로봇 어디에 어느 방향으로 달려 있는지를 ROS 전체에 알려주는 URDF(xacro)다.
`robot_state_publisher`가 `base_link` → 각 센서 frame TF를 발행한다.

```bash
ros2 launch poli_description description.launch.py      # 단독 실행
ros2 launch poli_hardware hardware.launch.py             # 하드웨어 launch에 기본 포함 (use_description:=false로 끔)
```

## frame
| frame | 무엇 | 비고 |
|---|---|---|
| `base_link` | 좌우 구동 바퀴 축의 가운데 | x 정면, y 왼쪽, z 위 |
| `laser` | RPLIDAR C1 레이저 | sllidar_ros2 기본 frame 이름과 같음. 레이저 높이 < 0.20 m 이어야 함 |
| `imu_link` | RRC Lite 내장 IMU | 보드 방향 실측 필요 |
| `camera_link` | AI Camera | 아래로 숙인 각 `camera_pitch` |
| `ultrasonic_front_link` / `_left_link` / `_right_link` / `_rear_link` | HC-SR04 | +x = 센서가 보는 방향. 좌측만 낮게, 나머지 지면 약 14 cm |
| `gripper_link` | 집게 | 들어 올리기 없음 |

odom → base_link는 여기서 발행하지 않는다 (robot_localization EKF 소유).

## 실측 후 고칠 것
`urdf/poli.urdf.xacro` 맨 위 property만 고치면 된다. `TODO_MEASURE` / `SIM_ONLY` 표시가 붙은 값이 실측 대상이다.
높이는 "지면에서 몇 m"로 적는다 (base_link 기준 z는 자동 계산).
`wheel_radius`, `wheel_separation`은 `poli_hardware/config/hardware.yaml`과 같은 값으로 맞춘다.

## 확인
```bash
xacro src/poli_description/urdf/poli.urdf.xacro > /tmp/poli.urdf && check_urdf /tmp/poli.urdf
ros2 run tf2_ros tf2_echo base_link ultrasonic_left_link    # 위치·방향 확인
colcon test --packages-select poli_description             # check_urdf 자동 검사
```
