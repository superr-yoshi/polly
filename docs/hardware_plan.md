# POLI Hardware Plan

> 부품별 상세 사양·치수·핀맵: `docs/hardware_reference.md` (원본: `docs/product-spec-claude/`)
> 대회 규정 준수 점검표: `docs/competition/rules_compliance.md` (크기 지름 400 × 높이 300 mm 원기둥 안, 10 kg 이내, 빨간 부품 금지, 두 임무 같은 하드웨어)

## 1. Main Computer
- Raspberry Pi 5
- Role: ROS 2, navigation, camera processing, LiDAR processing

## 2. Motor Control (2026-10-29 변경: RRC Lite → MDD10)
- Motor driver: **Cytron MDD10** (2채널), 모터 전원 = 12 V 배터리 직접 (B+/B−)
- Motor: JGB37-520 Encoder DC Motor × 2 (12 V, 30:1, 11 PPR)
- Speed control: Arduino Mega 2560 (아래 3장). MDD10 PWM1/DIR1 = D6/D30, PWM2/DIR2 = D7/D31
- Role:
  - Left / right wheel control (Mega 안의 PI + 앞먹임, 50 Hz)
  - Encoder feedback (Mega 4체배 → `/odom_raw` 엔코더 기반)
- RRC Lite: **사용 안 함** (옛 기록 `docs/rrc_protocol.md`, `docs/rrc_adapter_plan.md`)

## 3. Main MCU (주행·센서)
- Arduino Mega 2560 (NUCLEO-F446RE에서 변경)
- Connection: USB Serial → Raspberry Pi 5 (docs/serial_protocol.md v2.0)
- Role:
  - MDD10 모터 속도 제어 + 엔코더 2개 (D2/D3, D18/D19)
  - BNO085 IMU (I2C D20/D21)
  - HC-SR04 ultrasonic sensors × 4
  - 집게 서보 DS3218 (D9)
  - 안전: 명령이 0.3 s 끊기면 정지, 엔코더 방향 반대·신호 없음 감지 시 정지

## 4. LiDAR
- SLAMTEC RPLIDAR C1 (360°, 10 Hz, 0.05~12 m, 5 V 최대 260 mA)
- Connection: USB 어댑터 보드 → Raspberry Pi 5
- 레이저 높이: 장착면 + 29.8 mm → 외벽·장애물(200 mm)보다 낮게 장착
- Role:
  - Obstacle detection
  - SLAM
  - Navigation

## 5. Camera
- Raspberry Pi AI Camera (Sony IMX500, 수평 시야각 66.3°, 수동 초점 20 cm~∞)
- Connection: CSI → Raspberry Pi 5 **CAM0**
- Role:
  - Red rescue target detection
  - Visual alignment

## 6. IMU
- **BNO085** (2026-10-29, RRC Lite 내장 IMU에서 변경)
- Connection: Mega I2C (D20 SDA / D21 SCL) → `$IMU` → Raspberry Pi 5 (mega_bridge_node). 배선 TODO, 레벨 시프터 필요 여부 확인
- Role:
  - Robot heading
  - Rotation angle
  - Orientation
  - ROS 2 /imu/data

## 7. Ultrasonic Sensors
- HC-SR04 × 4
- Connection: Arduino Mega 2560 (D22~D29, docs/pin_map.md)
- Role:
  - Short-range obstacle detection
  - Collision prevention
  - ROS 2 /range/front, /range/left, /range/right, /range/rear
  - 장착: 전방·우측·후방은 지면에서 약 14 cm, 좌측만 낮게 (TODO_MEASURE)

## 8. Gripper
- Byte Robot Black Composite Claw 125mm (최대 개폐 125 mm, 파지력 500 g, 140 g)
- Servo: DS3218 × 1 (확정, **Mega D9**, 전원 5V 10A 컨버터 + GND 공통, 사양 4.8~6.8 V, PWM 500~2500 µs). 들어 올리기 없음
  - 2026-10-07 RRC PWM 포트로 옮겼다가 2026-10-29 Mega D9로 되돌림 (`docs/pin_map.md` 1장)
- Role:
  - Open / close gripper
  - Rescue object handling

## 9. Power
- Battery: **12 V** (2026-10-29 변경, 예전 XEON 6S 22.2V 5200mAh LiPo). 종류·용량 TODO
- 모터 전원: 배터리 12 V → MDD10 B+/B− (모터 정격 12 V)
- 5V 10A 컨버터 → Raspberry Pi 5 (USB-C) + 집게 서보 (2026-10-29 결정, 예전 XL4015 × 3·Daygreen 12 V 컨버터는 안 씀)
- **비상정지: 배터리 바로 뒤 전체 전원 차단 스위치** (docs/pin_map.md 4장)
- Raspberry Pi 5: PD가 없는 5 V 공급이라 USB 전류 600 mA 제한 → `usb_max_current_enable=1` 설정, 전원 부족 여부 실기 확인 (TODO_MEASURE)

## 10. Software Interface Plan
- /cmd_vel → motor command (mega_bridge_node → `$DRV` → Mega → MDD10)
- /odom_raw → 엔코더 odometry (Mega `$ENC`)
- /imu/data → BNO085 (Mega `$IMU`). 미연결이면 휠 각속도로 z축만
- /range/front, /range/left, /range/right, /range/rear → 초음파 (전방·좌측·우측·후방)
- /gripper/command ("open" / "grab", 들어 올리기 없음), /gripper/state → gripper (Mega D9)
- /battery_state → 없음 (2026-10-29, 전압 측정원 없음)
- /scan → RPLIDAR C1 LaserScan
- /odometry/filtered → robot_localization filtered odometry
- /camera/image_raw → camera image
