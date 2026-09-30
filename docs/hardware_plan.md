# POLI Hardware Plan

> 부품별 상세 사양·치수·핀맵: `docs/hardware_reference.md` (원본: `docs/product-spec-claude/`)

## 1. Main Computer
- Raspberry Pi 5
- Role: ROS 2, navigation, camera processing, LiDAR processing

## 2. Motor Control
- RRC Lite Controller
- Motor: JGB37-520 Encoder DC Motor × 2
- Role:
  - Left / right wheel control
  - Encoder feedback (공장 펌웨어는 STM32 내부 PID에만 사용, POLI 패치 펌웨어는 Pi로 50 Hz 보고)
  - Built-in 6-axis IMU data

## 3. Sensor MCU
- Arduino Mega 2560 (NUCLEO-F446RE에서 변경)
- Connection: USB Serial → Raspberry Pi 5 (docs/serial_protocol.md v1.1)
- Role:
  - HC-SR04 ultrasonic sensors × 4
  - Gripper servo (D9)
  - Send range / gripper state to Raspberry Pi, receive gripper commands

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
- RRC Lite built-in 6-axis IMU (QMI8658, BNO085에서 변경)
- Connection: RRC Lite → Raspberry Pi 5 (rrc_adapter_node, docs/rrc_protocol.md)
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
- Servo: DS3218 × 1 (확정, Mega D9 신호 / XL4015 6V 별도 전원, 사양 4.8~6.8 V, PWM 500~2500 µs). 들어 올리기 없음
- Role:
  - Open / close gripper
  - Rescue object handling

## 9. Power
- Battery: XEON 6S 22.2V 5200mAh LiPo
- DC-DC Converter: Daygreen B20-24-12
- RRC Lite motor power: 12V
- **비상정지: 배터리 바로 뒤 전체 전원 차단 스위치** (docs/pin_map.md 4장)
- Raspberry Pi 5: 5 V / 5 A 필요. 결선도상 XL4015 5 V → GPIO 5 V 핀 공급 → USB 장치 전원 부족 여부 실기 확인 (TODO_MEASURE)

## 10. Software Interface Plan
- /cmd_vel → motor command
- /odom_raw → RRC Lite odometry (공장 펌웨어 = 명령 기반 추정, POLI 패치 펌웨어 = 엔코더 실측)
- /imu/data → RRC Lite built-in 6-axis IMU
- /range/front, /range/left, /range/right, /range/rear → 초음파 (전방·좌측·우측·후방)
- /gripper/command ("open" / "grab", 들어 올리기 없음), /gripper/state → gripper (Arduino Mega)
- /battery_state → RRC Lite 입력 전압 (LiPo 잔량 아님)
- /scan → RPLIDAR C1 LaserScan
- /odometry/filtered → robot_localization filtered odometry
- /camera/image_raw → camera image
