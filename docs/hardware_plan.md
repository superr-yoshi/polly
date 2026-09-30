# POLI Hardware Plan

## 1. Main Computer
- Raspberry Pi 5
- Role: ROS 2, navigation, camera processing, LiDAR processing

## 2. Motor Control
- RRC Lite Controller
- Motor: JGB37-520 Encoder DC Motor × 2
- Role:
  - Left / right wheel control
  - Encoder feedback (STM32 내부 속도 PID용, Pi로는 보내지 않음)
  - Built-in 6-axis IMU data

## 3. Sensor MCU
- Arduino Mega 2560 (NUCLEO-F446RE에서 변경)
- Connection: USB Serial → Raspberry Pi 5 (docs/serial_protocol.md v1.1)
- Role:
  - HC-SR04 ultrasonic sensors × 4
  - Gripper servo (D9)
  - Send range / gripper state to Raspberry Pi, receive gripper commands

## 4. LiDAR
- SLAMTEC RPLIDAR C1
- Connection: USB → Raspberry Pi 5
- Role:
  - Obstacle detection
  - SLAM
  - Navigation

## 5. Camera
- Raspberry Pi AI Camera (Sony IMX500)
- Connection: CSI → Raspberry Pi 5
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
  - ROS 2 /range/front_left, /range/front_right, /range/rear_left, /range/rear_right

## 8. Gripper
- Byte Robot Black Composite Claw
- Servo: DS3218 × 1 (확정, Mega D9 신호 / XL4015 6V 별도 전원). 들어 올리기 없음
- Role:
  - Open / close gripper
  - Rescue object handling

## 9. Power
- Battery: XEON 6S 22.2V 5200mAh LiPo
- DC-DC Converter: Daygreen B20-24-12
- RRC Lite motor power: 12V

## 10. Software Interface Plan
- /cmd_vel → motor command
- /odom_raw → RRC Lite odometry (제조사 펌웨어가 엔코더를 보내지 않아 명령 기반 추정)
- /imu/data → RRC Lite built-in 6-axis IMU
- /range/front_left → front-left ultrasonic sensor
- /range/front_right → front-right ultrasonic sensor
- /range/rear_left → rear-left ultrasonic sensor
- /range/rear_right → rear-right ultrasonic sensor
- /gripper/command ("open" / "grab", 들어 올리기 없음), /gripper/state → gripper (Arduino Mega)
- /battery_state → RRC Lite 입력 전압 (LiPo 잔량 아님)
- /scan → RPLIDAR C1 LaserScan
- /odometry/filtered → robot_localization filtered odometry
- /camera/image_raw → camera image
