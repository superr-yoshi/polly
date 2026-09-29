# POLI Hardware Plan

## 1. Main Computer
- Raspberry Pi 5
- Role: ROS 2, navigation, camera processing, LiDAR processing

## 2. Motor Control
- RRC Lite Controller
- Motor: JGB37-520 Encoder DC Motor × 2
- Role:
  - Left / right wheel control
  - Encoder feedback
  - Built-in 6-axis IMU

## 3. Sensor MCU
- Arduino Mega 2560 (NUCLEO-F446RE에서 변경)
- Connection: USB Serial → Raspberry Pi 5 (docs/serial_protocol.md v1.1)
- Role:
  - HC-SR04 ultrasonic sensors × 4
  - Gripper servo (D9)
  - Send sensor data to Raspberry Pi, receive gripper commands

## 4. LiDAR
- SLAMTEC RPLIDAR A2M12
- Connection: USB → Raspberry Pi 5
- Role:
  - Obstacle detection
  - SLAM
  - Navigation

## 5. Camera
- Innomaker U20CAM-720P
- Connection: USB → Raspberry Pi 5
- Role:
  - Red rescue target detection
  - Visual alignment

## 6. IMU
- RRC Lite 내장 6축 IMU (BNO085에서 변경)
- Connection: RRC Lite → /imu/data (rrc_adapter_node)
- Role:
  - Robot heading
  - Rotation angle
  - Orientation

## 7. Ultrasonic Sensors
- HC-SR04 × 4
- Connection: Arduino Mega 2560 (D22~D29, docs/pin_map.md)
- Role:
  - Short-range obstacle detection
  - Collision prevention

## 8. Gripper
- Byte Robot Black Composite Claw
- Servo: DS3218 (결선도 기준 1개, Mega D9 신호 / XL4015 6V 별도 전원)
- Role:
  - Open / close gripper
  - Rescue object handling

## 9. Power
- Battery: XEON 6S 22.2V 5200mAh LiPo
- DC-DC Converter: Daygreen B20-24-12
- RRC Lite motor power: 12V

## 10. Software Interface Plan
- /cmd_vel → motor command
- /odom_raw → encoder odometry
- /imu/data → IMU data
- /scan → LiDAR data
- /range/front_left
- /range/front_right
- /range/rear_left
- /range/rear_right
- /gripper/state, /gripper/set → gripper
- /camera/image_raw → camera image