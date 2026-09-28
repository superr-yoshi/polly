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
  - Gripper servo control

## 3. Sensor MCU
- NUCLEO-F446RE
- Role:
  - BNO085 IMU
  - HC-SR04 ultrasonic sensors × 4
  - Send sensor data to Raspberry Pi

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
- BNO085
- Connection: NUCLEO-F446RE
- Role:
  - Robot heading
  - Rotation angle
  - Orientation

## 7. Ultrasonic Sensors
- HC-SR04 × 4
- Connection: NUCLEO-F446RE
- Role:
  - Short-range obstacle detection
  - Collision prevention

## 8. Gripper
- Byte Robot Black Composite Claw
- Servo: DS3218 × 2
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
- /camera/image_raw → camera image