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
  - Built-in 6-axis IMU data

## 3. Sensor MCU
- Arduino Mega 2560
- Connection: USB Serial bridge to Raspberry Pi 5
- Role:
  - HC-SR04 ultrasonic sensors × 4
  - Battery monitoring
  - Send range / battery data to Raspberry Pi

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
- RRC Lite built-in 6-axis IMU
- Connection: RRC Lite → Raspberry Pi 5
- Role:
  - Robot heading
  - Rotation angle
  - Orientation
  - ROS 2 /imu/data

## 7. Ultrasonic Sensors
- HC-SR04 × 4
- Connection: Arduino Mega 2560
- Role:
  - Short-range obstacle detection
  - Collision prevention
  - ROS 2 /range/front_left, /range/front_right, /range/rear_left, /range/rear_right

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
- /odom_raw → RRC Lite encoder odometry
- /imu/data → RRC Lite built-in 6-axis IMU
- /range/front_left → front-left ultrasonic sensor
- /range/front_right → front-right ultrasonic sensor
- /range/rear_left → rear-left ultrasonic sensor
- /range/rear_right → rear-right ultrasonic sensor
- /battery_state → battery status from Arduino Mega
- /scan → RPLIDAR C1 LaserScan
- /odometry/filtered → robot_localization filtered odometry

