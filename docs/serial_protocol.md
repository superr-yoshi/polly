# POLI Serial Protocol

## 목적
NUCLEO-F446RE에서 읽은 센서 데이터를
USB Serial을 통해 Raspberry Pi 5로 전달한다.

## 연결
NUCLEO-F446RE → USB Serial → Raspberry Pi 5

## 기본 형식
센서 데이터는 한 줄씩 전송한다.

### IMU
IMU,yaw,pitch,roll

예:
IMU,90.5,1.2,-0.8

### Ultrasonic
US,front_left,front_right,rear_left,rear_right

예:
US,32.4,30.8,55.2,53.9

## 아직 미정
- Serial 속도: 115200 baud
- IMU 전송 주기: 50 Hz
- 초음파 전송 주기: 8 Hz