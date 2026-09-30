# POLI Pin Map

> 기준: 해냄터 4팀 Arduino Mega 2560 결선도 + 전체 결선도.
> 실제 배선 후 다시 확인하고, 바뀌면 이 문서와 `firmware/mega_sensor_controller/config.h`를 함께 고친다.
> 원래 계획의 NUCLEO-F446RE + BNO085는 Arduino Mega 2560 + RRC Lite 내장 IMU로 대체되었다.

## 1. Arduino Mega 2560 (센서 MCU, 조원 A)

### HC-SR04 × 4
| 펌웨어 순서 | ROS 토픽 | 방향 | Trig | Echo | 장착 높이 |
|---|---|---|---|---|---|
| 0 | /range/front | 전방 | D22 | D23 | 지면에서 약 14 cm (TODO_MEASURE) |
| 1 | /range/left | 좌측 | D24 | D25 | **낮게** (정확한 높이 TODO_MEASURE) |
| 2 | /range/right | 우측 | D26 | D27 | 지면에서 약 14 cm (TODO_MEASURE) |
| 3 | /range/rear | 후방 | D28 | D29 | 지면에서 약 14 cm (TODO_MEASURE) |

- VCC ×4 → 브레드보드 위 + 레일 (Mega 5V), GND ×4 → 위 − 레일 (Mega GND)
- 2026-09-30 결정: 결선도대로 전방·좌측·우측·후방에 단다. 토픽 이름도 이에 맞춰 바꿨다 (예전 front_left/front_right/rear_left/rear_right).
- 센서 위치·방향은 `poli_description` 패키지(URDF)의 `ultrasonic_<이름>_link`에 들어 있다.

### 집게 서보 (DS3218)
| 항목 | 연결 |
|---|---|
| 신호 | D9 (PWM) |
| 전원 | XL4015 벅 컨버터 출력 6.0V → 브레드보드 아래 + 레일 (**Mega 5V 사용 금지**) |
| GND | 아래 − 레일 ↔ 위 − 레일 연결 (GND 공통, 필수) |

- ⚠ 위 + 레일(5V)과 아래 + 레일(6V)은 절대 연결하지 않는다.
- 열림/닫힘 각도: `config.h`의 `GRIP_OPEN_DEG` / `GRIP_CLOSE_DEG` (SIM_ONLY, 실제 집게로 보정)

## 2. RRC Lite (주행 하위제어)
| 항목 | 연결 |
|---|---|
| M1 (왼쪽 바퀴 추정, TODO_MEASURE) | JGB37-520 모터 M1 (M1+ / M1−, 엔코더 포함) |
| M2 (오른쪽 바퀴 추정, TODO_MEASURE) | JGB37-520 모터 M2 (M2+ / M2−, 엔코더 포함) |
| 전원 | XL4015 벅 컨버터 출력 → +BAT / −BAT (12V) |
| IMU | 보드 내장 6축 IMU |

## 3. Raspberry Pi 5
| 장치 | 연결 | udev 별칭 |
|---|---|---|
| RRC Lite | USB Serial | /dev/robot_rrc |
| Arduino Mega 2560 | USB Serial | /dev/robot_mega |
| RPLIDAR | USB (어댑터 보드) | /dev/robot_lidar |
| 카메라 | CSI | — |
| 전원 | XL4015 벅 컨버터 출력 5V → GPIO 5V/GND | — |

## 4. 전원
- 배터리: XEON 6S 22.2V 5200mAh LiPo → XT60 → 퓨즈(SZH-FU003, ATO 퓨즈 인라인 홀더) → WAGO 분배
- 분배 → XL4015 ×3: RRC Lite(12V) / Raspberry Pi(5V) / 서보(6V)
- 각 컨버터 실제 출력 전압(무부하 / 최대부하): TODO_MEASURE
