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

### (사용 안 함) 집게 서보 D9
- 2026-10-07: 집게 서보는 **RRC Lite PWM 서보 포트로 옮겼다** (아래 2장, 제품 사양서_E 연결 계획).
  Mega D9에는 아무것도 꽂지 않는다. 펌웨어의 집게 코드는 남아 있지만 `$GRIP`을 보내지 않으므로 동작하지 않는다.

## 2. RRC Lite (주행 하위제어)
| 항목 | 연결 |
|---|---|
| M1 (왼쪽 바퀴 추정, TODO_MEASURE) | JGB37-520 모터 M1 (M1+ / M1−, 엔코더 포함) |
| M2 (오른쪽 바퀴 추정, TODO_MEASURE) | JGB37-520 모터 M2 (M2+ / M2−, 엔코더 포함) |
| 전원 | XL4015 벅 컨버터 출력 → +BAT / −BAT (12V) |
| IMU | 보드 내장 6축 IMU |
| 모터 케이블 (PH2.0 6핀) | 모터 기판 M2·VCC·C1·C2·GND·M1 ↔ RRC Mx-F·5V·(엔코더 A)·(엔코더 B)·GND·Mx-B. **한쪽 끝에서 가운데 두 선(C1·C2)을 엇갈려** 연결 (일자로 꽂으면 엔코더 방향 반대 → 폭주, 2026-10-08 실측) |
| PWM 서보 포트 1 (TODO_MEASURE: 실제 꽂은 포트) | 집게 서보 DS3218 (신호·+·− 3핀 커넥터 그대로) |

### 집게 서보 (DS3218) — RRC PWM 서보 포트
- 포트 번호는 `config/hardware.yaml`의 `gripper_servo_id` (1~4)와 같아야 한다.
- ⚠ **서보 포트 전원 점퍼를 반드시 5V로 둔다.** 점퍼가 입력 전압(VIN, 12V) 쪽이면 DS3218(최대 6.8V)이 탄다.
  꽂기 전에 멀티미터로 서보 포트 + / − 사이가 약 5V인지 확인한다.
- 보드 실물 사진 (2026-10-28, 실측 전): 서보 포트는 `S3`(PA11), `S4`(PA12), `S5`(PC8), `S6`(PC9) 4개. 옆의 점퍼 `J2`, `J7`이
  `5V | VBAT` 선택이며 사진에서는 **둘 다 VBAT 쪽**으로 보였다 → 5V 쪽으로 옮길 것. J2/J7이 각각 어느 포트를 맡는지,
  펌웨어 서보 번호 1~4가 S3~S6 중 어디인지는 TODO_MEASURE (`scripts/pc_gripper_test.py --servo-id N`으로 확인).
  위쪽의 `S + -` 흰 커넥터 2개는 12 V가 나온다고 확인됨 (조원 확인) → DS3218을 꽂지 않는다.
- 사양서_E는 PWM 서보 포트 전압을 "5-12V"와 "5~8.4V" 두 가지로 적어 두었다 → 실물 점퍼로 확인 (TODO_MEASURE).
- RRC는 **전원을 켜면 PWM 서보 4포트 모두 1500 µs(가운데)를 낸다** (공장 펌웨어 `pwm_servos_init`).
  그래서 열림 위치 `gripper_open_us`를 1500으로 두었다. 집게를 조립할 때 1500 µs에서 "열림"이 되게 혼을 끼운다
  → 전원을 켤 때 집게가 움직이지 않는다 (규정 3.5.6, 전원 켜기는 시작 전 허용 Q7·Q17).
- 닫힘 `gripper_close_us`(1833 = 약 120°)는 임시값 (TODO_MEASURE: 50 mm 큐브를 빠지지 않게 잡는 값).
- 이전에 쓰던 XL4015 6.0V 별도 서보 전원은 필요 없다 (RRC 서보 포트가 전원까지 준다).

## 3. Raspberry Pi 5
| 장치 | 연결 | udev 별칭 |
|---|---|---|
| RRC Lite | USB Serial | /dev/robot_rrc |
| Arduino Mega 2560 | USB Serial | /dev/robot_mega |
| RPLIDAR | USB (어댑터 보드) | /dev/robot_lidar |
| 카메라 | CSI | — |
| IMU MPU6050 (GY-521) | 소프트웨어 I2C 버스 3, 주소 0x68 | /dev/i2c-3 |
| 전원 | XL4015 벅 컨버터 출력 5V → GPIO 5V/GND | — |

### 파이 40핀 사용 현황 (2026-10-30 조원 확인)
- 사용 중: **2, 3, 6, 7, 9, 18, 19, 30, 31번** (용도 기록 필요: TODO. 2·4·6번 = 전원 입력 계획과 관련)
- 3번(하드웨어 I2C SDA)이 사용 중이라 IMU는 **소프트웨어 I2C(15·16번)**로 연결한다.
  3번에 꽂힌 것이 I2C 장치로 확인되면 IMU를 3·5번에 함께 연결해도 된다 (그때 `i2c_bus: 1`).

### IMU MPU6050 (GY-521) — 파이 GPIO 직접 연결 (2026-10-30, RRC Lite 내장 IMU 대체)
| GY-521 | 파이 40핀 |
|---|---|
| VCC | **17번 3.3V** (5V 핀 금지: SDA/SCL이 5 V로 끌어올려질 수 있음) |
| GND | **14번 GND** (9·30번은 사용 중) |
| SDA | **15번 GPIO 22** |
| SCL | **16번 GPIO 23** |
| XDA, XCL, AD0, INT | 연결 안 함 (AD0 비움 = 주소 0x68) |

- 파이 설정 (한 번, 팀 상의 후): `/boot/firmware/config.txt`에 `dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=22,i2c_gpio_scl=23` 추가 → 재부팅.
  GY-521 모듈에 풀업 저항이 있어서 따로 달지 않는다.
- 코드: `hardware.yaml` imu_node `i2c_bus: 3`.
- 장착: 로봇 회전 중심 근처, 모듈의 **X 화살표 = 로봇 앞**, 칩이 위. 모터·드라이버·컨버터 옆은 피한다. 흔들리지 않게 고정.
- 위치·방향은 URDF `imu_x`, `imu_y`, `imu_h`, `imu_yaw` (TODO_MEASURE). 코드에서 축을 돌리지 않는다.
- 확인: `i2cdetect -y 3`에 `68`. 노드: `imu_node` (`hardware.launch.py` 실제 모드에서 자동 실행).
- 전원 LED가 빨간색이면 가린다 (규정 Q6).

## 4. 전원
- 배터리: XEON 6S 22.2V 5200mAh LiPo → XT60 → **비상정지 스위치** → 퓨즈(SZH-FU003, ATO 퓨즈 인라인 홀더) → WAGO 분배
- 분배 → XL4015 ×3: RRC Lite(12V) / Raspberry Pi(5V) / 서보(6V)
  - 2026-10-07: 서보는 RRC PWM 서보 포트 전원(점퍼 5V)을 쓰므로 서보용 6V 컨버터는 필요 없다. Pi 전원 방법(XL4015 5V 또는 RRC 5V 5A 출력)은 미정.
    ⚠ Pi(최대 5 A)까지 RRC 5V에서 받으면 서보(DS3218 잡을 때 순간 2 A 이상 가능, TODO_MEASURE)와 5 A를 나눠 쓰게 되어 여유가 없다.
- 각 컨버터 실제 출력 전압(무부하 / 최대부하): TODO_MEASURE

### 비상정지 스위치 (E-stop) — 2026-09-30 결정: 전체 전원 차단
- 위치: **배터리(XT60) 바로 뒤**, 퓨즈·WAGO 분배보다 앞. 누르면 로봇 전체(모터, Pi, Mega, 서보) 전원이 끊긴다.
- 스위치 조건 (TODO: 부품 선정)
  - 정격 전압: DC 25.2 V 이상 (6S 완충 전압)
  - 정격 전류: 전체 최대 전류 이상. 모터 2개 스톨 + Pi 5 + 서보를 더한 값 (TODO_MEASURE)
  - 누르면 잠기는(래치) 버섯형 권장, 로봇 윗면처럼 손이 바로 닿는 곳
- 주의: 누르면 Pi도 바로 꺼진다 (SD 카드 손상 가능). 평소 끄기는 소프트웨어 종료(`sudo poweroff`) 후 전원을 끈다.
- RRC Lite(공장 펌웨어)는 Pi가 멈추거나 USB가 빠지면 마지막 속도로 계속 달린다. **이 스위치가 하드웨어 쪽 유일한 안전장치다.**
