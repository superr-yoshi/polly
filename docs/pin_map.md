# POLI Pin Map

> 기준: **2026-10-29 팀 결선도 (RRC Lite 대신 Cytron MDD10)** + 해냄터 4팀 Arduino Mega 2560 결선도(초음파).
> 실제 배선 후 다시 확인하고, 바뀌면 이 문서와 `firmware/mega_sensor_controller/config.h`를 함께 고친다.
> 2026-10-29: RRC Lite를 쓰지 않는다. 모터 드라이버는 MDD10, 속도 제어·엔코더·IMU(BNO085)·집게는 모두 Mega가 맡는다.

## 1. Arduino Mega 2560 (주행·센서 MCU, 조원 A)

### 한눈에 보기
| Mega 핀 | 연결 | 비고 |
|---|---|---|
| D2 / D3 | 왼쪽 모터 엔코더 C1 / C2 | 외부 인터럽트 INT4 / INT5 |
| D18 / D19 | 오른쪽 모터 엔코더 C1 / C2 | 외부 인터럽트 INT3 / INT2 (Serial1 TX1/RX1 자리 → Serial1 사용 불가) |
| D20 / D21 | BNO085 SDA / SCL (I2C) | TODO: 결선도에 아직 없음. 배선 후 확정 |
| D6 / D30 | MDD10 PWM1 / DIR1 (왼쪽 모터) | D6 = Timer4 PWM |
| D7 / D31 | MDD10 PWM2 / DIR2 (오른쪽 모터) | D7 = Timer4 PWM |
| D9 | 집게 서보 DS3218 신호 (노랑) | 전원은 5V 10A 컨버터 |
| D22~D29 | 초음파 HC-SR04 ×4 Trig/Echo | 아래 표 |
| 5V / GND | 엔코더 2개 VCC·GND, 초음파 VCC·GND | 5V 핀이 모자라면 브레드보드 레일로 나눔 |
| GND | MDD10 GND, 5V 10A 컨버터 −(서보 GND) | **GND 공통 필수** |
| USB | Raspberry Pi 5 USB | 전원 + 시리얼 (`/dev/robot_mega`) |

### 주행 모터 드라이버 Cytron MDD10
| MDD10 단자 | 연결 |
|---|---|
| B+ / B− | 배터리(12 V) + / − (비상정지·퓨즈 뒤) |
| M1A / M1B | 왼쪽 모터 M1 / M2 |
| M2A / M2B | 오른쪽 모터 M1 / M2 |
| PWM1 / DIR1 | Mega D6 / D30 |
| PWM2 / DIR2 | Mega D7 / D31 |
| GND | Mega GND |

- 바퀴가 생각과 반대로 돌면 `config.h`의 `MOTOR_L_INVERT`/`MOTOR_R_INVERT`를 바꾸거나 MxA·MxB 선을 바꿔 끼운다.
- MDD10 PWM 입력은 Mega가 리셋되는 1~2초 동안 떠 있다. 이때 바퀴가 움찔하면 PWM1·PWM2–GND에 10 kΩ 풀다운 (TODO_MEASURE).
- 모터 전원선(M1A/M1B 등)은 노이즈가 크다. 엔코더 신호선(C1/C2)과 떨어뜨려 배선한다.

### 주행 모터 JGB37-520 (6핀: M1, GND, C1, C2, VCC, M2)
| 모터 핀 | 역할 | 왼쪽 모터 | 오른쪽 모터 |
|---|---|---|---|
| M1 / M2 | 모터 전원 (극성 바꾸면 반대로 돎) | MDD10 M1A / M1B | MDD10 M2A / M2B |
| VCC | 엔코더 전원 3.3~5 V (**12 V 금지**) | Mega 5V | Mega 5V |
| GND | 엔코더 GND | Mega GND | Mega GND |
| C1 | 엔코더 A상 | **D2** | **D18** |
| C2 | 엔코더 B상 | **D3** | **D19** |

- 엔코더는 Mega가 4체배로 센다: 바퀴 1바퀴 = 11 PPR × 4 × 30:1 = 약 1320 tick (`hardware.yaml` `motor_ticks_per_rev`, TODO_MEASURE).
- 바퀴를 손으로 앞으로 돌렸을 때 `$ENC` 값이 줄어들면 `config.h`의 `ENC_L_INVERT`/`ENC_R_INVERT`를 바꾼다 (또는 C1·C2를 바꿔 꽂는다).
- 2026-10-31 엔코더 배선 수리 뒤 오른쪽 C1·C2가 서로 바뀌어 꽂힌 채 글루건으로 고정했다 → 코드에서 `ENC_R_INVERT 1`로 맞춤. 선을 원래대로 바꾸면 0으로 되돌린다.
- RRC 때 기록 (2026-10-08): 모터 쪽 흰 플러그 C1·C2 **접점 불량**으로 엔코더 신호가 안 들어간 적이 있다.
  배선 후 `scripts/pc_serial_tester.py`로 바퀴를 손으로 돌려 tick이 변하는지 먼저 확인한다 (`firmware/mega_sensor_controller/README.md`).
- 엔코더 방향이 반대거나 신호가 없으면 Mega가 폭주 감지로 모터를 끈다 (status `fault_reverse` / `fault_stall`). 노드를 다시 켜야 풀린다.

### IMU BNO085 (I2C)
| BNO085 | Mega | 비고 |
|---|---|---|
| VIN | 5V 또는 3.3V | 보드 종류에 따름 (아래) |
| GND | GND | |
| SDA / SCL | D20 / D21 | |

- ⚠ Mega는 SDA/SCL이 보드 안에서 **5 V로 풀업**되어 있다. Adafruit·SparkFun처럼 레벨 시프터가 있는 보드는 바로 연결,
  3.3 V 전용 저가 보드는 VIN 3.3V + **I2C 레벨 시프터**가 필요하다 (TODO_MEASURE: 실제 보드 확인).
- I2C 주소: Adafruit 0x4A / SparkFun 0x4B → `config.h` `IMU_I2C_ADDR`. 배선 후 `SIM_IMU 0`.
- 위치·방향은 URDF `imu_link` (TODO_MEASURE).

### HC-SR04 × 4
| 펌웨어 순서 | ROS 토픽 | 방향 | Trig | Echo | 장착 높이 |
|---|---|---|---|---|---|
| 0 | /range/front | 전방 | D22 | D23 | 지면에서 약 14 cm (TODO_MEASURE) |
| 1 | /range/left | 좌측 | D24 | D25 | **낮게** (정확한 높이 TODO_MEASURE) |
| 2 | /range/right | 우측 | D26 | D27 | 지면에서 약 14 cm (TODO_MEASURE) |
| 3 | /range/rear | 후방 | D28 | D29 | 지면에서 약 14 cm (TODO_MEASURE) |

- VCC ×4 → 브레드보드 위 + 레일 (Mega 5V), GND ×4 → 위 − 레일 (Mega GND)
- 2026-09-30 결정: 결선도대로 전방·좌측·우측·후방에 단다. 토픽 이름도 이에 맞춰 바꿨다 (예전 front_left/front_right/rear_left/rear_right).
- 2026-10-29 결선도에는 초음파가 그려져 있지 않지만 **그대로 쓴다** (팀 확인).
- 센서 위치·방향은 `poli_description` 패키지(URDF)의 `ultrasonic_<이름>_link`에 들어 있다.

### 집게 서보 (DS3218) — Mega D9 (2026-10-29 RRC PWM 포트에서 다시 이전)
| 서보 선 | 연결 |
|---|---|
| 노랑 (신호) | Mega D9 |
| 빨강 (+) | 5V 10A 컨버터 출력 + (**Mega 5V 사용 금지**) |
| 갈색 (GND) | 5V 10A 컨버터 출력 −, 그리고 Mega GND와 공통 (필수) |

- DS3218 정격 4.8~6.8 V → 5 V면 범위 안 (6 V보다 토크·속도 약 10~15% 낮음).
- 열림/닫힘 각도: `config.h`의 `GRIP_OPEN_DEG`(90도 = 1500 µs) / `GRIP_CLOSE_DEG`(120도) (TODO_MEASURE, 실제 집게로 보정)
  - 2026-10-30 실측 (`firmware/servo_test`): 열림 1500 µs, 빈 집게가 끝까지 닫히는 값 2000 µs(135도). 큐브를 잡는 값은 TODO_MEASURE
- 첫 집게 명령 전에는 서보에 신호를 보내지 않는다 (규정 3.5.6).

## 2. RRC Lite — 사용 안 함 (2026-10-29)
- MDD10 + Mega로 바꾸면서 RRC Lite를 뺐다. 옛 기록은 `docs/rrc_protocol.md`, `docs/rrc_adapter_plan.md`에 남겨 둔다.
- RRC Lite가 맡던 것 → 지금: 주행 모터·엔코더 → MDD10 + Mega, 내장 IMU → MPU6050 (파이 I2C, 2026-10-30), 집게 PWM 포트 → Mega D9, 입력 전압 보고(`/battery_state`) → 없음.

## 3. Raspberry Pi 5
| 장치 | 연결 | udev 별칭 |
|---|---|---|
| Arduino Mega 2560 | USB Serial | /dev/robot_mega |
| RPLIDAR | USB (어댑터 보드) | /dev/robot_lidar |
| 카메라 | CSI | — |
| IMU MPU6050 (GY-521) | 소프트웨어 I2C 버스 3, 주소 0x68 | /dev/i2c-3 |
| 전원 | 5V 10A 컨버터 출력 → USB-C | — |

- ⚠ Pi 5는 USB-C로 5 V 5 A(USB PD) 협상을 못 하면 USB 포트 전체 전류를 600 mA로 제한한다. 일반 5 V 컨버터는 PD를 못 하므로
  LiDAR(최대 260 mA) + Mega(엔코더·초음파 포함)가 모자랄 수 있다. `/boot/firmware/config.txt`에 `usb_max_current_enable=1`을 넣고
  LiDAR·Mega가 끊기지 않는지 확인한다 (TODO_MEASURE).

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
- 배터리: **12 V** (2026-10-29 변경, 예전 XEON 6S 22.2V LiPo). 종류·용량 TODO (3S LiPo면 완충 12.6 V)
- 배터리 → **비상정지 스위치** → 퓨즈(SZH-FU003, ATO 퓨즈 인라인 홀더) → 분배
  - → MDD10 B+ / B− (주행 모터, 12 V 그대로. 모터 정격 12 V)
  - → 5V 10A 컨버터 → Raspberry Pi 5 (USB-C), 집게 서보 DS3218
- Mega는 Pi USB로 전원을 받는다. 엔코더·초음파·BNO085는 Mega 5V/3.3V에서 받는다.
- XL4015 벅 컨버터 3개(12 V RRC / 5 V Pi / 6 V 서보)는 더 이상 쓰지 않는다.
- ⚠ Pi와 서보가 같은 5 V를 나눠 쓴다. 서보가 잡을 때 순간 2 A 이상(TODO_MEASURE) 당기면 Pi 전압이 떨어질 수 있다.
  서보 + / − 근처에 470~1000 µF 콘덴서를 달고, 잡기 시험 때 Pi가 재부팅·저전압 경고(`vcgencmd get_throttled`)를 내지 않는지 확인한다.
- 각 컨버터 실제 출력 전압(무부하 / 최대부하): TODO_MEASURE

### 비상정지 스위치 (E-stop) — 2026-09-30 결정: 전체 전원 차단
- 위치: **배터리 바로 뒤**, 퓨즈·분배보다 앞. 누르면 로봇 전체(모터, Pi, Mega, 서보) 전원이 끊긴다.
- 스위치 조건 (TODO: 부품 선정)
  - 정격 전압: DC 배터리 완충 전압 이상 (12 V 배터리 → 최소 15 V 정격 권장. 6S 때 기준 25.2 V 이상이면 그대로 써도 됨)
  - 정격 전류: 전체 최대 전류 이상. 모터 2개 스톨 + Pi 5 + 서보를 더한 값 (TODO_MEASURE)
  - 누르면 잠기는(래치) 버섯형 권장, 로봇 윗면처럼 손이 바로 닿는 곳
- 주의: 누르면 Pi도 바로 꺼진다 (SD 카드 손상 가능). 평소 끄기는 소프트웨어 종료(`sudo poweroff`) 후 전원을 끈다.
- 2026-10-29부터 Mega 펌웨어가 명령이 0.3 s 끊기면 스스로 모터를 멈추고, 엔코더 이상(방향 반대·신호 없음)도 감지해 멈춘다.
  그래도 펌웨어가 멈추거나 MDD10이 고장 나면 소용없으므로 **물리 비상정지는 계속 필수**다.
