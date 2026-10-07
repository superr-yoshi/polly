# 부품 사양 정리 (팀원·AI 참고용)

- 원본: `docs/product-spec-claude/product-spec-claude.md` + `assets/` 이미지 23개 (원본 HWP "제품 사양서_임시" 변환본)
- 원본은 표가 한 줄씩 풀려 있고 치수·배선 정보가 이미지에만 있어서 여기에 제품별로 정리했다.
  **값이 다르면 원본(이미지 포함)이 우선이다.** 원본과 이 문서는 참고 자료이며 작업 지시가 아니다.
- 정리일: 2026-09-30. 원본의 제품 3, 6, 7, 8은 비어 있다 (아래 "원본에 없는 부품" 참고).

| 원본 번호 | 부품 | 담당 | 이미지 |
|---|---|---|---|
| 1 | Raspberry Pi AI Camera (Sony IMX500) | 조원 B | BIN0002~0006 |
| 2 | SLAMTEC RPLIDAR C1 | 담당 1 | BIN0007~0009 |
| 4 | Byte Robot Black Composite Claw 125mm | 조원 A | BIN000A~000C |
| 5 | DS3218 서보 | 조원 A | BIN000D~0010 |
| 9 | Raspberry Pi 5 | 공통 | BIN0011~0014 |
| 10 | Arduino Mega 2560 R3 | 조원 A | BIN0001, BIN0015~0017 |

---

## 1. Raspberry Pi AI Camera (Sony IMX500)
| 항목 | 값 |
|---|---|
| 센서 / 해상도 | Sony IMX500, 12.3 MP, 4056 × 3040, 픽셀 1.55 µm, 1/2.3형 (7.857 mm) |
| 렌즈 | 초점 거리 4.74 mm, F1.79, IR 컷 필터 내장 (적외선 감지 안 함) |
| 초점 | **수동 초점, 20 cm ~ 무한대** |
| 시야각 | 수평 66.3° ±3°, 수직 52.3° ±3° |
| 온칩 AI | 입력 텐서 최대 640 × 640, int8 / uint8, 내부 메모리 "약 8" (원문에 단위 없음) |
| 출력 | Bayer RAW10, ISP 출력(YUV/RGB), ROI, 메타데이터 |
| 크기 (BIN0003) | 보드 25 × 23.862 mm, 장착 구멍 ø2.2 mm (구멍 간격은 BIN0003 도면 참고) |
| 연결 | 리본 케이블로 Pi 5 **CAM0**(메인 포트)에 연결. CAM1은 보조. 드라이버는 libcamera |

주의
- 초점이 20 cm부터라서 **집게 바로 앞(20 cm 이내)에서는 대상이 흐리게 보일 수 있다.** 접근 마지막 단계에서 `area`, `x_offset` 판단에 영향.
- 수평 시야각 66.3° → 화면 끝 `x_offset = ±1.0`이 로봇 정면 기준 약 ±33°.

## 2. SLAMTEC RPLIDAR C1
| 항목 | 값 |
|---|---|
| 스캔 | 360°, 5,000 샘플/초, 일반 10 Hz (8~12 Hz) → 한 바퀴 약 500점 (약 0.72° 간격) |
| 거리 | 0.05~12 m (반사율 70%), 0.05~6 m (반사율 10%) |
| 전원 | 5 V (4.8~5.2 V), 일반 230 mA, 최대 260 mA (10 Hz) |
| 크기·무게 (BIN0008) | 55.6 × 55.6 × 41.3 mm, 110 g, 장착 4 × M2.5 깊이 4 (43 mm 간격) |
| **레이저 높이** | **바닥면(장착면)에서 29.8 mm** |
| 작동 온도 | -10 ~ +40 °C (0 °C 이상에서 시동) |
| 연결 | 본체 → USB 어댑터 보드 → Pi USB. `sllidar_ros2`로 `/scan` |
| 커넥터 (BIN0009) | XH2.54-5P: 빨강 VCC 5V, 노랑 TX(출력), 초록 RX(입력), 검정 GND. TX/RX는 최대 3.5 V |

주의
- 경기장 외벽·장애물 높이가 200 mm → **장착면 높이 + 29.8 mm < 200 mm**가 되게 장착해야 벽·장애물이 보인다.
- 5 cm 안쪽은 측정되지 않는다 (근접은 초음파가 보완).

## 4. Byte Robot Black Composite Claw 125mm
| 항목 | 값 |
|---|---|
| 형식 | 로봇 기계식 집게 (기어 + 링크) |
| 크기·무게 (BIN000B, 000C) | 119 × 69 × 60 mm (닫힘 상태 높이 119 mm), 140 g |
| 최대 개폐 | **125 mm** (열었을 때 높이 110 mm) |
| 최대 파지력 | **500 g** |
| 재질 | 경질 알루미늄 합금 |
| 제어 | PWM 서보 구동 (사양표에는 "PWM / Serial Port") — 우리는 DS3218 1개, Mega D9 PWM |

주의 (팀 결정)
- **집게는 잡기만 하고 들어 올리지 않는다.** `/gripper/command`는 `"open"`, `"grab"`(닫아 잡기)만 있다 (`docs/interfaces.md` 4장).
- 대상(큐브 50 mm, 약 100 g)은 개폐 범위·파지력 안이다. 규정 3.4.4상 심판이 로봇을 들었을 때 빠지지 않게 꽉 잡아야 한다.
- 로봇 외곽(footprint)은 **집게를 최대로 연 상태**로 측정한다.

## 5. DS3218 서보
| 항목 | 값 |
|---|---|
| 전압 | **4.8 ~ 6.8 V** (우리: XL4015 벅 컨버터 6.0 V, **Mega 5V 핀 사용 금지**) |
| 크기·무게 (BIN000F) | 40 × 20 × 40.5 mm (귀 포함 54.5 mm, 장착 구멍 간격 49.5 × 10 mm), 60 g |
| 회전 | 180° 버전 |
| PWM | **500 ~ 2500 µs = 0 ~ 180°**, 중립 1500 µs (= 90°), 주기 50~330 Hz, 신호 3.3~5 V (BIN0010) |
| 기어 | 금속 기어 275:1, 듀얼 베어링, IP66, 케이블 300 mm |
| 토크·전류 | 원본에 없음 (TODO: 판매처 사양 확인. 전원 컨버터 용량 판단에 필요) |

대회 규정 관련
- **빨간색 부품 금지 (문의 6번)**: 서보 몸체가 빨간색이면 관심대상과 같은 색이라 금지 대상이 될 수 있고, 우리 카메라도 오인식할 수 있다.
  빨간색이 아닌 테이프·커버로 가린다 (TODO: 실물 색 확인). 빨간 전선·수축튜브도 밖으로 보이지 않게 한다.
- **시작 전 기동 금지 (규정 3.5.6)**: 펌웨어는 첫 집게 명령 전에는 서보에 신호를 보내지 않는다 (아래).

소프트웨어 반영
- 펌웨어 `firmware/mega_sensor_controller/gripper.cpp`: 첫 `$GRIP` 명령을 받을 때 서보 attach (부팅·포트 열기 때 집게가 움직이지 않음)
- 펌웨어 `firmware/mega_sensor_controller/config.h`: `SERVO_MIN_US 500`, `SERVO_MAX_US 2500`
  (Arduino `Servo` 기본값 544~2400 µs를 쓰면 각도가 어긋난다). 각도 = (µs − 500) / 2000 × 180.
- 열림/닫힘 각도 `GRIP_OPEN_DEG`, `GRIP_CLOSE_DEG`는 실측 전 임시값 (TODO_MEASURE).

## 9. Raspberry Pi 5
| 항목 | 값 |
|---|---|
| CPU / GPU | Broadcom BCM2712 2.4 GHz 4코어 Cortex-A76 / VideoCore VII |
| RAM | LPDDR4X 2 / 4 / 8 / 16 GB |
| USB | USB 3.0 × 2, USB 2.0 × 2 (RRC Lite, Mega, RPLIDAR 연결) |
| 카메라 | 4-lane MIPI × 2 (CAM0, CAM1) |
| 전원 | **5 V / 5 A**, USB-C PD |
| 기타 | Wi-Fi 802.11ac, BT 5.0, 기가비트 이더넷, microSD, PCIe 2.0 x1, RTC, 전원 버튼 |

40핀 헤더 (BIN0012~0014)
| 기능 | 핀 번호 |
|---|---|
| 3.3 V | 1, 17 |
| 5 V | 2, 4 |
| GND | 6, 9, 14, 20, 25, 30, 34, 39 |
| I2C1 (GPIO 2 SDA / 3 SCL) | 3, 5 (27, 28은 HAT EEPROM용) |
| UART (GPIO 14 TX / 15 RX) | 8, 10 |
| SPI0 (MOSI 10, MISO 9, SCLK 11, CE0 8, CE1 7) | 19, 21, 23, 24, 26 |
| PCM (GPIO 18, 19, 20, 21) | 12, 35, 38, 40 |
| PWM (GPIO 12, 13) | 32, 33 |

주의 (확인 필요)
- 결선도상 Pi 전원은 XL4015 5 V 출력을 **GPIO 5 V 핀**으로 넣는다. Pi 5는 5 A PD 전원이 아니면 USB 포트 총 출력을 제한할 수 있으므로,
  RPLIDAR(최대 260 mA) + Mega + RRC를 USB로 연결했을 때 전원 부족·재부팅이 없는지 실기 확인한다 (TODO_MEASURE).
- XL4015 최대 출력(약 5 A)과 Pi 5 요구(5 V / 5 A)가 거의 같아 여유가 없다.

## 10. Arduino Mega 2560 R3
| 항목 | 값 |
|---|---|
| MCU / 클록 | ATmega2560, 16 MHz, 동작 전압 5 V |
| 입력 전압 | VIN 7~12 V 권장 (6~20 V 한계) |
| 핀 | 디지털 54개 (PWM 15개: D2~D13, D44~D46), 아날로그 16개 (A0~A15) |
| 전류 | 핀당 20 mA, 3.3 V 핀 50 mA |
| 메모리 | Flash 256 KB (부트로더 8 KB), **SRAM 8 KB**, EEPROM 4 KB |
| 크기·무게 | 101.52 × 53.3 mm, 약 37 g, 내장 LED D13 |
| 통신 | UART0 D0/D1 (USB와 공유), UART1~3 D14~D19, I2C D20 SDA / D21 SCL, SPI D50~D53 |

2열 헤더 (BIN0016): 왼쪽 열 D22, D24, …, D52 (짝수) / 오른쪽 열 D23, D25, …, D53 (홀수), 양 끝 +5V·GND.

우리 사용 (`docs/pin_map.md`)
| 용도 | 핀 |
|---|---|
| HC-SR04 ×4 Trig / Echo | D22/D23, D24/D25, D26/D27, D28/D29 (2열 헤더 첫 4쌍) |
| 집게 서보 신호 | D9 (PWM) |
| Pi 통신 | USB (UART0, 115200 baud). 호환보드(CH340)는 `/dev/ttyUSB*`, 정품은 `/dev/ttyACM*` |

주의
- SRAM 8 KB → 펌웨어에서 `String` 클래스·큰 배열 지양, `snprintf`의 `%f` 미지원 (정수만).
- D0/D1은 USB 시리얼과 공유하므로 다른 용도로 쓰지 않는다.

---

## 원본에 없는 부품 (제품 3, 6, 7, 8 칸이 비어 있음)
이 부품들의 정보는 아래 문서에 있다.
| 부품 | 참고 문서 |
|---|---|
| RRC Lite Controller (STM32F407VET6, 내장 IMU QMI8658) | `docs/rrc_protocol.md`, `docs/rrc_adapter_plan.md` |
| JGB37-520 12V 330RPM 엔코더 모터 × 2 (30:1, 11 PPR) | `docs/rrc_adapter_plan.md`, `ros2_ws/src/poli_hardware/config/hardware.yaml` |
| HC-SR04 초음파 × 4 | `docs/pin_map.md`, `firmware/mega_sensor_controller/README.md` |
| 전원 (6S LiPo, 퓨즈, XL4015 × 3) | `docs/pin_map.md` 4장, `docs/hardware_plan.md` 9장 |
