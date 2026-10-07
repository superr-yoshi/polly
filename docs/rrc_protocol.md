# RRC Lite ↔ Pi 시리얼 프로토콜 (제조사 펌웨어 분석)

- 대상: **Hiwonder RRC Lite Controller (STM32F407VET6)**. "ROS Robot Control Board"(별도 제품)가 아님
- 출처: 제조사 자료실 `RosRobotControllerLite_ros_250814.zip` → `RosRobotControllerLite_ros_250811` 소스
  - RRC Lite 전용 확인 근거: `Doc/log.txt`에 "RRC의 IIC가 RRCLite의 IIC2, IMU 칩 QMI8658", `.ioc`의 MCU = STM32F407VET6
- 구현: `ros2_ws/src/poli_hardware/poli_hardware/rrc_protocol.py` (순수 모듈, pytest 있음)
- 분석일: 2026-09-29. 보드에 다른 버전 펌웨어가 들어 있으면 다시 확인한다.

## 공통
| 항목 | 값 | 출처 |
|---|---|---|
| 통신 | USART1, **1,000,000 baud**, 8N1 | `RosRobotControllerM4.ioc` |
| 프레임 | `AA 55 │ func │ len │ data[len] │ crc8` | `Misc/packet.c` |
| CRC | CRC-8/MAXIM(표 방식), 범위 = func + len + data | `Misc/checksum.c` |
| 바이트 순서 | little-endian, 실수는 float32 | STM32 |

## func 번호
| func | 이름 | 우리가 쓰는 것 |
|---|---|---|
| 0 | SYS | 수신: 배터리 전압 (sub 0x04, uint16 mV, 약 1 Hz) — 현재 발행 안 함 |
| 3 | MOTOR | **송신: 모터 속도·정지·종류 설정** |
| 4 | PWM_SERVO | **송신: 집게 서보 펄스 / 수신: 현재 펄스** (2026-10-07부터) |
| 7 | IMU | **수신: 가속도·자이로 (약 50 Hz)** |
| 1, 2, 5, 6, 8~11 | LED, 부저, 버스 서보, 버튼, 게임패드, SBUS, OLED, RGB | 사용 안 함 |

## 모터 (func 3)
| sub | data | 뜻 |
|---|---|---|
| 1 | `01, n, [id(u8), rps(f32)] × n` | 여러 모터 속도 설정 |
| 3 | `03, mask(u8)` | mask 비트의 모터 정지 (0x0F = 전부) |
| 5 | `05, type(u8)` | 모터 종류: 0 JGB520, **1 JGB37**, 2 JGA27, 3 JGB528 |

- **id는 0부터**: 0 = M1, 1 = M2. 펌웨어가 id 범위를 검사하지 않으므로 Pi에서 0~3만 보낸다.
- **rps = 출력축 초당 회전수**. 펌웨어가 엔코더로 PID 제어한다.
- 부팅 직후 모터 파라미터는 JGA27(1040 ticks/rev) 기본값이다 → **시작할 때 반드시 type=1(JGB37) 전송**.
- 펌웨어의 JGB37 = 45:1 기어, 출력축 **1980 ticks/rev**, 제한 3.0 rps, PID 40/2/2 가정.
  우리 모터(JGB37-520 330RPM)의 기어비가 다르면 실제 속도가 비율만큼 달라진다 → `motor_ticks_per_rev`로 보정.
- 펌웨어 명령 경로는 rps 제한을 적용하지 않는다 → Pi에서 `max_motor_rps`로 제한.
- 제조사 탱크 차체 규칙: M1 = 왼쪽(부호 반전), M2 = 오른쪽.

## PWM 서보 (func 4) — 집게
| sub | data (`<` little-endian) | 뜻 |
|---|---|---|
| 0x03 | `<BHBH` = `03, duration_ms(u16), servo_id(u8), pulse_us(u16)` | 서보 1개를 `duration_ms` 동안 `pulse_us`로 이동 |
| 0x05 | `05, servo_id(u8)` | 현재 펄스 요청 → 보드가 `servo_id, 05, pulse_us(u16)` (func 4)로 응답 |

- **servo_id는 1~4** (보드 인쇄 번호). 펌웨어가 0을 검사하지 않으므로 Pi에서 1~4만 보낸다.
- 펄스 500~2500 µs (DS3218: 0~180°), 이동 시간 20~30000 ms로 Pi에서 제한한다.
- **전원을 켜면 펌웨어가 4포트 모두 1500 µs를 낸다** (`pwm_servos_init`) → 열림 위치를 1500 µs로 맞췄다.
  포트를 열어도 보드는 재부팅하지 않는다 (IMU가 끊기지 않고 50 Hz 유지 확인).
- 서보 포트 전원은 보드 점퍼로 5V / 입력 전압(VIN) 중 고른다. **DS3218은 5V 필수** (`docs/pin_map.md` 2장).
- 구현: `rrc_protocol.pwm_servo_set_frame`, `pwm_servo_read_frame`, `parse_pwm_servo_report`,
  상태 판단은 `poli_hardware/gripper.py` (`GripperController`). 이동 중에는 0.2 s마다 현재 펄스를 요청해
  목표에 닿으면 `closed`/`open`, 응답이 없어도 `move_ms + 0.5 s` 뒤에는 도착으로 본다.
- 첫 `/gripper/command`를 받기 전에는 서보 명령을 보내지 않는다 (규정 3.5.6).

## IMU (func 7, 24 bytes)
`ax, ay, az, gx, gy, gz` float32. 단위 **g, deg/s** → Pi에서 m/s², rad/s로 변환.
칩 QMI8658, 펌웨어 내부 `axis_convert` 적용 후 값. **ROS 축(x 전방, y 왼쪽, z 위)과 같은지는 실측 필요.**

## ⚠ 제조사 펌웨어의 한계 (중요)
1. **엔코더 값을 Pi로 보내지 않는다.** 엔코더는 STM32 속도 PID에만 쓰인다.
   → `/odom_raw`는 "보낸 명령 = 실제 속도"로 가정한 추정값이다. 바퀴가 미끄러지거나 막혀도 모른다.
   → EKF에서 회전은 IMU gyro z를 믿고, 위치 보정은 SLAM/AMCL(LiDAR)에 맡긴다.
2. **명령이 끊겨도 STM32가 스스로 멈추지 않는다.** 마지막 속도를 계속 유지한다.
   → Pi 노드가 25 Hz로 계속 명령(끊기면 0)을 보내고 종료할 때 정지 명령을 보내지만,
     **Pi 전원이 꺼지거나 USB가 빠지면 모터는 계속 돈다. 물리 E-stop 필수.**
3. 두 문제를 고친 POLI 패치 펌웨어(`firmware/rrc_lite_patch/`)를 만들어 두었으나 **보류**했다 (2026-10-01, 공장 펌웨어 그대로 사용).
   그래서 위 1·2의 한계는 그대로이고, 2번은 **물리 비상정지 스위치**로 대비한다.

## (보류) POLI 패치 펌웨어 추가 사항
| 항목 | 내용 |
|---|---|
| 명령 끊김 정지 | 모터 packet(func 3)이 500 ms 넘게 안 오면 모든 모터 `set_point = 0` |
| 엔코더 보고 | RRC → Pi, func 3, len 33, 20 ms 주기: `0x10`, `int32 count[4]`, `float rps[4]` |

- `rps`는 펌웨어의 모터 종류 ticks/rev(JGB37 = 1980) 기준이다. 실제 바퀴 속도는 Pi에서 `motor_ticks_per_rev`(1320)로 보정한다.
- Pi(`rrc_adapter_node`)는 보고가 오면 엔코더 기반, 0.2 s 넘게 안 오면 명령 기반 odom으로 자동 전환한다.
