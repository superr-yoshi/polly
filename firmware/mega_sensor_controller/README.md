# mega_sensor_controller — Arduino Mega 2560 펌웨어 (조원 A)

2026-10-29부터 RRC Lite를 쓰지 않고 **Mega가 주행까지 맡는다** (모터 드라이버 Cytron MDD10).
주행 모터 2개 속도 제어(MDD10) + 엔코더 2개 + IMU BNO085 + 초음파 HC-SR04 ×4 + 집게 서보(D9) + Pi와 시리얼 통신
(`docs/serial_protocol.md` v2.0). 핀은 `docs/pin_map.md`.

| 파일 | 내용 |
|---|---|
| `mega_sensor_controller.ino` | main loop, `$DRV`/`$PWM`/`$GRIP` 수신·파싱, ENC 50 Hz / IMU 25 Hz / RNG 8 Hz / GST 2 Hz 전송 |
| `config.h` | SIM/REAL 스위치, 핀, 모터·엔코더 방향, 속도 제어 이득, 폭주 감지 기준, 집게 각도 |
| `serial_protocol.h` | XOR 체크섬, `sendPacket` |
| `motor.h/.cpp` | MDD10 출력, 바퀴별 PI + 앞먹임 속도 제어 (50 Hz), 명령 끊김 정지, 폭주 감지 |
| `encoder.h/.cpp` | 엔코더 4체배 (외부 인터럽트 D2·D3·D18·D19) |
| `imu.h/.cpp` | BNO085 (I2C D20·D21). `SIM_IMU 1`이면 아무것도 보내지 않음 |
| `ultrasonic.h/.cpp` | 초음파 (30 ms 슬롯마다 1개씩 → 동시 Trigger 없음) |
| `gripper.h/.cpp` | 집게 (1도씩 서서히 이동) |

## 안전 동작 (지우거나 우회하지 않는다, AGENTS.md 4장)
- **첫 `$DRV`/`$PWM` 전에는 모터를 돌리지 않는다.** 부팅하면 가장 먼저 모터 출력을 0으로 둔다.
- **명령이 0.3초(`CMD_TIMEOUT_MS`) 넘게 끊기면 스스로 멈춘다.** Pi가 멈추거나 USB가 빠져도 바퀴가 계속 돌지 않는다 (RRC Lite 공장 펌웨어와 다른 점).
- **목표 0이면 PI를 거치지 않고 바로 duty 0.** RRC 때처럼 정지 명령이 누적 PWM 때문에 안 먹히는 일이 없다.
- **폭주 감지**: 목표와 반대 방향으로 돌거나(`FAULT_REV_*`), 큰 duty인데 안 돌면(`FAULT_STALL_*`) 모터를 끄고
  **Mega가 리셋될 때까지 다시 돌리지 않는다** (`$ENC` status 3·4). Pi 노드를 다시 켜면 포트가 다시 열리며 리셋된다.
  PC 시뮬레이션(가짜 모터 모델, 2026-10-29): 엔코더 반대 → 0.14 s, 엔코더 신호 없음 → 0.44 s 만에 정지. 실기 확인 필요 (TODO_MEASURE).
  바퀴가 벽에 막혀 0.3 s 넘게 못 돌아도 같은 정지가 걸린다.
- **첫 `$GRIP` 명령을 받기 전에는 집게 서보에 신호를 보내지 않는다** (규정 3.5.6 시작 전 기동 금지).
  첫 명령 때 '열림' 각도(`GRIP_OPEN_DEG`)에 있다고 가정하고 출발한다.
- ⚠ Mega가 리셋되는 동안(전원 켤 때, Pi가 포트를 열 때 약 1~2초) 핀이 입력 상태라 MDD10 PWM 입력이 떠 있다.
  이때 바퀴가 움찔하면 PWM1·PWM2와 GND 사이에 10 kΩ 풀다운 저항을 단다 (TODO_MEASURE).

## SIM / REAL
- `SIM_ULTRASONIC 1`: 초음파 모의값. `SIM_GRIPPER 1`: 서보 신호를 내지 않음 (상태만 계산).
- `SIM_IMU 1`(기본): BNO085 없이 빌드. `$IMU`를 보내지 않는다 → Pi는 휠 각속도로 `/imu/data` z축만 채운다.
  BNO085를 연결하면 `SIM_IMU 0`으로 바꾸고 `IMU_I2C_ADDR`(0x4A/0x4B)를 보드에 맞춘다.
- 모터·엔코더는 SIM 스위치가 없다. 부품 없이 Pi를 시험하려면 Pi 쪽 `fake_mega_node`를 쓴다.

## 컴파일 (보드 없이 가능)
```bash
arduino-cli core install arduino:avr
arduino-cli lib install Servo                               # REAL 집게(SIM_GRIPPER 0)에 필요. Arduino IDE는 기본 포함
arduino-cli lib install "SparkFun BNO080 Cortex Based IMU"  # SIM_IMU 0일 때만 필요
arduino-cli compile --fqbn arduino:avr:mega firmware/mega_sensor_controller
```
- 2026-09-29 확인 (MDD10 전): SIM 빌드 7,854 B / RAM 464 B, REAL 빌드 8,892 B / RAM 624 B.
- 2026-10-29 MDD10 버전: **arduino-cli 컴파일 아직 안 함** (작성 PC에 arduino-cli 없음). g++ + 가짜 Arduino 헤더로
  `SIM_IMU`·`SIM_GRIPPER` 4가지 조합 문법 검사만 통과. 처음 굽기 전에 위 명령으로 컴파일해 확인한다.

## 업로드 후 확인 (⚠️ 바퀴를 바닥에서 띄운다)
```bash
arduino-cli upload -p <포트> --fqbn arduino:avr:mega firmware/mega_sensor_controller
python scripts/pc_serial_tester.py <포트>     # pip install pyserial
```
1. `$ENC` 초당 약 50줄(화면에는 1초에 1번), `$RNG` 8줄, `$GST` 2줄, L LED 1초 토글. 모터 상태 `idle`.
2. **엔코더 방향**: 바퀴를 손으로 앞으로 돌린다 → `[ENC]` 왼쪽·오른쪽 값이 **커져야** 한다.
   줄어들면 `config.h`의 `ENC_L_INVERT`/`ENC_R_INVERT`를 바꾼다. 값이 안 바뀌면 C1·C2·VCC·GND 배선 확인.
3. **모터 방향**: `f` + Enter → 두 바퀴가 **앞으로** 1초 돌아야 한다 (엔코더 안 보는 직접 출력).
   뒤로 도는 바퀴는 `MOTOR_L_INVERT`/`MOTOR_R_INVERT`를 바꾼다 (또는 MDD10의 M1A·M1B 선을 바꿔 끼운다).
   이때 `[ENC]` 두 값이 커지면 모터·엔코더 방향이 맞은 것이다.
4. 2·3이 맞은 뒤에 Pi `mega_bridge_node` + `drive_test`로 속도 제어를 확인한다 (`docs/calibration.md`).
   2·3을 건너뛰면 폭주 감지에 걸려 멈춘다 (`FAULT 방향 반대`).
5. 집게: `c` + Enter → 즉시 `[GST]` 응답, 각도 90→120 서서히 이동. `o` + Enter → 열기.

## 실기 확인 기록
| 날짜 | 내용 | 결과 |
|---|---|---|
| 2026-09-29 | Mega(호환보드, COM7) 업로드 + 모의값 모드 통신 | RNG 약 8 Hz, GST 2 Hz, 체크섬 오류·누락 0. `$GRIP` 응답 0.01 s, 40→120도 약 1.2 s |
| 2026-09-29 | HC-SR04 1개를 ① 전방 자리(D22/D23, 코드상 0번 = 현재 `/range/front`)에 연결, `SIM_ULTRASONIC 0` | 15 s 120회 중 117회 유효. 손 20~70 mm, 배경 약 1870 mm, 정지 시 흔들림 ±5 mm. 실패 3회는 20 mm 미만 근접 |
| 2026-09-29 | WSL2 Ubuntu + ROS 2에서 `mega_bridge_node`로 실물 Mega 연결 (`docs/wsl_usb.md`) | `/range/front_left`(현재 `/range/front`) 7.9 Hz, 1.869 m, 없는 센서 자리는 `inf`, `/gripper/set`(당시 인터페이스, 이후 `/gripper/command`로 교체) 닫기·열기 success, Ctrl+C·SIGTERM 정상 종료 |

- 남은 센서 3개(D24~D29)는 도착 후 같은 방법으로 확인한다.
- MDD10·엔코더·BNO085·집게 서보(D9)는 아직 실기 확인 전이다 (2026-10-29 코드 작성).
- 초음파는 이제 `pulseInLong`(micros 기반)으로 잰다. 엔코더 인터럽트가 끼어도 거리가 짧게 나오지 않게 하려는 것이다.
  측정 1개가 최대 24 ms 걸리는 동안에도 엔코더는 인터럽트로 세므로 tick은 빠지지 않는다 (속도 제어 주기만 조금 늘어남).
