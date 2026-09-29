# mega_sensor_controller — Arduino Mega 2560 펌웨어 (조원 A)

초음파 HC-SR04 ×4 순차 측정 + 집게 서보(D9) + Pi와 시리얼 통신 (`docs/serial_protocol.md` v1.1).
모터·엔코더·IMU는 RRC Lite 담당이라 여기에 없다.

| 파일 | 내용 |
|---|---|
| `mega_sensor_controller.ino` | main loop, `$GRIP` 수신·파싱, RNG 8 Hz / GST 2 Hz 전송 |
| `config.h` | SIM/REAL 스위치, 핀, 집게 각도 |
| `serial_protocol.h` | XOR 체크섬, `sendPacket` |
| `ultrasonic.h/.cpp` | 초음파 (30 ms 슬롯마다 1개씩 → 동시 Trigger 없음) |
| `gripper.h/.cpp` | 집게 (1도씩 서서히 이동) |

## SIM / REAL
부품이 없는 동안 `config.h`의 `SIM_ULTRASONIC 1`, `SIM_GRIPPER 1`로 모의값을 보낸다.
부품이 오면 해당 스위치만 0으로 바꾸고, `TODO_MEASURE` 값(핀·집게 각도)을 실측으로 고친다.

## 컴파일 (보드 없이 가능)
```bash
arduino-cli core install arduino:avr
arduino-cli lib install Servo          # REAL(SIM_GRIPPER 0) 빌드에 필요. Arduino IDE는 기본 포함
arduino-cli compile --fqbn arduino:avr:mega firmware/mega_sensor_controller
```
2026-09-29 확인: SIM 빌드 7,854 B / RAM 464 B, REAL 빌드 8,892 B / RAM 624 B (Mega 8 KB 중 8% 미만).

## 업로드 후 확인 (보드 도착 후)
```bash
arduino-cli upload -p <포트> --fqbn arduino:avr:mega firmware/mega_sensor_controller
python scripts/pc_serial_tester.py <포트>     # pip install pyserial, o=열기 c=닫기 q=종료
```
- `$RNG` 초당 약 8줄, `$GST` 초당 2줄, L LED 1초 토글
- 시리얼 모니터(115200, "Both NL & CR")에 `$GRIP,1,1*0C` → 즉시 `$GST` 응답, 각도 40→120 서서히 이동
- 그다음 Pi에서 `ros2 run poli_hardware mega_bridge_node` → `/range/*`, `/gripper/set` 확인
