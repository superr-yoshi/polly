# RRC Lite 펌웨어 POLI 패치 (조원 A) — ⏸ 보류

> **2026-10-01 결정: 사용하지 않는다. RRC Lite는 공장 펌웨어 그대로 쓴다.**
> 조원 A가 다시 결정하기 전에는 이 펌웨어를 빌드·굽기 하지 않는다. 필요해질 때를 위해 남겨 둔 자료다.
> 공장 펌웨어의 한계(명령이 끊겨도 안 멈춤, 엔코더 미보고)는 `docs/rrc_protocol.md` 참고. 대비책은 물리 비상정지 스위치.

제조사(Hiwonder) RRC Lite 펌웨어 `RosRobotControllerLite_ros_250811`(공장 hex `..._250814.hex`)에 두 기능을 더한다.

| 기능 | 왜 | 동작 |
|---|---|---|
| **명령 끊김 자동 정지** | 공장 펌웨어는 Pi가 죽거나 USB가 빠져도 **마지막 속도로 계속 달린다** (테두리 넘으면 즉시 탈락) | 모터 명령이 **500 ms** 넘게 안 오면 모든 모터 목표 속도 = 0 |
| **엔코더 값 보고** | 공장 펌웨어는 엔코더 값을 Pi로 보내지 않아 `/odom_raw`가 명령 기반 추정이었다 | 20 ms(50 Hz)마다 모터 4개 누적 tick·회전 속도 전송 → `/odom_raw`가 실측 기반 |

Pi 쪽(`poli_hardware`)은 이미 대응되어 있다. 엔코더 보고가 오면 자동으로 엔코더 기반 odom을 쓰고,
안 오면(공장 펌웨어) 예전처럼 명령 기반으로 동작한다. **펌웨어를 굽기 전·후 모두 같은 Pi 코드로 동작한다.**

## 폴더
| 경로 | 내용 |
|---|---|
| `files/Hiwonder/System/app.c` | 10 ms 루프에 `poli_motor_watchdog_and_report()` 추가 |
| `files/Hiwonder/System/packet_handle.c` | 모터 명령 받은 시각 기록 (`poli_motor_cmd_tick`) |
| `files/Hiwonder/Portings/packet_reports.h` | 엔코더 보고 구조체 `PacketReportEncoderTypeDef` |
| `files/MDK-ARM/RosRobotControllerM4.uvprojx` | Keil 프로젝트를 **ARM Compiler 6**용으로 전환 (아래 설명) |
| `poli_rrc_patch.diff` | 제조사 원본 대비 바뀐 줄 전부 (검토용) |

제조사 소스 전체(162 MB)는 저장소에 넣지 않는다. 아래 1단계에서 받는다.

## 새 프로토콜 (docs/rrc_protocol.md에도 기록)
RRC → Pi, `AA 55 | 03 | 21 | data(33) | crc8`
| 바이트 | 내용 |
|---|---|
| 0 | `0x10` (엔코더 보고) |
| 1~16 | `int32 × 4` 모터 0~3 누적 tick (하위 32비트) |
| 17~32 | `float × 4` 모터 0~3 출력축 rev/s (펌웨어의 모터 종류 ticks/rev 기준, JGB37 = 1980) |

---

## 0. 준비 (Windows PC, 1회)
1. **Keil MDK Community Edition** (무료, 비상업·교육용) 설치: https://www.keil.arm.com/mdk-community/
   - Arm 계정으로 로그인해 Community 라이선스를 활성화한다.
   - Keil 실행 → Pack Installer → **Keil::STM32F4xx_DFP** 설치.
2. **ATK-XISP** (굽기 프로그램): 제조사 자료실 `Appendix/Factory Firmware/ATK-XISP.exe`
3. USB-C 케이블 (RRC Lite의 USB-C = UART1)

제조사 자료실: https://drive.google.com/drive/folders/1z6TAcHH-v3K-ZPH04aTxfE3NwCIeg-rl

## 1. 제조사 소스 받기
`Appendix/Source Code/RosRobotControllerLite_ros_250814.zip`을 받아 압축을 푼다 → `RosRobotControllerLite_ros_250811` 폴더.

## 2. 패치 적용
`files/` 안의 내용을 제조사 폴더에 **같은 경로로 덮어쓴다.**
```powershell
# PowerShell (경로는 자기 PC에 맞게)
Copy-Item -Recurse -Force C:\...\polly\firmware\rrc_lite_patch\files\* C:\...\RosRobotControllerLite_ros_250811\
```

## 3. 빌드
- `MDK-ARM\RosRobotControllerM4.uvprojx`를 Keil로 열고 **Project → Build Target (F7)**.
- 또는 명령줄: `"C:\Keil_v5\UV4\UV4.exe" -b MDK-ARM\RosRobotControllerM4.uvprojx -j0 -o build.log`
- 결과: `MDK-ARM\RosRobotControllerM4\RosRobotControllerM4.hex` (공장 hex는 약 50 KB 코드)

### 왜 Compiler 6으로 바꿨나
제조사 프로젝트는 옛 **ARM Compiler 5** 설정인데, 최신 MDK Community에는 **Compiler 6만** 들어 있다.
그래서 프로젝트 파일에서 두 가지를 바꿨다.
1. `uAC6` 0 → 1 (컴파일러 6 사용)
2. FreeRTOS 포트 `portable/RVDS/ARM_CM4F` → `portable/GCC/ARM_CM4F` (RVDS 포트는 Compiler 5 전용 문법. GCC 포트가 Compiler 6 공식 대응)

사전 확인 (2026-09-30): 전체 C 파일 132개를 `arm-none-eabi-gcc -fsyntax-only`로 검사 → 오류 없음
(`syscall.c` 1개는 Keil 전용 분기라 GCC에서만 걸림, Compiler 6 분기가 이미 있음). **실제 Keil 빌드는 아직 안 해봤다.**

빌드가 안 되면
- "compiler version 5 not found" → Options for Target → Target → ARM Compiler: **Use default compiler version 6**
- 시작 파일(`startup_stm32f407xx.s`) 어셈블 오류 → Options for Target → Asm → Assembler Option: **armasm (Arm Syntax)**
- 그 밖의 오류는 `build.log`를 조원 A에게 보낸다.

## 4. 굽기 (ATK-XISP, 제조사 "Firmware Flashing Tutorial.pdf"와 같음)
1. RRC Lite USB-C를 PC에 연결 (Pi·ROS 노드·WSL(usbipd)이 포트를 잡고 있으면 먼저 끊는다).
2. ATK-XISP: COM 포트 선택, **115200 bps**, 제조사 설명서의 체크박스 3개 선택,
   하단 **"Reset@DTR Low(<-3V), ISP@RTS High"** 선택.
3. 3단계의 `.hex` 선택 → **Start Programming** → 완료 표시 확인.

## 5. 되돌리기 (문제가 생기면)
같은 방법으로 공장 hex `Appendix/Factory Firmware/RosRobotControllerLite_ros_250814.hex`를 굽는다.
Pi 코드는 공장 펌웨어에서도 동작한다 (odom만 명령 기반으로 돌아감).

## 6. 시험 (반드시 바퀴를 띄우고)
```bash
ros2 launch poli_hardware hardware.launch.py use_fake_hardware:=false
```
| 시험 | 방법 | 통과 기준 |
|---|---|---|
| 엔코더 보고 | 노드 로그 | `RRC 엔코더 보고 수신 -> /odom_raw 엔코더 기반` |
| 엔코더 부호 | 바퀴를 손으로 앞으로 돌리며 `ros2 topic echo /odom_raw --field twist.twist.linear.x` | 앞으로 돌리면 + |
| **노드가 죽을 때** | teleop으로 전진 중 다른 터미널에서 `pkill -9 -f rrc_adapter_node` (정지 명령 없이 강제 종료) | **0.5초 안에 바퀴 정지** (공장 펌웨어는 계속 돈다) |
| **USB가 빠질 때** | 전진 중 RRC USB 케이블을 뽑는다 | **0.5초 안에 바퀴 정지** |
| 정상 주행 | teleop 연속 주행 | 끊김 없음 (Pi는 25 Hz로 계속 명령을 보내므로 500 ms에 걸리지 않음) |

결과는 `firmware/rrc_lite_patch/README.md` 아래 기록 표에 남긴다.

## 주의
- Pi 말고 다른 프로그램(제조사 앱 등)으로 모터를 돌릴 때도 **0.5초 안에 명령을 반복**해야 한다.
- 이 패치는 소프트웨어 안전장치다. **물리 비상정지 스위치(전체 전원 차단)를 대신하지 않는다** (`docs/pin_map.md` 4장).

## 기록
| 날짜 | 내용 | 결과 |
|---|---|---|
| 2026-09-30 | 패치 작성, GCC 문법 검사 (132개 파일) | 오류 없음. Keil 빌드·실기 시험 전 |
| 2026-10-01 | 보류 결정 (공장 펌웨어 사용) | — |
