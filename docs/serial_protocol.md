# Mega ↔ Pi 시리얼 프로토콜 (v2.0, 조원 A 범위)

팀 매뉴얼(21·21A장)의 v1 규칙을 기반으로 한다. **v1·v1.1과 달라진 점은 맨 아래 "변경 사항" 참고.**

> **2026-10-29 v2.0: RRC Lite를 쓰지 않고 Mega가 주행까지 맡는다** (모터 드라이버 Cytron MDD10, 핀은 `docs/pin_map.md`).
> 주행(`$DRV`·`$PWM`), 엔코더(`$ENC`), IMU(`$IMU`)를 추가했고, 집게(`$GRIP`·`$GST`)는 다시 Mega가 처리한다.
이 문서를 바꾸면 펌웨어(`firmware/mega_sensor_controller/`), `poli_hardware/mega_protocol.py`를 **함께** 고치고 Pi 쪽 담당(통합)에게 반드시 알린다.

## 공통 규칙
| 항목 | 규칙 |
|---|---|
| 전송 | 115200 baud, 8N1, 한 packet = 한 줄, 줄 끝 `\r\n` |
| 형식 | `$본문*CS` |
| CS | `$`와 `*` 사이 모든 문자의 XOR, 대문자 16진수 2자리 |
| 값 | 전부 정수 (AVR snprintf는 `%f` 미지원). `ENC`·`IMU`·`DRV`·`PWM`의 값은 음수 가능, seq·millis와 나머지 packet은 0 이상 |

## Mega → Pi
| 이름 | 형식 | 주기 | 설명 |
|---|---|---|---|
| ENC | `$ENC,seq,millis,left,right,status*CS` | 50 Hz | 엔코더 **누적** tick (4체배, 바퀴 1바퀴 약 1320). 로봇 전진 = + (펌웨어 `ENC_*_INVERT` 적용). status는 아래 표 |
| IMU | `$IMU,seq,millis,qx,qy,qz,qw,gx,gy,gz,ax,ay,az*CS` | 25 Hz | BNO085. q = 쿼터니언 ×10000 (Game Rotation Vector, 자기장 안 씀, 켤 때 yaw 0), g = 각속도 mrad/s, a = 가속도 mm/s² (중력 포함). **BNO085가 없으면(`SIM_IMU 1`·못 찾음) 보내지 않는다** |
| RNG | `$RNG,seq,millis,front,left,right,rear*CS` | 8 Hz | 초음파 4개 거리(mm). 0 = 측정 실패 또는 범위 밖 |
| GST | `$GST,seq,millis,last_id,state,angle*CS` | 2 Hz + 명령 직후 | 집게 상태. state 0=열림, 1=닫힘, 2=이동 중. angle = 서보 각도(도). last_id = 마지막으로 처리한 GRIP 명령 id |

ENC status:
| 값 | 이름 | 뜻 |
|---|---|---|
| 0 | idle | 정지. 첫 명령 전이거나 `$DRV`/`$PWM`이 0.3 s 넘게 끊김 |
| 1 | drive | `$DRV` 속도 제어 중 |
| 2 | pwm | `$PWM` 직접 출력 중 (배선 확인용) |
| 3 | fault_reverse | 목표와 반대 방향으로 돎 → 모터 끔. **Mega 리셋 전까지 명령 무시** |
| 4 | fault_stall | 큰 duty인데 거의 안 돎 (엔코더 신호 없음·바퀴 막힘) → 모터 끔. **Mega 리셋 전까지 명령 무시** |

- 초음파 순서: 전방(D22/23), 좌측(D24/25), 우측(D26/27), 후방(D28/29). 2026-09-30 이름 변경 (예전 fl/fr/rl/rr, 형식은 같음)
- seq는 packet 종류별로 따로 증가 (누락 검출용), millis는 Mega 시각
- ENC는 누적값이라 packet이 빠져도 이동 거리는 잃지 않는다. Pi는 두 ENC 사이 tick 차이를 millis 차이로 나눠 속도를 구한다.
- Mega가 리셋되면 tick·seq·millis가 0부터 다시 시작한다 (Pi는 millis가 줄면 리셋으로 본다).

## Pi → Mega
| 이름 | 형식 | 설명 |
|---|---|---|
| DRV | `$DRV,left,right*CS` | 목표 바퀴 속도 (엔코더 tick/s, 전진 = +). Pi가 25 Hz로 계속 보낸다 (정지 중에도 `$DRV,0,0`) |
| PWM | `$PWM,left,right*CS` | MDD10 duty 직접 출력 (-255~255). 엔코더를 보지 않는다. **바퀴를 띄우고 배선·방향 확인할 때만** |
| GRIP | `$GRIP,id,action*CS` | action 0 = 열기, 1 = 닫기. id는 Pi가 붙이는 번호 |

- **`$DRV`/`$PWM`이 0.3 s(`CMD_TIMEOUT_MS`) 넘게 안 오면 Mega가 스스로 모터를 멈춘다.** 노드가 죽거나 USB가 빠져도 바퀴가 계속 돌지 않는다.
- 첫 `$DRV`/`$PWM` 전에는 모터를 돌리지 않는다. 목표 0이면 바로 duty 0 (속도 제어를 거치지 않음).
- 속도 제어는 Mega 안에서 바퀴별 PI + 앞먹임, 50 Hz. 이득·폭주 감지 기준은 `config.h` (SIM_ONLY, 실기 보정 필요).
- Mega는 GRIP 명령을 처리하면 **즉시 GST를 한 번 보내** 응답한다. GST의 last_id가 보낸 id와 같아지면 명령이 수신·처리된 것이다.
- 체크섬 오류, 필드 오류, 모르는 명령은 조용히 폐기한다 (응답 없음).
- 집게는 목표 각도까지 1도씩 서서히 움직인다. 움직이는 동안 state = 2. 첫 GRIP 전에는 서보 신호를 내지 않는다 (규정 3.5.6).

## 예시 (체크섬 검증 완료, `test_mega_protocol.py`)
```
$ENC,10,200,1320,-660,1*7B
$IMU,4,160,0,0,0,10000,0,0,12,0,0,9806*57
$RNG,5,640,1000,1500,2000,800*63
$GST,3,1500,1,1,120*68
$DRV,1616,1616*40   ← 두 바퀴 0.25 m/s (반지름 32.5 mm, 1320 tick/rev)
$DRV,0,0*40         ← 정지
$PWM,80,-80*67      ← 직접 출력 (배선 확인용)
$GRIP,1,1*0C        ← 닫기, id 1
$GRIP,2,0*0E        ← 열기, id 2
```

## 대역폭
115200 baud ≈ 11.5 kB/s. ENC 50 Hz(약 2.1 kB/s) + IMU 25 Hz(약 2.1 kB/s) + RNG·GST(약 0.4 kB/s) → 약 40%.
IMU를 더 빠르게 하거나 packet을 늘리면 다시 계산한다.

## v1(팀 매뉴얼)·v1.1 대비 변경 사항
1. v1.1: `$IMU` 삭제(당시 IMU는 RRC Lite), `$BAT` 삭제(전압 측정 회로 없음), `$GST`·`$GRIP` 추가.
2. v1.1 (2026-10-07): 집게를 RRC Lite PWM 서보 포트로 옮겨 `$GRIP`·`$GST`를 쓰지 않음.
3. **v2.0 (2026-10-29)**: RRC Lite 사용 중단 → `$DRV`·`$PWM`·`$ENC` 추가, `$IMU`를 BNO085용 새 형식(필드 12개)으로 다시 추가,
   집게를 Mega D9로 되돌려 `$GRIP`·`$GST` 다시 사용. `$BAT`는 여전히 없음.

## Pi 쪽 구현 (poli_hardware 패키지)
- 파서: `ros2_ws/src/poli_hardware/poli_hardware/mega_protocol.py` (`FIELD_COUNT = {RNG: 6, GST: 5, ENC: 5, IMU: 12}`)
- 노드: `mega_bridge_node` 하나가 이 포트를 연다 — 포트를 열고 2초 대기 후 수신 (Mega 자동 리셋)
- ROS 인터페이스:

| 이름 | 종류 | 타입 | 내용 |
|---|---|---|---|
| `/cmd_vel` | Topic (구독) | geometry_msgs/Twist | → `$DRV` 25 Hz. 0.3 s 끊기면 0 |
| `/odom_raw` | Topic | nav_msgs/Odometry | `$ENC` 기반 (엔코더) |
| `/imu/data` | Topic | sensor_msgs/Imu | `$IMU` 기반. 없으면 휠 각속도로 z축만 |
| `/range/front`, `/range/left`, `/range/right`, `/range/rear` | Topic | sensor_msgs/Range | RNG의 4개 값 (m). frame `ultrasonic_<이름>_link`. 0(실패)은 `+inf` (REP-117 "감지 없음") |
| `/gripper/command` | Topic (구독) | std_msgs/String | `"open"` = 열기(GRIP 0), `"grab"` = 닫아 잡기(GRIP 1). 들어 올리기 없음. GST의 last_id로 수신 확인, 0.5초마다 최대 3번 전송 |
| `/gripper/state` | Topic | std_msgs/String | GST state → `open` / `closed` / `moving` |

- `/battery_state`는 없다 (2026-10-29, RRC Lite를 빼서 전압 측정원이 없음).
- 주의: 펌웨어는 20 mm 미만(너무 가까움)도 0으로 보내므로 `+inf`에 섞인다. 근접 정지는 LiDAR와 함께 판단한다.
