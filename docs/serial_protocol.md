# Mega ↔ Pi 시리얼 프로토콜 (v1.1, 조원 A 범위)

팀 매뉴얼(21·21A장)의 v1 규칙을 기반으로 한다. **v1과 달라진 점은 맨 아래 "변경 사항" 참고.**
이 문서를 바꾸면 Pi 쪽 담당(통합)에게 반드시 알린다.

## 공통 규칙
| 항목 | 규칙 |
|---|---|
| 전송 | 115200 baud, 8N1, 한 packet = 한 줄, 줄 끝 `\r\n` |
| 형식 | `$본문*CS` |
| CS | `$`와 `*` 사이 모든 문자의 XOR, 대문자 16진수 2자리 |
| 값 | 전부 정수 (AVR snprintf는 `%f` 미지원) |

## Mega → Pi
| 이름 | 형식 | 주기 | 설명 |
|---|---|---|---|
| RNG | `$RNG,seq,millis,front,left,right,rear*CS` | 8 Hz | 초음파 4개 거리(mm). 0 = 측정 실패 또는 범위 밖 |
| GST | `$GST,seq,millis,last_id,state,angle*CS` | 2 Hz + 명령 직후 | 집게 상태. state 0=열림, 1=닫힘, 2=이동 중. angle = 서보 각도(도). last_id = 마지막으로 처리한 GRIP 명령 id |

- 초음파 순서: 전방(D22/23), 좌측(D24/25), 우측(D26/27), 후방(D28/29). 2026-09-30 이름 변경 (예전 fl/fr/rl/rr, 형식은 같음)
- seq는 packet 종류별로 따로 증가 (누락 검출용), millis는 Mega 시각

## Pi → Mega
| 이름 | 형식 | 설명 |
|---|---|---|
| GRIP | `$GRIP,id,action*CS` | action 0 = 열기, 1 = 닫기. id는 Pi가 붙이는 번호 |

- Mega는 명령을 처리하면 **즉시 GST를 한 번 보내** 응답한다. GST의 last_id가 보낸 id와 같아지면 명령이 수신·처리된 것이다.
- 체크섬 오류, 필드 오류, 모르는 명령은 조용히 폐기한다 (응답 없음).
- 집게는 목표 각도까지 1도씩 서서히 움직인다. 움직이는 동안 state = 2.

## 예시 (체크섬 검증 완료)
```
$RNG,5,640,1000,1500,2000,800*63
$GST,3,1500,1,1,120*68
$GRIP,1,1*0C        ← 닫기, id 1
$GRIP,2,0*0E        ← 열기, id 2
```

## v1(팀 매뉴얼) 대비 변경 사항
1. `$IMU` 삭제: IMU는 RRC Lite에 달려 있어 Mega가 보내지 않는다.
2. `$BAT` 삭제: 전압 측정 회로가 부품표에 없고 Mega 담당 범위가 아니다. 필요해지면 다시 추가한다.
3. `$GST`(집게 상태)와 `$GRIP`(집게 명령)을 새로 추가.
4. 모터·엔코더는 RRC Lite가 처리하므로 이 프로토콜에 없다.

## Pi 쪽 구현 (poli_hardware 패키지)
- 파서: `ros2_ws/src/poli_hardware/poli_hardware/mega_protocol.py` (`FIELD_COUNT = {RNG: 6, GST: 5}`, IMU/BAT는 폐기)
- 노드: `mega_bridge_node` — 포트를 열고 2초 대기 후 수신 (Mega 자동 리셋)
- ROS 인터페이스:

| 이름 | 종류 | 타입 | 내용 |
|---|---|---|---|
| `/range/front`, `/range/left`, `/range/right`, `/range/rear` | Topic | sensor_msgs/Range | RNG의 4개 값 (m). frame `ultrasonic_<이름>_link`. 0(실패)은 `+inf` (REP-117 "감지 없음") |
| `/gripper/state` | Topic | std_msgs/String | GST state: `open` / `closed` / `moving` |
| `/gripper/command` | Topic (구독) | std_msgs/String | `"open"` = 열기(GRIP 0), `"grab"` = 닫아 잡기(GRIP 1). 들어 올리기 없음. GST의 last_id로 수신 확인, 0.5초마다 최대 3번 전송 |

- `/battery_state`는 Mega가 아니라 RRC Lite 노드가 발행한다 (RRC 입력 전압). Mega 프로토콜에 `$BAT`는 없다.
- 주의: 펌웨어는 20 mm 미만(너무 가까움)도 0으로 보내므로 `+inf`에 섞인다. 근접 정지는 LiDAR와 함께 판단한다.
