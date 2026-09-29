# RRC Lite 어댑터 구현 계획 (부품 도착 후)

`rrc_adapter_node`는 이미 만들어져 있고, **통신 부분(`RrcTransport`)만 비어 있다.**
/cmd_vel 처리, 속도 제한, watchdog, 휠 오도메트리 적분, /odom_raw·/imu/data 발행은
fake_rrc_node와 같은 코드(`rrc_node.py`, `diff_drive.py`)를 쓴다.

## 채워야 할 함수 (`ros2_ws/src/poli_hardware/poli_hardware/rrc_node.py`)
| 함수 | 입력/출력 | 비고 |
|---|---|---|
| `__init__(node)` | `port`, `baud` 파라미터로 시리얼 열기 | 포트는 `/dev/robot_rrc` |
| `set_wheel_speeds(left, right)` | 바퀴 각속도 rad/s | RRC 명령 단위(rps/rpm/PWM)로 변환. 전진이 + |
| `read_wheel_speeds()` | → (left, right) rad/s | 엔코더 기반. 전진이 + |
| `read_gyro_z()` | → rad/s 또는 None | 내장 IMU z축, ROS 축(위쪽 +, 반시계 +)으로 변환 |
| `close()` | | 정지 명령 후 포트 닫기 |

## 순서
1. **제조사 자료 확인**: Hiwonder RRC Lite의 시리얼 프로토콜/SDK(파이썬 예제)와 ROS 2 드라이버 유무·배포판.
   제조사 ROS 2 드라이버가 Jazzy에서 빌드되고 odom→base_link TF를 끌 수 있으면 그걸 쓰고,
   아니면 SDK를 `RrcTransport`에 감싼다.
2. **바퀴 띄운 상태**에서 `use_fake_hardware:=false`로 실행하고 `teleop_twist_keyboard`로
   전진/후진/좌회전/우회전/정지 확인. 방향이 반대면 코드에서 부호를 고친다 (배선을 바꾸지 말고 기록).
3. `/cmd_vel` 발행을 멈추고 0.3초 안에 바퀴가 서는지 확인 (watchdog).
4. `ros2 topic echo /odom_raw`로 엔코더 부호 확인: 전진 시 x 증가, 좌회전 시 yaw 증가.
5. 직선 1~2 m, 제자리 360° 시험으로 `wheel_radius`, `wheel_separation` 보정 → `config/hardware.yaml` +
   `docs/calibration.md`에 날짜와 함께 기록.
6. 내장 IMU 축 확인: 정지 시 gyro≈0, 좌회전 시 z > 0.

## 지켜야 할 것
- Topic 이름·타입·frame_id·parameter 이름을 바꾸지 않는다.
- RRC 쪽이 odom→base_link TF를 발행하면 끈다 (EKF 소유).
- 모터 정지 경로: watchdog(0.3 s) + 노드 종료 시 `set_wheel_speeds(0, 0)` + 물리 E-stop.
