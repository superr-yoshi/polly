# polly

자율 구조 로봇 POLI (Raspberry Pi 5 · Arduino Mega 2560 · Cytron MDD10 · ROS 2 Jazzy)

> 2026-10-29: RRC Lite를 쓰지 않는다. 모터 드라이버 MDD10, 속도 제어·엔코더·IMU(MPU-6050)·집게는 Mega가 맡는다 (`docs/pin_map.md`).

> AI 코딩 도구(Claude Code, Codex)로 작업할 때는 `AGENTS.md`(= `CLAUDE.md`)를 먼저 읽는다.
> 부품 사양: `docs/hardware_reference.md` · 팀 토픽 약속: `docs/interfaces.md` · 부품 도착 후 순서: `docs/bringup_checklist.md`

## 구조
| 경로 | 내용 | 담당 |
|---|---|---|
| `firmware/mega_sensor_controller/` | Mega 펌웨어: MDD10 모터 속도 제어, 엔코더 ×2, MPU-6050, 초음파 ×4, 집게 서보, Pi 시리얼 | A |
| `ros2_ws/src/poli_hardware/` | Mega 브리지(주행·센서·집게), fake 노드, launch, 보정 도구 | A |
| `ros2_ws/src/poli_navigation/` | 임무(mission2), 격자 지도, LiDAR 처리 | 담당 1 |
| `scripts/pc_serial_tester.py` | PC에서 Mega 시리얼 확인 도구 | A |
| `docs/` | 토픽 약속, 부품 사양, Mega 프로토콜(옛 RRC 기록 포함), 핀맵, 보정·udev·WSL 안내 | |
| `docs/product-spec-claude/` | 원본 제품 사양서 (HWP 변환본 + 이미지 23개, 참고 자료) | |
| `firmware/nucleo_controller/` | (구) NUCLEO-F446RE 계획. Mega로 대체되어 사용하지 않음 | |

## 빠른 확인
```bash
# Python 단위 테스트 (ROS 없이도 됨, 패키지별로 실행)
(cd ros2_ws/src/poli_hardware && python -m pytest test -q)
(cd ros2_ws/src/poli_navigation && python -m pytest test/test_grid_map.py test/test_mission2_logic.py test/test_scan_to_grid.py -q)
# Mega 펌웨어 컴파일 (보드 없이)
arduino-cli compile --fqbn arduino:avr:mega firmware/mega_sensor_controller
# ROS 2 (Ubuntu 24.04 + Jazzy)
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch poli_hardware hardware.launch.py use_fake_hardware:=true
```
