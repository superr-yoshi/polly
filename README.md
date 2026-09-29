# polly

자율 구조 로봇 POLI (Raspberry Pi 5 · RRC Lite · Arduino Mega 2560 · ROS 2 Jazzy)

## 구조
| 경로 | 내용 | 담당 |
|---|---|---|
| `firmware/mega_sensor_controller/` | Mega 펌웨어: 초음파 ×4, 집게 서보, Pi 시리얼 | A |
| `ros2_ws/src/poli_hardware/` | RRC 주행 어댑터, Mega 브리지, fake 노드, launch | A |
| `ros2_ws/src/poli_navigation/` | LiDAR·SLAM·Nav2 | 통합 |
| `scripts/pc_serial_tester.py` | PC에서 Mega 시리얼 확인 도구 | A |
| `docs/` | 프로토콜, 핀맵, 하드웨어 계획, udev, RRC 어댑터 계획 | |
| `firmware/nucleo_controller/` | (구) NUCLEO-F446RE 계획. Mega로 대체되어 사용하지 않음 | |

## 빠른 확인
```bash
# 단위 테스트 (ROS 없이)
python -m pytest ros2_ws/src/poli_hardware/test -q
# Mega 펌웨어 컴파일 (보드 없이)
arduino-cli compile --fqbn arduino:avr:mega firmware/mega_sensor_controller
# ROS 2 (Ubuntu 24.04 + Jazzy)
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch poli_hardware hardware.launch.py use_fake_hardware:=true
```
