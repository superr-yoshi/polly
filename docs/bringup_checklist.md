# 부품 도착 후 실행 순서 (담당 1: LiDAR · 통합 · 임무)

부품이 오면 위에서부터 순서대로 한다. 앞 단계가 안 되면 다음으로 넘어가지 않는다.
조원 A 주행 보정은 `docs/calibration.md`, udev는 `docs/udev_template.md`.

## 0. Pi 준비 (Ubuntu 24.04 + ROS 2 Jazzy)
```bash
git clone https://github.com/superr-yoshi/polly.git ~/polly
cd ~/polly/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
sudo usermod -aG dialout $USER   # 시리얼 권한. 다시 로그인해야 적용
```
- [ ] 빌드 성공 (`camera_vision`은 테스트가 없어 `colcon test`에서 실패로 나오지만 정상)

## 1. LiDAR 드라이버 (sllidar_ros2, RPLIDAR C1)
apt 패키지가 없어서 소스로 빌드한다. 우리 저장소와 섞이지 않게 작업 폴더를 따로 둔다.
```bash
mkdir -p ~/lidar_ws/src && cd ~/lidar_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/lidar_ws && colcon build --symlink-install
echo "source ~/lidar_ws/install/setup.bash" >> ~/.bashrc
```
- [ ] LiDAR만 켜기: `ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/ttyUSB0`
- [ ] `ros2 topic hz /scan` 약 10Hz → `mission1_node.SCAN_TIMEOUT`, `wall_localizer.MAX_ANGULAR_SPEED_FOR_SCAN` 확인
- [ ] udev 별칭 `/dev/robot_lidar` 만들기 (`docs/udev_template.md`). 안 했으면 launch에 `lidar_port:=/dev/ttyUSB0`

## 2. LiDAR 장착값 (URDF 한 곳에서만 고친다)
`ros2_ws/src/poli_description/urdf/poli.urdf.xacro`의 `lidar_x`, `lidar_y`, `lidar_yaw`.
임무 노드가 TF(`base_link` → `laser`)로 읽는다. 코드(`scan_to_grid.py`)는 고치지 않는다.
- [ ] `lidar_x`, `lidar_y`: 바퀴 축 가운데(base_link)에서 LiDAR 중심까지 (m, 앞 +x, 왼쪽 +y)
- [ ] `lidar_yaw`: 로봇 정면에 물체를 두고 `/scan`에서 그 물체가 0도에 보이는지. 뒤(180도)에 보이면 `lidar_yaw = pi`
- [ ] 임무 노드 로그 `LiDAR mount from TF: x=... y=... yaw=...`가 URDF 값과 같은지

## 3. 통합 실행 (처음엔 바퀴를 띄우고)
**mission2가 켜지자마자 `/cmd_vel`을 보낸다. 첫 실행은 바퀴를 바닥에서 띄우고, E-stop을 손 닿는 곳에 둔다.**
```bash
ros2 launch poli_navigation mission2.launch.py use_fake_hardware:=false
```
- [ ] 로그에 `LiDAR mount from TF` 나옴
- [ ] `ros2 topic hz /scan /odom_raw /vision/target` 모두 들어옴
- [ ] 경기장(또는 벽 4면)에 놓으면 `Wall correction active`
- [ ] Ctrl+C로 끄면 바퀴가 멈춤

## 4. 실측해서 고칠 값 (담당 1)
| 값 | 파일 | 지금 | 재는 방법 |
|---|---|---|---|
| `GRAB_AREA` | `mission2_logic.py`, `mission1_logic.py` | 15000 (임시) | 집게로 잡을 수 있는 거리에 대상을 두고 `/vision/target`의 `area` (640×480 기준, 조원 B와) |
| `GRIPPER_REACH_M` | `mission1_logic.py` | 0.25 | 로봇 중심 ~ 잡은 큐브 중심 거리 (줄자) |
| `GRASP_WAIT_S` | `mission2_logic.py`, `mission1_logic.py` | 1.5 | `/gripper/state`가 `closed`가 될 때까지 걸린 시간 + 여유 |
| `DANGER_MARGIN_M` | `mission2_logic.py` | 0.15 | 로봇 반지름 정도 |
| `WALL_TOLERANCE_M` | `wall_localizer.py` | 0.03 | 벽 앞에 세워 두고 `/scan` 거리 값이 흔들리는 폭 |
| `MAX_ANGULAR_SPEED_FOR_SCAN` | `wall_localizer.py` | 0.3 rad/s | 제자리 회전 중 `Wall correction` 위치가 튀는 속도 |
| `CORE_MARGIN_M` | `scan_to_grid.py` | 0.1 | 임무 1 격자 지도가 실제와 맞는지 보고 조정 |
| 속도·게인 (`SIM_ONLY`) | `mission2_logic.py`, `mission1_logic.py` | 시뮬레이션 값 | 실제 주행을 보고 조정 |

## 5. 다른 사람에게 확인할 것
- [ ] 조원 B: Ubuntu 24.04에서 `python3 -c "import picamera2"` 되는지 (Raspberry Pi OS용 라이브러리라 안 될 수 있다)
- [ ] 조원 B: 카메라 좌우 반전 (`CAMERA_HFLIP`) — 왼쪽 물체가 `x_offset` 음수인지
- [ ] 조원 A: 같은 집게 명령이 1초마다 와도 Mega 쪽 문제없는지

## 기록
| 날짜 | 항목 | 값 | 메모 |
|---|---|---|---|
| | | | |
