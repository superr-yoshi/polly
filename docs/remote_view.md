# 노트북에서 LiDAR·카메라 보기 (개발용)

Pi에 모니터가 없어도 노트북(WSL)의 RViz2 / rqt_image_view로 Pi의 ROS 토픽을 본다. 2026-10-28 확인.

> ⚠️ **개발용이다.** 임무가 시작되면 Wi-Fi를 끈다 (규정 1.3, `scripts/comm_off.sh`). 경기 중 확인은 `~/poli_logs/` 로그와 rosbag으로 한다.

## 준비 (노트북, 1회)
1. 노트북 WSL에 ROS 2 Jazzy (`rviz2`, `rqt_image_view` 포함)
2. `C:\Users\<이름>\.wslconfig`
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
3. 관리자 PowerShell (WSL로 들어오는 연결 허용 → Pi의 ROS 데이터가 WSL까지 온다)
   ```powershell
   Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
   wsl --shutdown
   ```
4. 노트북과 Pi를 **같은 Wi-Fi**에 연결한다. 학교 Wi-Fi(CWNUWIFI-5G)에서는 노트북 → Pi 접속이 안 됐고, 휴대폰 핫스팟에서는 됐다.
   Pi IP는 Windows에서 `ping poli-desktop.local`로 찾는다 (WSL에서는 `.local` 이름이 안 풀린다).

양쪽 모두 `ROS_DOMAIN_ID=0` (Pi `~/.bashrc`), `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`.
처음에 토픽이 안 보이면 `ros2 daemon stop` 후 `ros2 topic list --no-daemon --spin-time 8`.

## 한 줄로 보기 (노트북 VS Code 터미널, 저장소 폴더에서)
| 확인 | 명령 | 끝내기 |
|---|---|---|
| **LiDAR + 카메라 + 바퀴 한 번에** | `bash scripts/laptop/check_all.sh` | RViz 창 닫기 (Pi 쪽 모두 꺼짐). 바퀴는 띄우고, 비상정지(또는 Mega USB)를 바로 누를 수 있게 |
| LiDAR + 카메라 | `bash scripts/laptop/view_all.sh` | RViz 창 닫기 (Pi 쪽도 꺼짐) |
| LiDAR | `bash scripts/laptop/view_lidar.sh` | RViz 창 닫기 (Pi LiDAR도 꺼짐) |
| 카메라 영상 | `bash scripts/laptop/view_camera.sh` | 영상 창 닫기 (Pi 영상 발행도 꺼짐) |
| 빨간 물체 인식 값 | `bash scripts/laptop/watch_target.sh` | Ctrl+C |

Pi IP는 자동으로 찾는다. 못 찾으면 뒤에 IP를 붙인다: `bash scripts/laptop/view_lidar.sh 172.25.114.24`.
카메라 스크립트는 Pi 저장소에 `scripts/camera_stream_test.py`가 있어야 한다 (Pi에서 `git pull`).

## LiDAR
- Pi: LiDAR 켜기 (임무 launch를 켜면 같이 켜진다)
  ```bash
  ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/robot_lidar
  ```
- 노트북: `rviz2 -f laser` → Add → By topic → `/scan` → LaserScan.
  sllidar_ros2의 `rviz/sllidar_ros2.rviz` 설정 파일을 쓰면 바로 보인다.
- `Global Status: No tf data` 경고는 LiDAR만 켰을 때 정상 (URDF·TF가 없음).

## 카메라
`camera_vision`(조원 B)은 인식 결과 `/vision/target`만 발행하고 영상은 발행하지 않는다.
- 인식 결과만: 노트북에서 `ros2 topic echo /vision/target`
- 영상 시험: Pi에서 `python3 ~/polly/scripts/camera_stream_test.py` (320×240, 5 Hz) → 노트북에서
  `ros2 run rqt_image_view rqt_image_view /camera/image_raw`
  - 카메라는 한 프로그램만 쓸 수 있어 `red_object_detector`와 동시에 못 켠다.
  - 임무 중 영상을 보려면 조원 B 노드에 영상 발행(디버그 파라미터)을 넣어야 한다 → 조원 B와 상의.

## Pi 모니터에서 직접 볼 때
- Pi 모니터 터미널(**SSH 창이 아닌** 로컬 터미널)에서 `rviz2` / `ros2 run camera_vision red_object_detector --ros-args -p show_debug:=true`.
- SSH로 접속한 창에서 실행하면 화면을 못 찾아 RViz가 죽는다 (Qt `xcb` 오류).
