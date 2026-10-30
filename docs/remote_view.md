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
| **앞으로 가기** (바닥 주행, 조립 후) | `bash scripts/laptop/forward_test.sh` | Enter 후 RViz 창 닫기 (Pi 쪽 모두 꺼짐). 아래 "앞으로 가기 시험" |
| LiDAR + 카메라 | `bash scripts/laptop/view_all.sh` | RViz 창 닫기 (Pi 쪽도 꺼짐) |
| LiDAR | `bash scripts/laptop/view_lidar.sh` | RViz 창 닫기 (Pi LiDAR도 꺼짐) |
| 카메라 영상 | `bash scripts/laptop/view_camera.sh` | 영상 창 닫기 (Pi 영상 발행도 꺼짐) |
| 빨간 물체 인식 값 | `bash scripts/laptop/watch_target.sh` | Ctrl+C |

Pi IP는 자동으로 찾는다. 못 찾으면 뒤에 IP를 붙인다: `bash scripts/laptop/view_lidar.sh 172.25.114.24`.
카메라 스크립트는 Pi 저장소에 `scripts/camera_stream_test.py`가 있어야 한다 (Pi에서 `git pull`).

## 앞으로 가기 시험 (`forward_test.sh`)
RViz로 LiDAR·카메라를 보면서 로봇을 바닥에서 **0.25 m/s로 2초(약 0.5 m)** 전진시키고,
앞·뒤 벽까지 거리가 바뀐 만큼을 LiDAR로 재서 실제로 앞으로 갔는지 알려 준다 (Pi에서 `scripts/forward_test.py`).
y를 누를 때마다 한 번씩 달리므로, 로봇을 다시 놓고 여러 번 할 수 있다.

- 준비: 로봇을 바닥에 두고 **앞쪽 1 m 이상 비우기**, **1~3 m 앞에 벽이나 상자** (거리 변화를 잰다),
  비상정지 스위치와 Mega USB를 바로 누르거나 뽑을 수 있게.
- 저절로 멈추는 경우: 출발 전 앞쪽(정면 ±30도)이 1 m 안으로 막혀 있으면 출발 안 함,
  달리는 중 앞쪽 0.35 m 안에 뭐가 보이거나 LiDAR가 0.5초 넘게 끊기면 정지,
  Ctrl+C·SSH 끊김이면 정지 명령을 보내고 끝남 (그래도 안 오면 `mega_bridge_node`와 Mega 펌웨어가 0.3초 뒤 정지).
- LiDAR 정면은 URDF(`poli_description`의 `lidar_yaw`)에서 TF로 읽는다. LiDAR에서 0.15 m 안의 값은
  로봇 몸체로 보고 무시한다 (`SELF_HIT_M`, 조립 후 실측 필요).

| 결과 | 뜻 / 할 일 |
|---|---|
| ✅ 앞으로 갔어요 (명령의 80~120 %) | 정상. 20 % 넘게 다르면 바퀴 미끄러짐·`wheel_radius` 보정 (조원 A) |
| ❌ LiDAR로 보면 뒤로 갔어요 | 눈으로도 뒤로 갔다 → 바퀴 방향(`left_sign`·`right_sign`, 조원 A). 눈으로는 앞으로 갔다 → URDF `lidar_yaw` 180도 |
| ❌ 옆으로 움직인 것처럼 보여요 | 눈으로는 앞으로 갔다 → URDF `lidar_yaw` 90도 |
| ❌ 거의 안 움직였어요 | 배터리·비상정지 스위치, MDD10 전원, 모터 선. 바퀴 노드 로그 `/tmp/forward_drive.log` (Pi) |
| ⚠️ ○도 휘었어요 (IMU) | 두 바퀴 속도 차이 또는 바퀴 방향 설정 |

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
