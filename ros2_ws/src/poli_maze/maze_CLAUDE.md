# polly 저장소 지침 (Claude 용)

로봇 대회용 ROS 2 (Jazzy) 프로젝트. 9x9 미로 경기장(칸 400 mm)에서 목표 칸까지 가서 빨간 물건을 집고 시작점으로 돌아온다.
답변과 코드 주석, 커밋 메시지는 한국어로 쓴다.

## 팀과 담당

| 패키지 / 폴더 | 담당 | 내용 |
|---|---|---|
| `ros2_ws/src/poli_hardware`, `poli_description` | 조원 A (하드웨어/구동) | 모터, 집게, 센서 드라이버, URDF |
| `ros2_ws/src/poli_navigation` | 담당 1 (미션) | 미션 로직, `scan_to_grid.py` 등 |
| `ros2_ws/src/camera_vision` | 조원 B (비전) | 빨간 대상 감지 `red_object_detector` |
| `ros2_ws/src/poli_interfaces` | 조원 B | 전용 메시지 `TargetDetection` |
| `ros2_ws/src/poli_maze` | 조원 B | 미로 탐색 + 집기 + 복귀 `maze_runner` |
| `docs/` | 전체 | `interfaces.md`(토픽 약속), `mission_strategy.md`, `hardware_plan.md`, `pin_map.md`, `serial_protocol.md` |

- **내가 요청받은 패키지만 수정한다.** 다른 사람 패키지를 고쳐야 할 것 같으면 고치지 말고 무엇을 왜 바꿔야 하는지 먼저 알려준다.
- 패키지 안에 다른 패키지 복사본(같은 이름 폴더)을 만들지 않는다. colcon 이 "Duplicate package names" 로 실패한다.

## 토픽 약속 (자세한 건 docs/interfaces.md)

| 토픽 | 형식 | 보내는 쪽 → 받는 쪽 |
|---|---|---|
| `/vision/target` | `poli_interfaces/msg/TargetDetection` (`detected`, `x_offset`, `area`) | 비전 → 미션. 못 찾아도 `detected=false` 로 계속 보냄. `x_offset` 음수 = 화면 왼쪽 |
| `/cmd_vel` | `geometry_msgs/Twist` | 미션/미로 → 구동 |
| `/gripper/command` | `std_msgs/String` (`"open"`, `"grab"`) | 미션/미로 → 집게. `grab` 은 잡기만 하고 들어 올리지 않음 |
| `/gripper/holding` | (아직 없음) | 센서 추가 예정. 있다고 가정하지 않는다 |
| `/odom_raw`, `/imu/data`, `/range/*`, `/battery_state` | 하드웨어 노드 | 구동 → 미션 |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR 드라이버 → 미션 |

토픽 이름, 필드, 부호를 바꾸는 일은 `docs/interfaces.md` 를 먼저 고치고 팀 확인을 받은 뒤에 한다.

## 미로 알고리즘 규칙 (poli_maze, 바꾸면 안 되는 약속)

시뮬레이터와 같은 규칙이다. 다른 방식으로 "개선"하지 말고, 바꾸자는 의견이 있으면 이유와 함께 먼저 제안한다.

**갈 때**
1. 칸 숫자 = 목표까지 맨해튼 거리 + 1 (목표 = 1, 시작 칸 = 9). 경기장 9x9, 목표 (4,4) 정가운데, 시작 (8,8) 오른쪽 아래 구석, 위쪽을 보고 출발
   - **칸 번호는 코드에서 0 부터 센다.** 사람이 1 부터 세는 (5,5) 가 코드의 (4,4), (9,9) 가 (8,8). 사용자에게 칸을 말할 때는 둘 다 적는다
   - 경기장 크기와 목표/시작 칸은 `maze_logic.py` 맨 위 `N`, `GOAL`, `START` 한 곳에서만 바꾼다
2. 점수 = 칸 숫자 + 방문 횟수 x 10 (지날 때마다 누적)
3. 라이다로 찾은 벽 칸은 50 으로 표시하고 절대 가지 않는다
4. 점수가 가장 작은 칸으로 간다. 동점이면 직진 → 후진 → 오른쪽 → 왼쪽
5. 뒤쪽 칸은 돌지 않고 후진, 옆 칸은 제자리 90도 회전 후 전진
6. 달리면서 계속 맵핑. 다음 칸 중앙 '결정 지점'(감속 거리 + 5 cm 전)에서 방향을 정해, 직진이면 멈추지 않고 회전할 때만 칸 중앙에 멈춘다

**돌아올 때**
1. 목표에서 `"grab"` 발행 후 1.5초 대기 (`GRASP_WAIT_S`)
2. 올 때 지나간 칸에만, 시작점 1 부터 물결처럼(BFS) 새 숫자를 매긴다
3. 가장 작은 숫자로 간다 (동점 규칙 같음). 직선 구간은 한 번에 달린다

로직은 `poli_maze/maze_logic.py`(ROS 없음), 맵핑은 `maze_mapper.py`, 주행은 `maze_runner.py`. 알고리즘을 고치면 `test/test_maze_logic.py` 가 통과해야 한다.
- 처음 손으로 그린 7x7 경기장 테스트 (시뮬레이터와 비교 기준, 크기를 따로 지정): 갈 때 12칸, 복귀 숫자표 목표 = 13, 복귀 첫 칸 후진
- 9x9 대회 경기장 테스트: 설정값, 벽 없을 때 최단 8칸, 벽이 있을 때 목표 도착과 복귀

## 코드 규칙

- 파이썬, ament_python. 모듈/함수/클래스 docstring 은 한국어 한 줄 이상
- 조정할 숫자는 파일 위쪽 대문자 상수로 모은다. 코드 중간에 숫자를 박지 않는다
- 표시 규칙
  - `# TODO_MEASURE:` 로봇으로 재서 넣어야 하는 숫자
  - `# TODO_HW:` 하드웨어가 정해진 뒤 고쳐야 하는 코드
  - 새로 추가하면 `poli_maze/HARDWARE_TODO.md` 목록에도 추가한다
- 노드는 `main()` 이 있고 `setup.py` 의 `console_scripts` 에 등록해 `ros2 run` 으로 실행되게 한다
- 노드 종료 시 `/cmd_vel` 0 을 보내 로봇을 세운다
- 대회에는 모니터가 없다. 화면 창(`cv2.imshow` 등)은 파라미터로 켤 때만 띄우고 기본값은 꺼짐
- 로그는 반복되는 것에 `throttle_duration_sec` 를 건다

## 빌드와 테스트

```bash
cd ~/polly/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select <패키지>
colcon test --packages-select <패키지> && colcon test-result --verbose
source install/setup.bash
```

- 이 PC 의 WSL 은 Ubuntu 26.04 라 Jazzy 가 제대로 돌지 않는다 (Python 3.14). 여기서 빌드가 깨져도 시스템 Python 을 바꾸지 않는다 (`/usr/bin/python3` 링크 수정 금지).
- ROS 없이 확인할 수 있는 순수 로직(예: `maze_logic.py`)은 `python3` 로 직접 테스트한다. 실제 빌드와 실행 확인은 라즈베리파이(또는 Ubuntu 24.04)에서 한다.
- `build/`, `install/`, `log/` 는 커밋하지 않는다.

## Git 규칙

- `main` 에 직접 커밋/푸시하지 않는다. 기능 브랜치(`feature/vision`, `feature/maze` 등)에서 작업하고 GitHub PR 로 합친다
- 작업 시작 전 `git pull`, 커밋 후 바로 `git push`. 푸시 뒤 `git log --oneline origin/<브랜치> -3` 으로 올라갔는지 확인
- **커밋 전에 `git status` 와 바뀐 내용 요약을 보여주고 확인을 받는다.** 다른 사람 패키지 파일이 섞여 있으면 멈추고 알린다
- `git reset --hard`, `git push --force`, 파일 삭제처럼 되돌리기 어려운 명령은 실행 전에 반드시 물어본다
- 커밋 메시지는 한국어로 무엇을 바꿨는지 한 줄 (예: `camera_vision 실행 명령어 주석 수정`)
- 작업 폴더는 WSL 의 `~/polly` 하나만 쓴다. 바탕화면(OneDrive)의 `polly` 복사본은 쓰지 않는다

## 설명 방식

- 사용자는 git 과 ROS 를 배우는 중이다. 명령어를 줄 때는 무엇을 하는 명령인지 짧게 같이 쓴다
- 여러 단계 작업은 순서대로 번호를 붙이고, 각 단계 결과를 확인할 방법을 같이 준다
- 에러가 나면 원인을 먼저 짚고, 확인 명령 → 해결 명령 순서로 안내한다
