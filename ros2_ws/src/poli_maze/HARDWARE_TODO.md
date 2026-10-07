# poli_maze 하드웨어 완성 후 손댈 곳

코드에서 `TODO_HW`(하드웨어에 따라 고칠 코드)와 `TODO_MEASURE`(로봇으로 재서 넣을 숫자)로 표시한 곳을 한곳에 모았어요.
위에서부터 순서대로 하면 돼요. 줄 번호는 이 패키지 처음 버전 기준이라, 고치다 보면 조금 밀릴 수 있어요. 그럴 땐 파일에서 `TODO_` 로 검색하세요.

```bash
grep -rn "TODO_HW\|TODO_MEASURE" poli_maze/
```

---

## 1. 조립 끝나자마자 확인 (코드가 돌아가려면 꼭 필요)

| 할 일 | 위치 |
|---|---|
| 조원 A 하드웨어 노드의 토픽 이름과 형식 확인. `/odom_raw` 가 `nav_msgs/Odometry` 가 아니면 `odom_callback` 수정 | `maze_runner.py:36` |
| 라이다 장착 위치(앞쪽 `LASER_X`, 왼쪽 `LASER_Y`, 돌아간 각도 `LASER_YAW`)를 재서 넣기. URDF 값과 같아야 함 | `maze_mapper.py:28` |
| 라이다를 뒤집어 달았으면 `LASER_FLIPPED = True` | `maze_mapper.py:33` |
| 라이다 0도가 로봇 정면이 아니면, 진행 방향 안전 거리 계산도 같이 확인 | `maze_runner.py:170` |
| 제자리 회전 가능한지: 바퀴 축 가운데 ~ 가장 먼 끝(집게, 들고 있는 물건 포함) 거리가 18 cm 이하인지 | `maze_runner.py:63` |
| 대회 규정의 목표 칸, 시작 칸, 시작 방향 (지금: 9x9, 목표 (4,4) = 사람 기준 (5,5), 시작 (8,8) = (9,9), 위쪽 출발) | `maze_logic.py:26` |
| 실제 칸 한 변 길이 `CELL_SIZE` (지금 0.40 m) | `maze_mapper.py:23` |

## 2. 로봇을 달려 보면서 재서 넣을 숫자

### 직진 / 후진 (`maze_runner.py`)

| 값 | 지금 | 재는 법 | 위치 |
|---|---|---|---|
| `MAX_SPEED` | 0.30 m/s | 칸 중앙에 정확히 설 수 있는 가장 빠른 속도 | :48 |
| `MAX_REVERSE_SPEED` | 0.20 m/s | 후진도 같은 방법으로 | :49 |
| `ACCEL` | 0.6 m/s² | 출발·정지 때 바퀴가 헛돌지 않는 최대값 | :50 |
| `MIN_SPEED` | 0.05 m/s | 이보다 느리면 모터가 안 도는 값 | :51 |
| `POS_TOL` | 0.02 m | 칸 중앙 도착으로 볼 오차 | :53 |
| `HEADING_KP` | 2.5 | 직진 중 좌우로 흔들리면 낮추고, 방향이 계속 틀어지면 높이기 | :54 |
| `CROSS_KP` | 2.0 | 칸 가운데 줄로 돌아오는 힘. 지그재그면 낮추기 | :55 |
| `SAFETY_STOP_DIST` | 0.10 m | 진행 방향 이 안에 물체가 있으면 정지 | :76 |

### 제자리 회전 (`maze_runner.py`)

| 값 | 지금 | 재는 법 | 위치 |
|---|---|---|---|
| `TURN_KP` | 3.0 | 90도 돌 때 지나치면 낮추고, 느리면 높이기 | :65 |
| `MAX_TURN_SPEED` | 1.5 rad/s | 미끄러지지 않는 최대 회전 속도 | :66 |
| `MIN_TURN_SPEED` | 0.3 rad/s | 이보다 느리면 회전이 멈추는 값 | :67 |
| `YAW_TOL` | 2° | 회전 완료로 볼 오차 | :68 |

### 라이다 벽 판단 (`maze_mapper.py`)

| 값 | 지금 | 재는 법 | 위치 |
|---|---|---|---|
| `MAX_RANGE` | 1.3 m | 이보다 먼 점은 버림. 먼 벽을 잘못 찍으면 줄이기 | :39 |
| `MIN_RANGE` | 0.05 m | 로봇 몸체나 집게에 맞는 점 거르기. 라이다 바로 옆 부품 거리보다 크게 | :40 |
| `HIT_MIN_PER_SCAN` | 3 | 빈칸이 벽으로 찍히면 올리고, 벽을 못 찾으면 내리기 | :42 |
| `CONFIRM_SCANS` | 3 | 위와 같음. 올리면 안전하지만 벽 판단이 늦어짐 | :43 |

## 3. 기능 추가가 필요한 곳 (하드웨어가 정해지면)

| 할 일 | 위치 |
|---|---|
| 대회 시작 신호(버튼 등)를 기다렸다가 출발하게 하기. 지금은 odom, scan 이 들어오고 1초 뒤 바로 출발 | `maze_runner.py:136` |
| 바퀴 odom 의 방향(yaw)이 많이 틀어지면 `/imu/data` 의 yaw 로 바꾸기 | `maze_runner.py:147` |
| `/gripper/holding` 센서가 생기면 잡았는지 확인하고, 못 잡았으면 다시 시도 | `maze_runner.py:205` |
| 회전 전에 라이다로 벽까지 거리를 재서 칸 중앙에 맞추기 (회전 여유 약 4 cm) | `maze_runner.py:276` |
| 물건이 칸 안 어디에 있는지에 따라 비전(`/vision/target`)으로 정렬 후 집기 | `maze_runner.py:378` |
| 규정상 시작점에서 물건을 내려놓아야 하면 `'open'` 발행 | `maze_runner.py:404` |

## 4. 아직 안 만든 것 (실제 주행에서 필요해지면)

- **위치 보정**: 지금은 바퀴 odom 만 믿어요. 칸을 여러 개 지나며 위치가 밀리면, 경기장 바깥 테두리나 옆 벽까지의 라이다 거리로 위치를 맞추는 코드를 추가해야 해요.
- **벽 모양 가정**: 벽이 칸 하나를 차지하는 물체라고 가정했어요(손그림의 세모). 실제 벽이 칸 사이 판자라면 맵핑과 알고리즘을 바꿔야 해요.

## 5. 확인 방법

```bash
# 알고리즘 테스트 (로봇 없이)
cd ~/polly/ros2_ws
colcon build --packages-select poli_maze
colcon test --packages-select poli_maze && colcon test-result --verbose

# 실행
source install/setup.bash
ros2 run poli_maze maze_runner

# 다른 터미널에서 상태 보기
ros2 topic echo /maze/status
ros2 topic echo /maze/map
```

처음 달릴 때는 `MAX_SPEED` 를 0.15 정도로 낮추고, 바퀴를 띄운 상태에서 방향이 맞게 도는지부터 확인하세요.
