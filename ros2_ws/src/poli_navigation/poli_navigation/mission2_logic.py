"""
임무 2 (배틀럼블) 판단 로직.

ROS2와 무관한 순수 Python 모듈이다.
센서 값(Observation)을 받아 지금 할 동작(Command)을 결정한다.
토픽 연결은 별도 ROS2 노드에서 한다.

전략 (docs/mission_strategy.md 3장, 2026-10-31 임무 1 개선을 옮김):
  DRIVE(출발하자마자 가속하며 중앙 쪽으로 곡선) -> 카메라에 보이면 바로 APPROACH
  -> PUSH(밀어 넣으며 닫기) -> GRASP -> GUARD(180도 돌아 중앙에서 떨어진 원 위로 빠져서 지키기)
  못 잡았거나 놓치면(/gripper/holding false) LURK: GUARD처럼 원 위로 빠져 상대를 피하고,
  빨간 대상이 보이면 집게를 열고 다시 잡으러 간다 (LURK_AFTER_FAIL, 2026-11-02 팀)
  다시 잡으러 가는 중(ALIGN·APPROACH)에도 상대가 DODGE_START_M 안으로 오면 LURK로 돌아가 피한다 (2026-11-03)
  (LURK_AFTER_FAIL False면 SPIN: 끝까지 제자리 회전만 (SPIN_AFTER_FAIL, 2026-11-01 팀),
   둘 다 False면 예전처럼 BACKOFF(집게 열고 후진) -> SEARCH -> 다시 잡기)
  중앙 근처까지 와도 안 보이면 SEARCH -> ALIGN -> APPROACH
  + 언제나 탈락선 감시 (RETREAT)

GUARD (2026-11-01 팀 결정 "빠르게 잡고 도망"):
  잡으면 중앙(상대가 모이는 곳)에서 GUARD_RADIUS_M 떨어진 원 위, 우리 출발 쪽으로 천천히 빠진다.
  거기서 서 있다가 LiDAR에 상대가 가까워지면 원을 따라 상대 반대쪽으로 피한다.
  차체가 약해 부딪히지 않는 것이 우선이고, 큐브가 빠지지 않게 느리게·부드럽게 움직인다.
  원은 탈락선에서 충분히 떨어져 있고, 원을 따라 움직이므로 바깥(탈락선) 쪽으로 피하지 않는다.

좌표:
- 경기장 좌표(m), 왼쪽 아래 모서리가 (0, 0)
- 네 출발 위치는 중앙 기준 90도씩 돌린 모양이고 장애물이 없으므로
  로봇은 항상 "(8, 2) 격자 중심에서 +y 방향을 보고 출발"한 것으로 계산한다.
"""

import collections
from dataclasses import dataclass
import math
from typing import Optional


CELL_SIZE_M = 0.4
ARENA_SIZE_M = 9 * CELL_SIZE_M
# 카메라로 계산한 큐브 위치가 외벽에서 이 안이면 경기장 밖 물체로 보고 버린다
ARENA_MARGIN_M = 0.05

START_POSE = (7.5 * CELL_SIZE_M, 1.5 * CELL_SIZE_M, math.pi / 2)
CENTER_M = (4.5 * CELL_SIZE_M, 4.5 * CELL_SIZE_M)

# 추정 위치가 외벽보다 이만큼 넘게 밖이면 위치를 잃은 것: RETREAT를 포함해 움직이지 않는다.
# 2026-11-03 20:20 실기: /odom_raw가 둘이라 위치가 (0.17, 9.91)처럼 경기장 밖으로 계산됐고, RETREAT가 그 위치로
# '안쪽'을 정해 실제로는 선 밖으로 최대 출력 후진했다.
POSE_LOST_MARGIN_M = 0.15

# 탈락 영역: 바깥 테두리 한 줄 (노란 선 바깥)
SAFE_ZONE_MIN_M = CELL_SIZE_M
SAFE_ZONE_MAX_M = ARENA_SIZE_M - CELL_SIZE_M

# TODO_MEASURE: 로봇 중심이 노란 선에서 이 거리 안으로 들어오면 중앙 쪽으로 복귀
# 2026-11-01 0.15 -> 0.3: 0.15면 차체 옆·앞 끝(중심에서 약 0.15~0.22 m)이 이미 선에 닿는다.
# 우리 로봇은 미는 힘이 약해 크게 밀릴 수 있어 일찍 돌아온다.
DANGER_MARGIN_M = 0.3
# 출발 중(TURN, DRIVE)에만: 출발 위치가 선에서 0.2 m라 0.3이면 출발하자마자 RETREAT가 된다.
# 출발하면 바로 중앙 쪽으로 가므로 선에서 멀어진다.
START_DANGER_MARGIN_M = 0.15
# 복귀 시작 거리보다 이만큼 더 들어오면 원래 하던 동작으로
RETREAT_EXIT_EXTRA_M = 0.15
# 큐브를 잡고 복귀할 때는 중앙이 아니라 선에서 곧장 안쪽으로, 복귀 끝 거리보다 이만큼 더 들어간 점을
# 향하고, 앞에 상대가 있어도 서지 않고 밀고 들어간다 (상대 밀기는 허용 3.4.6, 선 밖은 즉시 탈락).
# 2026-11-01 실기: 사방에서 조이면 중앙 쪽에서 미는 상대 앞에서 서 버려 그대로 선 밖으로 밀려 나갔다.
RETREAT_GOAL_EXTRA_M = 0.1
# RETREAT 중 이 시간 동안 선에서 RETREAT_STUCK_PROGRESS_M도 못 멀어지면(미는 상대에 막혀 바퀴만 돎) 곧장 안쪽 대신
# RETREAT_SIDESTEP_RAD 비스듬히 비켜 간다. 또 막히면 반대쪽으로. 2026-11-02 21:47 실기: 선 앞에서 4초 동안
# 0.8 m/s로 맞서 밀다 바퀴만 헛돌고(엔코더 2.8 m, LiDAR 위치 그대로) 선 밖으로 밀려남. TODO_MEASURE
RETREAT_STUCK_S = 0.8
RETREAT_STUCK_PROGRESS_M = 0.03
RETREAT_SIDESTEP_RAD = math.radians(50.0)
# RETREAT는 최대 출력 (2026-11-03 팀: 선 밖은 즉시 탈락이라 무슨 상황에서든 최우선). 방향이 RETREAT_FULL_SPEED_RAD
# 안이면 바로 RETREAT_SPEED(hardware.yaml max_linear)로, 앞뒤 중 덜 도는 쪽으로, 목표 앞에서 감속하지 않는다.
# 예전에는 방향 오차만큼 속도를 줄이고(60도면 절반) 회전 2.6 rad/s, 목표 앞 감속.
RETREAT_SPEED = 0.8
RETREAT_MAX_ANGULAR = 3.5  # hardware.yaml max_angular 3.6 안
RETREAT_FULL_SPEED_RAD = math.radians(80.0)

# 밀림 감지 (2026-11-02 팀: 외력으로 밀리면 바로 빠져나오기). GUARD·LURK에서 서 있는데(명령 0인 지 PUSH_SETTLE_S)
# 엔코더 속도가 PUSH_SPEED_M_S를 PUSH_CONFIRM_STEPS번 연속 넘으면 밀린 것. 21:47 실기: 서 있을 때 엔코더는 0.000,
# 밀리면 0.05~0.28 m/s (MDD10이 바퀴를 꽉 잡지 않아 밀리면 바퀴가 돈다). 가속도계는 x축에 -1.1 m/s^2이 늘 실려 못 쓴다.
# 미는 상대가 밀리는 방향 반대쪽 PUSH_OPPONENT_M에 있다고 보고 PUSH_MEMORY_S 동안 상대 목록에 넣는다
# -> 원을 따라 피하기(_keep_away)가 바로 시작된다. 밀리는 쪽으로 도망가면 선 쪽일 수 있어 원을 따라 비켜 간다.
PUSH_REACT = True
PUSH_SPEED_M_S = 0.03
PUSH_CONFIRM_STEPS = 2
PUSH_SETTLE_S = 0.3
PUSH_OPPONENT_M = 0.3
PUSH_MEMORY_S = 1.5
# 바퀴가 안 돌고 미끄러지며 밀릴 때 (엔코더 0): 서 있는 동안 LiDAR 벽 보정 위치(wall_offset)가 PUSH_SLIDE_WINDOW_S 안에
# PUSH_SLIDE_M 넘게 한쪽으로 움직이면 밀린 것. 갱신이 PUSH_SLIDE_MIN_UPDATES번 이상 있고 마지막 갱신도 같은 쪽이어야
# 한다 (빨리 돈 뒤 첫 스캔에서 한 번에 따라잡는 보정을 밀림으로 보지 않게).
# 2026-11-03 17:33 실기: 엔코더 0.000인 채로 벽 보정이 -0.02 -> -0.16 m로 바뀌며 밀렸다. TODO_MEASURE
PUSH_SLIDE_M = 0.04
PUSH_SLIDE_WINDOW_S = 0.5
PUSH_SLIDE_MIN_UPDATES = 3

# 이 거리(중앙까지)에 오면 카메라로 넘긴다 (그 전에라도 보이면 바로 넘긴다).
VISION_HANDOFF_M = 0.6

# 속도·가감속 (2026-10-31 임무 1과 같은 값, mission1_logic.py 참고)
DRIVE_SPEED = 0.8
# 2026-11-01 "전체적으로 느림" -> 가속 1.2 -> 1.6, 감속은 최대(LINEAR_DECEL)로 큐브 바로 앞에서 한 번에.
# TODO_MEASURE: 출발 때 바퀴가 헛돌거나(엔코더 거리 틀어짐) 앞뒤로 들리면 낮춘다
LINEAR_ACCEL = 1.6
# 2026-11-01 2.0 -> 3.5 급제동 ("이 속도면 급제동해도 문제없음"). 0.8 m/s에서 약 9 cm에 선다.
LINEAR_DECEL = 3.5
DRIVE_DECEL = LINEAR_DECEL  # 큐브(중앙이라고 가정) 앞 감속 지점까지 일정하게 감속
# 명령한 속도가 실제 바퀴 속도가 되기까지의 지연. 감속을 이만큼 일찍 시작한다.
# (0.8 m/s에서 0.2 s = 16 cm. 이걸 안 빼면 큐브를 지나쳤다, 시뮬레이션) TODO_MEASURE
# 2026-11-01 실기 6회차 로그: 명령 대비 실제 속도가 약 0.1~0.15 s 늦다. 0.2 -> 0.15
MOTOR_LAG_S = 0.15
ANGULAR_ACCEL = 15.0
# 이만큼 판단이 끊기면 그동안 노드가 정지 명령을 보냈다고 보고 0에서 다시 가속한다.
ACCEL_RESET_S = 0.15
TURN_SPEED = 3.5
MIN_TURN_SPEED = 0.6  # 바퀴 약 0.084 m/s = 하드웨어 최저 속도 근처
# 출발 곡선: 방향 오차 x 이 값으로 돌면서, 방향 오차의 cos만큼 전진한다 (45도면 70% 속도).
# 가속 구간(약 0.7초)에 회전을 겹쳐 제자리 45도 회전(약 0.5초)을 없앤다. 반지름은 처음 약 7~10 cm.
# TODO_MEASURE: 출발 때 크게 휘청이면 낮춘다
HEADING_GAIN = 4.0
# 회전 예측: 모터 지연만큼 지금 회전 속도로 더 돈다고 보고 미리 줄인다 (mission1_logic.TURN_LEAD_S와 같음)
TURN_LEAD_S = 0.12
# 2026-11-01 1.0 -> 1.5 (카메라 지연 0.3 s 동안 26도 = 화면 반폭 33도 안)
SEARCH_SPEED = 1.5
# 탐색: 먼저 오른쪽·왼쪽 이 각도까지 훑고(카메라 ±33도라 앞 ±83도), 없으면 왼쪽으로 계속 돈다.
# 2026-11-01 실기: 큐브가 오른쪽 44도에 있었는데 왼쪽으로만 돌아 300도(5.8 s) 걸렸다.
SEARCH_SWEEP_RAD = math.radians(50.0)
SEARCH_SWEEP_DONE_RAD = math.radians(5.0)
# 계속 돌며 한 바퀴를 다 봐도 없으면, 중앙에서 SEARCH_CENTER_M 거리까지 다가가 다시 돈다 (RECENTER).
# 카메라는 약 1.4 m(면적 300)까지만 큐브를 알아본다. 탈락한 팀의 큐브는 중앙으로 돌아온다(3.4.6).
SEARCH_CENTER_M = 1.0
RECENTER_SPEED = 0.4

# 카메라: x_offset 1.0 = 화면 끝 = 약 33도
CAMERA_HALF_FOV_RAD = math.radians(33.0)
# 카메라 결과는 이만큼 늦게 온다. 그 시각의 위치로 큐브 위치를 계산한다. TODO_MEASURE
# 2026-11-01 실기 6회차 로그: 0.77 -> 0.4 m/s로 줄일 때 카메라 면적 거리 + 이동 거리가 3.5 cm만
# 늘었다 -> 지연 약 0.1 s (0.3이면 11 cm). 0.3으로 두면 0.8 m/s에서 16 cm 일찍 감속했다. 0.3 -> 0.12
CAMERA_LAG_S = 0.12
# 큐브 거리 추정: 면적은 거리 제곱에 반비례.
# 2026-11-01 실측: 카메라 렌즈 정면 30 cm에서 7070 -> 50 cm 환산 약 2550
# (mission1_logic.py의 9000은 실측 전 가정. 임무 1은 그 값으로 실기 성공해 그대로 둔다)
# TODO_MEASURE: 50 cm·1 m에서도 재서 확인
TARGET_AREA_AT_50CM = 7070.0 * (0.3 / 0.5) ** 2
# 카메라가 로봇 중심(바퀴 축)보다 앞에 있는 거리 (mission1_logic.py와 같음, TODO_MEASURE)
CAMERA_AHEAD_M = 0.10
# 새 카메라 결과를 큐브 위치 추정에 섞는 비율 (나머지는 이전 추정)
TARGET_SMOOTHING = 0.5
# 위치 기록 사이가 이보다 벌어졌으면(판단이 멈춤) 그 구간 위치로 큐브 위치를 계산하지 않는다.
POSE_HISTORY_GAP_S = 0.12
# 제자리 맞추기(ALIGN)는 큐브 방향이 이 안이면 끝내고 전진하며 맞춘다.
ALIGN_DONE_RAD = math.radians(15.0)
# 이보다 빨리 돌 때는 큐브 위치를 갱신하지 않는다 (카메라 지연 오차 = 회전 속도 x 지연 차이).
# 2026-11-01 실기: 접근 중 앞에서 좌우로 흔들림. 시뮬레이션에서 실제 지연이 0.15 s면 추정이 ±15 cm 튀었다.
TARGET_UPDATE_MAX_ANGULAR = 1.5
# 큐브 접근 중 회전: 부드럽게 (위치 추정 흔들림을 키우지 않게)
APPROACH_HEADING_GAIN = 2.0
APPROACH_MAX_ANGULAR = 1.2
# 이 크기 이상으로 보이면 DRIVE 중에도 바로 카메라 접근 (경기장 밖 빨간 물체는 작게 보여 무시)
# 350 = 카메라에서 약 1.35 m (2026-11-01 실측 기준, 카메라 노드 최소 300). 예전 2000은 약 56 cm,
# 600은 SEARCH로 넘어간 0.1 s 뒤에야 756으로 보여 놓쳤다 (2026-11-01 실기).
SEE_TARGET_MIN_AREA = 350.0
# DRIVE 중에 이 안(화면 가운데 쪽)에 보이면 바로 접근한다.
APPROACH_MAX_OFFSET = 0.6
# 접근 속도: 멀면 빠르게, 큐브 위치 추정까지 남은 거리로 일정하게 감속해
# APPROACH_SLOW_AREA(잡는 위치 약 5 cm 전)에서 APPROACH_SLOW_SPEED가 된다.
# 남은 거리는 마지막 카메라 결과 + 그 뒤 엔코더 이동으로 계산하므로 카메라 지연만큼 지나치지 않는다.
APPROACH_FAST_SPEED = DRIVE_SPEED  # 2026-11-01 0.5 -> 0.8 (접근 흔들림을 고친 뒤)
# 2026-11-01 "발견하고 너무 느리게 잡음" 0.12 -> 0.25. 잡는 시점을 늦게 오는 카메라 면적 대신
# 엔코더 위치(고정한 큐브 위치까지 거리)로 정하므로 빨리 들어가도 지나치지 않는다.
APPROACH_SLOW_SPEED = 0.4  # 2026-11-01 0.25 -> 0.4 ("조금 비켜 잡아도 되니 속도를 줄이지 말자")
APPROACH_DECEL = DRIVE_DECEL
# 이만큼 가까우면(카메라 약 25 cm) 더 틀지 않고 곧게 들어간다 (조금 비켜 잡아도 됨, 2026-11-01).
# 가까울수록 위치 추정의 작은 오차에도 방향이 크게 바뀌어 막판에 꺾였다 (시뮬레이션).
APPROACH_STRAIGHT_AREA = 10000.0
# 곧게 들어가기 전에 큐브가 화면 가운데(x_offset) 이 안이어야 한다. 밖이면 가면서 카메라 방향으로 맞춘다.
# 2026-11-01 실기 19:30: 면적 13500, x_offset -0.43(약 14도)에서 곧게 고정해 큐브 옆을 집었다.
# 적외선이 없을 때는 APPROACH_GRAB_MAX_OFFSET 밖이면 잡기를 시작하지 않는다. TODO_MEASURE: 집게 폭에 맞춰
APPROACH_STRAIGHT_MAX_OFFSET = 0.15
APPROACH_GRAB_MAX_OFFSET = 0.3
APPROACH_CENTER_GAIN = 2.0   # rad/s 당 카메라 방향 오차(rad)
APPROACH_SLOW_AREA = 20000.0  # 카메라에서 약 18 cm (잡기 GRAB_AREA는 약 13 cm, 2026-11-01 실측 기준)
# 카메라가 대상을 잠깐 놓쳐도 이 시간은 기다린다 (카메라 지연 약 0.3초)
TARGET_LOST_S = 0.5

# 잡기 (임무 1과 같음)
GRAB_AREA = 35000.0
GRAB_PUSH_M = 0.04
# 2026-11-01 0.02 -> GRAB_PUSH_M: 밀기 시작하자마자 닫는다. 0.4 m/s로 닫히는 0.18 s 동안 약 7 cm 더 들어간다.
GRAB_CLOSE_LEAD_M = GRAB_PUSH_M
GRAB_PUSH_SPEED = APPROACH_SLOW_SPEED
GRAB_PUSH_TIMEOUT_S = 1.0
# 2026-11-01 팀 요청: "큐브를 조금 밀어도 되니 주춤하지 말고 확실하게 잡기".
# 집게 안 적외선(VL53L0X, /range/gripper)이 살아 있으면 카메라 면적으로 닫지 않고, 밀며 들어가다가
# 큐브가 이 거리 안에 보이는 순간 닫는다. 실측: 잡은 큐브 28 mm, 빈 집게 닫힘 108 mm.
# TODO_MEASURE: 열린 집게 안으로 큐브가 충분히 들어온 거리
GRAB_TOF_M = 0.05
# 적외선 방식으로 밀며 들어가는 최대 거리. 이만큼 밀어도 집게 안에 안 들어오면(옆으로 비켜 감)
# 닫지 않고 물러나 다시 찾는다 (헛 집게질 대신, BACKOFF)
GRAB_TOF_PUSH_MAX_M = 0.15
APPROACH_STALL_M = 0.06
AREA_GROW_RATIO = 1.05

# 집게 기다리기: /gripper/state가 closed면 바로, 모르면 최대 GRASP_WAIT_S
GRASP_WAIT_S = 1.5
GRIPPER_MIN_WAIT_S = 0.15  # 2026-11-01 0.25 -> 0.15 (집게 닫힘 0.36 -> 0.18 s)
GRIPPER_SETTLE_S = 0.1
# 집게가 닫힌 뒤 /gripper/holding(ToF)이 false여도 이만큼은 더 기다린다.
# tof_node는 가까운 값이 3번(약 30 Hz, 0.1 s) 이어져야 true라서 바로 보면 잡았는데도 실패로 볼 수 있다.
HOLDING_CONFIRM_S = 0.3

# 못 잡았거나 놓쳤으면 집게를 열고 이만큼 곧게 후진한 뒤 다시 찾는다 (2026-11-01).
# 큐브가 집게 바로 앞(카메라 아래)에 있으면 화면에 안 보여 SEARCH가 한 바퀴를 다 돈다.
# 물러나면 화면에 들어와 바로 다시 접근한다. TODO_MEASURE: 카메라에 다시 보이는 거리
BACKOFF_M = 0.18
BACKOFF_SPEED = 0.3
BACKOFF_TIMEOUT_S = 1.5

# 2026-11-01 팀 결정: 잡기에 실패하거나(헛 집게질·밀어도 안 들어옴) 잡았다가 놓치면(뺏김)
# 큐브를 쫓아가지 않고 그 자리에서 끝까지 돌기만 한다 (SPIN). 탈락선 감시(RETREAT)는 그대로.
# 집게는 그대로 둔다 (닫힌 채라 차체가 작고, ToF가 잘못 '놓침'으로 봐도 큐브를 떨어뜨리지 않는다).
SPIN_AFTER_FAIL = True
SPIN_SPEED = 1.5  # rad/s (SEARCH_SPEED와 같게). TODO_MEASURE: 차체가 흔들리면 낮춘다

# 2026-11-02 팀: 실패·놓침 뒤 SPIN 대신 LURK. 잡았을 때(GUARD)와 같이 원 위로 빠지고 상대가 오면 피한다.
# 원 위에 서면 제자리에서 SPIN_SPEED로 돌며 둘러보고, 피하는 중이 아닐 때 빨간 대상이 보이면
# 집게를 열고 다시 잡으러 간다. False면 SPIN_AFTER_FAIL대로 (되돌리기용, git 태그 backup/m2-spin-before-lurk).
# 집게는 다시 잡으러 갈 때까지 그대로 둔다 (ToF가 잘못 '놓침'으로 봐도 큐브를 떨어뜨리지 않는다).
LURK_AFTER_FAIL = True
# 큐브가 카메라에서 이보다 가까우면(면적이 크면) 아직 다시 잡으러 가지 않는다. 집게가 닫힌 채
# 큐브 바로 앞에서 실패했으므로, 원 쪽으로 물러나 집게가 다 열릴 거리를 번 뒤에 간다.
# 약 25 cm면 0.4 m/s로 다가가도 집게가 다 열리는 0.18 s(약 7 cm) 뒤에 큐브까지 여유가 있다.
LURK_RETRY_MAX_AREA = APPROACH_STRAIGHT_AREA  # 카메라에서 약 25 cm
# 큐브 위치가 상대 로봇(LiDAR 중심)에서 이 안이면 상대가 잡은 큐브로 보고 가지 않는다 (차체가 약해
# 부딪히지 않는 것이 우선). 0이면 언제나 간다. TODO_MEASURE: 상대 집게 길이에 맞춰
LURK_IGNORE_NEAR_OPPONENT_M = 0.3
# 원 위 한 자리에서 제자리로 한 바퀴(LURK_TURN_BEFORE_MOVE_RAD) 돌아도 다시 잡으러 가지 못하면 원을 따라
# LURK_MOVE_RAD 옮겨 다시 돈다 (2026-11-03 팀). 한 자리에서는 멀거나(약 1.35 m 넘게) 상대에 가린 큐브를 끝내 못 봤다.
# 시뮬레이션 300가지(상대 0~3대가 가림): 30초 안에 찾음 242 -> 272, 지금 방식이 찾는 경우는 똑같이 빠름,
# 부딪힘·탈락선 0.2 m 안 0번. (원을 돌며 중앙만 보는 방식은 253, 평균이 더 느렸다). TODO_MEASURE
LURK_TURN_BEFORE_MOVE_RAD = 2 * math.pi
LURK_MOVE_RAD = math.radians(60.0)
# 상대가 잡은 큐브(상대에서 LURK_IGNORE_NEAR_OPPONENT_M 안)를 보면 그 상대를 가장 가까운 노란 선 쪽으로 민다
# (2026-11-03 팀: 연습에서 조금만 더 밀면 선 밖으로 밀 수 있었다. 상대 밀기 허용 3.4.6, 탈락한 팀의 큐브는 중앙으로).
# 상대 뒤편(선 반대쪽) ATTACK_STANDOFF_M 지점으로 가서(우리가 상대와 선 사이면 옆으로 돌아서) 선 쪽을 보고
# ATTACK_SPEED로 들이민다. 우리가 선 DANGER_MARGIN_M 앞에 오면 RETREAT가 끊는다 (최우선).
# 상대가 선을 ATTACK_DONE_MARGIN_M 넘었거나, ATTACK_LOST_S 동안 못 봤거나, ATTACK_TIMEOUT_S가 지나면 LURK로.
ATTACK_HELD_CUBE = True
ATTACK_STANDOFF_M = 0.45
ATTACK_START_M = 0.1     # 자리 잡기 점에서 이 안이면 밀기 시작
ATTACK_DETOUR_M = 0.5    # 상대 옆으로 돌아갈 때 상대 중심에서 옆으로
ATTACK_AIM_AHEAD_M = 0.3  # 밀 때 겨누는 점: 상대 중심에서 선 쪽으로
ATTACK_RAM_RAD = math.radians(25.0)  # 방향이 이 안이면 밀고, 밖이면 제자리에서 먼저 돈다
ATTACK_SPEED = 0.8       # hardware.yaml max_linear
ATTACK_TRACK_M = 0.5     # 상대 위치를 이어서 찾는 거리
ATTACK_LOST_S = 1.0
ATTACK_TIMEOUT_S = 8.0
ATTACK_DONE_MARGIN_M = -0.05

# 시작할 때 집게 열기 명령을 보낼지. 2026-11-01 False로 했다가 True로 되돌림:
# 임무를 켜면 Mega가 리셋돼 서보 힘이 빠지고 집게가 조금 닫힌다. 열기를 안 보내면 덜 열린 채로
# 큐브까지 갔다. (시작 때 "한 번 오므려 보인" 것은 힘 빠진 집게를 열림 90도로 다시 잡는 동작)
# 주행과 동시에 열리므로 시간 손해는 없다.
OPEN_GRIPPER_AT_START = True
# 출발: True = 제자리에서 중앙 방향(45도)으로 최대 속도로 돈 뒤 곧게 직진.
# False = 출발하자마자 가속하며 곡선으로 튼다 (10-31 방식, 기본). 2026-11-01 시뮬레이션(지연 포함):
# 밀어 넣기 시작 곡선 2.35 s / 제자리 회전 2.80 s -> 곡선 유지 (팀 결정)
START_PIVOT = False
PIVOT_DECEL = 10.0                   # rad/s^2, 목표각 앞에서 일정하게 감속 (임무 1 TURN_DECEL과 같음)
PIVOT_DONE_RAD = math.radians(4.0)   # 남은 각도가 이 안이면 직진 시작 (나머지는 직진하며 맞춤)

# 잡은 뒤 지키기 (GUARD). False면 예전처럼 잡은 자리에 서 있는다 (HOLD).
GUARD_ENABLED = True
# True = 잡자마자 원 위 우리 출발 쪽 점까지 빠진 뒤 지킨다 (2026-11-01 '잡고 도망'). 빠지는 방식은 아래.
# False = 잡은 자리에 서 있다가 상대가 DODGE_START_M 안으로 오면 원을 따라 피하고, 멀어지면 그 자리에 선다
# (2026-11-02 팀이 잠깐 정했다가 되돌림: 뒤로 빠지되 거리만 줄인다 -> GUARD_RADIUS_M 1.0 -> 0.85).
GUARD_ESCAPE_FIRST = True
# True = 처음 빠질 때 후진하지 않고 제자리에서 180도 돌아 전진으로 원 위 점까지 간다 (2026-11-03 팀:
# 중앙 쪽 상대가 큐브를 못 보게 차체로 가린다. 도착하면 큐브가 바깥을 본다). 도는 동안 약 1.3 s 더 걸린다.
# False = 예전처럼 중앙을 본 채 후진. TODO_MEASURE: 큐브를 끌고 돌 때 빠지면 GUARD_MAX_ANGULAR를 낮춘다
GUARD_ESCAPE_TURN_AROUND = True
# 돌아설 때 목표 점 방향과 이 안으로 맞을 때까지는 제자리에서만 돈다 (곡선으로 크게 휘면 도착했을 때
# 큐브가 바깥이 아니라 옆을 본다, 시뮬레이션 40도)
GUARD_TURN_AROUND_DONE_RAD = math.radians(20.0)
# 지키는 원의 반지름 (중앙에서). 0.85 m면 대각선(처음 빠지는 점)에서 탈락선까지 0.80 m, 원이 선과 가장 가까운
# 정면 방향이면 0.55 m 남는다 (RETREAT 0.3 m보다 멀다). 2026-11-01 0.8 -> 1.0 ("너무 조금 후진", 약 0.87 m 후진)
# TODO_MEASURE: 연습장에서 상대 로봇 접근을 보고 조정 (제출 전에 확정, 경기장 공개 후 수정 금지)
# 2026-11-02 1.0 -> 0.85 (팀: 빠지는 거리를 조금 줄이자). 잡는 자리(중앙에서 약 0.13 m)에서 약 0.87 -> 0.72 m 후진.
GUARD_RADIUS_M = 0.85
# 처음 빠지는 방향 (중앙 기준). 우리 출발 위치 쪽 대각선 = 상대가 출발하지 않는 쪽.
GUARD_HOME_ANGLE = math.atan2(START_POSE[1] - CENTER_M[1], START_POSE[0] - CENTER_M[0])
# 큐브를 잡고 움직일 때 속도·가감속·회전 (빠르거나 급하면 큐브가 빠진다). TODO_MEASURE
# 2026-11-01 0.3 -> 0.5, 가감속 0.6 -> 1.0 (실기: 집게 힘이 충분해 큐브가 빠지지 않음, 피하기가 느렸다)
# 2026-11-02 0.5 -> 0.65 (팀: 피하는 속도를 더 올리자). 21:24 실기: 큐브를 잡고 0.65 m/s 후진·0.5 m/s 피하기 내내
# 집게 ToF 22~30 mm로 큐브가 빠지지 않았다. TODO_MEASURE: 피하다 큐브가 틀어지거나 빠지면 낮춘다
# 2026-11-02 0.65 -> 0.8 (팀: 올릴 수 있는 만큼). 0.8 = hardware.yaml max_linear (바퀴 약 235 rpm, 정격 250)
GUARD_SPEED = 0.8
# 잡자마자 원 위 처음 점까지 빠질 때만 (상대가 중앙에 오기 전에 벗어나기, 2026-11-01).
# 정격 근처(0.8 m/s, hardware.yaml max_linear)는 큐브를 끌면 속도가 흔들린다.
# 2026-11-01 0.5 -> 0.65 (가감속 1.0이면 0.8 m 안에서 약 0.89 m/s까지 낼 수 있다)
# 2026-11-02 21:24 실기: 0.65 m/s로 후진했다 서도 큐브가 빠지지 않음 (ToF 22~30 mm)
GUARD_ESCAPE_SPEED = 0.8  # 2026-11-02 0.65 -> 0.8 (max_linear)
# 2026-11-02 1.0 -> 1.3: 피하기 시작해서 0.65 m/s까지 약 0.65 -> 0.5 s. TODO_MEASURE: 큐브가 빠지면 1.0으로
GUARD_ACCEL = 1.6  # 2026-11-02 1.3 -> 1.6 (= LINEAR_ACCEL, 전체 출발 가속 한도)
GUARD_HEADING_GAIN = 2.0
# 2026-11-01 1.0 -> 1.6: GUARD_SPEED 0.3 -> 0.5와 같은 비율 (같은 회전 반경, 피할 때 원 밖으로 크게 돌지 않게)
# 2026-11-02 1.6 -> 2.1: GUARD_SPEED 0.5 -> 0.65와 같은 비율 (회전 반경 그대로)
# 2026-11-02 2.1 -> 2.6: GUARD_SPEED 0.65 -> 0.8과 같은 비율
GUARD_MAX_ANGULAR = 2.6
# 목표 점에서 이 안이면 선다
GUARD_ARRIVE_M = 0.08
# 앞·뒤 중 덜 도는 쪽으로 간다. 지금 방향을 바꾸려면 이만큼 더 틀어져야 한다 (앞뒤 왔다 갔다 방지).
GUARD_REVERSE_HYSTERESIS_RAD = math.radians(20.0)
# 상대가 이 거리 안으로 오면 피하기 시작하고, DODGE_CLEAR_M 밖으로 가면 그 자리에 선다.
DODGE_START_M = 0.7
DODGE_CLEAR_M = 0.9
# 피할 때 원 위에서 한 번에 앞서 보는 각도 (원을 따라가도록 목표 점을 조금씩 옮긴다)
DODGE_ARC_STEP_RAD = 0.5
# 피할 방향 고르기: 원을 따라 양쪽으로 DODGE_LOOKAHEAD_RAD까지 길을 그려 보고,
# 길에서 상대와 가장 가까워지는 거리가 더 큰 쪽으로 간다 (상대가 여럿이면 더 빈 쪽).
DODGE_LOOKAHEAD_RAD = math.radians(90.0)
DODGE_PATH_STEP_RAD = math.radians(15.0)
# 반대쪽이 이만큼 더 넓어야 피하던 방향을 바꾼다 (왔다 갔다 방지)
DODGE_SWITCH_M = 0.1
# 양쪽 길 모두 상대와 이보다 가까우면(끼였다) 들이받지 않게 그 자리에 선다 (로봇 중심 ~ 상대 표면)
DODGE_BLOCKED_M = 0.35
# 가는 방향 ±GUARD_BLOCK_RAD 안, 이 거리 + 지금 속도로 서는 거리 안에 상대가 있으면
# 전진·후진을 멈춘다 (들이받지 않기). 서 있으면 0.3 m, 0.3 m/s면 약 0.42 m, 0.5 m/s면 약 0.58 m.
# (로봇 중심 ~ 상대 로봇 표면. 우리 차체 앞뒤 끝이 중심에서 약 0.15~0.22 m)
GUARD_BLOCK_M = 0.3
GUARD_BLOCK_RAD = math.radians(60.0)

GRIPPER_OPEN = 'open'
GRIPPER_GRAB = 'grab'
GRIPPER_CLOSED = 'closed'

TURN = 'TURN'  # 출발 제자리 회전 (START_PIVOT)
DRIVE = 'DRIVE'
SEARCH = 'SEARCH'
ALIGN = 'ALIGN'
APPROACH = 'APPROACH'
PUSH = 'PUSH'
GRASP = 'GRASP'
HOLD = 'HOLD'
GUARD = 'GUARD'
RETREAT = 'RETREAT'
RECENTER = 'RECENTER'
BACKOFF = 'BACKOFF'
SPIN = 'SPIN'
LURK = 'LURK'
ATTACK = 'ATTACK'  # 상대가 잡은 큐브를 보면 그 상대를 선 쪽으로 밀기
# 큐브를 잡고 있는 상태
HOLDING_STATES = (HOLD, GUARD)


@dataclass
class Observation:
    x: float
    y: float
    yaw: float
    now: float
    # 조원 B Vision 결과. x_offset -1 ~ 1, 음수 = 대상이 화면 왼쪽
    detected: bool = False
    x_offset: float = 0.0
    area: float = 0.0
    # 조원 A 집게 상태. 알 수 없으면 None
    holding: Optional[bool] = None
    # 조원 A /gripper/state: 'open' / 'closed' / 'moving'. 알 수 없으면 None
    gripper_state: Optional[str] = None
    # 카메라 결과를 받은 시각. 같은 결과를 두 번 쓰지 않는다. None이면 매번 새 결과로 본다.
    vision_stamp: Optional[float] = None
    # LiDAR 벽 보정값 (odom 위치에 더하는 x, y). 서 있는데 바뀌면 미끄러지며 밀리는 중. 모르면 None
    wall_offset: Optional[tuple] = None
    # LiDAR로 찾은 가장 가까운 상대 로봇 (경기장 좌표). 없거나 모르면 None (opponent_detector.py)
    opponent: Optional[tuple] = None
    # 찾은 상대 로봇 전부 (가까운 순). 비어 있으면 opponent만 쓴다.
    opponents: tuple = ()
    # 집게 안 적외선 거리 (m, 범위 밖이면 inf). 센서가 없거나 끊기면 None (예전 카메라 면적 방식)
    gripper_range: Optional[float] = None
    # 엔코더로 잰 앞뒤 속도 (m/s, 앞 +). 모르면 None (밀림 감지 안 함)
    measured_linear: Optional[float] = None


@dataclass
class Command:
    linear: float = 0.0
    angular: float = 0.0
    # None = 집게 명령 없음 (현재 상태 유지)
    gripper: Optional[str] = None


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def heading_error_to(obs, target):
    target_yaw = math.atan2(target[1] - obs.y, target[0] - obs.x)
    return normalize_angle(target_yaw - obs.yaw)


def distance_to(obs, target):
    return math.hypot(target[0] - obs.x, target[1] - obs.y)


def distance_to_danger_line(obs):
    """노란 선(탈락 영역 경계)까지의 가장 가까운 거리. 선을 넘으면 음수."""
    return min(
        obs.x - SAFE_ZONE_MIN_M,
        SAFE_ZONE_MAX_M - obs.x,
        obs.y - SAFE_ZONE_MIN_M,
        SAFE_ZONE_MAX_M - obs.y,
    )


def retreat_goal(obs, exit_margin):
    """선에서 곧장 안쪽으로, 복귀 끝 거리보다 RETREAT_GOAL_EXTRA_M 더 들어간 가장 가까운 점."""
    low = SAFE_ZONE_MIN_M + exit_margin + RETREAT_GOAL_EXTRA_M
    high = SAFE_ZONE_MAX_M - exit_margin - RETREAT_GOAL_EXTRA_M
    return clamp_range(obs.x, low, high), clamp_range(obs.y, low, high)


def clamp_range(value, low, high):
    return min(max(value, low), high)


def guard_point(angle):
    """중앙에서 GUARD_RADIUS_M 떨어진 원 위의 점."""
    return (CENTER_M[0] + GUARD_RADIUS_M * math.cos(angle),
            CENTER_M[1] + GUARD_RADIUS_M * math.sin(angle))


def angle_around_center(point):
    return math.atan2(point[1] - CENTER_M[1], point[0] - CENTER_M[0])


def clamp(value, limit):
    return max(-limit, min(limit, value))


def area_to_distance(area):
    """카메라 면적 -> 큐브까지 거리 추정(m). 면적은 거리 제곱에 반비례한다."""
    return 0.5 * math.sqrt(TARGET_AREA_AT_50CM / max(area, 1.0))


def braking_speed(remaining, current, final_speed, decel=LINEAR_DECEL):
    """남은 거리에서 final_speed까지 decel로 줄일 수 있는 최고 속도. 모터 지연만큼 덜 남았다고 본다."""
    remaining -= abs(current) * MOTOR_LAG_S
    return math.sqrt(final_speed ** 2 + 2.0 * decel * max(0.0, remaining))


def ramp(target, last, accel, decel, dt):
    """last에서 target으로 dt 동안 accel(빨라질 때)·decel(느려질 때)만큼만 바꾼다."""
    if target * last < 0.0:
        # 방향이 바뀌면 먼저 0까지 줄인다.
        target = 0.0
    rate = accel if abs(target) > abs(last) else decel
    step = rate * dt
    return last + max(-step, min(step, target - last))


class Mission2Logic:

    def __init__(self):
        self.state = TURN if START_PIVOT else DRIVE
        self.state_started_at = None
        self.gripper_opened = False
        self._state_before_retreat = DRIVE
        self._retreat_exit_margin = DANGER_MARGIN_M + RETREAT_EXIT_EXTRA_M
        self._target_seen_at = None
        # 카메라로 계산한 큐브 위치 (경기장 좌표). 카메라 지연 때문에 화면 값으로 바로 돌지 않고
        # 이 점을 향해 엔코더·IMU 위치로 돈다 (위치는 지연이 작다).
        self.target = None
        self._target_updated_at = None
        self._poses = collections.deque()
        self._vision_stamp = None
        self._search_direction = 1.0
        self._search_start_yaw = 0.0
        self._search_step = 2
        # 계속 돌기 단계에서 돈 각도 (한 바퀴 다 봤는지)
        self._search_turned = 0.0
        self._search_last_yaw = 0.0
        self._area_best = 0.0
        self._area_best_at = None
        self._push_start = None
        self._grab_sent_at = None
        # 이번 밀기를 적외선으로 판단하는지 (밀기 시작할 때 센서가 살아 있었는지)
        self._push_tof = False
        self._gripper_reached_at = None
        self._last_linear = 0.0
        self._last_angular = 0.0
        self._last_command_at = None
        # GUARD: 지금 가려는 원 위 각도(중앙 기준), 피하는 중인지, 피하는 방향, 후진으로 가는지
        self._guard_angle = GUARD_HOME_ANGLE
        self._dodging = False
        self._dodge_direction = 1.0
        self._guard_reverse = False
        self._escaping = False
        # 집게가 닫혔다고 본 시각 (GRASP), 후진 시작 위치 (BACKOFF)
        self._grasp_closed_at = None
        self._backoff_start = None
        # 잡기에 실패했거나 놓쳐서 LURK·SPIN으로 간 뒤인지 (RETREAT 뒤에도 그리로 돌아간다)
        self.gave_up = False
        # 밀림 감지: 명령 0으로 서 있기 시작한 시각, 연속으로 밀린 판단 횟수, 미는 상대 (x, y, 언제까지)
        self._stand_since = None
        self._push_steps = 0
        self.push_opponent = None
        # 마지막 밀림 감지 근거 (로그용)
        self.push_reason = ''
        # 추정 위치가 경기장 밖이라 서 있는 중인지 (POSE_LOST_MARGIN_M)
        self.pose_lost = False
        # ATTACK: 미는 상대 위치, 미는 방향(단위 벡터), 들이미는 중인지
        self.attack_victim = None
        self.attack_dir = None
        self._attack_seen_at = None
        self._attack_ramming = False
        # 서 있는 동안의 벽 보정값 기록 [(시각, (x, y))] (미끄러짐 감지)
        self._slide_history = []
        # LURK: 지금 자리에서 제자리로 돈 각도 (한 바퀴 돌면 원을 따라 옮긴다)
        self._lurk_turned = 0.0
        self._lurk_last_yaw = None
        # RECENTER로 갈 중앙 둘레 각도 (None이면 지금 자리 쪽)
        self._recenter_angle = None
        # RETREAT 중 선에서 가장 멀었던 거리와 그 시각, 비켜 가는 쪽 (0 = 곧장 안쪽)
        self._retreat_best = 0.0
        self._retreat_best_at = 0.0
        self._retreat_side = 0.0

    def _change_state(self, new_state, obs):
        if new_state == SEARCH and new_state != self.state:
            # 전에 본 큐브가 있으면 그쪽으로 계속 돈다 (도는 중에 방향을 바꾸지 않는다).
            # 없으면 오른쪽 -> 왼쪽으로 앞쪽을 먼저 훑는다.
            self._search_direction = 1.0
            self._search_start_yaw = obs.yaw
            self._search_step = 0
            self._search_turned = 0.0
            self._search_last_yaw = obs.yaw
            if self.target is not None and distance_to(obs, self.target) > CAMERA_AHEAD_M:
                self._search_direction = math.copysign(1.0, heading_error_to(obs, self.target))
                self._search_step = 2
        self.state = new_state
        self.state_started_at = obs.now
        self._gripper_reached_at = None
        self._grasp_closed_at = None

    def step(self, obs):
        if self.state_started_at is None:
            self.state_started_at = obs.now
        self.pose_lost = not all(-POSE_LOST_MARGIN_M <= v <= ARENA_SIZE_M + POSE_LOST_MARGIN_M
                                 for v in (obs.x, obs.y))
        if self.pose_lost:
            return self._limit_accel(Command(), obs.now)
        self._poses.append((obs.now, obs.x, obs.y, obs.yaw))
        while self._poses and obs.now - self._poses[0][0] > 2.0 * CAMERA_LAG_S:
            self._poses.popleft()
        self._update_target(obs)
        self._detect_push(obs)

        command = self._safety_check(obs)
        if command is None:
            command = self._run_state(obs)

        # 시작 시 집게를 한 번 연다 (OPEN_GRIPPER_AT_START일 때만).
        if OPEN_GRIPPER_AT_START and not self.gripper_opened and command.gripper is None:
            command.gripper = GRIPPER_OPEN
            self.gripper_opened = True

        return self._limit_accel(command, obs.now)

    def _limit_accel(self, command, now):
        """급출발·급정지 방지: 직전 명령에서 가감속 한도만큼만 바꾼다 (상태가 바뀌어도 이어서)."""
        dt = None if self._last_command_at is None else now - self._last_command_at
        if dt is None or dt < 0.0 or dt > ACCEL_RESET_S:
            self._last_linear = self._last_angular = 0.0
            dt = ACCEL_RESET_S / 2.0
        accel, decel = LINEAR_ACCEL, LINEAR_DECEL
        if self.holding_cube() and self.state != RETREAT:
            # 큐브를 잡고 있으면 급출발·급제동하지 않는다 (큐브가 빠진다).
            # RETREAT는 예외: 선 밖은 즉시 탈락이라 최대 감속으로 꺾는다.
            accel = decel = GUARD_ACCEL
        command.linear = ramp(command.linear, self._last_linear, accel, decel, dt)
        command.angular = ramp(
            command.angular, self._last_angular, ANGULAR_ACCEL, ANGULAR_ACCEL, dt)
        self._last_linear = command.linear
        self._last_angular = command.angular
        self._last_command_at = now
        return command

    def _safety_check(self, obs):
        margin = distance_to_danger_line(obs)
        danger = START_DANGER_MARGIN_M if self.state in (TURN, DRIVE) else DANGER_MARGIN_M

        if self.state != RETREAT and margin < danger:
            self._state_before_retreat = self.state
            self._retreat_exit_margin = danger + RETREAT_EXIT_EXTRA_M
            self._retreat_best, self._retreat_best_at, self._retreat_side = margin, obs.now, 0.0
            self._change_state(RETREAT, obs)

        if self.state != RETREAT:
            return None

        if margin >= self._retreat_exit_margin:
            resume = self._resume_state_after_retreat()
            if resume in (GUARD, LURK):
                # 밀려난 자리에서 가까운 원 위 점으로 돌아간다 (처음 빠진 점까지 가로지르지 않는다)
                self._guard_angle = angle_around_center((obs.x, obs.y))
                self._dodging = False
                self._escaping = False
            self._change_state(resume, obs)
            return None

        if margin > self._retreat_best + RETREAT_STUCK_PROGRESS_M:
            self._retreat_best, self._retreat_best_at = margin, obs.now
        elif obs.now - self._retreat_best_at > RETREAT_STUCK_S:
            # 막혔다 (미는 상대에 맞서 바퀴만 돈다) -> 비스듬히 비켜 간다, 또 막히면 반대쪽
            self._retreat_side = -self._retreat_side if self._retreat_side else 1.0
            self._retreat_best_at = obs.now
        # 큐브를 잡았든 아니든 선에서 곧장 안쪽으로, 앞뒤 중 덜 도는 쪽으로 최대 출력.
        # 상대가 막아도 서지 않는다 (RETREAT_GOAL_EXTRA_M 주석).
        goal = self._sidestep(obs, retreat_goal(obs, self._retreat_exit_margin))
        return self._retreat_drive(obs, goal)

    def _retreat_drive(self, obs, goal):
        """RETREAT: goal 쪽으로 앞뒤 중 덜 도는 방향, RETREAT_FULL_SPEED_RAD 안이면 바로 최대 속도."""
        error = heading_error_to(obs, goal)
        if self._guard_reverse:
            self._guard_reverse = abs(error) > math.pi / 2 - GUARD_REVERSE_HYSTERESIS_RAD
        else:
            self._guard_reverse = abs(error) > math.pi / 2 + GUARD_REVERSE_HYSTERESIS_RAD
        if self._guard_reverse:
            error = normalize_angle(error + math.pi)
        linear = RETREAT_SPEED if abs(error) < RETREAT_FULL_SPEED_RAD else 0.0
        angular = clamp(GUARD_HEADING_GAIN * error, RETREAT_MAX_ANGULAR)
        return Command(linear=-linear if self._guard_reverse else linear, angular=angular)

    def _sidestep(self, obs, goal):
        """RETREAT가 막혔으면 goal 방향을 로봇 기준 RETREAT_SIDESTEP_RAD만큼 돌린 점."""
        if not self._retreat_side:
            return goal
        angle = self._retreat_side * RETREAT_SIDESTEP_RAD
        dx, dy = goal[0] - obs.x, goal[1] - obs.y
        return (obs.x + dx * math.cos(angle) - dy * math.sin(angle),
                obs.y + dx * math.sin(angle) + dy * math.cos(angle))

    def _detect_push(self, obs):
        """GUARD·LURK에서 서 있는데 엔코더가 움직이면 밀린 것: 미는 상대를 push_opponent에 기억한다."""
        if self.push_opponent is not None and obs.now > self.push_opponent[2]:
            self.push_opponent = None
        standing = abs(self._last_linear) < 1e-3 and abs(self._last_angular) < 1e-3
        if not standing:
            self._stand_since = None
        elif self._stand_since is None:
            self._stand_since = obs.now
        if (not PUSH_REACT or self.state not in (GUARD, LURK)
                or self._stand_since is None or obs.now - self._stand_since < PUSH_SETTLE_S):
            self._push_steps = 0
            self._slide_history = []
            return
        if self._detect_slide(obs) or obs.measured_linear is None:
            self._push_steps = 0
            return
        if abs(obs.measured_linear) <= PUSH_SPEED_M_S:
            self._push_steps = 0
            return
        self._push_steps += 1
        if self._push_steps < PUSH_CONFIRM_STEPS:
            return
        # 밀리는 방향 반대쪽에 미는 상대가 있다
        back = -math.copysign(PUSH_OPPONENT_M, obs.measured_linear)
        self.push_opponent = (obs.x + back * math.cos(obs.yaw), obs.y + back * math.sin(obs.yaw),
                              obs.now + PUSH_MEMORY_S)
        self.push_reason = f'엔코더 {obs.measured_linear:+.2f} m/s'
        self._push_steps = 0

    def _detect_slide(self, obs):
        """서 있는데 벽 보정 위치가 한쪽으로 계속 움직이면(바퀴가 안 돌고 미끄러짐) 미는 상대를 기억한다."""
        if obs.wall_offset is None:
            return False
        history = self._slide_history
        if not history or history[-1][1] != obs.wall_offset:
            history.append((obs.now, obs.wall_offset))
        while history and obs.now - history[0][0] > PUSH_SLIDE_WINDOW_S:
            history.pop(0)
        if len(history) < PUSH_SLIDE_MIN_UPDATES:
            return False
        dx = history[-1][1][0] - history[0][1][0]
        dy = history[-1][1][1] - history[0][1][1]
        moved = math.hypot(dx, dy)
        if moved <= PUSH_SLIDE_M:
            return False
        last_dx = history[-1][1][0] - history[-2][1][0]
        last_dy = history[-1][1][1] - history[-2][1][1]
        if last_dx * dx + last_dy * dy <= 0.0:
            return False
        # 밀려 간 쪽의 반대쪽에 미는 상대가 있다
        self.push_opponent = (obs.x - PUSH_OPPONENT_M * dx / moved,
                              obs.y - PUSH_OPPONENT_M * dy / moved, obs.now + PUSH_MEMORY_S)
        self.push_reason = f'LiDAR 미끄러짐 {moved * 100:.0f} cm'
        self._slide_history = []
        return True

    def _resume_state_after_retreat(self):
        # 실패 뒤 다시 잡았으면(LURK에서 재시도 성공) 잡은 상태가 먼저다
        if self._state_before_retreat in HOLDING_STATES:
            return self._state_before_retreat
        if self.gave_up:
            return LURK if LURK_AFTER_FAIL else SPIN
        return SEARCH

    def wants_opponents(self):
        """상대 위치(LiDAR)가 필요한 상태인지. 출발·첫 접근 중에는 판단을 늦추지 않게 계산하지 않는다."""
        return self.holding_cube() or self.state in (RECENTER, LURK, ATTACK) or self._retrying()

    def _retrying(self):
        """실패 뒤 LURK에서 다시 잡으러 가는 중인지 (이때도 GUARD·LURK처럼 상대를 피한다)."""
        return self.gave_up and LURK_AFTER_FAIL and self.state in (ALIGN, APPROACH)

    def _opponent_too_close(self, obs):
        """다시 잡으러 가는 중에 상대가 DODGE_START_M 안으로 왔는지 -> LURK로 돌아가 피한다."""
        return self._retrying() and any(
            math.dist((obs.x, obs.y), other) < DODGE_START_M for other in self._opponents(obs))

    def holding_cube(self):
        if self.state == RETREAT:
            return self._state_before_retreat in HOLDING_STATES
        return self.state in HOLDING_STATES

    def _run_state(self, obs):
        handlers = {
            TURN: self._pivot,
            DRIVE: self._drive,
            SEARCH: self._search,
            ALIGN: self._align,
            APPROACH: self._approach,
            PUSH: self._push,
            GRASP: self._grasp,
            HOLD: self._hold,
            GUARD: self._guard,
            BACKOFF: self._backoff,
            SPIN: self._spin,
            LURK: self._lurk,
            ATTACK: self._attack,
            RECENTER: self._recenter,
        }
        return handlers[self.state](obs)

    def _drive_toward(self, obs, target, speed, gain=HEADING_GAIN, max_angular=TURN_SPEED):
        """곡선 주행: 방향 오차만큼 돌면서 cos(오차)만큼 전진한다 (90도 넘게 틀어지면 제자리)."""
        error = heading_error_to(obs, target)
        predicted = error - self._last_angular * TURN_LEAD_S
        if predicted * error < 0.0:
            predicted = 0.0  # 예측으로 반대쪽으로 꺾지는 않는다
        angular = clamp(gain * predicted, max_angular)
        linear = speed * max(0.0, math.cos(error))
        return Command(linear=linear, angular=angular)

    def _pose_at(self, when):
        """
        주어진 시각 when(또는 바로 전)의 위치.

        그 근처 기록이 없으면(판단이 잠깐 멈췄던 구간) None. 2026-11-01 실기: 판단이 0.46 s 멈춘 뒤
        멈추기 전 위치로 큐브 위치를 계산해 13 cm 가깝게 잡고 일찍 기어갔다.
        """
        later = None
        for pose in reversed(self._poses):
            if pose[0] <= when:
                # when 앞뒤 기록 사이가 벌어졌으면 그 사이 위치를 모른다
                if later is not None and later[0] - pose[0] > POSE_HISTORY_GAP_S:
                    return None
                return pose[1:]
            later = pose
        # 기록이 아직 짧으면(막 시작) 가장 오래된 위치
        return self._poses[0][1:]

    def _update_target(self, obs):
        if not obs.detected or obs.area <= 0.0:
            return
        if obs.vision_stamp is not None:
            if obs.vision_stamp == self._vision_stamp:
                return
            self._vision_stamp = obs.vision_stamp
        if abs(self._last_angular) > TARGET_UPDATE_MAX_ANGULAR and self.target is not None:
            return
        # 가까우면(잡기 직전) 위치를 고정하고 곧게 들어간다. 큐브가 화면을 채워 x_offset이 흔들린다.
        if (self.state in (APPROACH, PUSH) and self.target is not None
                and self._area_best >= APPROACH_SLOW_AREA):
            return
        pose = self._pose_at(obs.now - CAMERA_LAG_S)
        if pose is None:
            return
        x, y, yaw = pose
        bearing = yaw - obs.x_offset * CAMERA_HALF_FOV_RAD
        distance = CAMERA_AHEAD_M + area_to_distance(obs.area)
        point = (x + distance * math.cos(bearing), y + distance * math.sin(bearing))
        # 경기장 밖으로 계산되면 큐브가 아니다 (2026-11-01 실기: 출발 전 오른쪽 경기장 밖 빨간 물체가
        # 면적 430으로 보여 큐브 위치에 섞였다)
        if not all(ARENA_MARGIN_M < c < ARENA_SIZE_M - ARENA_MARGIN_M for c in point):
            return
        self._target_updated_at = obs.now
        if self.target is None:
            self.target = point
        else:
            self.target = tuple(
                old + TARGET_SMOOTHING * (new - old) for old, new in zip(self.target, point))

    def _camera_distance(self, obs):
        """카메라에서 추정한 큐브 위치까지 거리 (엔코더 위치 기준이라 카메라 지연이 없다)."""
        return distance_to(obs, self.target) - CAMERA_AHEAD_M

    def _target_lost(self, obs):
        if obs.detected:
            self._target_seen_at = obs.now
            return False
        return self._target_seen_at is None or obs.now - self._target_seen_at > TARGET_LOST_S

    def _sees_target(self, obs):
        return obs.detected and obs.area >= SEE_TARGET_MIN_AREA

    def _start_approach(self, obs):
        self._area_best = 0.0
        self._area_best_at = (obs.x, obs.y)
        self._change_state(APPROACH, obs)
        return self._approach(obs)

    def _pivot(self, obs):
        """출발 제자리 회전: 중앙 방향으로 최대 속도로 돌고, 목표각 앞에서 일정하게 감속한다."""
        error = heading_error_to(obs, CENTER_M)
        # 지금 회전 속도로 더 돌 각도를 빼고 판단 (모터·IMU 지연으로 넘어가지 않게)
        predicted = error - self._last_angular * TURN_LEAD_S
        if abs(predicted) < PIVOT_DONE_RAD or predicted * error < 0.0:
            self._change_state(DRIVE, obs)
            return self._drive(obs)
        remaining = abs(predicted) - PIVOT_DONE_RAD
        speed = min(TURN_SPEED, math.sqrt(MIN_TURN_SPEED ** 2 + 2.0 * PIVOT_DECEL * remaining))
        return Command(angular=math.copysign(speed, predicted))

    def _drive(self, obs):
        # 가는 길에 카메라에 보이면 멈추지 않고 바로 카메라 접근으로
        if self._sees_target(obs) and abs(obs.x_offset) < APPROACH_MAX_OFFSET:
            return self._start_approach(obs)
        distance = distance_to(obs, CENTER_M)
        if distance <= VISION_HANDOFF_M:
            self._target_seen_at = obs.now
            self._change_state(ALIGN if obs.detected else SEARCH, obs)
            return self._run_state(obs)
        # 큐브가 중앙에 있다고 보고, 잡기 직전 감속 지점에서 APPROACH_SLOW_SPEED가 되도록 일정하게 감속
        remaining = distance - CAMERA_AHEAD_M - area_to_distance(APPROACH_SLOW_AREA)
        speed = min(DRIVE_SPEED, braking_speed(
            remaining, self._last_linear, APPROACH_SLOW_SPEED, DRIVE_DECEL))
        return self._drive_toward(obs, CENTER_M, speed)

    def _search(self, obs):
        if obs.detected:
            self._target_seen_at = obs.now
            self._change_state(ALIGN, obs)
            return self._align(obs)

        if self._search_step < 2:
            goal = self._search_start_yaw + (
                -SEARCH_SWEEP_RAD if self._search_step == 0 else SEARCH_SWEEP_RAD)
            error = normalize_angle(goal - obs.yaw)
            if abs(error) < SEARCH_SWEEP_DONE_RAD:
                self._search_step += 1
            else:
                self._search_last_yaw = obs.yaw
                return Command(angular=math.copysign(SEARCH_SPEED, error))
        self._search_turned += abs(normalize_angle(obs.yaw - self._search_last_yaw))
        self._search_last_yaw = obs.yaw
        if self._search_turned >= 2.0 * math.pi:
            # 한 바퀴 돌아도 없다: 멀면 중앙 쪽으로 다가가고, 가까우면 중앙 둘레를 따라 LURK_MOVE_RAD 옮겨
            # 다시 찾는다 (2026-11-03 19:42 실기: 중앙에서 1.0 m 자리에서 6초 넘게 제자리로만 돎)
            angle = angle_around_center((obs.x, obs.y))
            if distance_to(obs, CENTER_M) <= SEARCH_CENTER_M + GUARD_ARRIVE_M:
                angle += LURK_MOVE_RAD
            self._recenter_angle = angle
            self._change_state(RECENTER, obs)
            return self._recenter(obs)
        return Command(angular=self._search_direction * SEARCH_SPEED)

    def _recenter_goal(self, obs):
        """중앙에서 SEARCH_CENTER_M 떨어진 점: _recenter_angle 쪽 (없으면 지금 자리 쪽)."""
        angle = (angle_around_center((obs.x, obs.y)) if self._recenter_angle is None
                 else self._recenter_angle)
        return (CENTER_M[0] + SEARCH_CENTER_M * math.cos(angle),
                CENTER_M[1] + SEARCH_CENTER_M * math.sin(angle))

    def _recenter(self, obs):
        """한 바퀴 돌아도 없으면 중앙 쪽으로 다가가 다시 찾는다. 상대가 앞에 있으면 멈춘다."""
        if obs.detected:
            self._target_seen_at = obs.now
            self._change_state(ALIGN, obs)
            return self._align(obs)
        goal = self._recenter_goal(obs)
        if distance_to(obs, goal) < GUARD_ARRIVE_M:
            self._change_state(SEARCH, obs)
            return Command()
        return self._guard_drive(obs, goal, RECENTER_SPEED, self._opponents(obs))

    def _turn_speed(self, error):
        speed = max(MIN_TURN_SPEED, min(TURN_SPEED, HEADING_GAIN * abs(error)))
        return math.copysign(speed, error)

    def _search_again(self, obs):
        """대상을 놓쳤다. 한 번 실패한 뒤(다시 잡으러 가던 중)면 LURK로, 아니면 SEARCH."""
        if self.gave_up and LURK_AFTER_FAIL:
            return self._start_lurk(obs)
        self._change_state(SEARCH, obs)
        return Command()

    def _align(self, obs):
        if self._opponent_too_close(obs):
            return self._start_lurk(obs)
        if self._target_lost(obs) or self.target is None:
            return self._search_again(obs)

        error = heading_error_to(obs, self.target)
        if abs(error) < ALIGN_DONE_RAD:
            return self._start_approach(obs)
        return Command(angular=self._turn_speed(error))

    def _approach(self, obs):
        if obs.gripper_range is not None and obs.gripper_range <= GRAB_TOF_M:
            # 적외선이 큐브를 이미 집게 안에서 본다 (카메라 아래로 사라졌어도) -> 바로 닫는다
            return self._start_push(obs)
        if self._opponent_too_close(obs):
            return self._start_lurk(obs)
        if self._target_lost(obs) or self.target is None:
            if self._area_best >= APPROACH_SLOW_AREA:
                # 바로 앞까지 왔다가 카메라 아래로 사라졌다 = 집게 앞에 있다. 밀며 잡는다.
                return self._start_push(obs)
            return self._search_again(obs)

        if self._area_best_at is None:
            self._area_best_at = (obs.x, obs.y)
        if obs.detected and obs.area > self._area_best * AREA_GROW_RATIO:
            self._area_best = obs.area
            self._area_best_at = (obs.x, obs.y)
        stalled = (obs.detected and obs.area >= APPROACH_SLOW_AREA
                   and math.dist((obs.x, obs.y), self._area_best_at) >= APPROACH_STALL_M)
        # 가까이 와서(면적 APPROACH_SLOW_AREA, 천천히 가며 본 위치로 고정한 뒤) 고정한 큐브 위치까지
        # 카메라 거리가 GRAB_AREA 거리 안이면 잡는다 (엔코더 위치라 카메라 지연 없음).
        # 빨리 달릴 때 본 위치는 카메라 지연 차이 x 속도만큼 틀려서(0.8 m/s면 0.15 s에 12 cm) 쓰지 않는다.
        close_by_pose = (self._area_best >= APPROACH_SLOW_AREA
                         and self._camera_distance(obs) <= area_to_distance(GRAB_AREA))
        off_center = obs.detected and abs(obs.x_offset) > APPROACH_GRAB_MAX_OFFSET
        grab = close_by_pose or (obs.detected and (obs.area >= GRAB_AREA or stalled))
        # 적외선이 있으면 비켜 있어도 밀며 들어간다 (집게 안에 들어와야만 닫으므로 헛 집게질이 없다)
        if grab and (not off_center or self._tof_ok(obs)):
            return self._start_push(obs)

        # 잠깐 놓쳤을 때나 가까우면 천천히
        speed = APPROACH_SLOW_SPEED
        if obs.detected and self._area_best < APPROACH_SLOW_AREA:
            # 남은 거리 = 지금 카메라 면적으로 잰 거리 - 카메라 지연 동안 간 거리.
            # (위치 추정은 빨리 달리며 본 값이라 틀어지기 쉬워 감속에는 쓰지 않는다. 2026-11-01 실기:
            #  면적으로는 37 cm인데 추정 20 cm라 일찍 0.25 m/s로 0.85 s를 기어갔다)
            seen = area_to_distance(obs.area) - abs(self._last_linear) * CAMERA_LAG_S
            remaining = seen - area_to_distance(APPROACH_SLOW_AREA)
            speed = min(APPROACH_FAST_SPEED, braking_speed(
                remaining, self._last_linear, APPROACH_SLOW_SPEED, APPROACH_DECEL))
        command = self._drive_toward(
            obs, self.target, speed, APPROACH_HEADING_GAIN, APPROACH_MAX_ANGULAR)
        if self._area_best >= APPROACH_STRAIGHT_AREA:
            if obs.detected and abs(obs.x_offset) > APPROACH_STRAIGHT_MAX_OFFSET:
                # 가운데가 아니다 -> 서지 않고 가면서 카메라 방향으로 맞춘다 (큐브를 조금 밀어도 된다.
                # 2026-11-01: 서서 맞추니 큐브 앞에서 주춤했다)
                command = Command(linear=speed, angular=self._center_turn(obs))
            else:
                command = Command(linear=speed)
        return command

    def _tof_ok(self, obs):
        return obs.gripper_range is not None

    def _center_turn(self, obs):
        """카메라에 보이는 큐브를 화면 가운데로 가져오는 회전 (안 보이면 0)."""
        if not obs.detected:
            return 0.0
        bearing = -obs.x_offset * CAMERA_HALF_FOV_RAD
        return clamp(APPROACH_CENTER_GAIN * bearing, APPROACH_MAX_ANGULAR)

    def _start_push(self, obs):
        self._push_start = (obs.x, obs.y)
        self._grab_sent_at = None
        self._push_tof = self._tof_ok(obs)
        self._change_state(PUSH, obs)
        return self._push(obs)

    def _push(self, obs):
        """
        큐브를 밀며 집게 안으로 넣고 닫는다.

        적외선이 있으면: 큐브가 GRAB_TOF_M 안에 보이는 순간 닫고, GRAB_TOF_PUSH_MAX_M을 밀어도 안 들어오면
        닫지 않고 물러난다. 없으면: GRAB_PUSH_M만큼 곧게 더 들어가며 끝 GRAB_CLOSE_LEAD_M에서 닫는다.
        """
        pushed = math.hypot(obs.x - self._push_start[0], obs.y - self._push_start[1])
        if self._push_tof:
            if obs.gripper_range is not None and obs.gripper_range <= GRAB_TOF_M:
                self._change_state(GRASP, obs)
                self._grab_sent_at = obs.now
                return Command(gripper=GRIPPER_GRAB)
            pushed_too_long = obs.now - self.state_started_at > GRAB_PUSH_TIMEOUT_S
            if pushed >= GRAB_TOF_PUSH_MAX_M or pushed_too_long:
                return self._start_backoff(obs)
            return Command(linear=GRAB_PUSH_SPEED, angular=self._center_turn(obs))
        if pushed >= GRAB_PUSH_M or obs.now - self.state_started_at > GRAB_PUSH_TIMEOUT_S:
            closing = self._grab_sent_at is not None
            self._change_state(GRASP, obs)
            if closing:
                return Command()
            self._grab_sent_at = obs.now
            return Command(gripper=GRIPPER_GRAB)
        command = Command(linear=GRAB_PUSH_SPEED)
        if self._grab_sent_at is None and pushed >= GRAB_PUSH_M - GRAB_CLOSE_LEAD_M:
            command.gripper = GRIPPER_GRAB
            self._grab_sent_at = obs.now
        return command

    def _gripper_closed(self, obs):
        """집게가 닫혔다고 알린 뒤 GRIPPER_SETTLE_S가 지났거나, 최대 시간이 지났는지."""
        waited = obs.now - (self.state_started_at if self._grab_sent_at is None
                            else self._grab_sent_at)
        if waited >= GRASP_WAIT_S:
            return True
        if waited < GRIPPER_MIN_WAIT_S or obs.gripper_state != GRIPPER_CLOSED:
            return False
        if self._gripper_reached_at is None:
            self._gripper_reached_at = obs.now
        return obs.now - self._gripper_reached_at >= GRIPPER_SETTLE_S

    def _grasp(self, obs):
        if self._grasp_closed_at is None:
            if not self._gripper_closed(obs):
                return Command()
            self._grasp_closed_at = obs.now
            self._grab_sent_at = None

        if obs.holding is False:
            if obs.now - self._grasp_closed_at < HOLDING_CONFIRM_S:
                return Command()
            return self._start_backoff(obs)

        # TODO: 집게 상태를 알 수 없으면(None) 잡았다고 가정한다.
        if GUARD_ENABLED:
            self._guard_angle = GUARD_HOME_ANGLE
            self._dodging = False
            self._escaping = GUARD_ESCAPE_FIRST
            self._change_state(GUARD, obs)
            return Command()
        self._change_state(HOLD, obs)
        return Command()

    def _lost_cube(self, obs):
        """큐브를 놓쳤으면 집게를 열고 물러나 다시 찾는다. 놓치지 않았으면 None."""
        if obs.holding is not False:
            return None
        return self._start_backoff(obs)

    def _start_backoff(self, obs):
        if LURK_AFTER_FAIL:
            return self._start_lurk(obs)
        if SPIN_AFTER_FAIL:
            self.gave_up = True
            self._change_state(SPIN, obs)
            return self._spin(obs)
        self._target_seen_at = obs.now
        self._backoff_start = (obs.x, obs.y)
        self._change_state(BACKOFF, obs)
        return Command(gripper=GRIPPER_OPEN)

    def _backoff(self, obs):
        """집게를 열고 BACKOFF_M 곧게 후진한 뒤 SEARCH (탈락선은 RETREAT가 먼저 본다)."""
        moved = math.dist((obs.x, obs.y), self._backoff_start)
        if moved >= BACKOFF_M or obs.now - self.state_started_at > BACKOFF_TIMEOUT_S:
            self._target_seen_at = obs.now
            self._change_state(SEARCH, obs)
            return Command()
        return Command(linear=-BACKOFF_SPEED)

    def _spin(self, obs):
        """잡기 실패·놓침 뒤: 아무것도 쫓지 않고 제자리에서 계속 돈다 (SPIN_AFTER_FAIL)."""
        return Command(angular=SPIN_SPEED)

    def _start_lurk(self, obs):
        self.gave_up = True
        # 실패 전에 본 큐브 위치는 버린다 (큐브가 밀렸거나 상대가 가져갔을 수 있다)
        self.target = None
        # 지금 자리에서 가까운 원 위 점으로 빠진다 (큐브 쪽을 보고 있으므로 보통 후진)
        self._guard_angle = angle_around_center((obs.x, obs.y))
        self._dodging = False
        self._escaping = True
        self._lurk_turned = 0.0
        self._lurk_last_yaw = None
        self._change_state(LURK, obs)
        return self._lurk(obs)

    def _lurk(self, obs):
        """
        잡지 못했거나 놓친 뒤 (LURK_AFTER_FAIL): GUARD처럼 원 위에서 상대를 피하며 기다린다.

        원 위에 서 있는 동안은 제자리에서 돌며 카메라로 둘러보고, 피하는 중이 아니고 상대가 DODGE_CLEAR_M 밖일 때
        빨간 대상이 보이면 집게를 열고 다시 잡으러 간다 (상대가 잡고 있는 큐브로 보이면 가지 않는다).
        """
        if obs.gripper_range is not None and obs.gripper_range <= GRAB_TOF_M:
            # 큐브가 이미 집게 안에 있다 (ToF가 잠깐 '놓침'으로 봤다) -> 열지 않고 다시 닫는다
            return self._start_push(obs)
        opponents = self._opponents(obs)
        command = self._keep_away(obs, opponents)
        # 큐브 위치는 이번에 새로 계산된 것만 쓴다 (빨리 돌 때는 갱신하지 않아 옛 위치가 남는다)
        # 상대가 DODGE_CLEAR_M 밖일 때만 간다 (그 안이면 가다가 바로 다시 피하게 된다)
        clear = all(math.dist((obs.x, obs.y), other) > DODGE_CLEAR_M for other in opponents)
        # 원 위에 서서 둘러볼 때만 판단한다 (옮겨 가는 중에는 카메라 지연만큼 큐브 위치가 틀어져
        # 상대가 잡은 큐브를 빈 큐브로 볼 수 있다)
        retry = (command is None and clear and self._sees_target(obs) and self.target is not None
                 and self._target_updated_at == obs.now and obs.area < LURK_RETRY_MAX_AREA
                 and not self._target_near_opponent(opponents))
        if retry:
            self._target_seen_at = obs.now
            self._change_state(ALIGN, obs)
            command = self._align(obs)
            if command.gripper is None:
                command.gripper = GRIPPER_OPEN
            return command
        if (ATTACK_HELD_CUBE and command is None and self._sees_target(obs)
                and self.target is not None and self._target_updated_at == obs.now
                and self._target_near_opponent(opponents)):
            victim = min(opponents, key=lambda other: math.dist(self.target, other))
            return self._start_attack(obs, victim)
        if command is None:
            return self._lurk_look_around(obs)
        # 원 위 점으로 가거나 피하는 중: 새 자리에서 다시 한 바퀴를 센다
        self._lurk_turned = 0.0
        self._lurk_last_yaw = None
        return command

    def _lurk_look_around(self, obs):
        """원 위에 서서 제자리로 돌며 둘러본다. 한 바퀴 돌아도 못 찾으면 원을 따라 LURK_MOVE_RAD 옮긴다."""
        if self._lurk_last_yaw is not None:
            self._lurk_turned += abs(normalize_angle(obs.yaw - self._lurk_last_yaw))
        self._lurk_last_yaw = obs.yaw
        if self._lurk_turned >= LURK_TURN_BEFORE_MOVE_RAD:
            self._lurk_turned = 0.0
            self._lurk_last_yaw = None
            self._guard_angle += LURK_MOVE_RAD
            return Command()
        return Command(angular=SPIN_SPEED)

    def _start_attack(self, obs, victim):
        """상대 victim을 가장 가까운 노란 선 쪽으로 밀기 시작한다."""
        x, y = victim
        sides = ((x - SAFE_ZONE_MIN_M, (-1.0, 0.0)), (SAFE_ZONE_MAX_M - x, (1.0, 0.0)),
                 (y - SAFE_ZONE_MIN_M, (0.0, -1.0)), (SAFE_ZONE_MAX_M - y, (0.0, 1.0)))
        self.attack_dir = min(sides)[1]
        self.attack_victim = victim
        self._attack_seen_at = obs.now
        self._attack_ramming = False
        self._change_state(ATTACK, obs)
        return self._attack(obs)

    def _attack(self, obs):
        """자리 잡기(상대 뒤편) -> 선 쪽으로 들이밀기. 선 앞에서는 RETREAT가 먼저 끊는다."""
        opponents = self._opponents(obs)
        near = [other for other in opponents
                if math.dist(other, self.attack_victim) < ATTACK_TRACK_M]
        if near:
            self.attack_victim = min(near, key=lambda other: math.dist(other, self.attack_victim))
            self._attack_seen_at = obs.now
        vx, vy = self.attack_victim
        victim_margin = min(vx - SAFE_ZONE_MIN_M, SAFE_ZONE_MAX_M - vx,
                            vy - SAFE_ZONE_MIN_M, SAFE_ZONE_MAX_M - vy)
        if (obs.now - self._attack_seen_at > ATTACK_LOST_S
                or obs.now - self.state_started_at > ATTACK_TIMEOUT_S
                or victim_margin < ATTACK_DONE_MARGIN_M):
            return self._start_lurk(obs)
        dx, dy = self.attack_dir
        others = tuple(other for other in opponents if other != self.attack_victim)
        if not self._attack_ramming:
            staging = (vx - dx * ATTACK_STANDOFF_M, vy - dy * ATTACK_STANDOFF_M)
            if math.dist((obs.x, obs.y), staging) < ATTACK_START_M:
                self._attack_ramming = True
            else:
                # 우리가 상대 옆이나 선 쪽에 있으면 상대를 옆으로 돌아서 뒤편으로 간다
                rel_x, rel_y = obs.x - vx, obs.y - vy
                along = rel_x * dx + rel_y * dy
                lateral = -rel_x * dy + rel_y * dx
                goal = staging
                if along > -ATTACK_STANDOFF_M / 2 and abs(lateral) < ATTACK_DETOUR_M:
                    side = 1.0 if lateral >= 0.0 else -1.0
                    goal = (vx - dy * side * ATTACK_DETOUR_M - dx * ATTACK_STANDOFF_M / 2,
                            vy + dx * side * ATTACK_DETOUR_M - dy * ATTACK_STANDOFF_M / 2)
                return self._guard_drive(obs, goal, ATTACK_SPEED, others)
        aim = (vx + dx * ATTACK_AIM_AHEAD_M, vy + dy * ATTACK_AIM_AHEAD_M)
        error = heading_error_to(obs, aim)
        linear = ATTACK_SPEED if abs(error) < ATTACK_RAM_RAD else 0.0
        angular = clamp(GUARD_HEADING_GAIN * error, RETREAT_MAX_ANGULAR)
        return Command(linear=linear, angular=angular)

    def _target_near_opponent(self, opponents):
        return any(math.dist(self.target, other) < LURK_IGNORE_NEAR_OPPONENT_M
                   for other in opponents)

    def _hold(self, obs):
        # 규정 3.4.7.3: 끝날 때까지 연속으로 오래 확보한 팀이 유리하다.
        # 잡은 뒤에는 움직이지 않고 계속 유지한다.
        lost = self._lost_cube(obs)
        if lost is not None:
            return lost
        return Command()

    def _guard(self, obs):
        """
        잡은 큐브를 지킨다: 상대가 오면 원을 따라 반대쪽으로 피한다.

        GUARD_ESCAPE_FIRST면 먼저 원 위 점으로 빠져 거기 서 있고, 아니면 지금 자리에 서 있는다.
        """
        lost = self._lost_cube(obs)
        if lost is not None:
            return lost
        command = self._keep_away(obs, self._opponents(obs), stay=not GUARD_ESCAPE_FIRST)
        return Command() if command is None else command

    def _keep_away(self, obs, opponents, stay=False):
        """
        원 위 점으로 가고, 상대가 오면 원을 따라 반대쪽으로 피한다 (GUARD·LURK 공통).

        stay면 피할 때만 움직이고 그 밖에는 지금 자리에 서 있는다.
        서 있을 때(원 위 점에 도착, 또는 stay)는 None.
        """
        opponent_distance = min(
            (math.dist((obs.x, obs.y), other) for other in opponents), default=math.inf)
        if opponent_distance < DODGE_START_M:
            self._dodging = True
        elif opponent_distance > DODGE_CLEAR_M:
            if self._dodging:
                # 상대가 멀어졌다. 지금 자리에서 원 위에 선다.
                self._guard_angle = angle_around_center((obs.x, obs.y))
            self._dodging = False

        if self._dodging:
            self._escaping = False
            own_angle = angle_around_center((obs.x, obs.y))
            clearance = {direction: self._dodge_clearance(own_angle, direction, opponents)
                         for direction in (1.0, -1.0)}
            other = -self._dodge_direction
            if clearance[other] > clearance[self._dodge_direction] + DODGE_SWITCH_M:
                self._dodge_direction = other
            # 끼임 판단은 LiDAR로 본 상대만 (밀어서 안 상대는 바로 붙어 있어 늘 막혀 보인다. 미는 쪽에서 비켜 가야 한다)
            seen = tuple(o for o in opponents
                         if self.push_opponent is None or o != self.push_opponent[:2])
            pinned = opponent_distance < DODGE_BLOCKED_M
            if pinned:
                # 붙어 있는 상대의 반대쪽으로 원을 따라 빠진다 (양쪽 길이 다 막혀 보여도)
                nearest = min(opponents, key=lambda other: math.dist((obs.x, obs.y), other))
                side = math.atan2(math.sin(angle_around_center(nearest) - own_angle),
                                  math.cos(angle_around_center(nearest) - own_angle))
                if side != 0.0:
                    self._dodge_direction = -math.copysign(1.0, side)
            if (self._dodge_clearance(own_angle, self._dodge_direction, seen) < DODGE_BLOCKED_M
                    and not pinned):
                # 양쪽 다 막혔다. 안쪽은 상대가 모이고 바깥은 탈락선이라 서 있는다.
                # 단 상대가 이미 붙어 있으면(pinned) 서 있으면 그대로 밀려 나가므로 덜 막힌 쪽으로 빠진다
                # (2026-11-03 17:33 실기: 0.27 m에서 미는 상대와 반대쪽 0.45 m 물체 사이에서 2초 서 있다가
                # 탈락선 쪽으로 밀림). 가는 방향 바로 앞 상대는 _guard_drive가 멈춘다.
                return Command()
            self._guard_angle = own_angle + self._dodge_direction * DODGE_ARC_STEP_RAD

        if stay and not self._dodging:
            return None
        goal = guard_point(self._guard_angle)
        if not self._dodging and distance_to(obs, goal) < GUARD_ARRIVE_M:
            self._escaping = False
            return None
        speed = GUARD_ESCAPE_SPEED if self._escaping else GUARD_SPEED
        forward_only = self._escaping and self.holding_cube() and GUARD_ESCAPE_TURN_AROUND
        return self._guard_drive(obs, goal, speed, opponents, forward_only)

    def _opponents(self, obs):
        if obs.opponents:
            found = tuple(obs.opponents)
        else:
            found = () if obs.opponent is None else (obs.opponent,)
        if self.push_opponent is not None:
            # 밀어서 알게 된 상대 (LiDAR에 안 보여도)
            found = (self.push_opponent[:2],) + found
        return found

    @staticmethod
    def _dodge_clearance(own_angle, direction, opponents):
        """원을 따라 direction 쪽으로 갈 길에서 상대와 가장 가까워지는 거리."""
        steps = round(DODGE_LOOKAHEAD_RAD / DODGE_PATH_STEP_RAD)
        path = [guard_point(own_angle + direction * k * DODGE_PATH_STEP_RAD)
                for k in range(1, steps + 1)]
        return min((math.dist(point, other) for point in path for other in opponents),
                   default=math.inf)

    def _guard_drive(self, obs, goal, speed, opponents=(), forward_only=False):
        """
        큐브를 잡은 채 goal로 천천히 간다. 앞·뒤 중 덜 도는 쪽으로 간다.

        forward_only면 후진하지 않고 goal 쪽으로 돌아서 전진한다 (GUARD_ESCAPE_TURN_AROUND).
        가는 방향 바로 앞에 상대가 있으면 들이받지 않게 멈추고 방향만 맞춘다.
        """
        error = heading_error_to(obs, goal)
        if forward_only:
            self._guard_reverse = False
        elif self._guard_reverse:
            self._guard_reverse = abs(error) > math.pi / 2 - GUARD_REVERSE_HYSTERESIS_RAD
        else:
            self._guard_reverse = abs(error) > math.pi / 2 + GUARD_REVERSE_HYSTERESIS_RAD
        if self._guard_reverse:
            error = normalize_angle(error + math.pi)

        # 큐브를 잡고 있으면 부드럽게(GUARD_ACCEL) 서고, 없으면(LURK) 최대 감속으로 선다
        decel = GUARD_ACCEL if self.holding_cube() else LINEAR_DECEL
        remaining = distance_to(obs, goal)
        speed = min(speed, braking_speed(remaining, self._last_linear, 0.0, decel))
        linear = speed * max(0.0, math.cos(error))
        if forward_only and abs(error) > GUARD_TURN_AROUND_DONE_RAD:
            linear = 0.0
        if self._guard_reverse:
            linear = -linear
        angular = clamp(GUARD_HEADING_GAIN * error, GUARD_MAX_ANGULAR)

        current = abs(self._last_linear)
        block = GUARD_BLOCK_M + current * MOTOR_LAG_S + current ** 2 / (2.0 * decel)
        heading = obs.yaw + (math.pi if self._guard_reverse else 0.0)
        for other in opponents:
            if math.dist((obs.x, obs.y), other) >= block:
                continue
            toward = math.atan2(other[1] - obs.y, other[0] - obs.x)
            if abs(normalize_angle(toward - heading)) < GUARD_BLOCK_RAD:
                linear = 0.0
        return Command(linear=linear, angular=angular)
