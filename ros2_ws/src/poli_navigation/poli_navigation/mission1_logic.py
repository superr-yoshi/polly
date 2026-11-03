"""
임무 1 (자율탐색 및 요구조자 구출) 판단 로직.

ROS2와 무관한 순수 Python 모듈이다.
센서 값(Observation)을 받아 지금 할 동작(Command)을 결정한다.
토픽 연결은 별도 ROS2 노드에서 한다.

전략 (docs/mission_strategy.md 2장):
  [EXPLORE] SCAN -> TURN -> DRIVE -> SCAN ... (중앙 옆 격자까지 한 칸씩)
            FACE -> SEARCH / ALIGN -> APPROACH -> PUSH -> GRASP
  [RETURN]  SCAN -> TURN -> DRIVE -> SCAN ... (출발 격자까지 한 칸씩)
            RELEASE -> DONE
  탐색은 poli_maze의 거리 + 방문 횟수 점수로 이웃 한 칸을 선택한다.
  복귀는 실제 방문한 칸 안에서 출발점까지 경로를 계산한다.
  다음 칸이 뒤쪽이면 돌지 않고 후진한다 (출발 격자에 넣는 마지막 칸 제외).

좌표:
- 경기장 좌표(m), 왼쪽 아래 모서리가 (0, 0)
- 장애물이 점대칭이므로 로봇은 항상 "(9, 1) 격자 중심에서 +y 방향을 보고
  출발"한 것으로 계산한다. (grid_map.py, scan_to_grid.py와 같다)
"""

from collections import deque
from dataclasses import dataclass
import math
from typing import Optional

from poli_navigation import scan_to_grid
from poli_navigation.grid_map import (
    BLOCKED,
    cell_center_m,
    CELL_SIZE_M,
    CENTER,
    FREE,
    GridMap,
    MISSION1_START,
    UNKNOWN,
)
from poli_navigation.mission1_maze import Mission1Maze
from poli_navigation.mission2_logic import (
    clamp,
    Command,
    distance_to,
    GRIPPER_GRAB,
    GRIPPER_OPEN,
    heading_error_to,
    ramp,
)
from poli_navigation.scan_to_grid import (
    GUARD_AHEAD_M,
    GUARD_MIN_POINTS,
    nearest_ahead,
    neighbor_states,
    obstacle_ahead,
    point_to_cell,
    rect_ray_counts,
    scan_to_cells,
    wall_in_direction,
)
from poli_navigation.turn_pulse import HW_MIN_WHEEL_SPEED, TurnPulse


CENTER_M = cell_center_m(CENTER)

# 2026-10-31 사용자 실측: 바퀴 축 중앙 ~ 잡은 큐브 중심 약 13 cm.
# TODO_MEASURE: 근삿값이므로 출발 칸 안에 블록 전체가 들어오는지 실기 재확인.
# 큐브는 들지 않고 바닥에 끌고 간다.
# 출발 격자에서 놓을 때 로봇 중심이 아니라 큐브가 격자 가운데에 오게 한다.
GRIPPER_REACH_M = 0.13

# SIM_ONLY: 멈춘 뒤 스캔할 때까지 기다리는 시간 (움직이며 찍힌 스캔을 쓰지 않게)
# TODO_MEASURE: 최소 안정 시간. 실물 노드는 정지 확인 후 이 시간이 지난 스캔만 쓴다.
# 2026-10-31 0.2 -> 0.1: 칸마다 판단이 오래 걸림. 차체 흔들림은 노드가 IMU·엔코더로 따로 확인한다.
# 멈춘 뒤 지도가 이상하게(통로가 막힘으로) 바뀌면 다시 늘린다.
# 2026-10-31 0.1 -> 0.05: 칸마다 판단 0.43초 중 스캔 대기를 줄인다 (팀 요청). 차체 흔들림은 노드가 따로 본다
SCAN_SETTLE_S = 0.05
# 주행 중 다음 칸이 막혀 보이는 스캔이 이만큼 연속이어야 멈춘다.
# 출발 순간 차체가 숙여져 LiDAR가 바닥·장애물 윗변을 앞칸 장애물로 보는 것을 거른다 (2026-10-31 실기)
DRIVE_BLOCK_SCANS = 3
# 주행 중 지도(격자) 방식 안전 확인 켜기/끄기. 2026-10-31 실기에서 엉뚱하게 서서 끔.
# 2026-11-01부터는 진행선 앞만 보는 앞 벽 감시(_wall_before_target)를 모든 전진에 쓴다 (GUARD_REPEAT_MAX)
DRIVE_BLOCK_CHECK = False
# 탐색 중 카메라에 대상이 이 크기 이상으로 보이면 (5,5) 옆이 아니어도 잡으러 간다 (2026-10-31 팀 결정).
# 너무 작으면(멀면) 경기장 밖 빨간 물체일 수 있어 무시한다. 50 cm에서 약 9000 (TODO_MEASURE)
# 단, 칸 가운데에 서 있고 대상이 상하좌우 축 방향(±TARGET_AXIS_TOL_RAD)이며 그 사이 칸이 막히지 않았을 때만.
# 2026-10-31 실기: 주행 중 보이자마자 대각선으로 다가가다 옆 장애물에 박았다 (통로 여유 한쪽 약 4.7 cm).
SEE_TARGET_MIN_AREA = 2000.0
TARGET_AXIS_TOL_RAD = math.radians(15.0)
# 큐브가 통로 가운데 선에서 옆으로 이보다 멀면 (거리 x 각도) 접근하지 않는다.
# 2026-10-31 실기 (17:16): 67 cm 앞 14도(옆 약 16 cm) 큐브에 접근하다 화면 밖으로 놓침. TODO_MEASURE
TARGET_LATERAL_MAX_M = 0.06
# 카메라 x_offset 1.0 = 화면 끝 = 약 33도 (TODO_MEASURE: 실제 화각)
CAMERA_HALF_FOV_RAD = math.radians(33.0)
# 대상 거리 추정: 면적은 거리 제곱에 반비례, 50 cm에서 약 9000 (TODO_MEASURE)
TARGET_AREA_AT_50CM = 9000.0
# 큐브가 보였는데 지금 칸에서 곧게 다가갈 수 없으면, 위치를 기억해 두고 큐브 칸의 옆 칸으로 가서 잡는다
# (2026-10-31 팀 결정: 중앙에 없어도 보이면 잡으러 간다). 큐브가 칸 안에서 치우친 방향이 진행 방향과
# 같아지는 옆 칸(진행 방향과 직각인 치우침이 이 값 이하)을 고른다. TODO_MEASURE
CUBE_NEIGHBOR_LATERAL_MAX_M = 0.08
# 카메라가 바퀴 축보다 앞에 있는 거리 (큐브 위치 추정용). TODO_MEASURE
CAMERA_AHEAD_M = 0.10
# 탐색 단계에서 대상을 놓치고 이 시간 동안 못 찾으면 다시 격자 탐색으로 돌아간다 (SEARCH_SPEED로 한 바퀴 남짓).
# 찾지 못한 중앙 옆 칸은 목표에서 뺀다 (2026-10-31: 큐브가 중앙에 없으면 FACE·SEARCH를 끝없이 반복).
SEARCH_GIVE_UP_S = 7.0

# 격자 가운데에 도착했다고 보는 거리, 가운데에서 벗어났다고 보는 거리
ARRIVE_TOLERANCE_M = 0.03
# 2026-10-31 0.10 -> 0.15: 상자가 40 cm 격자에서 조금만 어긋나도 가운데로 되돌아가며 후진했다 (팀 요청)
CELL_CENTER_TOLERANCE_M = 0.15

# 속도와 제어 값 (임무 2와 같은 기준, turn_pulse.py 참고)
# 조원 A 결정: 주행은 0.25 m/s. 하드웨어가 0.25 m/s 아래로 내지 않으므로
# 감속(MIN_DRIVE_SPEED)·느린 접근은 실제로는 0.25 m/s가 된다.
# 2026-10-31 0.6 -> 0.8 (전부 써서 빠르게). hardware.yaml max_linear와 같게. 바퀴 약 235 rpm (정격 250)
# 2026-11-01 0.8 -> 0.6: 여러 칸 직진에서 속도가 붙으면 거리·방향 오차가 커져 옆을 박거나 빗나감 (팀).
# 10-31 전체 임무 36.6 s 기록도 0.6이었다.
DRIVE_SPEED = 0.6
# 복귀(큐브를 끌고 감)는 빠를 필요 없이 적당히 (2026-11-01 팀). 출발을 부드럽게 해 큐브가 빠지지 않게
# (임무 2 GUARD도 큐브를 끌 때 가속 0.6). 감속은 큐브를 닫힌 집게 쪽으로 미니 그대로 (도착 후 바로 섬).
# TODO_MEASURE: 큐브가 흔들리면 더 낮춘다
# 2026-11-03 0.45 -> 0.35 (팀: 복귀는 조금 느려도 안전하게. 옆으로 밀림·지나침이 줄어든다)
RETURN_DRIVE_SPEED = 0.35
# 복귀를 안전하게 (2026-11-03 팀): 꺾는 칸에 도착하면 서서 스캔해(벽으로 위치를 바로잡고) 계획하고,
# 새로 갈 방향 기준 옆으로 RETURN_RECENTER_M 넘게 비켜 있으면 먼저 칸 가운데(RECENTER_ARRIVE_M 안)로 옮긴 뒤 돈다.
# 2026-11-03 22:46·22:48 실기: 꺾는 칸에 3~4 cm 덜 와서 그대로 돌아 새 통로 입구 모서리에 걸렸다 (여유 4.7 cm).
# 꺾는 칸마다 약 0.3~0.5 s 더 걸린다.
RETURN_SAFE_TURNS = True
RETURN_RECENTER_M = 0.02
RECENTER_ARRIVE_M = 0.01
RETURN_LINEAR_ACCEL = 0.6
RETURN_LINEAR_DECEL = 2.0  # LINEAR_DECEL과 같게
# 복귀 회전은 부드럽게 (2026-11-01 23:11 실기: 잡은 큐브가 제자리 회전 직후 화면 면적 35000 -> 16000~23000으로
# 집게 안에서 틀어지고, 복귀 중 벽에 몇 번 부딪힘). 바닥에 끌리는 큐브가 빠른 회전에 비틀리지 않게
# 최고 회전 속도·각가속도를 낮추고, 방향을 거의 다 맞춘 뒤 출발한다. TODO_MEASURE: 큐브가 계속 틀어지면 더 낮춘다
# 2026-11-02 팀: 복귀 회전도 요구조자 찾으러 갈 때(탐색)와 똑같이 (2.0 rad/s·6 rad/s^2·6도는 너무 느림).
# 탐색 값 TURN_SPEED·ANGULAR_ACCEL·TURN_EXIT_RAD와 같게 둔다. 큐브가 다시 틀어지면 예전 값으로 낮춘다.
RETURN_TURN_SPEED = 3.5       # rad/s (예전 2.0)
RETURN_ANGULAR_ACCEL = 15.0   # rad/s^2 (예전 6.0)
RETURN_TURN_EXIT_RAD = math.radians(10.0)  # (예전 6도)
MIN_DRIVE_SPEED = HW_MIN_WHEEL_SPEED
# 도착 감속 (m/s^2): 남은 거리 d에서 sqrt(MIN^2 + 2 * DRIVE_DECEL * d)로 일정하게 줄인다.
# 예전 1.5 * d는 끝에서 너무 오래 기어가고 0.08 m/s에서 뚝 멈췄다 (2026-10-31 실기).
# 40 cm 한 칸 약 2.0초 -> 약 1.6초. TODO_MEASURE: 도착 위치가 밀리면 낮춘다
DRIVE_DECEL = 1.2  # 2026-10-31 0.8 -> 1.2
# 모든 상태 공통 가감속 제한 (step()의 _limit_accel). 급출발·급정지 방지.
# 속도를 줄일 때는 더 빨리 줄여 계획하지 않은 정지(대상 발견 등)에서 덜 밀리게 한다.
# TODO_MEASURE: 출발할 때 앞이 들리거나 바퀴가 헛돌면 LINEAR_ACCEL을 낮춘다
LINEAR_ACCEL = 1.2   # m/s^2, 2026-10-31 0.8 -> 1.2. TODO_MEASURE: 바퀴 헛돎·잡은 큐브 놓침이 보이면 낮춘다
LINEAR_DECEL = 2.0   # m/s^2, 0.8 m/s -> 0 약 0.4초 (약 16 cm)
ANGULAR_ACCEL = 15.0  # rad/s^2, 2026-10-31 10 -> 15 (0 -> 3.5 rad/s 약 0.23초)
# 이만큼 판단이 끊기면 그동안 노드가 정지 명령을 보냈다고 보고 0에서 다시 가속한다.
ACCEL_RESET_S = 0.15
# 카메라 접근: 멀 때는 빠르게, 큐브가 커 보이면 천천히 (면적 판단 지연으로 지나치지 않게)
APPROACH_SPEED = 0.25  # 2026-10-31 0.2 -> 0.25 (가까우면 APPROACH_SLOW_SPEED)
APPROACH_SLOW_SPEED = 0.12
APPROACH_SLOW_AREA = 20000.0  # TODO_MEASURE: 잡는 위치 약 15 cm 전
# 2026-10-31 1.2 -> 1.8 (회전이 느리고 끊김). hardware.yaml max_angular 2.0 안.
# TODO_MEASURE: 목표각을 지나쳐 되돌아 돌면 TURN_SPEED 또는 TURN_DECEL을 낮춘다 (IMU 지연)
# 2026-10-31 2.5 -> 3.5. hardware.yaml max_angular 3.6 안.
# TODO_MEASURE: '회전 후 오차'가 계속 음수(넘침)로 크면 TURN_LEAD_S를 올린다
TURN_SPEED = 3.5
TURN_DECEL = 10.0  # rad/s^2 (2026-10-31 6 -> 10), 목표각 앞에서 일정하게 감속해 MIN_TURN_SPEED로 도착
# 연속 회전 감속 하한. 바퀴 속도 약 0.084 m/s = 하드웨어 최저 속도(0.08) 근처.
# 더 낮추면 바퀴가 섰다 갔다 하며 끊겨 돈다.
MIN_TURN_SPEED = 0.6
# 2026-10-31 0.6 -> 1.0 (잡기 전체가 느림). 카메라 20 Hz면 한 번에 약 3도라 놓치지 않는다.
SEARCH_SPEED = 1.3  # 2026-10-31 1.0 -> 1.3
HEADING_GAIN = 1.5
# 진행선에서 옆으로 벗어났을 때 돌아오는 방향 = 옆 오차 x 이득, 최대 각도
DRIVE_LATERAL_GAIN = 2.0  # rad/m (5 cm -> 약 6도)
DRIVE_MAX_CORRECTION_RAD = math.radians(15.0)
# 직진 중 방향 오차가 DRIVE_FULL_SPEED_RAD를 넘으면 속도를 줄이고(DRIVE_PIVOT_RAD에서 0), 그 이상이면 제자리에서 돈다.
# 2026-11-02 20:03 실기: 복귀 중 곡선·회전 뒤 방향 오차 8~20도인 채 최고 속도로 출발해 사선으로 감
# (예전에는 45도 안이면 최고 속도, HEADING_GAIN 1.5라 20도를 맞추는 동안 몇 cm 옆으로 감).
# 2026-11-02 5 -> 10도, 25 -> 30도, 진행선 기준 방향 오차만 본다: 20:3x 실기에서 주행 중 벽 보정(2~3 cm)으로
# 진행선 복귀 각도가 5도를 넘을 때마다 중간중간 감속했다.
DRIVE_FULL_SPEED_RAD = math.radians(10.0)
DRIVE_PIVOT_RAD = math.radians(30.0)
# 직진 중 방향 맞추는 이득 (HEADING_GAIN 1.5는 20도를 맞추는 데 약 1초). 시뮬레이션(모터 지연 0.12 s):
# 20도 틀어진 출발에서 옆 오차 3.8 -> 1.8 cm, 좌우 흔들림 없음 (3.5까지도 없음).
# TODO_MEASURE: 직진 중 좌우로 흔들리면 낮춘다
DRIVE_HEADING_GAIN = 2.5
# 한 번 주행에서 진행선 옆으로 이보다 많이 벗어났으면 도착할 때 로그로 남긴다 (사선 주행 확인용)
DRIVE_LATERAL_LOG_M = 0.03
# 진행선에서 이보다 벗어나면 DRIVE_LATERAL_SLOW_SPEED로 줄인다: 빠르면 약한 쪽 바퀴가 돌 여유가 없어 못 돌아온다.
# 2026-11-02 21:21·21:55 실기: 오른쪽 바깥 벽을 따라 오른쪽으로 19·23 cm 밀려 벽에 박음 (방향을 고치라는
# 명령은 나갔는데 0.6 m/s에서 한쪽 바퀴가 그만큼 더 빨리 못 돎). TODO_MEASURE
DRIVE_LATERAL_SLOW_M = 0.05
DRIVE_LATERAL_SLOW_SPEED = 0.35
# 위 감속과 옆 벽 감시 감속(SIDE_GUARD_SPEED)을 쓸지. 2026-11-02 팀: 감속 없이 -> 끔. 옆 벽 감시를 일찍(0.19 m)
# 시작해 작게 자주 틀어 바퀴 속도 차이를 작게 둔다. 그래도 벽에 박으면 True로 켠다.
DRIVE_SLOW_FOR_WALLS = False
# 옆 벽 감시: 직진 중 차체 옆(축 기준 앞뒤 SIDE_GUARD_BACK_M ~ SIDE_GUARD_FRONT_M)의 LiDAR 점이 축에서 옆으로
# SIDE_GUARD_M보다 가까우면 반대쪽으로 SIDE_GUARD_GAIN x 들어온 거리만큼 더 틀고 SIDE_GUARD_SPEED로 줄인다.
# 차체 반폭 0.1525 m(바퀴 포함 30.5 cm), 통로 가운데면 벽은 0.2 m. 2026-11-02 21:57 실기: 오른쪽 벽이 축에서
# 0.15~0.17 m(바퀴와 0~2 cm)인 채 달리다 사선으로 박음. 축에서 SIDE_GUARD_SELF_M 안 점은 우리 부품으로 본다.
# 2026-11-02 0.18 -> 0.19: 통로 가운데(벽 0.2 m)에서 1 cm만 비켜도 작게 틀기 시작 (감속 없이 일찍 고친다)
SIDE_GUARD_M = 0.19
# 2026-11-02 끔: 22:1x 실기에서 지그재그로 가며 벽에 박음. 연습장 통로는 좌우 벽이 축에서 17~19 cm라 감시 거리(19 cm)
# 안에 늘 양쪽 벽이 있고, LiDAR 흔들림(10 Hz, ±1 cm)마다 좌우로 틀었다. 'Drive:' 로그(옆 거리)는 그대로 남긴다.
SIDE_GUARD_ENABLED = False
# 2026-11-02 20 -> 12: 22:08 실기에서 주행 방향이 65~116도로 흔들림 (집게 날 오판과 겹침). 3 cm면 0.36 rad/s 더
SIDE_GUARD_GAIN = 12.0       # rad/s 당 m
SIDE_GUARD_SPEED = 0.3
SIDE_GUARD_BACK_M = -0.15
# 2026-11-02 0.20 -> 0.05 (차체 앞 끝): 22:08 실기에서 열린 집게 날(축 앞 +10~17 cm, 옆 약 12 cm)을 좌우 벽 12 cm로 보고
# 1.4 rad/s씩 틀었다. 차체 옆면(축 16 cm 뒤 ~ 5 cm 앞)만 본다
SIDE_GUARD_FRONT_M = 0.05
SIDE_GUARD_SELF_M = 0.12

# TODO_MEASURE: 펄스 한 번 회전량(turn_pulse.PULSE_S)보다 범위(±)가 넓어야 멈출 수 있다
TURN_TOLERANCE_RAD = math.radians(5.0)
# 칸 사이 TURN은 남은 오차가 이 안이면 끝내고 나머지는 달리면서 맞춘다 (2026-10-31: 회전당 약 0.15~0.2초 단축)
TURN_EXIT_RAD = math.radians(10.0)
# 회전 속도에 맞춘 정지 보정: 지금 각속도로 이 시간만큼 더 돈다고 보고 미리 감속·정지한다.
# (명령 감속 약 0.1초 + Mega·모터 지연). 2026-10-31 회전을 빠르게 한 뒤 목표각을 넘겨 벽에 박음.
# TODO_MEASURE: 로그 '회전 후 오차'가 목표를 넘친 쪽(같은 부호로 계속 큼)이면 늘리고, 덜 돈 쪽이면 줄인다
TURN_LEAD_S = 0.12
# 회전을 끝내고 이 시간 뒤 방향 오차를 로그로 남긴다 (TURN_LEAD_S 조정용)
TURN_CHECK_S = 0.3
# 주행 중 판단 (2026-10-31 팀 요청: 판단할 때 멈추지 말 것).
# 칸 가운데에 가까워지면 ROLL_SPEED까지만 줄이고, 그 속도로 일정하게 달리며 찍힌 스캔으로 다음 칸을 정한다.
# 다음 칸이 직진이면 멈추지 않고 이어 가고, 회전이면 가운데에 서자마자 (판단 대기 없이) 돈다.
# 조건에 맞는 스캔을 못 받으면 예전처럼 서서 판단한다 (SCAN).
# 2026-10-31 실기 (16:54): 끔. 달리는 동안 위치 추정이 늦어 칸마다 목표를 지나치고(주행 중 보정이 늘
# 진행 방향 +3 cm 상한), 지도가 어긋나 왕복·STUCK. 서서 판단할 때는 앞뒤 오차 평균 1 cm 이하.
# TODO: odom 지연을 재고 보정한 뒤 다시 켠다.
# 2026-11-01 저녁: 대회 모드도 켬 (팀: 최대한 안 멈추고). 그날 연습은 모두 켠 상태(--test-noview)로 했다.
# 옆 칸 오판은 막힘 기준 70%와 LiDAR에서 이웃 벽면을 겨누는 판단(scan_to_grid.neighbor_states)으로 막는다.
ROLLING_DECISION = True
# TODO_MEASURE: 판단 속도. 스캔 한 바퀴(0.1초) 동안 2 cm 움직인다. 지도가 이상하면 낮춘다
# 2026-11-01 0.2 -> 0.35, 0.14 -> 0.20: (5,5)에서 멀어지는 직진은 칸마다 이 속도로 줄여 판단해 칸마다 주춤했다
# (팀). 벽 판단 기준을 콘 광선 50%로 올려(scan_to_grid) 조금 흐린 스캔으로도 판단이 안정적이다.
# 0.35 m/s면 스캔 한 바퀴(0.1 s) 동안 3.5 cm. TODO_MEASURE: 지도가 다시 어긋나면 낮춘다
ROLL_SPEED = 0.35
# 2026-11-01 0.20 -> 0.28, 0.10 -> 0.18: 0.35 m/s에서는 창이 좁아 스캔이 도착할 때 이미 가운데라 칸마다 서서
# 스캔했다 (21:30 실기, 칸마다 0.4 s 주춤). 창은 칸 안(가운데 0.2 m 앞까지), 일정 속도는 그보다 먼저 시작한다
ROLL_ZONE_M = 0.28     # 칸 가운데 이 거리 앞부터 ROLL_SPEED로 달린다
ROLL_WINDOW_M = 0.18   # 스캔을 찍은 위치가 칸 가운데에서 이 안이어야 판단에 쓴다
# 속도를 바꾼(가감속) 뒤 차체 숙여짐이 가라앉을 때까지. 이 시간 뒤 시작한 스캔만 쓴다. TODO_MEASURE
ROLL_SETTLE_S = 0.15
ROLL_STRAIGHT_RAD = math.radians(10.0)  # 다음 칸 방향이 이 안이면 직진으로 보고 멈추지 않는다
# 스캔 한 바퀴 시간을 모를 때 (LaserScan.scan_time이 0)
DEFAULT_SCAN_DURATION_S = 0.1
# 접근은 축 방향으로 먼저 돌고, 통로 가운데 선에서 APPROACH_LATERAL_MAX_M 안으로만 다닌다.
# 그 안에서는 카메라로 큐브 쪽으로 방향을 맞추되 축에서 APPROACH_MAX_HEADING_RAD까지만 튼다.
# TODO_MEASURE: 통로 여유(한쪽 약 4.7 cm) 안. 큐브가 통로 가운데에서 많이 벗어나 있으면 못 잡을 수 있다
APPROACH_LATERAL_MAX_M = 0.035  # 2026-10-31 0.02 -> 0.035 (조금 비켜난 큐브도 잡게)
APPROACH_LATERAL_GAIN = 4.0  # rad/m, 가운데 선으로 돌아오는 방향 = 옆 오차 x 이 값 (최대 아래 각도)
APPROACH_MAX_HEADING_RAD = math.radians(10.0)
# 접근 중 카메라가 대상을 이 시간 넘게 놓치면 칸 가운데로 돌아가 다시 판단한다
APPROACH_LOST_S = 0.5
# 면적이 APPROACH_SLOW_AREA를 넘은 뒤 이만큼 더 갔는데 면적이 AREA_GROW_RATIO만큼도 안 커지면
# 큐브가 집게에 닿아 같이 밀리는 것으로 보고 잡는다 (GRAB_AREA에 못 미쳐도).
# 2026-10-31 실기 (17:45): 면적이 25000에서 안 커진 채 큐브를 80 cm 밀고 감. TODO_MEASURE
APPROACH_STALL_M = 0.06
# 두 칸 한 번에 갈 때 목표 칸 가운데보다 앞(CHAIN_WALL_MARGIN_M 여유)에 벽이 보이면 멈춘다.
# 막힌 두 번째 칸의 면은 목표 가운데 0.2 m 앞, 막다른 칸 너머 벽은 0.2 m 뒤에 있다.
# LiDAR가 상자 위로 넘어가지 않는 거리(축에서 CHAIN_GUARD_RANGE_M) 안의 점만 쓴다. TODO_MEASURE
CHAIN_WALL_MARGIN_M = 0.12
# 두 칸 묶기 전 벽 검사의 광선 범위. ±15도면 옆 상자 모서리(약 18도, 0.6 m)가 위치 오차로 들어와
# 사진 1->3번 같은 통로에서 묶지 않았다 (시뮬레이션 27가지 중 6가지). ±8도면 0가지 (2026-10-31)
CHAIN_CHECK_CONE_RAD = math.radians(8.0)
# 두 칸 주행 감시가 보는 폭 (진행선 ±). 막힌 두 번째 칸의 정면 벽은 통로 폭 전체에 걸쳐 있어 잡히고,
# 통로 옆 상자 면(±0.2 m, 좁은 통로·로봇이 비켜 있으면 더 안쪽)은 빠진다 (2026-10-31 실기: 옆면으로 멈춤)
CHAIN_GUARD_HALF_WIDTH_M = 0.10
CHAIN_GUARD_RANGE_M = 0.6
# 후진으로 칸 가운데에 설 때 목표 칸 너머(차체 뒤) 벽이 LiDAR에 보이면 그 벽까지 거리로도 도착을 본다.
# 축 ~ 차체 뒤끝 약 0.16 m라 가운데에 서면 뒤 벽까지 4 cm뿐이다. 2026-11-03 14:50 실기: 큐브를 잡고
# (5,5)->(7,5) 후진 중 엔코더 위치가 3 cm 넘게 덜 간 것으로 나와(벽 보정 상한 +3 cm) (8,5) 상자에 박았다.
# 벽으로 본 남은 거리가 엔코더로 본 것과 REVERSE_WALL_MATCH_M 넘게 다르면 목표 칸 너머 벽이 아닌 것으로 본다.
REVERSE_WALL_MATCH_M = 0.10
# 목표 칸 너머가 이미 막힘(또는 외벽)으로 알려져 있으면 그 벽이 확실하므로 더 넓게 믿고, 마지막 구간은 천천히 간다.
# 2026-11-03 16:26 실기: 큐브를 잡고 (6,5)->(7,5) 후진. 엔코더가 약 10 cm 덜 간 것으로 나와(주행 중 벽 보정은
# 오차의 절반, 한 번에 3 cm까지) 10 cm 창에 늦게 들어왔고, 0.32 m/s에서 서느라 밀려 (8,5) 상자에 닿았다.
REVERSE_WALL_MATCH_KNOWN_M = 0.25
REVERSE_WALL_SLOW_ZONE_M = 0.25
REVERSE_WALL_SLOW_SPEED = 0.2  # TODO_MEASURE: 이 속도에서 서는 거리(지연 + 감속)가 뒤 여유 4 cm 안인지
# 후진으로 칸 가운데에 설 때는 가운데 ARRIVE_TOLERANCE_M(3 cm) 앞이 아니라 이 안까지 가서 선다.
# 2026-11-03 22:46·22:48 실기: (5,5)->(5,6), (6,1)->(7,1) 후진이 3~4 cm 덜 돼(뒤 벽 앞에서는 느려서 관성으로
# 더 밀리지 않음) 돌고 나서 새 통로 한쪽으로 치우쳐 입구 모서리에 걸렸다 (통로 여유 한쪽 4.7 cm).
# 뒤가 벽이면 그 거리로 위치를 알고 느리게 들어가므로 뒤 벽(가운데에서 4 cm 여유)에 닿지 않고, 뒤가 비었으면
# 박을 것이 없다. 전진은 앞 벽과 집게 끝 사이가 3 cm뿐이라 ARRIVE_TOLERANCE_M 그대로.
REVERSE_ARRIVE_M = 0.01
# 꺾어 들어갈 옆 칸이 '모름'이면(콘 광선 중 가까운 점이 70%가 안 되고 빈칸도 아님) 그 칸 앞쪽 절반
# (칸 경계 - UNKNOWN_WALL_BEFORE_M ~ 경계 + UNKNOWN_WALL_DEPTH_M, 칸 가운데 선 ±CHAIN_GUARD_HALF_WIDTH_M)에
# 돌아온 벽 점이 GUARD_MIN_POINTS개 이상 있는지 본다. 있으면 막힘으로 보고 다시 계획한다.
# 콘 비율은 반사가 안 돌아온 광선도 분모에 넣어, 검은 판처럼 반사가 약한 벽은 '모름'이 된다.
# 2026-11-03 15:05·15:23 실기: (4,7)에서 (4,8), (9,4)에서 (9,3)이 서서 봐도 '?'라 그대로 돌아 들어갔고,
# 회전 직후 0.3 s는 앞 벽 감시를 쉬어(GUARD_AFTER_TURN_S) 상자 앞면에 닿은 뒤 멈췄다.
# 달리면서 판단할 때도 같은 검사로 막힘을 알면 서서 다시 보지 않는다 (오후 실행마다 4~5번 서던 것).
UNKNOWN_WALL_BEFORE_M = 0.03
UNKNOWN_WALL_DEPTH_M = CELL_SIZE_M / 2.0
# 같은 사각형에 맞은 점이 없고, 사각형을 지나 더 멀리 맞은 광선이 이만큼 있으면 빈칸으로 본다.
# 2026-11-03 15:52 실기: 달리며 찍은 스캔(칸 가운데 15~18 cm 앞, LiDAR는 축 8 cm 뒤)에서는 꺾을 칸 쪽
# 콘 광선 일부가 지나온 쪽 옆 상자 면((8,1)->(8,2)면 (9,2))을 가까이 맞혀 빈칸인 (8,2)·(7,5)가 '?'가 됐고,
# 회전마다 서서 다시 봤다. 시뮬레이션: 빈칸은 안 0개·지나감 41~54개, 막힘은 안 18~48개·지나감 4~6개
# (광선 1도 간격, 실제 C1은 0.5도라 두 배). TODO_MEASURE: 실기 로그로 확인
UNKNOWN_CLEAR_MIN_RAYS = 15
# 회전이 덜 끝났을 때는 앞 벽 감시를 하지 않는다: 빨리 도는 동안 찍힌 스캔은 한 바퀴(0.1 s) 동안 20도 가까이
# 비틀려 옆 상자 모서리가 진행선 위로 온다 (2026-11-01 20:25 실기: (9,2)에서 157도로 아직 돌며 '앞에 벽' 급정지)
GUARD_MAX_TURN_RATE = 0.8                 # rad/s
GUARD_MAX_HEADING_RAD = math.radians(20.0)  # 진행 방향과 로봇 방향 차이
# 회전을 끝내고 이 시간 동안도 하지 않는다: 명령은 줄었어도 차체는 아직 돈다 (21:30 실기: (7,2)에서 회전 직후 오판)
GUARD_AFTER_TURN_S = 0.3
# 2026-11-01: 위 앞 벽 감시를 한 칸 전진에도 쓴다 (꺼 둔 DRIVE_BLOCK_CHECK 대신, 상대 로봇이 앞 칸에 들어옴).
# 서서 본 스캔에는 빈칸인데 감시만 계속 멈추면(같은 목표 칸 연속 이 횟수) 그 목표에서는 감시를 끈다
# (같은 계획·정지 무한 반복 방지). TODO_MEASURE: 실기에서 한 칸 주행이 엉뚱하게 서는지 확인
GUARD_REPEAT_MAX = 3
# 탐색 중 새 칸을 이 시간 동안 못 밟으면 막힘 표시를 모두 지우고 서서 다시 본다.
# 잘못 찍힌 막힘(LiDAR 오인식)에 갇혀 같은 칸을 오가는 것을 푼다 (2026-10-31 실기: (9,3)<->(9,5) 왕복).
# 진짜 벽은 다음 스캔에서 다시 막힘이 된다. 실기 전체 임무 약 37초 기준. TODO_MEASURE
NO_PROGRESS_S = 40.0
# 지나온 칸 어디에도 안 가 본 빈 이웃이 없으면(지도상 닫힌 영역) 40초를 기다리지 않고 바로 지운다.
# 2026-11-01 연습장 실기 4번: 출발 영역이 지도상 닫혀 (7,6)~(9,7)을 1~2분 왕복. 이 간격보다 자주는 안 한다.
FORGET_MIN_INTERVAL_S = 10.0
# 지우고 다시 봐도 똑같이 닫혀 있으면 간격을 두 배씩 (20, 40, 60 s). 대회 경기장은 길이 있어야 하므로
# 진짜로 닫힌 것은 연습장뿐이다 (2026-11-01 20:53: 닫힌 연습장 영역을 10초마다 다시 돌며 후진 반복)
FORGET_MAX_INTERVAL_S = 60.0
AREA_GROW_RATIO = 1.05

# 2026-11-01 실기 영상 대책 (요청서: 좁은 통로 왕복, 큐브 없이 RETURN).
# TURN·ALIGN 중 남은 각도의 부호가 이만큼 바뀌면(목표를 넘었다 돌아오기 반복) 그 자리에서 끝낸다.
TURN_FLIP_MAX = 3
# 같은 칸에서 '칸 가운데로 복귀'가 이 횟수를 넘게 이어지면 복귀를 건너뛰고 바로 다음 칸으로 간다.
RECENTER_MAX = 2
# 잡기(PUSH) 조건: 대상이 연속 GRAB_MIN_FRAMES번(20 Hz) 보이고, 접근 시작 뒤 GRAB_MIN_TRAVEL_M 이상
# 다가갔고, 가장 큰 면적이 처음 면적의 GRAB_MIN_GROWTH배 이상일 때만. 화면에 붙박인 빨간 물체(집게 부품 등)는
# 다가가도 커지지 않는다. 옆 칸 가운데(약 0.3 m)에서 잡는 위치까지 면적은 약 1.3배 이상 커진다. TODO_MEASURE
GRAB_MIN_FRAMES = 5
GRAB_MIN_TRAVEL_M = 0.03
GRAB_MIN_GROWTH = 1.15
# 접근으로 이만큼 갔는데도 면적이 GRAB_MIN_GROWTH배 안 되면 오탐으로 보고 접근을 그만둔다.
APPROACH_FALSE_TRAVEL_M = 0.30
# /gripper/holding이 없을 때(None, 지금 실물은 보내는 노드가 없음) 잡았는지 판단하는 방법:
# 'camera' = 집게를 닫은 뒤 큐브가 화면에 HELD_MIN_AREA 이상으로 보이면 잡은 것 (2026-11-01 기본)
# 'assume' = 예전처럼 잡았다고 가정. 나중에 VL53L0X 등 센서를 달면 /gripper/holding을 보내면 된다.
HOLDING_UNKNOWN = 'camera'
# TODO_MEASURE: 집게가 큐브를 잡은 채 카메라 면적 (잡는 위치 약 33000, 집게 날에 가려지면 작아짐)
HELD_MIN_AREA = 15000.0
# 출발 확인: (9,1) 가운데에 바퀴 축을 두고 위(+y)를 보면 오른쪽(+x)·뒤(-y) 외벽이 축에서 약 0.2 m.
START_WALL_M = CELL_SIZE_M / 2.0
START_WALL_TOL_M = 0.07
# 이 폭(축선 ±) 안의 점만 그 방향 벽으로 본다. 로봇 몸체 점을 빼려고 축에서 이보다 가까운 점은 버린다.
START_WALL_BAND_M = 0.08
START_WALL_MIN_M = 0.12

# 곡선 꺾기 (2026-11-01): 지도가 있는 길(복귀·막다른 길 되돌아가기)에서 꺾는 칸에 서서 제자리 회전하지 않고,
# 네 칸이 만나는 안쪽 모서리 점을 중심으로 반지름 ARC_RADIUS_M 원호를 ARC_SPEED로 돈다 (회전마다 약 1초 단축).
# 원호에서 바깥 앞모서리가 원래 방향의 다음 칸 쪽으로 약 19 cm(벽까지 8 mm) 나가므로, 그 칸이 빈칸(FREE)으로
# 확인됐을 때만 한다. 차체 치수 BODY_* 실측 후 재확인 (TODO_MEASURE)
# 2026-11-02 팀: 끔. 복귀 때 곡선(드리프트처럼 도는 것)이 불안하다 -> 갈 때처럼 칸 가운데에 서서 제자리 회전.
# 20:03 실기: 곡선 뒤 방향 오차 -8.7도, 옆 오차 4.2 cm로 곡선 취소 뒤 -19.6도.
ARC_TURNS = False
ARC_RADIUS_M = CELL_SIZE_M / 2.0
ARC_SPEED = 0.4           # m/s -> 회전 2 rad/s. TODO_MEASURE: 바퀴 미끄러짐·큐브 빠짐이 보이면 낮춘다
RETURN_ARC_SPEED = 0.3    # 복귀 중(큐브를 끌 때) 곡선 속도
ARC_RADIAL_GAIN = 3.0     # rad/m, 원에서 벗어난 거리만큼 안·밖으로 튼다 (최대 ARC_MAX_CORRECTION_RAD)
ARC_MAX_CORRECTION_RAD = math.radians(30.0)
ARC_EXIT_RAD = math.radians(10.0)   # 나갈 방향과 이만큼 안이면 원호를 끝내고 직진
# 원호 반지름은 들어가는 순간 로봇과 모서리 점 거리로 맞춘다. 그러면 안쪽 여유가 직선 주행 때와 같게 유지된다
# (시뮬레이션: 고정 반지름으로 5 cm 비켜 들어가면 원 안쪽으로 5 cm 들어갔다).
# 진행선에서 옆으로 이보다 비켜 있으면 곡선을 포기하고 칸 가운데에서 제자리 회전한다.
ARC_MAX_LATERAL_M = 0.04

# 큐브(50mm)가 이 크기 이상으로 보이면 집을 수 있는 거리 (카메라 픽셀 면적, 640x480)
# 2026-10-30 실측 (주황 시험 대상, 같은 크기): 열린 집게 사이 잡는 위치에서 area 약 33000
# (아래가 화면 밖으로 잘려 사실상 최댓값) -> 집게 안에 들어온 순간 잡도록 조금 낮춤 (TODO_MEASURE: 경기장에서 재확인)
# TODO_MEASURE: 2026-10-31 사용자 요청으로 35000. 실제 접근 중 이 면적에 도달하는지 재확인.
# 35000에서도 큐브가 집게 안쪽까지 안 들어왔다 (면적이 화면 아래 잘림으로 포화) ->
# 면적을 더 올리지 않고 GRAB_AREA에 닿은 뒤 엔코더로 GRAB_PUSH_M만큼 더 들어가서 닫는다.
GRAB_AREA = 35000.0
# TODO_MEASURE: 큐브가 집게 안쪽 끝에 닿는 거리. 아직 얕으면 늘리고, 큐브를 앞으로 밀어내면 줄인다
GRAB_PUSH_M = 0.04
# 집게 안 적외선(VL53L0X, /range/gripper)에 큐브가 이 거리 안에 보이면 밀어 넣기(PUSH)를 시작한다.
# 실측: 잡은 큐브 28 mm, 빈 집게 닫힘 108 mm. TODO_MEASURE: 열린 집게 안으로 충분히 들어온 거리
GRAB_TOF_M = 0.05
# 밀어 넣으며 닫는 거리 (GRAB_TOF_M보다 깊게). 2026-11-01 23:11 실기: 43 mm에서 닫아 잡은 뒤 32~38 mm,
# 집게 힘이 약해 복귀 회전 때 큐브가 틀어짐 -> 큐브를 집게 안쪽(가득 들어감 28 mm)까지 넣고 닫는다.
# TODO_MEASURE: 'm 밀어도 적외선에 안 들어옴'이 나오면 올린다
GRAB_TOF_CLOSE_M = 0.035
# 적외선에 이 거리 안으로 무언가 보이고 LiDAR(17 cm 높이)는 앞에 벽을 못 보면 낮은 큐브가 집게 앞에 있는 것.
# 2026-11-01 21:30 실기: 회전 직후 큐브가 바로 앞이라 카메라 면적이 그대로인 채 끌고 가다
# '면적이 안 커짐(오탐)'으로 접근을 두 번 그만두며 1.2 m 밀었다
TOF_NEAR_M = 0.15
# 적외선에 큐브가 가까이 보이는 동안 접근 중 벽 감시 거리 (집게 끝 앞). 평소는 scan_to_grid.GUARD_AHEAD_M 6 cm
APPROACH_WALL_GUARD_NEAR_CUBE_M = 0.01
# 적외선으로 잡기 시작하는 것은 카메라가 대상을 CAMERA_CONFIRM_S 넘게 이어서 봤고(사이 끊김 CAMERA_GAP_S까지는
# 이어진 것으로 봄), 마지막으로 본 지 TOF_GRAB_CAMERA_S 안일 때만 (가까워지면 집게 앞으로 들어가 화면에서 사라진다).
# 2026-11-02 19:32·19:52 실기: 큐브가 없는 (6,2)를 지날 때마다 적외선이 1·33·0 mm를 내 헛 집게질 -> 360도 SEARCH.
# 그 실행 내내 카메라는 빨간색을 못 봤다. 팀: 요구조자는 일정 거리부터 1~2초 카메라에 담기며 다가간다.
# TODO_MEASURE: 회전 직후 바로 앞(약 40 cm) 큐브도 이만큼 보이는지 실기 로그로 확인
CAMERA_CONFIRM_S = 1.0
CAMERA_GAP_S = 0.5
TOF_GRAB_CAMERA_S = 2.0
# 적외선 방식으로 밀며 들어가는 최대 거리 (이만큼 밀어도 안 들어오면 닫지 않고 다시 판단)
GRAB_TOF_PUSH_MAX_M = 0.15
# 큐브가 집게 입구(GRAB_TOF_MOUTH_M 안)에 있는데 GRAB_TOF_STALL_S 동안 GRAB_TOF_PROGRESS_M도 더 안 들어오면(같이 밀려감)
# 깊이(GRAB_TOF_CLOSE_M)까지 기다리지 않고 그 자리에서 닫는다. 벽은 집게 끝이 먼저 닿아 적외선이 약 90 mm 아래로 안 온다.
# 2026-11-02 22:25 실기: 55 mm에서 더 안 들어온 채 1초 동안 11 cm 밀다 '안 들어옴'으로 그만둬 큐브가 옆으로 빠짐.
GRAB_TOF_MOUTH_M = 0.07
GRAB_TOF_PROGRESS_M = 0.005
GRAB_TOF_STALL_S = 0.3
# 주행 중 카메라에 큐브가 이만큼 크게(약 45 cm 안) 보이면 그 칸으로 밀고 들어가지 않고 진행선으로 바로 접근한다.
# 21:30 실기: 멀리서 작게 본 큐브 거리를 멀게 계산해 큐브 칸으로 들어가며 밀었다. TODO_MEASURE
DRIVE_CUBE_AREA = 11000.0
DRIVE_CUBE_MAX_OFFSET = 0.5
GRAB_PUSH_SPEED = APPROACH_SLOW_SPEED
# 바퀴가 미끄러져 엔코더 거리가 안 늘어도 이 시간 뒤에는 닫는다 (보통 약 0.35초)
GRAB_PUSH_TIMEOUT_S = 1.0
# 밀어 넣기가 이만큼 남았을 때 집게를 닫기 시작한다 (닫기와 밀기를 겹쳐 시간 단축). 0 = 다 밀고 닫기.
# 펌웨어 1도/8ms에서 처음 약 20도(약 0.16초, 0.12 m/s로 약 2 cm)는 집게 끝이 큐브(50 mm)에
# 닿지 않는 각도라고 보고, 그 동안만 겹친다. 더 일찍 닫으면 큐브가 깊이 들어가기 전에 집게가 닿는다.
# TODO_MEASURE: 큐브가 다시 얕게 잡히거나 집게에 밀려 튕기면 줄이거나 0
GRAB_CLOSE_LEAD_M = 0.02

# 집게가 닫히고 열리기를 기다리는 최대 시간. /gripper/state가 없을 때(모를 때)는 이만큼 기다린다.
# 2026-10-31 펌웨어 값: 90<->135도, 1도/8ms = 약 0.36초 (옛 추정 1.2초로 정한 1.5초를 그대로 기다려 느렸다)
GRASP_WAIT_S = 1.5
RELEASE_WAIT_S = 1.5
# /gripper/state가 원하는 값(closed/open)이 되면 최대 시간을 기다리지 않고 넘어간다 (interfaces.md 4-2).
# 명령 직후에는 지난 상태가 남아 있을 수 있어 최소 시간은 기다린다 (실제 동작은 약 0.36초라 그 전엔 끝날 수 없음).
GRIPPER_MIN_WAIT_S = 0.25
# Mega가 목표 각도에 닿았다고 알린 뒤 서보가 실제로 따라가 힘을 줄 때까지의 여유. TODO_MEASURE
GRIPPER_SETTLE_S = 0.1
GRIPPER_CLOSED = 'closed'
GRIPPER_OPENED = 'open'

# 출발 칸에 와서 집게를 열지 (True, 기본) 잡은 채 끝낼지 (False). _deliver 참고
RELEASE_AT_START = True

# 갈 길이 없을 때 다시 스캔하기 전까지 기다리는 시간
STUCK_RETRY_S = 1.0

EXPLORE = 'EXPLORE'
RETURN = 'RETURN'

SCAN = 'SCAN'
TURN = 'TURN'
DRIVE = 'DRIVE'
FACE = 'FACE'
SEARCH = 'SEARCH'
ALIGN = 'ALIGN'
APPROACH = 'APPROACH'
PUSH = 'PUSH'
GRASP = 'GRASP'
RELEASE = 'RELEASE'
DONE = 'DONE'
STUCK = 'STUCK'
ARC = 'ARC'


def side_clearance(scan):
    """차체 옆 구간의 LiDAR 점까지 옆 거리 (왼쪽, 오른쪽) (m, 축 기준). 없으면 inf."""
    left = right = math.inf
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        angle = scan_to_grid.LASER_YAW_RAD + scan.angle_min + i * scan.angle_increment
        # 로봇 기준 점 (앞 = +x, 왼쪽 = +y)
        px = scan_to_grid.LASER_OFFSET_X_M + distance * math.cos(angle)
        py = scan_to_grid.LASER_OFFSET_Y_M + distance * math.sin(angle)
        if not SIDE_GUARD_BACK_M <= px <= SIDE_GUARD_FRONT_M or abs(py) < SIDE_GUARD_SELF_M:
            continue
        if py > 0.0:
            left = min(left, py)
        else:
            right = min(right, -py)
    return left, right


def start_wall_check(scan):
    """
    출발 자세 확인: 오른쪽·뒤 외벽이 축에서 약 START_WALL_M에 있는지 (로그용 한 줄).

    로봇은 (9,1) 가운데에 바퀴 축을 두고 위(+y)를 본다고 가정한다 (scan_to_grid.START_POSE).
    그러면 오른쪽(+x)과 뒤(-y)가 경기장 외벽이다. 다르면 놓은 방향·위치가 틀렸을 수 있다.
    """
    right, rear = [], []
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        angle = scan_to_grid.LASER_YAW_RAD + scan.angle_min + i * scan.angle_increment
        # 로봇 기준 점 (앞 = +x, 왼쪽 = +y)
        px = scan_to_grid.LASER_OFFSET_X_M + distance * math.cos(angle)
        py = scan_to_grid.LASER_OFFSET_Y_M + distance * math.sin(angle)
        if math.hypot(px, py) < START_WALL_MIN_M:
            continue
        if py < 0.0 and abs(px) <= START_WALL_BAND_M:
            right.append(-py)
        if px < 0.0 and abs(py) <= START_WALL_BAND_M:
            rear.append(-px)
    found = {'오른쪽': min(right, default=None), '뒤': min(rear, default=None)}
    text = ', '.join(f'{name} {"없음" if d is None else f"{d:.2f} m"}' for name, d in found.items())
    if all(d is not None and abs(d - START_WALL_M) <= START_WALL_TOL_M for d in found.values()):
        return f'출발 자세 확인 OK: 외벽 {text} (예상 {START_WALL_M:.2f} m)'
    return (f'⚠️ 출발 자세 확인: 외벽 {text} (예상 {START_WALL_M:.2f} m). '
            '(9,1) 칸 가운데에 바퀴 축, 위(+y)를 보게 놓았는지 확인')


@dataclass
class Observation:
    x: float
    y: float
    yaw: float
    now: float
    # 최근 LiDAR 스캔 (sensor_msgs/LaserScan 또는 같은 속성을 가진 객체)
    scan: object = None
    # 조원 B Vision 결과 (docs/interfaces.md /vision/target)
    detected: bool = False
    x_offset: float = 0.0
    area: float = 0.0
    # 조원 A 집게 상태. 알 수 없으면 None
    holding: Optional[bool] = None
    # 조원 A /gripper/state: 'open' / 'closed' / 'moving'. 알 수 없으면 None
    gripper_state: Optional[str] = None
    # scan의 첫 광선 촬영 시각 (LaserScan header.stamp). 모르면 None (주행 중 판단 안 함)
    scan_time: Optional[float] = None
    # 집게 안 적외선 거리 (m, 범위 밖 inf). 센서가 없거나 끊기면 None
    gripper_range: Optional[float] = None


class Mission1Logic:

    def __init__(self, point_symmetry=True, rolling_decision=None):
        # point_symmetry: 대회 규정상 장애물은 점대칭이라 한 칸이 막히면 반대편도 막힘으로 칠한다.
        # 대칭이 아닌 연습 경기장에서는 끈다 (start_mission.sh --test).
        self.grid = GridMap(use_point_symmetry=point_symmetry)
        # 주행 중 판단 (칸마다 안 서기). 기본은 ROLLING_DECISION, 노드 파라미터로 켤 수 있다 (연습 --test)
        self.rolling = ROLLING_DECISION if rolling_decision is None else rolling_decision
        self.phase = EXPLORE
        self.state = SCAN
        self.state_started_at = None
        self.gripper_opened = False
        # 지금 가는 지점 (경기장 좌표 m)
        self.target = None
        # True면 지금 가는 지점이 큐브를 놓을 위치다.
        self.delivering = False
        # 마지막으로 계산한 경로 (확인용)
        self.path = None
        self.turn_pulse = TurnPulse()
        self.maze = Mission1Maze()
        self.reverse = False
        self._drive_block_scans = 0
        self._last_drive_scan = None
        self._drive_heading = None
        self._push_start = None
        self._gripper_reached_at = None
        self._grab_sent_at = None
        # 대상 접근 축 방향(rad)과 출발 칸 가운데, 접근 중 대상을 마지막으로 본 시각
        self._approach_axis = None
        self._approach_origin = None
        self._target_seen_at = None
        # 접근 중 가장 컸던 면적과 그때 위치 (면적이 안 커지는지 본다)
        self._area_best = 0.0
        self._area_best_at = None
        # 장애물에 막혀 접근을 그만둔 칸 (그 칸에서는 다시 접근하지 않는다)
        self._approach_blocked = set()
        # 큐브를 찾다가 못 찾은 중앙 옆 칸 (목표에서 뺀다)
        self._searched_goals = set()
        # 기억한 큐브 위치 (경기장 m)와 그때 면적 (클수록 가까이서 본 것이라 정확)
        self.cube = None
        self._cube_area = 0.0
        # 카메라가 마지막으로 대상을 본 시각과 이어서 보기 시작한 시각 (적외선 잡기 확인용)
        self._camera_seen_at = None
        self._camera_streak_start = None
        self._scan_at_arrival = False
        # 두 칸을 한 번에 가는 중인지 (두 번째 칸이 미관측일 수 있어 앞을 계속 본다)
        self._chained = False
        # 이번 주행에서 진행선 옆으로 가장 많이 벗어난 거리, 옆 벽 감시를 로그로 남겼는지 (로그용)
        self._drive_max_lateral = 0.0
        # 후진 도착을 뒤 벽 거리로 앞당긴 만큼 (로그용)
        self._reverse_wall_gap = 0.0
        # 지금 주행이 칸 가운데로 옮기기(recenter)인지
        self._recentering = False
        # 스캔에서 벽이 보인 칸 사이 연결 (frozenset 쌍). 복귀·되돌아가기는 이 연결로 지나가지 않는다.
        self._wall_edges = set()
        self._side_guard_noted = False
        # 여러 칸 주행 중 앞 벽으로 멈춘 칸 (그 칸에서는 다시 묶지 않는다)
        self._no_chain_from = set()
        self._drive_origin_cell = None
        # 한 칸 주행 앞 벽 감시: 연속으로 멈춘 목표 칸과 횟수, 감시를 끈 목표 칸
        self._guard_stop_cell = None
        self._guard_stops = 0
        self._guard_off_cell = None
        self._guard_ahead = None
        # 막다른 길에서 되돌아 나오는 경로 (지나온 칸, 끝 = 안 가 본 이웃이 있는 칸). 없으면 None
        self._backtrack = None
        # 지금 직선 구간 끝에서 곡선으로 꺾을 계획 (_arc_plan). 없으면 None
        self._arc = None
        # 임무 시작 시각(첫 step), 마지막으로 처음 가 본 칸에 들어간 시각
        self.started_at = None
        self._new_cell_at = None
        # 직전 상태와 바뀐 이유 (노드가 상태 로그에 쓴다). 이유가 없으면 None
        self.prev_state = None
        self.state_reason = None
        # TURN·ALIGN 중 남은 각도의 부호와 바뀐 횟수
        self._turn_sign = 0
        self._turn_flips = 0
        # 연속으로 '칸 가운데 복귀'를 한 칸과 횟수
        self._recenter_cell = None
        self._recenters = 0
        # 접근: 시작 위치, 처음 본 면적, 연속으로 보인 횟수
        self._approach_start = None
        self._area_first = None
        self._seen_frames = 0
        self._start_checked = False
        # 이번 밀기를 적외선으로 판단하는지 (밀기 시작할 때 센서가 살아 있었는지)
        self._push_tof = False
        # 밀기 중 집게 입구 큐브: 어느 밀기 것인지, 가장 가까웠던 거리와 그 시각
        self._mouth_for = None
        self._mouth_best = None
        self._forgot_at = -math.inf
        self._forget_interval = FORGET_MIN_INTERVAL_S
        self._last_forgot_blocked = None
        # 직전에 내보낸 (linear, angular)와 시각. 가감속 제한에 쓴다.
        self._last_linear = 0.0
        self._last_angular = 0.0
        self._last_command_at = None
        # 마지막으로 속도를 바꾼(가감속) 시각. 주행 중 판단은 그 뒤 안정된 스캔만 쓴다.
        self._speed_changed_at = 0.0
        # 주행 중 판단을 이미 끝낸 목표 (회전이 필요해 가운데에 서면 바로 _plan)
        self._decided_for = None
        self._has_scan_time = False
        self._turn_heading = None
        self._turn_ended_at = None
        # 노드가 로그로 남길 한 줄 (남긴 뒤 None)
        self.event = None
        self.stop_reason = None

    def _change_state(self, new_state, obs, reason=None):
        self.prev_state = self.state
        self.state_reason = reason
        self.state = new_state
        self.state_started_at = obs.now
        self.turn_pulse.reset()
        self._turn_sign = 0
        self._turn_flips = 0
        if new_state not in (TURN, DRIVE, ARC):
            self._arc = None
        if new_state not in (TURN, DRIVE):
            self._chained = False
        if new_state == DRIVE and self.prev_state != DRIVE:
            self._drive_max_lateral = 0.0
            self._side_guard_noted = False
        if new_state != DRIVE:
            # 다른 일(대상 발견 등)로 주행이 끊기면 미리 한 판단은 버린다.
            self._decided_for = None
            self._scan_at_arrival = False
        self._gripper_reached_at = None

    def step(self, obs):
        known = len(self.maze.state.visits)
        self.maze.observe(point_to_cell(obs.x, obs.y), obs.yaw)
        if self.started_at is None:
            self.started_at = self._new_cell_at = obs.now
        if len(self.maze.state.visits) > known:
            self._new_cell_at = obs.now
        if self.state_started_at is None:
            self.state_started_at = obs.now
        self._remember_cube(obs)
        if obs.detected:
            if self._camera_seen_at is None or obs.now - self._camera_seen_at > CAMERA_GAP_S:
                self._camera_streak_start = obs.now
            self._camera_seen_at = obs.now

        command = self._check_dropped(obs)
        if command is None:
            command = self._check_trapped(obs)
        if command is None:
            command = self._run_state(obs)

        # 시작 시 집게를 한 번 연다.
        if not self.gripper_opened and command.gripper is None:
            command.gripper = GRIPPER_OPEN
            self.gripper_opened = True

        return self._limit_accel(command, obs.now)

    def _limit_accel(self, command, now):
        """급출발·급정지 방지: 직전 명령에서 가감속 한도만큼만 바꾼다 (상태가 바뀌어도 이어서)."""
        dt = None if self._last_command_at is None else now - self._last_command_at
        if dt is None or dt < 0.0 or dt > ACCEL_RESET_S:
            # 처음이거나 판단이 끊긴 동안 노드가 정지 명령을 보냈다 -> 정지 상태에서 시작
            self._last_linear = self._last_angular = 0.0
            dt = ACCEL_RESET_S / 2.0
        accel, decel = ((RETURN_LINEAR_ACCEL, RETURN_LINEAR_DECEL) if self.phase == RETURN
                        else (LINEAR_ACCEL, LINEAR_DECEL))
        command.linear = ramp(command.linear, self._last_linear, accel, decel, dt)
        if abs(command.linear - self._last_linear) > 0.005:
            self._speed_changed_at = now
        angular_accel = RETURN_ANGULAR_ACCEL if self.phase == RETURN else ANGULAR_ACCEL
        command.angular = ramp(
            command.angular, self._last_angular, angular_accel, angular_accel, dt)
        self._last_linear = command.linear
        self._last_angular = command.angular
        self._last_command_at = now
        return command

    def _check_dropped(self, obs):
        # 돌아가는 중에 큐브를 놓치면 다시 찾아서 잡는다.
        if self.phase != RETURN or obs.holding is not False:
            return None
        if self.state not in (SCAN, TURN, DRIVE, ARC):
            return None

        self.phase = EXPLORE
        self.delivering = False
        self._change_state(SEARCH, obs)
        return Command(gripper=GRIPPER_OPEN)

    def _run_state(self, obs):
        handlers = {
            SCAN: self._scan,
            TURN: self._turn,
            DRIVE: self._drive,
            FACE: self._face,
            SEARCH: self._search,
            ALIGN: self._align,
            APPROACH: self._approach,
            PUSH: self._push,
            GRASP: self._grasp,
            RELEASE: self._release,
            DONE: self._done,
            STUCK: self._stuck,
            ARC: self._arc_step,
        }
        return handlers[self.state](obs)

    # ------------------------------------------------------------
    # 격자 이동 (SCAN -> TURN -> DRIVE)
    # ------------------------------------------------------------

    def _scan(self, obs):
        # 멈춘 뒤 새 스캔이 들어올 때까지 기다린다.
        if obs.now - self.state_started_at < SCAN_SETTLE_S:
            return Command()
        if obs.scan is None:
            return Command()

        if not self._start_checked:
            self._start_checked = True
            self._note(start_wall_check(obs.scan))
        self._map_scan(obs.scan, (obs.x, obs.y, obs.yaw))
        if self._target_approach(obs):
            self._change_state(ALIGN, obs)
            return Command()
        return self._plan(obs)

    def _check_trapped(self, obs):
        """
        갇힌 것 같으면 막힘 표시를 지우고 멈춰 서서 다시 본다 (탐색 중, 큐브 위치를 모를 때).

        2026-11-01 20:25 실기: 주행 중 판단(연습 모드)은 서서 스캔(SCAN)하지 않아, SCAN에만 있던 이 검사가
        한 번도 돌지 않고 닫힌 영역 안 두 칸을 30초 넘게 오갔다(방향 바꿀 때마다 급정지). 그래서 매 step 본다.
        """
        if self.phase != EXPLORE or self.cube is not None or self.state not in (SCAN, TURN, DRIVE):
            return None
        if obs.now - self._forgot_at < self._forget_interval:
            return None
        if obs.now - self._new_cell_at > NO_PROGRESS_S:
            why = f'새 칸 없이 {NO_PROGRESS_S:.0f}s'
        elif not self._has_frontier():
            why = '지도상 갈 곳 없음 (닫힌 영역)'
        else:
            return None
        # 지운 뒤 다시 본 지도가 지난번과 같으면(진짜 닫힌 영역) 간격을 두 배로 (최대 FORGET_MAX_INTERVAL_S)
        blocked = frozenset(c for c, v in self.grid.cells.items() if v == BLOCKED)
        if blocked == self._last_forgot_blocked:
            self._forget_interval = min(2.0 * self._forget_interval, FORGET_MAX_INTERVAL_S)
        self._last_forgot_blocked = blocked
        self._forget_blocked(obs, why)
        self._change_state(SCAN, obs, why)
        return Command()

    def _has_frontier(self):
        """지나온 칸 중 안 가 본, 막히지 않은 이웃이 있는 칸이 있는지."""
        return any(self._new_neighbors(cell) for cell in self.maze.state.visits)

    def _forget_blocked(self, obs, why):
        """갇힌 것 같으면 막힘 표시와 포기한 칸들을 지우고 처음처럼 다시 본다 (진짜 벽은 다시 막힘)."""
        for cell, value in self.grid.cells.items():
            if value == BLOCKED:
                self.grid.cells[cell] = UNKNOWN
        self._no_chain_from.clear()
        self._approach_blocked.clear()
        self._searched_goals.clear()
        self._guard_off_cell = None
        self._backtrack = None
        self._new_cell_at = obs.now
        self._forgot_at = obs.now
        self._note(f'{obs.now - self.started_at:.0f}s: {why} -> 막힘 표시를 지우고 다시 탐색')

    def _note(self, message):
        """노드 로그 한 줄에 덧붙인다 (같은 step의 앞 내용을 지우지 않게)."""
        self.event = message if self.event is None else f'{self.event}; {message}'

    def _map_scan(self, scan, pose, center=None):
        """
        스캔으로 상하좌우 이웃 칸만 지도에 반영한다 (방향별 ±15도 레이저, center 기준 거리).

        멀리 있는 칸까지 광선으로 칠하는 갱신(update_grid_from_scan)은 쓰지 않는다. 멀리 맞은 점은
        위치·방향 오차가 커져 통로 칸을 막힘으로 잘못 찍었다 (2026-10-31 실기: (8,3)을 못 골라
        돌아감, 주행 중 스캔으로는 STUCK·왕복). 미로는 이웃 칸만 쓰고, 먼 칸은 '모름'으로 두었다가
        옆에 가서 직접 본다. 시뮬레이션 규정 예시 63.4 s -> 63.5 s로 시간 차이 없음.
        """
        visits = self.maze.state.visits
        origin = point_to_cell(*(center if center is not None else pose[:2]))
        for cell, state in neighbor_states(scan, pose, center).items():
            if not self.grid.in_bounds(cell):
                continue
            # 칸 사이에 벽이 보였는지 연결마다 기억한다 (두 칸 다 가 봤어도 그 사이로 지나가지 않게)
            if state == BLOCKED:
                self._wall_edges.add(frozenset((origin, cell)))
            else:
                self._wall_edges.discard(frozenset((origin, cell)))
            if state == BLOCKED:
                # 로봇이 실제로 지나온 칸은 벽일 수 없다 (2026-11-01 실기: 출발 칸 (9,1)이 막힘으로 찍혀
                # 닫힌 영역이 됨). 상대 로봇이 서 있는 경우는 주행 중 앞 벽 감시가 막는다.
                if cell in visits:
                    continue
                self.grid.mark_blocked(cell)
            else:
                self.grid.mark_free(cell)

    def scan_pose(self, obs):
        """스캔을 찍던 순간(한 바퀴 가운데)의 로봇 위치. 달리는 중이면 그만큼 뒤로 돌린다."""
        if obs.scan_time is None:
            return obs.x, obs.y, obs.yaw
        duration = getattr(obs.scan, 'scan_time', 0.0) or DEFAULT_SCAN_DURATION_S
        back = self._last_linear * max(0.0, obs.now - (obs.scan_time + duration / 2.0))
        return obs.x - back * math.cos(obs.yaw), obs.y - back * math.sin(obs.yaw), obs.yaw

    def rolling_scan_ok(self, obs):
        """이 스캔으로 주행 중 판단을 할 수 있는지 (노드도 같은 스캔으로 벽 보정을 한다)."""
        if not self.rolling or self.state != DRIVE or self.delivering or self.target is None:
            return False
        if self._recentering:
            return False  # 가운데로 옮기는 중: 도착하면 _plan이 다음 칸을 정한다
        if obs.scan is None or obs.scan_time is None or self._decided_for == self.target:
            return False
        if abs(self._last_linear) > ROLL_SPEED + 0.01:
            return False
        if obs.scan_time < self._speed_changed_at + ROLL_SETTLE_S:
            return False
        x, y, _ = self.scan_pose(obs)
        target_cell = point_to_cell(*self.target)
        if target_cell is None or point_to_cell(x, y) != target_cell:
            return False
        if math.dist(cell_center_m(target_cell), self.target) > 1e-6:
            return False  # 칸 가운데로 가는 주행만 (놓기 위치 등은 제외)
        return math.hypot(self.target[0] - x, self.target[1] - y) <= ROLL_WINDOW_M

    def _rolling_decision(self, obs):
        """
        달리는 중에 다음 칸을 정한다. 직진이면 목표를 바꿔 계속 가고 True.

        꺾어야 하면 가운데에 서자마자 (스캔 대기 없이) 돈다 (2026-11-01 팀: 서서 판단하면 너무 오래 걸림).
        달리며 찍은 스캔에 옆 상자 모서리가 몇 개 걸려 옆 칸을 막힘으로 보던 것(20:53 실기)은
        scan_to_grid의 막힘 기준을 콘 광선의 25%로 올려 막는다 (예전 3개).
        """
        x, y, yaw = self.scan_pose(obs)
        self._map_scan(obs.scan, (x, y, yaw), center=self.target)
        if self._target_approach(obs, at=self.target, commit=False):
            # 가운데에서 대상에 축 방향으로 다가갈 수 있다 -> 서서 SCAN에서 다시 확인
            self._decided_for = self.target
            self._scan_at_arrival = True
            self.event = '주행 중 판단: 대상이 보여 가운데에 선다'
            return False
        saved = (self.target, self._drive_heading, self.reverse, self.delivering, self.path,
                 self.state, self.state_started_at, self.stop_reason)
        # 미리 해 보는 _plan이 가운데 옮기기 횟수를 지우면 RECENTER_MAX가 안 먹는다 (2026-11-03 23:31 실기)
        saved_recenter = (self._recenter_cell, self._recenters, self._recentering)
        old_reverse = self.reverse
        # 가운데에 도착했다고 보고 계획한다 (미로 탐색은 이미 이 칸에 들어온 것으로 셌다).
        center = Observation(x=self.target[0], y=self.target[1], yaw=obs.yaw, now=obs.now)
        self._plan(center)
        straight = (
            self.state == TURN and not self.reverse and not old_reverse
            and not self.delivering and self.target != saved[0]
            and saved[1] is not None
            and abs(math.atan2(math.sin(self._drive_heading - saved[1]),
                               math.cos(self._drive_heading - saved[1]))) < ROLL_STRAIGHT_RAD)
        if straight:
            self.state = DRIVE
            self._decided_for = None
            self.event = f'주행 중 판단: 직진 계속 -> {self.path}'
            return True
        next_cell = self.path[1] if self.state == TURN and self.path else None
        (self.target, self._drive_heading, self.reverse, self.delivering, self.path,
         self.state, self.state_started_at, self.stop_reason) = saved
        self._recenter_cell, self._recenters, self._recentering = saved_recenter
        self._decided_for = self.target
        judged = (None if next_cell is None else self._judge_unknown(
            obs.scan, (x, y, yaw), point_to_cell(*self.target), next_cell))
        if judged == BLOCKED:
            # '모름'이지만 그 칸 앞쪽 절반에 벽 점이 있다 -> 막힘으로 보고 서지 않고 다시 판단
            self.grid.mark_blocked(next_cell)
            self._note(f'주행 중 {next_cell}은 모름이지만 벽 점이 보여 막힘으로 봄')
            return self._rolling_decision(obs)
        if judged == FREE:
            # 콘으로는 애매했지만 그 칸 앞쪽 절반이 비었고 광선이 지나간다 -> 빈칸으로 보고 바로 돈다
            self.grid.mark_free(next_cell)
            self._note(f'주행 중 {next_cell}은 모름이지만 광선이 지나가 빈칸으로 봄')
        if (next_cell is not None and self.grid.get(next_cell) != FREE
                and next_cell not in self.maze.state.visits):
            # 꺾어 들어갈 옆 칸을 빈칸으로 못 봤다 (애매해서 '모름'). 미로는 모름을 지나갈 수 있다고 보지만
            # 옆 칸 벽은 집게 끝에서 몇 cm라 돌고 나서는 앞 벽 감시로 못 막는다 -> 서서 다시 본다.
            # 2026-11-02 19:24 실기: (8,5)->(8,6), (8,6)->(7,6) 둘 다 '?'인 왼쪽 칸으로 꺾어 벽을 뚫고 감.
            self._scan_at_arrival = True
            self.event = f'주행 중 판단: 꺾을 칸 {next_cell}을 못 봐서 가운데에 서서 다시 본다'
            return False
        self.event = '주행 중 판단: 가운데에 서서 회전 (판단 대기 없음)'
        return False

    def _plan(self, obs, after_recenter=False):
        current = point_to_cell(obs.x, obs.y)
        self.stop_reason = None
        self.reverse = False
        if current is None:
            self.stop_reason = '추정 위치가 경기장 밖입니다'
            self._change_state(STUCK, obs)
            return Command()

        # 지금 서 있는 격자는 막혀 있을 수 없다. (잘못 인식한 경우 되돌린다)
        if self.grid.get(current) == BLOCKED:
            self.grid.mark_free(current)

        cube_goals = self._cube_goals() if self.phase == EXPLORE else []
        if cube_goals:
            goals = cube_goals
        elif self.phase == EXPLORE:
            goals = [cell for cell in self.grid.neighbors(CENTER)
                     if cell not in self._searched_goals]
        else:
            goals = [MISSION1_START]

        if current in goals:
            return self._arrived_at_goal(obs)

        path = None
        self._backtrack = None
        if cube_goals:
            # 기억한 큐브의 옆 칸까지 지도(모르는 칸은 지나갈 수 있다고 봄)로 최단 경로
            paths = [self.grid.find_path(current, goal) for goal in cube_goals]
            paths = [p for p in paths if p is not None]
            path = min(paths, key=len)[:2] if paths else None
        if path is None and self.phase == EXPLORE:
            path = self._backtrack = self._backtrack_path(current)
        if path is None and self.phase == EXPLORE:
            next_cell = self.maze.next_cell(self.grid)
            path = [current, next_cell] if next_cell is not None else None
        elif path is None:
            path = self._shortest_path(current, goals)
        if path is None:
            self.path = None
            self.stop_reason = (
                '갈 수 있는 이웃 칸이 없습니다' if self.phase == EXPLORE
                else '방문한 칸 안에서 복귀 경로를 찾지 못했습니다')
            self._change_state(STUCK, obs)
            return Command()

        if obs.scan is not None and self._judge_unknown(
                obs.scan, (obs.x, obs.y, obs.yaw), current, path[1]) == BLOCKED:
            self.grid.mark_blocked(path[1])
            self._note(f'{path[1]}은 모름이지만 그쪽 진행선에 벽 -> 막힘으로 보고 다시 계획')
            return self._plan(obs)

        self.path = path
        center = cell_center_m(current)
        drive_origin = center
        self._drive_origin_cell = center

        recenter = distance_to(obs, center) > CELL_CENTER_TOLERANCE_M
        if RETURN_SAFE_TURNS and self.phase == RETURN and not recenter and not after_recenter:
            # 새로 갈 방향 기준 옆으로 비켜 있으면 그 통로 입구 모서리에 걸린다 -> 먼저 가운데로.
            # 방금 가운데로 옮겼으면 다시 재지 않는다: 뒤 벽으로 도착을 봤으면 엔코더는 몇 cm 어긋나 있어
            # 옮기기를 끝없이 되풀이했다 (2026-11-03 23:24 (8,1), 23:31 (9,9) 실기에서 멈춤).
            step_x, step_y = path[1][0] - path[0][0], path[1][1] - path[0][1]
            lateral = abs(-(obs.x - center[0]) * step_y + (obs.y - center[1]) * step_x)
            recenter = lateral > RETURN_RECENTER_M
        if recenter:
            # 같은 칸에서 가운데 복귀가 이어지면(지나침·왕복) RECENTER_MAX번 뒤에는 건너뛴다.
            same = self._recenter_cell == current
            self._recenters = self._recenters + 1 if same else 1
            self._recenter_cell = current
            if self._recenters > RECENTER_MAX:
                recenter = False
                self._note(f'{current} 가운데 복귀가 {self._recenters}번째 -> 건너뛰고 바로 {path[1]}로')
        else:
            self._recenter_cell = None
            self._recenters = 0
        self._recentering = recenter
        if recenter:
            # 격자 가운데에서 벗어나 있으면 먼저 가운데로 돌아간다.
            # (옆 격자로 비스듬히 가다가 장애물 모서리에 걸리지 않게)
            self.target = center
            drive_origin = (obs.x, obs.y)
            self.delivering = False
            # 가운데가 뒤쪽이면 (접근을 그만둔 뒤 등) 돌지 않고 후진한다.
            self.reverse = abs(heading_error_to(obs, self.target)) > 3 * math.pi / 4
        elif self.phase == RETURN and path[1] == MISSION1_START:
            self.target = self._release_point(path[0], path[1])
            self.delivering = True
        else:
            # 직진이 이어지면 중간 칸에서 서지 않고 한 번에 간다 (2026-10-31: 칸마다 서면 느림).
            end = self._straight_end(obs, path)
            self.target = cell_center_m(path[end])
            self.delivering = False
            # 다음 칸이 뒤쪽이면 180도 돌지 않고 후진한다 (탐색·복귀 모두, 2026-10-31 수아: 복귀도).
            # 출발 격자에 넣는 마지막 칸(위 delivering)은 전진: 큐브가 로봇 앞에 있고 출발 격자는
            # 구석이라 후진으로는 큐브 전체를 격자 안에 넣을 수 없다 (문의 16).
            # TODO_MEASURE: 복귀 중 후진으로 끌 때 큐브가 집게에서 빠지지 않는지 실기 확인
            self.reverse = abs(heading_error_to(obs, self.target)) > 3 * math.pi / 4
            self._arc = None if self.reverse else self._arc_plan(path, end)

        self._drive_heading = math.atan2(
            self.target[1] - drive_origin[1], self.target[0] - drive_origin[0])
        self._change_state(TURN, obs)
        return Command()

    def _judge_unknown(self, scan, pose, current, nxt):
        """
        아직 '모름'인 이웃 칸 nxt를 그 칸 앞쪽 절반 사각형으로 다시 본다: BLOCKED, FREE, 또는 None(그대로 모름).

        안에 돌아온 벽 점이 GUARD_MIN_POINTS개 이상이면 막힘, 하나도 없고 지나간 광선이
        UNKNOWN_CLEAR_MIN_RAYS개 이상이면 빈칸.
        """
        if (self.phase != EXPLORE or self.grid.get(nxt) == FREE
                or nxt in self.maze.state.visits
                or abs(nxt[0] - current[0]) + abs(nxt[1] - current[1]) != 1):
            return None
        heading = math.atan2(nxt[1] - current[1], nxt[0] - current[0])
        half = CELL_SIZE_M / 2.0
        inside, through = rect_ray_counts(
            scan, pose, cell_center_m(current), heading, half - UNKNOWN_WALL_BEFORE_M,
            half + UNKNOWN_WALL_DEPTH_M, CHAIN_GUARD_HALF_WIDTH_M)
        if inside >= GUARD_MIN_POINTS:
            return BLOCKED
        if inside == 0 and through >= UNKNOWN_CLEAR_MIN_RAYS:
            return FREE
        return None

    def _arc_plan(self, path, end):
        """
        path[end](꺾는 칸)에서 곡선으로 꺾을 수 있으면 그 계획, 아니면 None.

        지도가 있는 길(복귀·되돌아가기)에서 전진으로 들어와 90도 옆으로 나가고, 원래 방향의 다음 칸이
        빈칸(FREE)으로 확인됐을 때만. 다음 구간이 출발 격자에 넣는 칸(delivering)이면 하지 않는다.
        """
        if not ARC_TURNS or (self.phase != RETURN and self._backtrack is None):
            return None
        if end < 1 or end + 1 >= len(path):
            return None
        corner = path[end]
        d1 = (corner[0] - path[end - 1][0], corner[1] - path[end - 1][1])
        d2 = (path[end + 1][0] - corner[0], path[end + 1][1] - corner[1])
        if d1[0] * d2[0] + d1[1] * d2[1] != 0:
            return None
        if self.phase == RETURN and path[end + 1] == MISSION1_START:
            return None
        # 바깥: 원래 방향의 다음 칸 (바깥 앞모서리가 약 19 cm 나감).
        # 안쪽 대각선 칸은 보지 않는다: 반지름을 들어갈 때 거리로 맞추므로 그 칸에 상자가 있어도
        # 모서리까지 여유가 직선으로 그 옆을 달릴 때와 같고, 차체가 그 칸 쪽으로 들어가지 않는다.
        outer = (corner[0] + d1[0], corner[1] + d1[1])
        if not self.grid.in_bounds(outer) or self.grid.get(outer) != FREE:
            return None
        return {'corner': corner, 'd1': d1, 'd2': d2, 'next': path[end:]}

    def _arc_step(self, obs):
        """안쪽 모서리 점을 중심으로 원호를 따라 돌고, 나갈 방향이 되면 다음 직선 구간으로 이어 간다."""
        arc = self._arc
        if arc is None:
            self._change_state(SCAN, obs)
            return Command()
        (d1x, d1y), (d2x, d2y) = arc['d1'], arc['d2']
        cx, cy = self._arc_center(arc)
        radius = arc.get('radius', ARC_RADIUS_M)
        turn = 1.0 if d1x * d2y - d1y * d2x > 0 else -1.0   # +1 = 왼쪽(반시계)
        goal_yaw = math.atan2(d2y, d2x)
        left = math.atan2(math.sin(goal_yaw - obs.yaw), math.cos(goal_yaw - obs.yaw))
        past = (obs.x - cx) * d2x + (obs.y - cy) * d2y >= 0.0
        if abs(left) < ARC_EXIT_RAD or past or turn * left < 0.0:
            return self._after_arc(obs, arc, goal_yaw)
        rx, ry = obs.x - cx, obs.y - cy
        # 원의 접선 방향 + 원 밖이면 안쪽으로, 안이면 바깥으로
        tangent = math.atan2(ry, rx) + turn * math.pi / 2.0
        correction = clamp(ARC_RADIAL_GAIN * (math.hypot(rx, ry) - radius),
                           ARC_MAX_CORRECTION_RAD)
        desired = tangent + turn * correction
        error = math.atan2(math.sin(desired - obs.yaw), math.cos(desired - obs.yaw))
        arc_speed = self._arc_speed()
        angular = turn * arc_speed / radius + HEADING_GAIN * error
        return Command(linear=arc_speed, angular=clamp(angular, self._turn_speed()))

    def _arc_speed(self):
        return RETURN_ARC_SPEED if self.phase == RETURN else ARC_SPEED

    def _max_speed(self):
        return RETURN_DRIVE_SPEED if self.phase == RETURN else DRIVE_SPEED

    def _turn_speed(self):
        return RETURN_TURN_SPEED if self.phase == RETURN else TURN_SPEED

    @staticmethod
    def _arc_center(arc):
        """원 중심 = 꺾는 칸 가운데에서 들어온 쪽으로 R, 나갈 쪽으로 R (네 칸이 만나는 모서리 점)."""
        (d1x, d1y), (d2x, d2y) = arc['d1'], arc['d2']
        bx, by = cell_center_m(arc['corner'])
        return (bx + ARC_RADIUS_M * (d2x - d1x), by + ARC_RADIUS_M * (d2y - d1y))

    def _after_arc(self, obs, arc, heading):
        """원호 다음 직선 구간 (지나온 칸이라 같은 방향이 이어지는 데까지, 서지 않고)."""
        path = arc['next']
        end = self._straight_end(obs, path)
        self.target = cell_center_m(path[end])
        self.path = path
        self.delivering = False
        self.reverse = False
        self._drive_origin_cell = cell_center_m(arc['corner'])
        self._drive_heading = heading
        self._arc = self._arc_plan(path, end)
        # 곡선 직후에도 차체가 아직 돌고 있어 GUARD_AFTER_TURN_S 동안 앞 벽 감시를 쉰다
        # (21:30 실기: 곡선 직후 '앞 칸 (9,2)에 장애물'로 1초 STUCK)
        self._turn_heading = heading
        self._turn_ended_at = obs.now
        self._change_state(DRIVE, obs)
        return self._drive(obs)

    def _straight_end(self, obs, path):
        """
        path[1]부터 서지 않고 곧게 갈 수 있는 마지막 칸의 번호.

        복귀·막다른 길 되돌아가기: 지나온 칸이라 지도가 있으므로 같은 방향이 이어지는 데까지
        (복귀는 출발 격자 앞에서 멈춤).
        탐색: 그다음 칸들도 막힘이 아니고 점수가 계속 작아지면(목표 쪽) 이어서 (중앙 옆 칸에서 끝).
        서서 찍은 스캔으로 만든 지도만 쓴다 (주행 중 스캔은 위치가 늦어 쓰지 않음).
        """
        self._chained = False
        step_dir = (path[1][0] - path[0][0], path[1][1] - path[0][1])
        if self.phase == RETURN or self._backtrack is not None:
            end = 1
            # 복귀는 출발 격자 앞에서 멈춘다 (마지막 칸은 큐브를 넣는 전진, _plan의 delivering)
            while (end + 1 < len(path)
                   and (path[end + 1] != MISSION1_START or self._backtrack is not None)
                   and (path[end + 1][0] - path[end][0],
                        path[end + 1][1] - path[end][1]) == step_dir):
                end += 1
            return end
        middle = path[1]
        if (self._sees_target(obs) or self.cube is not None
                or middle in self.grid.neighbors(CENTER)
                or path[0] in self._no_chain_from
                # 후진(뒤쪽 칸)일 때만 묶지 않는다. 90도 옆 칸은 돌고 나서 두 칸 간다.
                # 2026-10-31 실기: 정확히 90도 옆인 (9,2)->(8,2)->(7,2)가 각도 오차 1도로 끊겼다.
                or abs(heading_error_to(obs, cell_center_m(middle))) > 3 * math.pi / 4):
            return 1
        # 2026-10-31 팀 규칙: 앞 두 칸이 다 뚫려 있고 두 번째 칸 점수(목표 거리 + 방문 x 10)가
        # 첫 번째 칸보다 작으면 두 칸을 쭉 가고, 크거나 같으면 한 칸씩 가서 확인한다.
        # 두 번째 칸은 막힘으로 표시되지 않았으면 된다. LiDAR가 약 0.55 m 너머 상자 위로 넘어가
        # 두 번째 칸은 대개 미관측이라, 빈칸 확인을 요구하면 거의 묶이지 않았다 (2026-10-31 실기).
        # 대신 두 칸 주행 중 목표 칸 가운데보다 앞에 벽이 보이면 멈추고 다시 판단한다 (_drive).
        after = (middle[0] + step_dir[0], middle[1] + step_dir[1])
        if not self.grid.in_bounds(after) or self.grid.get(after) == BLOCKED:
            return 1
        # 서서 찍은 스캔에 두 번째 칸 앞면(가운데에서 0.6 m) 근처 벽이 보이면 처음부터 묶지 않는다.
        # 2026-10-31 실기: 묶고 출발하자마자 앞 벽 감시로 서는 일이 잦았다 (실제로 막힌 칸).
        travel_yaw = math.atan2(step_dir[1], step_dir[0])
        if obs.scan is not None and wall_in_direction(
                obs.scan, (obs.x, obs.y, obs.yaw), travel_yaw,
                CELL_SIZE_M * 1.5 - CHAIN_WALL_MARGIN_M, CELL_SIZE_M * 1.5 + CHAIN_WALL_MARGIN_M,
                CHAIN_CHECK_CONE_RAD):
            return 1
        if not self._can_pass(middle, after, step_dir):
            return 1
        path.append(after)
        # 2026-11-01: 세 칸 이상도 같은 규칙(막힘 아님 + 점수가 계속 작아짐)이면 쭉 간다.
        # 먼 칸은 서서 본 스캔으로 확인할 수 없어(약 0.55 m 너머는 레이저가 상자 위로 넘어감)
        # 주행 중 앞 벽 감시(_wall_before_target)에 맡긴다. 중앙 옆 칸에 닿으면 서서 큐브를 본다.
        while not self._next_to_center(path[-1]):
            nxt = (path[-1][0] + step_dir[0], path[-1][1] + step_dir[1])
            if not self._can_pass(path[-1], nxt, step_dir):
                break
            path.append(nxt)
        self._note(f'{len(path) - 1}칸 한 번에: {path[0]} -> {path[-1]}')
        self._chained = True
        return len(path) - 1

    def _can_pass(self, cell, nxt, step_dir):
        """
        cell에서 서지(판단하지) 않고 nxt까지 곧게 가도 되는지.

        nxt가 막힘이 아니고, (5,5)에 가까워지거나(점수가 작아짐) cell에서 새로 판단할 것이 없을 때
        (양옆이 막힘·경기장 밖, 또는 지나온 칸인데 nxt는 안 가 본 칸 -> 어차피 직진).
        2026-11-01 팀 "최대한 안 멈추고": (5,5)에서 멀어지는 통로는 칸마다 줄였다 가속해 주춤했다.
        """
        if (not self.grid.in_bounds(nxt) or self.grid.get(nxt) == BLOCKED
                or self._next_to_center(cell)):
            return False
        if self.maze.state.score(nxt) < self.maze.state.score(cell):
            return True
        visits = self.maze.state.visits
        for side in ((-step_dir[1], step_dir[0]), (step_dir[1], -step_dir[0])):
            n = (cell[0] + side[0], cell[1] + side[1])
            if not self.grid.in_bounds(n) or self.grid.get(n) == BLOCKED:
                continue
            if n in visits and nxt not in visits:
                continue
            return False
        return True

    @staticmethod
    def _next_to_center(cell):
        return abs(cell[0] - CENTER[0]) + abs(cell[1] - CENTER[1]) == 1

    def _new_neighbors(self, cell):
        """안 가 본, 막히지 않은 이웃 칸 (모르는 칸 포함)."""
        visits = self.maze.state.visits
        candidates = ((cell[0] + 1, cell[1]), (cell[0] - 1, cell[1]),
                      (cell[0], cell[1] + 1), (cell[0], cell[1] - 1))
        return [n for n in candidates
                if self.grid.in_bounds(n) and n not in visits and self.grid.get(n) != BLOCKED]

    def _backtrack_path(self, current):
        """
        막다른 길(안 가 본 이웃이 없음)에서 안 가 본 이웃이 있는 가장 가까운 지나온 칸까지의 경로.

        지나온 칸은 지도가 있어 다시 볼 것이 없으므로 복귀처럼 칸마다 서지 않고 한 번에 간다
        (2026-11-01: 예전에는 maze 방문 점수로 한 칸씩 서서 스캔하며 나왔다).
        거리가 같으면 새 이웃이 목표에 가까운 칸. 안 가 본 이웃이 있거나 갈 곳이 없으면 None (maze가 고른다).
        """
        if self._new_neighbors(current):
            return None
        visits = self.maze.state.visits
        traversed = self.maze.traversed_edges()
        came_from = {current: None}
        depth = {current: 0}
        queue = deque([current])
        best, best_key = None, None
        while queue:
            cell = queue.popleft()
            if best_key is not None and depth[cell] > best_key[0]:
                break
            new = self._new_neighbors(cell)
            if new:
                key = (depth[cell], min(self.maze.state.base_value(n) for n in new))
                if best_key is None or key < best_key:
                    best, best_key = cell, key
                continue
            # 지나온 칸은 지도에 막힘으로 바뀌어 있어도 지나갈 수 있다고 본다 (_shortest_path와 같음)
            for n in ((cell[0] + 1, cell[1]), (cell[0] - 1, cell[1]),
                      (cell[0], cell[1] + 1), (cell[0], cell[1] - 1)):
                if n in visits and n not in came_from and self._edge_ok(cell, n, traversed):
                    came_from[n] = cell
                    depth[n] = depth[cell] + 1
                    queue.append(n)
        if best is None:
            return None
        path = [best]
        while came_from[path[-1]] is not None:
            path.append(came_from[path[-1]])
        path.reverse()
        self._note(f'막다른 길 -> 서지 않고 되돌아감: {path[0]} -> {path[-1]}')
        return path

    def _edge_ok(self, a, b, traversed):
        """
        가 본 두 이웃 칸 a, b 사이로 지나가도 되는지: 실제로 지나간 연결이거나, 그 사이에 벽이 보인 적이 없을 때.

        2026-11-03 17:02 실기: (6,5)와 (7,5)를 둘 다 가 봤지만 그 사이는 얇은 판이었다. 칸 지도는 칸 사이 벽을
        나타내지 못해 복귀가 (6,5)->(7,5)로 판을 뚫고 후진했다. (대회 장애물은 칸 크기 상자라 이런 벽은 없지만
        이웃 칸 오판에도 안전하다.)
        """
        edge = frozenset((a, b))
        return edge in traversed or edge not in self._wall_edges

    def _shortest_path(self, current, goals):
        visited = self.maze.visited_cells()
        traversed = self.maze.traversed_edges()
        came_from = {current: None}
        queue = deque([current])
        while queue:
            cell = queue.popleft()
            if cell in goals:
                path = [cell]
                while came_from[path[-1]] is not None:
                    path.append(came_from[path[-1]])
                return path[::-1]
            for n in ((cell[0] + 1, cell[1]), (cell[0] - 1, cell[1]),
                      (cell[0], cell[1] + 1), (cell[0], cell[1] - 1)):
                if n in visited and n not in came_from and self._edge_ok(cell, n, traversed):
                    came_from[n] = cell
                    queue.append(n)
        # 벽 연결을 빼면 길이 없다 (기억이 틀렸을 수 있다) -> 예전처럼 가 본 칸끼리 잇는다
        self._note('복귀: 지나간 길로 이어지지 않아 가 본 칸끼리 잇는다')
        return self._visited_shortest_path(current, goals)

    def _visited_shortest_path(self, current, goals):
        return_grid = GridMap(use_point_symmetry=False)
        visited = self.maze.visited_cells()
        # 실제로 지나온 칸은 지도에 막힘으로 바뀌어 있어도 지나갈 수 있다고 본다.
        # 2026-10-31 실기: 지나온 (7,5)가 대칭 칸 (3,5) 때문에 막힘이 되어 복귀 경로가 없었다.
        for cell in return_grid.cells:
            if cell not in visited:
                return_grid.mark_blocked(cell)
        paths = [return_grid.find_path(current, goal) for goal in goals]
        paths = [path for path in paths if path is not None]
        if not paths:
            return None
        return min(paths, key=len)

    def _release_point(self, from_cell, to_cell):
        """큐브가 to_cell 가운데에 놓이도록 로봇 중심이 멈출 위치."""
        from_x, from_y = cell_center_m(from_cell)
        to_x, to_y = cell_center_m(to_cell)
        length = math.hypot(to_x - from_x, to_y - from_y)
        dir_x = (to_x - from_x) / length
        dir_y = (to_y - from_y) / length

        return (to_x - dir_x * GRIPPER_REACH_M, to_y - dir_y * GRIPPER_REACH_M)

    def _sees_target(self, obs):
        """탐색 중 대상이 충분히 크게 보이는지."""
        return self.phase == EXPLORE and obs.detected and obs.area >= SEE_TARGET_MIN_AREA

    def _remember_cube(self, obs):
        """탐색 중 큐브가 보이면 경기장 위치를 계산해 기억한다 (더 크게 보일 때만 바꾼다)."""
        if not self._sees_target(obs) or obs.area <= self._cube_area:
            return
        bearing = obs.yaw - obs.x_offset * CAMERA_HALF_FOV_RAD
        distance = CAMERA_AHEAD_M + 0.5 * math.sqrt(TARGET_AREA_AT_50CM / obs.area)
        x = obs.x + distance * math.cos(bearing)
        y = obs.y + distance * math.sin(bearing)
        if point_to_cell(x, y) is None:
            return
        self.cube = (x, y)
        self._cube_area = obs.area

    def _cube_goals(self):
        """
        기억한 큐브를 곧게 잡으러 갈 수 있는 큐브 칸의 옆 칸들.

        큐브가 칸 안에서 치우친 방향이 진행 방향과 같아지는 옆 칸만 (직각 치우침 <= 8 cm).
        없으면 직각 치우침이 가장 작은 옆 칸. 막힌 칸, 찾다 실패한 칸은 뺀다.
        """
        if self.cube is None:
            return []
        cube_cell = point_to_cell(*self.cube)
        center = cell_center_m(cube_cell)
        offset = (self.cube[0] - center[0], self.cube[1] - center[1])
        candidates = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            cell = (cube_cell[0] + dx, cube_cell[1] + dy)
            if (not self.grid.in_bounds(cell) or self.grid.get(cell) == BLOCKED
                    or cell in self._searched_goals):
                continue
            # 옆 칸 -> 큐브 칸 방향과 직각인 치우침
            lateral = abs(offset[0] * dy - offset[1] * dx)
            candidates.append((lateral, cell))
        good = [cell for lateral, cell in candidates if lateral <= CUBE_NEIGHBOR_LATERAL_MAX_M]
        if good:
            return good
        return [min(candidates)[1]] if candidates else []

    def _target_approach(self, obs, at=None, commit=True):
        """
        칸 가운데(at, 없으면 지금 위치)에서 대상에 축 방향으로 곧게 다가갈 수 있는지.

        대상이 상하좌우 축에서 ±TARGET_AXIS_TOL_RAD 안이고, 대상 칸까지 사이 칸이 막히지
        않았을 때만 True. commit이면 접근 축과 출발 칸을 기억한다.
        """
        if not self._sees_target(obs):
            return False
        if at is None:
            cell = point_to_cell(obs.x, obs.y)
            if cell is None or distance_to(obs, cell_center_m(cell)) > CELL_CENTER_TOLERANCE_M:
                return False
        else:
            cell = point_to_cell(*at)
        if cell is None or cell in self._approach_blocked:
            return False
        # x_offset 음수 = 왼쪽 = 반시계(+)
        bearing = obs.yaw - obs.x_offset * CAMERA_HALF_FOV_RAD
        axis = round(bearing / (math.pi / 2)) * (math.pi / 2)
        off_axis = math.atan2(math.sin(bearing - axis), math.cos(bearing - axis))
        if abs(off_axis) > TARGET_AXIS_TOL_RAD:
            return False
        distance = 0.5 * math.sqrt(TARGET_AREA_AT_50CM / obs.area)
        if distance * abs(math.sin(off_axis)) > TARGET_LATERAL_MAX_M:
            return False
        cells_away = max(1, round(distance / CELL_SIZE_M))
        step_x, step_y = round(math.cos(axis)), round(math.sin(axis))
        for k in range(1, cells_away + 1):
            between = (cell[0] + k * step_x, cell[1] + k * step_y)
            if not self.grid.in_bounds(between):
                return False
            if k < cells_away and self.grid.get(between) == BLOCKED:
                return False
        if commit:
            self._approach_axis = axis
            self._approach_origin = cell_center_m(cell)
        return True

    def _arrived_at_goal(self, obs):
        if self.phase == EXPLORE:
            self._change_state(FACE, obs)
            return Command()

        # TODO: 출발 격자 안에서 돌아가기 시작한 경우 (보통은 없다)
        return self._deliver(obs)

    def _turn_toward(self, error, obs):
        """IMU 각도 오차에 따라 연속 회전하고 목표각 앞에서 일정하게 감속한다."""
        remaining = max(0.0, abs(error) - TURN_TOLERANCE_RAD)
        speed = min(self._turn_speed(),
                    math.sqrt(MIN_TURN_SPEED ** 2 + 2.0 * TURN_DECEL * remaining))
        return math.copysign(speed, error)

    def _turn(self, obs):
        error = self._travel_heading_error(obs)
        # 지금 돌고 있는 속도로 더 돌 각도를 빼고 판단한다 (빠를수록 일찍 감속·정지).
        predicted = error - self._last_angular * TURN_LEAD_S

        flipped = self._turn_flipped(predicted)
        exit_rad = RETURN_TURN_EXIT_RAD if self.phase == RETURN else TURN_EXIT_RAD
        if abs(predicted) < exit_rad or flipped:
            self._turn_heading = obs.yaw + error
            self._turn_ended_at = obs.now
            self._change_state(DRIVE, obs, self._flip_reason(error) if flipped else None)
            return self._drive(obs)

        return Command(angular=self._turn_toward(predicted, obs))

    def _turn_flipped(self, predicted):
        """남은 각도의 부호가 TURN_FLIP_MAX번 바뀌었는지 (목표를 넘었다 돌아오기 반복)."""
        sign = 1 if predicted > 0.0 else -1
        if self._turn_sign and sign != self._turn_sign:
            self._turn_flips += 1
        self._turn_sign = sign
        return self._turn_flips >= TURN_FLIP_MAX

    def _flip_reason(self, error):
        self._note(f'회전이 목표를 {self._turn_flips}번 넘나듦 -> 남은 {math.degrees(error):+.1f}도에서 끝냄')
        return '회전 왕복 제한'

    def _drive(self, obs):
        # 주행 중 대상이 보여도 바로 꺾지 않는다 (대각선 접근 방지). 칸 가운데에서 판단한다.
        # 단 큐브가 바로 앞이면(적외선, 또는 카메라에 크게) 그 칸으로 밀고 들어가지 않고 진행선으로 바로 잡는다.
        if self.phase == EXPLORE and not self.reverse and not self.delivering:
            if self._tof_cube(obs) and self._camera_confirmed(obs):
                self._begin_approach(obs, self._drive_heading, self._drive_origin_cell)
                return self._start_push(obs)
            if (obs.detected and obs.area >= DRIVE_CUBE_AREA
                    and abs(obs.x_offset) <= DRIVE_CUBE_MAX_OFFSET):
                self._begin_approach(obs, self._drive_heading, self._drive_origin_cell)
                self._change_state(APPROACH, obs, f'주행 중 큐브가 바로 앞 (면적 {obs.area:.0f})')
                return self._approach(obs)
        target_cell = point_to_cell(*self.target)
        if (not self.reverse and obs.scan is not None and self._drive_heading is not None
                and target_cell != self._guard_off_cell and self._wall_before_target(obs)):
            stop_cell = self._cell_before_wall(obs) if self._chained else None
            if stop_cell is not None:
                # 여러 칸 가는 중 앞 칸이 막혀 있었다 -> 그 자리에 급정지하지 않고 벽 바로 앞 칸 가운데를
                # 새 목표로 해 부드럽게 선다 (2026-11-01 20:25 실기: 0.8 m/s 급정지로 칸을 17 cm 지나쳐 후진)
                self.target = cell_center_m(stop_cell)
                target_cell = stop_cell
                self._chained = False
                self._note(f'여러 칸 주행 중 앞에 벽 -> {stop_cell} 가운데에서 선다')
            elif self._chained:
                # 벽 앞 칸까지 멈출 거리가 안 된다 -> 바로 멈추고 칸 가운데에서 다시 판단.
                # 출발 칸에서는 다시 묶지 않는다 (2026-10-31 실기: 같은 계획·정지를 0.3초마다 반복).
                self._no_chain_from.add(point_to_cell(*self._drive_origin_cell))
                self.event = '여러 칸 주행 중 앞에 벽 -> 멈추고 다시 판단'
                self._chained = False
                self._change_state(SCAN, obs)
                return Command()
            else:
                # 한 칸 주행인데 목표 칸에 무언가 있다 (상대 로봇 등). 서서 본 스캔이 빈칸이라고 하는데
                # 같은 목표에서 GUARD_REPEAT_MAX번 연속 멈추면 감시를 잘못 본 것으로 보고 그 목표는 감시 끔.
                same = self._guard_stop_cell == target_cell
                self._guard_stops = self._guard_stops + 1 if same else 1
                self._guard_stop_cell = target_cell
                if self._guard_stops >= GUARD_REPEAT_MAX:
                    self._guard_off_cell = target_cell
                    self.event = f'{target_cell} 앞 벽 감시가 {self._guard_stops}번 연속 -> 이 칸은 감시 끔'
                # 상대 로봇이 지나가는 중일 수 있어 STUCK_RETRY_S 기다렸다가 다시 본다.
                self.stop_reason = f'주행 중 앞 칸 {target_cell}에 장애물 ({self._guard_stops}번째)'
                self._chained = False
                self._change_state(STUCK, obs)
                return Command()
        # 진행 중 다음 칸에 장애물이 나타나면 멈춘 뒤 다시 계획한다.
        if (DRIVE_BLOCK_CHECK and obs.scan is not None
                and obs.scan is not self._last_drive_scan):
            # 이동 중 스캔과 odom의 시각 차이로 벽이 옆 칸에 찍힐 수 있다.
            # 이때는 정지만 하고, 정지 후 받은 스캔으로 영구 지도를 갱신한다.
            # 새 스캔마다 한 번만 세고, 연속 DRIVE_BLOCK_SCANS번 막혀 보일 때만 멈춘다.
            self._last_drive_scan = obs.scan
            blocked, _ = scan_to_cells(obs.scan, (obs.x, obs.y, obs.yaw))
            target_cell = point_to_cell(*self.target)
            current_cell = point_to_cell(obs.x, obs.y)
            if target_cell != current_cell and target_cell in blocked:
                self._drive_block_scans += 1
            else:
                self._drive_block_scans = 0
            if self._drive_block_scans >= DRIVE_BLOCK_SCANS:
                self._drive_block_scans = 0
                self._change_state(SCAN, obs)
                return Command()
        if self._turn_ended_at is not None and obs.now - self._turn_ended_at >= TURN_CHECK_S:
            left = math.atan2(math.sin(self._turn_heading - obs.yaw),
                              math.cos(self._turn_heading - obs.yaw))
            self.event = f'회전 후 오차 {math.degrees(left):+.1f}도 (음수 = 목표를 넘침)'
            self._turn_ended_at = None
        if self.rolling_scan_ok(obs) and self._rolling_decision(obs):
            return self._drive(obs)
        distance = distance_to(obs, self.target)
        error = self._travel_heading_error(obs)

        endpoint_reached = False
        remaining = distance
        lateral = 0.0
        wall_known = False
        if self._drive_heading is not None:
            # 목표 점을 겨누지 않고 계획한 진행선(출발 가운데 -> 목표 가운데)을 따라간다.
            # 2026-10-31 실기: 위치 보정으로 목표 점이 옆으로 옮겨지면 점을 겨누느라 대각선·45도 꺾임.
            # 선에서 벗어나면 최대 DRIVE_MAX_CORRECTION_RAD까지만 틀고, 앞뒤 거리로 도착을 판단한다.
            # 옆으로 많이 벗어난 채 도착하면 다음 계획에서 칸 가운데로 짧게 옮긴다.
            axis = self._drive_heading
            dx, dy = self.target[0] - obs.x, self.target[1] - obs.y
            along = dx * math.cos(axis) + dy * math.sin(axis)
            lateral = -dx * math.sin(axis) + dy * math.cos(axis)  # 진행선이 로봇 왼쪽이면 +
            remaining = along
            wall_known = self.reverse and self._wall_known_beyond(target_cell)
            if self.reverse and obs.scan is not None:
                by_wall = self._reverse_wall_left(
                    obs, along, REVERSE_WALL_MATCH_KNOWN_M if wall_known else REVERSE_WALL_MATCH_M)
                if by_wall is not None and by_wall < remaining:
                    self._reverse_wall_gap = along - by_wall
                    remaining = by_wall
            endpoint_reached = remaining <= self._arrive_tolerance()
            # 목표 바로 앞에서는 작은 옆 오차가 큰 회전각으로 증폭되므로 진행축만 유지한다.
            correction = (0.0 if distance < CELL_CENTER_TOLERANCE_M
                          else clamp(DRIVE_LATERAL_GAIN * lateral, DRIVE_MAX_CORRECTION_RAD))
            desired_yaw = axis + correction + (math.pi if self.reverse else 0.0)
            error = math.atan2(math.sin(desired_yaw - obs.yaw), math.cos(desired_yaw - obs.yaw))

        arc = self._arc is not None and self._arc['corner'] == target_cell
        if arc and remaining <= ARC_RADIUS_M and abs(lateral) > ARC_MAX_LATERAL_M:
            self._note(f'옆 오차 {lateral * 100:+.1f} cm -> 곡선 꺾기 취소, 서서 회전')
            self._arc = None
            arc = False
        if arc and remaining <= ARC_RADIUS_M:
            # 꺾는 칸 가운데 R 앞(칸 경계)에서 서지 않고 원호로 들어간다.
            cx, cy = self._arc_center(self._arc)
            self._arc['radius'] = math.hypot(obs.x - cx, obs.y - cy)
            self._change_state(ARC, obs)
            return self._arc_step(obs)

        # 도착했거나, 가까운 곳에서 목표를 지나쳤으면 (목표가 뒤에 있음) 멈춘다.
        passed = distance < CELL_CENTER_TOLERANCE_M and abs(error) > math.pi / 2
        if distance < self._arrive_tolerance() or passed or endpoint_reached:
            if self._drive_max_lateral > DRIVE_LATERAL_LOG_M:
                self._note(f'진행선 옆 오차 최대 {self._drive_max_lateral * 100:.1f} cm '
                           f'({target_cell}까지)')
            self._drive_max_lateral = 0.0
            if self._reverse_wall_gap > 0.01:
                self._note(f'후진 도착을 뒤 벽으로 봄: 엔코더보다 {self._reverse_wall_gap * 100:.1f} cm '
                           f'먼저 섬 ({target_cell})')
            self._reverse_wall_gap = 0.0
            self._guard_stop_cell = None
            self._guard_stops = 0
            if target_cell == self._guard_off_cell:
                self._guard_off_cell = None
            if self.delivering:
                return self._deliver(obs)
            if self._decided_for == self.target and not self._scan_at_arrival:
                # 달리면서 이미 판단했다 -> 서서 기다리지 않고 바로 돈다.
                self._decided_for = None
                return self._plan(obs)
            if self.phase == RETURN:
                if RETURN_SAFE_TURNS and not self._recentering:
                    # 꺾는 칸: 서서 스캔해 벽으로 위치를 바로잡은 뒤 계획한다 (가운데에서 비켜 있으면 먼저 옮김)
                    self._change_state(SCAN, obs)
                    return Command()
                # 가운데로 옮긴 뒤(또는 RETURN_SAFE_TURNS 끔): 새로 볼 것이 없으니 바로 다음 구간으로
                # (2026-10-31 팀 요청: 왔던 길은 계산 없이 빠르게).
                return self._plan(obs, after_recenter=self._recentering)
            if self._backtrack is not None and target_cell != self._backtrack[-1]:
                # 막다른 길에서 되돌아가는 중 꺾는 칸: 지나온 칸이라 복귀처럼 서지 않는다.
                return self._plan(obs)

            self._change_state(SCAN, obs)
            return Command()

        # 목표에 가까워지면 일정한 감속으로 줄여 최저 속도로 도착한다. (출발 가속은 _limit_accel)
        if wall_known and remaining < REVERSE_WALL_SLOW_ZONE_M:
            slow_cap = REVERSE_WALL_SLOW_SPEED
        else:
            slow_cap = math.inf
        remaining = max(0.0, remaining - ARRIVE_TOLERANCE_M)
        speed = min(self._max_speed(), slow_cap,
                    math.sqrt(MIN_DRIVE_SPEED ** 2 + 2.0 * DRIVE_DECEL * remaining))
        if arc:
            # 원호 시작점(꺾는 칸 가운데 R 앞)에서 ARC_SPEED가 되게 줄인다 (멈추지 않음)
            to_arc = max(0.0, remaining - ARC_RADIUS_M)
            speed = min(self._max_speed(),
                        math.sqrt(self._arc_speed() ** 2 + 2.0 * DRIVE_DECEL * to_arc))
        if obs.scan_time is not None:
            self._has_scan_time = True
        if (self.rolling and self._has_scan_time and not self.delivering
                and self._decided_for != self.target):
            # 판단 전에는 가운데 앞 ROLL_ZONE_M부터 ROLL_SPEED로 달린다 (서지 않고 판단할 수 있게).
            roll_left = max(0.0, remaining - ROLL_ZONE_M)
            roll = math.sqrt(ROLL_SPEED ** 2 + 2.0 * DRIVE_DECEL * roll_left)
            speed = min(speed, roll)
        angular = clamp(DRIVE_HEADING_GAIN * error, self._turn_speed())
        # 방향이 틀어져 있으면 그만큼 천천히 가며 맞추고(사선 방지), 많이 틀어져 있으면 제자리에서 먼저 돈다.
        # 진행선으로 돌아오려고 일부러 트는 각도(옆 오차 보정)는 빼고 진행선 기준으로 본다.
        misaligned = error
        if self._drive_heading is not None:
            facing = self._drive_heading + (math.pi if self.reverse else 0.0)
            misaligned = math.atan2(math.sin(obs.yaw - facing), math.cos(obs.yaw - facing))
        linear = self._heading_limited(speed, misaligned)
        if DRIVE_SLOW_FOR_WALLS and abs(lateral) > DRIVE_LATERAL_SLOW_M:
            linear = min(linear, DRIVE_LATERAL_SLOW_SPEED)
        if SIDE_GUARD_ENABLED and obs.scan is not None:
            left, right = side_clearance(obs.scan)
            # 오른쪽 벽이 가까우면 왼쪽(+)으로, 왼쪽 벽이 가까우면 오른쪽으로 (후진이면 반대로 돌아야 같은 쪽으로 비킨다)
            push = max(0.0, SIDE_GUARD_M - right) - max(0.0, SIDE_GUARD_M - left)
            if push:
                angular = clamp(angular + (-1.0 if self.reverse else 1.0) * SIDE_GUARD_GAIN * push,
                                self._turn_speed())
                if DRIVE_SLOW_FOR_WALLS:
                    linear = min(linear, SIDE_GUARD_SPEED)
                if not self._side_guard_noted and min(left, right) < SIDE_GUARD_M - 0.02:
                    side = '오른쪽' if push > 0.0 else '왼쪽'
                    self._note(f'옆 벽 감시: {side} 벽이 축에서 {min(left, right) * 100:.1f} cm -> 비켜 감')
                    self._side_guard_noted = True
        self._drive_max_lateral = max(self._drive_max_lateral, abs(lateral))
        return Command(linear=-linear if self.reverse else linear, angular=angular)

    @staticmethod
    def _heading_limited(speed, error):
        """방향 오차에 따른 직진 속도: DRIVE_FULL_SPEED_RAD까지 그대로, DRIVE_PIVOT_RAD에서 0."""
        if abs(error) >= DRIVE_PIVOT_RAD:
            return 0.0
        scale = min(1.0, (DRIVE_PIVOT_RAD - abs(error)) / (DRIVE_PIVOT_RAD - DRIVE_FULL_SPEED_RAD))
        return max(MIN_DRIVE_SPEED, speed * scale)

    def _cell_before_wall(self, obs):
        """
        여러 칸 주행 중 앞에 벽(self._guard_ahead, 축 기준 거리)이 보일 때 부드럽게 설 수 있는 칸.

        진행선 위 칸 중 칸 가운데 + 반 칸(앞면)이 벽 앞(여유 5 cm)이고, 지금 속도로 DRIVE_DECEL 감속해
        가운데에 설 수 있는 가장 먼 칸. 없으면 None (바로 멈춘다).
        """
        if self._guard_ahead is None or not self.path:
            return None
        cos_h, sin_h = math.cos(self._drive_heading), math.sin(self._drive_heading)
        braking = self._last_linear ** 2 / (2.0 * DRIVE_DECEL)
        best = None
        for cell in self.path:
            cx, cy = cell_center_m(cell)
            along = (cx - obs.x) * cos_h + (cy - obs.y) * sin_h
            if along < braking or along + CELL_SIZE_M / 2.0 > self._guard_ahead + 0.05:
                continue
            if best is None or along > best[0]:
                best = (along, cell)
        return None if best is None else best[1]

    def _wall_before_target(self, obs):
        """
        두 칸 주행 중 목표 칸 가운데보다 앞에 벽이 보이는지 (두 번째 칸이 막혀 있었다).

        목표 칸 너머 벽(막다른 칸)은 목표 가운데에서 0.2 m 뒤라 멈추지 않는다.
        2026-10-31 실기: 집게 앞 20 cm만 보고 멈추니 막다른 칸 (7,2)에 다 와서 너머 벽에 섰다.
        """
        if self._turn_ended_at is not None and obs.now - self._turn_ended_at < GUARD_AFTER_TURN_S:
            return False
        heading_error = math.atan2(math.sin(self._drive_heading - obs.yaw),
                                   math.cos(self._drive_heading - obs.yaw))
        if (abs(self._last_angular) > GUARD_MAX_TURN_RATE
                or abs(heading_error) > GUARD_MAX_HEADING_RAD):
            return False
        # 스캔을 찍던 순간의 위치로 벽 거리와 목표 거리를 같이 잰다. 지금 위치를 쓰면 스캔이 늦게 온 만큼
        # (0.6 m/s, 0.15~0.2 s면 9~12 cm) 벽이 멀게 보여, 칸 사이 벽(목표 가운데 0.2 m 앞, 여유 8 cm)을 끝내
        # 못 잡았다 (2026-11-03 22:12 실기: (9,5)->(6,5) 세 칸 중 (7,5)/(6,5) 사이 벽을 뚫음, 시뮬레이션 재현).
        sx, sy, _ = self.scan_pose(obs)
        cos_h, sin_h = math.cos(self._drive_heading), math.sin(self._drive_heading)
        ahead = nearest_ahead(obs.scan, (sx, sy, obs.yaw), self._drive_heading,
                              CHAIN_GUARD_HALF_WIDTH_M)
        # _cell_before_wall은 지금 위치 기준 거리를 쓴다
        moved = (obs.x - sx) * cos_h + (obs.y - sy) * sin_h
        self._guard_ahead = None if ahead is None else ahead - moved
        if ahead is None or ahead > CHAIN_GUARD_RANGE_M:
            return False
        dx, dy = self.target[0] - sx, self.target[1] - sy
        to_target = dx * cos_h + dy * sin_h
        return ahead < to_target - CHAIN_WALL_MARGIN_M

    def _arrive_tolerance(self):
        """칸 가운데 이 거리 안이면 도착: 가운데로 옮기기·후진은 1 cm, 그 밖(전진)은 ARRIVE_TOLERANCE_M."""
        if self._recentering:
            return RECENTER_ARRIVE_M
        return REVERSE_ARRIVE_M if self.reverse else ARRIVE_TOLERANCE_M

    def _wall_known_beyond(self, target_cell):
        """주행 방향으로 목표 칸 바로 너머가 막힘이거나 경기장 밖인지 (그 벽까지 거리를 믿을 수 있다)."""
        if target_cell is None or self._drive_heading is None:
            return False
        beyond = (target_cell[0] + round(math.cos(self._drive_heading)),
                  target_cell[1] + round(math.sin(self._drive_heading)))
        return not self.grid.in_bounds(beyond) or self.grid.get(beyond) == BLOCKED

    def _reverse_wall_left(self, obs, along, match=REVERSE_WALL_MATCH_M):
        """
        후진 중 차체 뒤(진행 방향) 벽까지 거리로 본 목표 가운데까지 남은 거리.

        목표 칸 너머 벽(가운데에서 반 칸 뒤)으로 보이지 않으면 None (지나가는 칸, 옆 상자 등).
        스캔은 찍힌 뒤 움직인 만큼 빼서 지금 위치 기준으로 바꾼다.
        """
        sx, sy, syaw = self.scan_pose(obs)
        wall = nearest_ahead(obs.scan, (sx, sy, syaw), self._drive_heading,
                             CHAIN_GUARD_HALF_WIDTH_M)
        if wall is None:
            return None
        cos_a, sin_a = math.cos(self._drive_heading), math.sin(self._drive_heading)
        moved = (obs.x - sx) * cos_a + (obs.y - sy) * sin_a
        left = wall - moved - CELL_SIZE_M / 2.0
        if abs(left - along) > match:
            return None
        return left

    def _travel_heading_error(self, obs):
        """후진할 때 목표 반대쪽으로 차체 정면을 유지한다."""
        error = heading_error_to(obs, self.target)
        if self.reverse:
            error = math.atan2(math.sin(error + math.pi), math.cos(error + math.pi))
        return error

    def _stuck(self, obs):
        # 상대 로봇이 길을 막고 있을 수 있다. 기다렸다가 다시 스캔한다.
        # (상대가 떠났으면 다시 볼 때 neighbor_states가 빈칸으로 바꾼다)
        if obs.now - self.state_started_at < STUCK_RETRY_S:
            return Command()

        self._change_state(SCAN, obs)
        return Command()

    # ------------------------------------------------------------
    # 큐브 잡기 (FACE -> SEARCH / ALIGN -> APPROACH -> PUSH -> GRASP)
    # ------------------------------------------------------------

    def _face(self, obs):
        # 기억한 큐브가 있으면 큐브 칸 쪽, 없으면 중앙을 본다.
        cube_cell = point_to_cell(*self.cube) if self.cube is not None else None
        error = heading_error_to(obs, cell_center_m(cube_cell) if cube_cell else CENTER_M)

        if abs(error) < TURN_TOLERANCE_RAD:
            self._change_state(ALIGN if self._target_approach(obs) else SEARCH, obs)
            return Command()

        return Command(angular=self._turn_toward(error, obs))

    def _search(self, obs):
        if self._target_approach(obs):
            self._change_state(ALIGN, obs)
            return Command()
        # 대상이 보이는데 이 칸에서 축 방향으로 닿을 수 없으면 더 돌아도 소용없다
        # (축 판단은 로봇 방향이 아니라 대상 위치로 정해진다). 2026-10-31 실기: 계속 360도 돎.
        unreachable = self._sees_target(obs)
        if self.phase == EXPLORE and (
                unreachable or obs.now - self.state_started_at > SEARCH_GIVE_UP_S):
            cell = point_to_cell(obs.x, obs.y)
            self._searched_goals.add(cell)
            reason = '대상이 보이지만 축 방향이 아님' if unreachable else '대상을 못 찾음'
            self.event = f'{cell}에서 {reason} -> 목표에서 빼고 다시 탐색'
            if not unreachable:
                # 기억한 위치에 큐브가 없었다 (잘못 잰 거리 또는 상대가 옮김) -> 잊고 다시 찾는다
                self.cube = None
                self._cube_area = 0.0
            self._change_state(SCAN, obs)
            return Command()

        # TODO: 큐브가 중앙에 없을 때 (상대가 먼저 가져감) 대응
        return Command(angular=SEARCH_SPEED)

    def _align(self, obs):
        """제자리에서 접근 축 방향으로 정확히 돈다 (대상 쪽이 아니라 통로 방향)."""
        if self._approach_axis is None:
            self._change_state(SCAN, obs)
            return Command()
        error = math.atan2(math.sin(self._approach_axis - obs.yaw),
                           math.cos(self._approach_axis - obs.yaw))
        predicted = error - self._last_angular * TURN_LEAD_S
        flipped = self._turn_flipped(predicted)
        if abs(predicted) < TURN_TOLERANCE_RAD or flipped:
            self._begin_approach(obs, self._approach_axis, self._approach_origin)
            self._change_state(APPROACH, obs, self._flip_reason(error) if flipped else None)
            return self._approach(obs)
        return Command(angular=self._turn_toward(predicted, obs))

    def _begin_approach(self, obs, axis, origin):
        """접근 축·출발점과 면적 기록을 새로 시작한다."""
        self._approach_axis = axis
        self._approach_origin = origin
        self._target_seen_at = obs.now
        self._area_best = 0.0
        self._area_best_at = (obs.x, obs.y)
        self._approach_start = (obs.x, obs.y)
        self._area_first = None
        self._seen_frames = 0

    def _tof_cube(self, obs, within=GRAB_TOF_M):
        """
        적외선에 within 안으로 무언가 보이는데 LiDAR(17 cm 높이)는 집게 앞에 벽을 못 보면 낮은 큐브다.

        칸 가운데에서 막힌 칸을 보고 서면 적외선에 벽(약 3~5 cm)이 보이므로 LiDAR로 벽을 걸러낸다.
        """
        if obs.gripper_range is None or obs.gripper_range > within:
            return False
        if within <= GRAB_TOF_M:
            # 5 cm 안은 집게 사이뿐이다: 센서가 집게 끝보다 약 9 cm 뒤라 벽은 그 전에 집게 끝에 닿는다
            # (실측: 잡은 큐브 28 mm). 큐브가 벽에 붙어 있어도 잡게 벽 확인을 하지 않는다 (21:43 실기)
            return True
        wall = obs.scan is not None and obstacle_ahead(
            obs.scan, (obs.x, obs.y, obs.yaw), ahead=within + 0.05)
        return not wall

    def _camera_confirmed(self, obs):
        """카메라가 대상을 CAMERA_CONFIRM_S 넘게 이어서 봤고, 마지막으로 본 지 TOF_GRAB_CAMERA_S 안인지."""
        return (self._camera_seen_at is not None
                and obs.now - self._camera_seen_at <= TOF_GRAB_CAMERA_S
                and self._camera_seen_at - self._camera_streak_start >= CAMERA_CONFIRM_S)

    def _start_push(self, obs):
        self._push_start = (obs.x, obs.y)
        self._push_tof = obs.gripper_range is not None
        self._change_state(PUSH, obs)
        return self._push(obs)

    def _abort_approach(self, obs, reason):
        """접근을 그만두고 칸 가운데로 돌아가 다시 판단한다."""
        self._approach_blocked.add(point_to_cell(*self._approach_origin))
        self._change_state(SCAN, obs, f'접근 중단: {reason}')
        return Command()

    def _approach(self, obs):
        if self._tof_cube(obs) and self._camera_confirmed(obs):
            return self._start_push(obs)
        if obs.detected:
            self._target_seen_at = obs.now
        elif obs.now - self._target_seen_at > APPROACH_LOST_S:
            return self._abort_approach(obs, '대상을 놓침')

        if self._approach_start is None:
            self._approach_start = (obs.x, obs.y)
        self._seen_frames = self._seen_frames + 1 if obs.detected else 0
        if obs.detected and self._area_first is None:
            self._area_first = obs.area
        if obs.detected and obs.area > self._area_best * AREA_GROW_RATIO:
            self._area_best = obs.area
            self._area_best_at = (obs.x, obs.y)
        traveled = math.dist((obs.x, obs.y), self._approach_start)
        grown = (self._area_first is not None
                 and self._area_best >= self._area_first * GRAB_MIN_GROWTH)
        pushing_cube = self._tof_cube(obs, TOF_NEAR_M)
        if not grown and traveled >= APPROACH_FALSE_TRAVEL_M and not pushing_cube:
            return self._abort_approach(
                obs, f'{traveled:.2f} m 다가가도 면적이 {self._area_first:.0f} -> '
                     f'{self._area_best:.0f} (오탐?)')
        stalled = (obs.detected and obs.area >= APPROACH_SLOW_AREA
                   and math.dist((obs.x, obs.y), self._area_best_at) >= APPROACH_STALL_M)
        # 연속으로 보이고, 실제로 다가갔고, 면적이 커졌을 때만 잡는다 (붙박인 빨간 물체 오탐 방지)
        trusted = (self._seen_frames >= GRAB_MIN_FRAMES and traveled >= GRAB_MIN_TRAVEL_M
                   and grown) or pushing_cube
        if obs.detected and trusted and (obs.area >= GRAB_AREA or stalled):
            reason = (f'면적 {obs.area:.0f} >= {GRAB_AREA:.0f}' if obs.area >= GRAB_AREA
                      else f'면적이 {obs.area:.0f}에서 안 커짐 (큐브가 닿음)')
            reason += f', 처음 {self._area_first or 0:.0f}, {traveled:.2f} m 다가감'
            self._push_start = (obs.x, obs.y)
            self._push_tof = obs.gripper_range is not None
            self._change_state(PUSH, obs, reason)
            return self._push(obs)
        # 적외선에 무언가 가까이 보이면(큐브) 벽 바로 앞 큐브까지 집게가 들어가게 감시 거리를 줄인다
        # (21:43 실기: 벽에 붙은 큐브 앞에서 집게 끝이 벽 6 cm 앞에 와 '집게 앞에 장애물'로 그만둠)
        near_cube = obs.gripper_range is not None and obs.gripper_range <= TOF_NEAR_M
        guard = APPROACH_WALL_GUARD_NEAR_CUBE_M if near_cube else GUARD_AHEAD_M
        if obs.scan is not None and obstacle_ahead(obs.scan, (obs.x, obs.y, obs.yaw), ahead=guard):
            return self._abort_approach(obs, '집게 앞에 장애물')

        # 통로 가운데 선에서 옆으로 벗어난 거리 (진행 방향 왼쪽이 +)
        axis = self._approach_axis
        lateral = (-(obs.x - self._approach_origin[0]) * math.sin(axis)
                   + (obs.y - self._approach_origin[1]) * math.cos(axis))
        heading = math.atan2(math.sin(obs.yaw - axis), math.cos(obs.yaw - axis))
        if abs(lateral) > APPROACH_LATERAL_MAX_M or not obs.detected:
            # 가운데 선으로 돌아온다 (대상을 잠깐 놓쳤을 때도 축을 따라 곧게).
            offset = -clamp(APPROACH_LATERAL_GAIN * lateral, APPROACH_MAX_HEADING_RAD)
        else:
            # 카메라로 큐브 쪽을 보되 축에서 APPROACH_MAX_HEADING_RAD까지만 튼다.
            bearing = heading - obs.x_offset * CAMERA_HALF_FOV_RAD
            offset = clamp(bearing, APPROACH_MAX_HEADING_RAD)
        desired = axis + offset
        error = math.atan2(math.sin(desired - obs.yaw), math.cos(desired - obs.yaw))
        speed = APPROACH_SLOW_SPEED if obs.area >= APPROACH_SLOW_AREA else APPROACH_SPEED
        return Command(linear=speed, angular=clamp(HEADING_GAIN * error, TURN_SPEED))

    def _mouth_stalled(self, obs):
        """밀기 중 큐브가 집게 입구에 있는데 GRAB_TOF_STALL_S 동안 더 안 들어오는지 (같이 밀려감)."""
        if self._mouth_for != self.state_started_at:
            self._mouth_for = self.state_started_at
            self._mouth_best = None
        distance = obs.gripper_range
        if distance is None or distance > GRAB_TOF_MOUTH_M:
            return False
        if self._mouth_best is None or distance < self._mouth_best[0] - GRAB_TOF_PROGRESS_M:
            self._mouth_best = (distance, obs.now)
            return False
        return obs.now - self._mouth_best[1] >= GRAB_TOF_STALL_S

    def _push(self, obs):
        """GRAB_AREA에 닿은 뒤 GRAB_PUSH_M만큼 곧게 더 들어가 큐브를 집게 안쪽까지 넣고 닫는다."""
        # 큐브가 너무 가까워 카메라 값(잘림·놓침)은 믿지 않고 엔코더 거리만 본다.
        pushed = math.hypot(obs.x - self._push_start[0], obs.y - self._push_start[1])
        if self._push_tof and self._grab_sent_at is None:
            # 적외선: 큐브가 집게 안쪽(GRAB_TOF_CLOSE_M)까지 들어오면 닫는다. 안 들어오면 닫지 않고 다시 판단
            if obs.gripper_range is not None and obs.gripper_range <= GRAB_TOF_CLOSE_M:
                self._change_state(GRASP, obs, f'적외선 {obs.gripper_range * 1000:.0f} mm')
                self._grab_sent_at = obs.now
                return Command(gripper=GRIPPER_GRAB)
            if self._mouth_stalled(obs):
                self._change_state(
                    GRASP, obs, f'적외선 {obs.gripper_range * 1000:.0f} mm에서 더 안 들어옴 -> 입구에서 닫음')
                self._grab_sent_at = obs.now
                return Command(gripper=GRIPPER_GRAB)
            too_long = obs.now - self.state_started_at > GRAB_PUSH_TIMEOUT_S
            if pushed >= GRAB_TOF_PUSH_MAX_M or too_long:
                return self._abort_approach(obs, f'{pushed:.2f} m 밀어도 적외선에 안 들어옴')
            return Command(linear=GRAB_PUSH_SPEED)
        if pushed >= GRAB_PUSH_M or obs.now - self.state_started_at > GRAB_PUSH_TIMEOUT_S:
            closing = self._grab_sent_at is not None
            self._change_state(GRASP, obs)
            if closing:
                return Command()
            self._grab_sent_at = obs.now
            return Command(gripper=GRIPPER_GRAB)
        command = Command(linear=GRAB_PUSH_SPEED)
        if self._grab_sent_at is None and pushed >= GRAB_PUSH_M - GRAB_CLOSE_LEAD_M:
            # 남은 거리를 밀면서 닫기 시작한다.
            command.gripper = GRIPPER_GRAB
            self._grab_sent_at = obs.now
        return command

    def _gripper_done(self, obs, wanted, max_wait, since=None):
        """집게가 wanted 상태에 닿고 GRIPPER_SETTLE_S가 지났거나, 최대 시간이 지났는지."""
        waited = obs.now - (self.state_started_at if since is None else since)
        if waited >= max_wait:
            return True
        if waited < GRIPPER_MIN_WAIT_S or obs.gripper_state != wanted:
            return False
        if self._gripper_reached_at is None:
            self._gripper_reached_at = obs.now
        return obs.now - self._gripper_reached_at >= GRIPPER_SETTLE_S

    def _grasp(self, obs):
        # 닫기는 밀어 넣는 동안 이미 시작했을 수 있다 -> 명령을 보낸 시각부터 센다.
        if not self._gripper_done(obs, GRIPPER_CLOSED, GRASP_WAIT_S, since=self._grab_sent_at):
            return Command()
        self._grab_sent_at = None

        if obs.holding is None and HOLDING_UNKNOWN == 'camera':
            # 잡았는지 알려 주는 센서가 없다 -> 큐브가 집게 앞(화면)에 크게 보이는지로 판단한다.
            held = obs.detected and obs.area >= HELD_MIN_AREA
            how = f'카메라 확인 (detected={obs.detected}, area={obs.area:.0f}, 기준 {HELD_MIN_AREA:.0f})'
        elif obs.holding is None:
            held, how = True, '집게 센서 없음 -> 잡았다고 가정 (HOLDING_UNKNOWN=assume)'
        else:
            held, how = obs.holding, f'/gripper/holding={obs.holding}'
        if not held:
            self._change_state(SEARCH, obs, f'못 잡음: {how}')
            return Command(gripper=GRIPPER_OPEN)

        self.phase = RETURN
        self._change_state(SCAN, obs, f'잡음: {how}')
        return Command()

    # ------------------------------------------------------------
    # 놓기 (RELEASE -> DONE)
    # ------------------------------------------------------------

    def _deliver(self, obs):
        """
        출발 칸에 큐브를 가져왔다.

        규정 3.3.4: 시작 지점으로 '운송'하면 승리, 문의 16: 블록 전체가 시작 격자 안이면 인정.
        2026-10-31에는 놓지 않고 끝냈으나 2026-11-01 팀 결정으로 다시 놓는다 (RELEASE_AT_START).
        놓는 위치는 _release_point (큐브가 격자 가운데).
        """
        if RELEASE_AT_START:
            self._change_state(RELEASE, obs)
            return Command(gripper=GRIPPER_OPEN)
        self._change_state(DONE, obs)
        return Command()

    def _release(self, obs):
        if not self._gripper_done(obs, GRIPPER_OPENED, RELEASE_WAIT_S):
            return Command()

        self._change_state(DONE, obs)
        return Command()

    def _done(self, obs):
        # 놓은 뒤에는 움직이지 않는다. (상대가 가져가지 못하게 막는 효과도 있다)
        return Command()
