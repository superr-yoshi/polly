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

from dataclasses import dataclass
import math
from typing import Optional

from poli_navigation.grid_map import (
    BLOCKED,
    cell_center_m,
    CELL_SIZE_M,
    CENTER,
    GridMap,
    MISSION1_START,
)
from poli_navigation.mission1_maze import Mission1Maze
from poli_navigation.mission2_logic import (
    clamp,
    Command,
    distance_to,
    GRIPPER_GRAB,
    GRIPPER_OPEN,
    heading_error_to,
)
from poli_navigation.scan_to_grid import (
    nearest_ahead,
    neighbor_states,
    obstacle_ahead,
    point_to_cell,
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
SCAN_SETTLE_S = 0.1
# 주행 중 다음 칸이 막혀 보이는 스캔이 이만큼 연속이어야 멈춘다.
# 출발 순간 차체가 숙여져 LiDAR가 바닥·장애물 윗변을 앞칸 장애물로 보는 것을 거른다 (2026-10-31 실기)
DRIVE_BLOCK_SCANS = 3
# 주행 중 안전 확인 켜기/끄기. 2026-10-31 실기 시험을 위해 끔 (팀 요청). 대회 전 다시 검토 (TODO)
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
DRIVE_SPEED = 0.45  # 2026-10-31 0.35 -> 0.45 (전체가 느림, 팀 요청). hardware.yaml max_linear와 같게
MIN_DRIVE_SPEED = HW_MIN_WHEEL_SPEED
# 도착 감속 (m/s^2): 남은 거리 d에서 sqrt(MIN^2 + 2 * DRIVE_DECEL * d)로 일정하게 줄인다.
# 예전 1.5 * d는 끝에서 너무 오래 기어가고 0.08 m/s에서 뚝 멈췄다 (2026-10-31 실기).
# 40 cm 한 칸 약 2.0초 -> 약 1.6초. TODO_MEASURE: 도착 위치가 밀리면 낮춘다
DRIVE_DECEL = 0.5
# 모든 상태 공통 가감속 제한 (step()의 _limit_accel). 급출발·급정지 방지.
# 속도를 줄일 때는 더 빨리 줄여 계획하지 않은 정지(대상 발견 등)에서 덜 밀리게 한다.
# TODO_MEASURE: 출발할 때 앞이 들리거나 바퀴가 헛돌면 LINEAR_ACCEL을 낮춘다
LINEAR_ACCEL = 0.5   # m/s^2, 0 -> 0.45 m/s 약 0.9초
LINEAR_DECEL = 1.2   # m/s^2, 0.45 m/s -> 0 약 0.4초 (약 8 cm)
ANGULAR_ACCEL = 6.0  # rad/s^2, 0 -> 1.8 rad/s 약 0.3초
# 이만큼 판단이 끊기면 그동안 노드가 정지 명령을 보냈다고 보고 0에서 다시 가속한다.
ACCEL_RESET_S = 0.15
# 카메라 접근: 멀 때는 빠르게, 큐브가 커 보이면 천천히 (면적 판단 지연으로 지나치지 않게)
APPROACH_SPEED = 0.2  # 2026-10-31 0.15 -> 0.2 (전체가 느림)
APPROACH_SLOW_SPEED = 0.12
APPROACH_SLOW_AREA = 20000.0  # TODO_MEASURE: 잡는 위치 약 15 cm 전
# 2026-10-31 1.2 -> 1.8 (회전이 느리고 끊김). hardware.yaml max_angular 2.0 안.
# TODO_MEASURE: 목표각을 지나쳐 되돌아 돌면 TURN_SPEED 또는 TURN_DECEL을 낮춘다 (IMU 지연)
TURN_SPEED = 1.8
TURN_DECEL = 4.0  # rad/s^2, 목표각 앞에서 일정하게 감속해 MIN_TURN_SPEED로 도착
# 연속 회전 감속 하한. 바퀴 속도 약 0.084 m/s = 하드웨어 최저 속도(0.08) 근처.
# 더 낮추면 바퀴가 섰다 갔다 하며 끊겨 돈다.
MIN_TURN_SPEED = 0.6
# 2026-10-31 0.6 -> 1.0 (잡기 전체가 느림). 카메라 20 Hz면 한 번에 약 3도라 놓치지 않는다.
SEARCH_SPEED = 1.0
HEADING_GAIN = 1.5
# 진행선에서 옆으로 벗어났을 때 돌아오는 방향 = 옆 오차 x 이득, 최대 각도
DRIVE_LATERAL_GAIN = 2.0  # rad/m (5 cm -> 약 6도)
DRIVE_MAX_CORRECTION_RAD = math.radians(15.0)

# TODO_MEASURE: 펄스 한 번 회전량(turn_pulse.PULSE_S)보다 범위(±)가 넓어야 멈출 수 있다
TURN_TOLERANCE_RAD = math.radians(5.0)
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
ROLLING_DECISION = False
# TODO_MEASURE: 판단 속도. 스캔 한 바퀴(0.1초) 동안 2 cm 움직인다. 지도가 이상하면 낮춘다
ROLL_SPEED = 0.2
ROLL_ZONE_M = 0.14     # 칸 가운데 이 거리 앞부터 ROLL_SPEED로 달린다
ROLL_WINDOW_M = 0.10   # 스캔을 찍은 위치가 칸 가운데에서 이 안이어야 판단에 쓴다
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
CHAIN_GUARD_RANGE_M = 0.6
AREA_GROW_RATIO = 1.05

# 큐브(50mm)가 이 크기 이상으로 보이면 집을 수 있는 거리 (카메라 픽셀 면적, 640x480)
# 2026-10-30 실측 (주황 시험 대상, 같은 크기): 열린 집게 사이 잡는 위치에서 area 약 33000
# (아래가 화면 밖으로 잘려 사실상 최댓값) -> 집게 안에 들어온 순간 잡도록 조금 낮춤 (TODO_MEASURE: 경기장에서 재확인)
# TODO_MEASURE: 2026-10-31 사용자 요청으로 35000. 실제 접근 중 이 면적에 도달하는지 재확인.
# 35000에서도 큐브가 집게 안쪽까지 안 들어왔다 (면적이 화면 아래 잘림으로 포화) ->
# 면적을 더 올리지 않고 GRAB_AREA에 닿은 뒤 엔코더로 GRAB_PUSH_M만큼 더 들어가서 닫는다.
GRAB_AREA = 35000.0
# TODO_MEASURE: 큐브가 집게 안쪽 끝에 닿는 거리. 아직 얕으면 늘리고, 큐브를 앞으로 밀어내면 줄인다
GRAB_PUSH_M = 0.04
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


def ramp(target, last, accel, decel, dt):
    """last에서 target으로 dt 동안 accel(빨라질 때)·decel(느려질 때)만큼만 바꾼다."""
    if target * last < 0.0:
        # 방향이 바뀌면 먼저 0까지 줄인다.
        target = 0.0
    rate = accel if abs(target) > abs(last) else decel
    step = rate * dt
    return last + max(-step, min(step, target - last))


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


class Mission1Logic:

    def __init__(self, point_symmetry=True):
        # point_symmetry: 대회 규정상 장애물은 점대칭이라 한 칸이 막히면 반대편도 막힘으로 칠한다.
        # 대칭이 아닌 연습 경기장에서는 끈다 (start_mission.sh --test).
        self.grid = GridMap(use_point_symmetry=point_symmetry)
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
        self._scan_at_arrival = False
        # 두 칸을 한 번에 가는 중인지 (두 번째 칸이 미관측일 수 있어 앞을 계속 본다)
        self._chained = False
        # 두 칸 주행 중 앞 벽으로 멈춘 칸 (그 칸에서는 다시 묶지 않는다)
        self._no_chain_from = set()
        self._drive_origin_cell = None
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

    def _change_state(self, new_state, obs):
        self.state = new_state
        self.state_started_at = obs.now
        self.turn_pulse.reset()
        if new_state not in (TURN, DRIVE):
            self._chained = False
        if new_state != DRIVE:
            # 다른 일(대상 발견 등)로 주행이 끊기면 미리 한 판단은 버린다.
            self._decided_for = None
            self._scan_at_arrival = False
        self._gripper_reached_at = None

    def step(self, obs):
        self.maze.observe(point_to_cell(obs.x, obs.y), obs.yaw)
        if self.state_started_at is None:
            self.state_started_at = obs.now
        self._remember_cube(obs)

        command = self._check_dropped(obs)
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
        command.linear = ramp(command.linear, self._last_linear, LINEAR_ACCEL, LINEAR_DECEL, dt)
        if abs(command.linear - self._last_linear) > 0.005:
            self._speed_changed_at = now
        command.angular = ramp(
            command.angular, self._last_angular, ANGULAR_ACCEL, ANGULAR_ACCEL, dt)
        self._last_linear = command.linear
        self._last_angular = command.angular
        self._last_command_at = now
        return command

    def _check_dropped(self, obs):
        # 돌아가는 중에 큐브를 놓치면 다시 찾아서 잡는다.
        if self.phase != RETURN or obs.holding is not False:
            return None
        if self.state not in (SCAN, TURN, DRIVE):
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

        self._map_scan(obs.scan, (obs.x, obs.y, obs.yaw))
        if self._target_approach(obs):
            self._change_state(ALIGN, obs)
            return Command()
        return self._plan(obs)

    def _map_scan(self, scan, pose, center=None):
        """
        스캔으로 상하좌우 이웃 칸만 지도에 반영한다 (방향별 ±15도 레이저, center 기준 거리).

        멀리 있는 칸까지 광선으로 칠하는 갱신(update_grid_from_scan)은 쓰지 않는다. 멀리 맞은 점은
        위치·방향 오차가 커져 통로 칸을 막힘으로 잘못 찍었다 (2026-10-31 실기: (8,3)을 못 골라
        돌아감, 주행 중 스캔으로는 STUCK·왕복). 미로는 이웃 칸만 쓰고, 먼 칸은 '모름'으로 두었다가
        옆에 가서 직접 본다. 시뮬레이션 규정 예시 63.4 s -> 63.5 s로 시간 차이 없음.
        """
        for cell, state in neighbor_states(scan, pose, center).items():
            if not self.grid.in_bounds(cell):
                continue
            if state == BLOCKED:
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
        if not ROLLING_DECISION or self.state != DRIVE or self.delivering or self.target is None:
            return False
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
        """달리는 중에 다음 칸을 정한다. 직진이면 목표를 바꿔 계속 가고 True."""
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
        (self.target, self._drive_heading, self.reverse, self.delivering, self.path,
         self.state, self.state_started_at, self.stop_reason) = saved
        self._decided_for = self.target
        self.event = '주행 중 판단: 가운데에 서서 회전 (판단 대기 없음)'
        return False

    def _plan(self, obs):
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
        if cube_goals:
            # 기억한 큐브의 옆 칸까지 지도(모르는 칸은 지나갈 수 있다고 봄)로 최단 경로
            paths = [self.grid.find_path(current, goal) for goal in cube_goals]
            paths = [p for p in paths if p is not None]
            path = min(paths, key=len)[:2] if paths else None
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

        self.path = path
        center = cell_center_m(current)
        drive_origin = center
        self._drive_origin_cell = center

        if distance_to(obs, center) > CELL_CENTER_TOLERANCE_M:
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

        self._drive_heading = math.atan2(
            self.target[1] - drive_origin[1], self.target[0] - drive_origin[0])
        self._change_state(TURN, obs)
        return Command()

    def _straight_end(self, obs, path):
        """
        path[1]부터 서지 않고 곧게 갈 수 있는 마지막 칸의 번호.

        복귀: 지나온 칸이라 지도가 있으므로 같은 방향이 이어지는 데까지 (출발 격자 앞에서 멈춤).
        탐색: 그다음 칸도 빈칸이고 점수가 더 작으면(목표 쪽) 한 칸 더.
        서서 찍은 스캔으로 만든 지도만 쓴다 (주행 중 스캔은 위치가 늦어 쓰지 않음).
        """
        self._chained = False
        step_dir = (path[1][0] - path[0][0], path[1][1] - path[0][1])
        if self.phase == RETURN:
            end = 1
            while (end + 1 < len(path) and path[end + 1] != MISSION1_START
                   and (path[end + 1][0] - path[end][0],
                        path[end + 1][1] - path[end][1]) == step_dir):
                end += 1
            return end
        middle = path[1]
        if (self._sees_target(obs) or self.cube is not None
                or middle in self.grid.neighbors(CENTER)
                or path[0] in self._no_chain_from
                or abs(heading_error_to(obs, cell_center_m(middle))) > math.pi / 2):
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
                CELL_SIZE_M * 1.5 - CHAIN_WALL_MARGIN_M, CELL_SIZE_M * 1.5 + CHAIN_WALL_MARGIN_M):
            return 1
        if self.maze.state.score(after) >= self.maze.state.score(middle):
            return 1
        path.append(after)
        self.event = f'두 칸 한 번에: {path[0]} -> {after}'
        self._chained = True
        return 2

    def _shortest_path(self, current, goals):
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
        self._change_state(RELEASE, obs)
        return Command(gripper=GRIPPER_OPEN)

    def _turn_toward(self, error, obs):
        """IMU 각도 오차에 따라 연속 회전하고 목표각 앞에서 일정하게 감속한다."""
        remaining = max(0.0, abs(error) - TURN_TOLERANCE_RAD)
        speed = min(TURN_SPEED, math.sqrt(MIN_TURN_SPEED ** 2 + 2.0 * TURN_DECEL * remaining))
        return math.copysign(speed, error)

    def _turn(self, obs):
        error = self._travel_heading_error(obs)
        # 지금 돌고 있는 속도로 더 돌 각도를 빼고 판단한다 (빠를수록 일찍 감속·정지).
        predicted = error - self._last_angular * TURN_LEAD_S

        if abs(predicted) < TURN_TOLERANCE_RAD:
            self._turn_heading = obs.yaw + error
            self._turn_ended_at = obs.now
            self._change_state(DRIVE, obs)
            return self._drive(obs)

        return Command(angular=self._turn_toward(predicted, obs))

    def _drive(self, obs):
        # 주행 중 대상이 보여도 바로 꺾지 않는다 (대각선 접근 방지). 칸 가운데에서 판단한다.
        if self._chained and obs.scan is not None and self._wall_before_target(obs):
            # 두 칸 한 번에 가는데 두 번째 칸이 막혀 있었다 -> 멈추고 첫 칸 가운데에서 다시 판단.
            # 출발 칸에서는 다시 묶지 않는다 (2026-10-31 실기: 같은 두 칸 계획·정지를 0.3초마다 반복).
            self._chained = False
            self._no_chain_from.add(point_to_cell(*self._drive_origin_cell))
            self.event = '두 칸 주행 중 앞에 벽 -> 멈추고 다시 판단'
            self._change_state(SCAN, obs)
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
        if self._drive_heading is not None:
            # 목표 점을 겨누지 않고 계획한 진행선(출발 가운데 -> 목표 가운데)을 따라간다.
            # 2026-10-31 실기: 위치 보정으로 목표 점이 옆으로 옮겨지면 점을 겨누느라 대각선·45도 꺾임.
            # 선에서 벗어나면 최대 DRIVE_MAX_CORRECTION_RAD까지만 틀고, 앞뒤 거리로 도착을 판단한다.
            # 옆으로 많이 벗어난 채 도착하면 다음 계획에서 칸 가운데로 짧게 옮긴다.
            axis = self._drive_heading
            dx, dy = self.target[0] - obs.x, self.target[1] - obs.y
            along = dx * math.cos(axis) + dy * math.sin(axis)
            lateral = -dx * math.sin(axis) + dy * math.cos(axis)  # 진행선이 로봇 왼쪽이면 +
            endpoint_reached = along <= ARRIVE_TOLERANCE_M
            remaining = along
            # 목표 바로 앞에서는 작은 옆 오차가 큰 회전각으로 증폭되므로 진행축만 유지한다.
            correction = (0.0 if distance < CELL_CENTER_TOLERANCE_M
                          else clamp(DRIVE_LATERAL_GAIN * lateral, DRIVE_MAX_CORRECTION_RAD))
            desired_yaw = axis + correction + (math.pi if self.reverse else 0.0)
            error = math.atan2(math.sin(desired_yaw - obs.yaw), math.cos(desired_yaw - obs.yaw))

        # 도착했거나, 가까운 곳에서 목표를 지나쳤으면 (목표가 뒤에 있음) 멈춘다.
        passed = distance < CELL_CENTER_TOLERANCE_M and abs(error) > math.pi / 2
        if distance < ARRIVE_TOLERANCE_M or passed or endpoint_reached:
            if self.delivering:
                self._change_state(RELEASE, obs)
                return Command(gripper=GRIPPER_OPEN)
            if self._decided_for == self.target and not self._scan_at_arrival:
                # 달리면서 이미 판단했다 -> 서서 기다리지 않고 바로 돈다.
                self._decided_for = None
                return self._plan(obs)
            if self.phase == RETURN:
                # 복귀는 지나온 칸으로만 가므로 새로 볼 것이 없다 -> 꺾는 칸에서도 서서 스캔하지 않고
                # 바로 다음 구간으로 (2026-10-31 팀 요청: 왔던 길은 계산 없이 빠르게).
                return self._plan(obs)

            self._change_state(SCAN, obs)
            return Command()

        # 목표에 가까워지면 일정한 감속으로 줄여 최저 속도로 도착한다. (출발 가속은 _limit_accel)
        remaining = max(0.0, remaining - ARRIVE_TOLERANCE_M)
        speed = min(DRIVE_SPEED, math.sqrt(MIN_DRIVE_SPEED ** 2 + 2.0 * DRIVE_DECEL * remaining))
        if obs.scan_time is not None:
            self._has_scan_time = True
        if (ROLLING_DECISION and self._has_scan_time and not self.delivering
                and self._decided_for != self.target):
            # 판단 전에는 가운데 앞 ROLL_ZONE_M부터 ROLL_SPEED로 달린다 (서지 않고 판단할 수 있게).
            roll_left = max(0.0, remaining - ROLL_ZONE_M)
            roll = math.sqrt(ROLL_SPEED ** 2 + 2.0 * DRIVE_DECEL * roll_left)
            speed = min(speed, roll)
        angular = clamp(HEADING_GAIN * error, TURN_SPEED)
        # 방향이 많이 틀어져 있으면 제자리에서 먼저 돈다.
        linear = speed if abs(error) < math.radians(45.0) else 0.0
        return Command(linear=-linear if self.reverse else linear, angular=angular)

    def _wall_before_target(self, obs):
        """
        두 칸 주행 중 목표 칸 가운데보다 앞에 벽이 보이는지 (두 번째 칸이 막혀 있었다).

        목표 칸 너머 벽(막다른 칸)은 목표 가운데에서 0.2 m 뒤라 멈추지 않는다.
        2026-10-31 실기: 집게 앞 20 cm만 보고 멈추니 막다른 칸 (7,2)에 다 와서 너머 벽에 섰다.
        """
        ahead = nearest_ahead(obs.scan, (obs.x, obs.y, obs.yaw))
        if ahead is None or ahead > CHAIN_GUARD_RANGE_M:
            return False
        dx, dy = self.target[0] - obs.x, self.target[1] - obs.y
        to_target = dx * math.cos(self._drive_heading) + dy * math.sin(self._drive_heading)
        return ahead < to_target - CHAIN_WALL_MARGIN_M

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
        if abs(predicted) < TURN_TOLERANCE_RAD:
            self._target_seen_at = obs.now
            self._area_best = 0.0
            self._area_best_at = (obs.x, obs.y)
            self._change_state(APPROACH, obs)
            return self._approach(obs)
        return Command(angular=self._turn_toward(predicted, obs))

    def _abort_approach(self, obs, reason):
        """접근을 그만두고 칸 가운데로 돌아가 다시 판단한다."""
        self._approach_blocked.add(point_to_cell(*self._approach_origin))
        self.event = f'접근 중단: {reason}'
        self._change_state(SCAN, obs)
        return Command()

    def _approach(self, obs):
        if obs.detected:
            self._target_seen_at = obs.now
        elif obs.now - self._target_seen_at > APPROACH_LOST_S:
            return self._abort_approach(obs, '대상을 놓침')

        if obs.detected and obs.area > self._area_best * AREA_GROW_RATIO:
            self._area_best = obs.area
            self._area_best_at = (obs.x, obs.y)
        stalled = (obs.detected and obs.area >= APPROACH_SLOW_AREA
                   and math.dist((obs.x, obs.y), self._area_best_at) >= APPROACH_STALL_M)
        if obs.detected and (obs.area >= GRAB_AREA or stalled):
            if stalled and obs.area < GRAB_AREA:
                self.event = f'면적이 {obs.area:.0f}에서 안 커짐 -> 큐브가 닿은 것으로 보고 잡기'
            self._push_start = (obs.x, obs.y)
            self._change_state(PUSH, obs)
            return self._push(obs)
        if obs.scan is not None and obstacle_ahead(obs.scan, (obs.x, obs.y, obs.yaw)):
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

    def _push(self, obs):
        """GRAB_AREA에 닿은 뒤 GRAB_PUSH_M만큼 곧게 더 들어가 큐브를 집게 안쪽까지 넣고 닫는다."""
        # 큐브가 너무 가까워 카메라 값(잘림·놓침)은 믿지 않고 엔코더 거리만 본다.
        pushed = math.hypot(obs.x - self._push_start[0], obs.y - self._push_start[1])
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

        if obs.holding is False:
            self._change_state(SEARCH, obs)
            return Command(gripper=GRIPPER_OPEN)

        # TODO: 집게 상태를 알 수 없으면(None) 잡았다고 가정한다.
        self.phase = RETURN
        self._change_state(SCAN, obs)
        return Command()

    # ------------------------------------------------------------
    # 놓기 (RELEASE -> DONE)
    # ------------------------------------------------------------

    def _release(self, obs):
        if not self._gripper_done(obs, GRIPPER_OPENED, RELEASE_WAIT_S):
            return Command()

        self._change_state(DONE, obs)
        return Command()

    def _done(self, obs):
        # 놓은 뒤에는 움직이지 않는다. (상대가 가져가지 못하게 막는 효과도 있다)
        return Command()
