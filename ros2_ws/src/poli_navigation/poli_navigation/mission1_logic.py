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
    neighbor_states,
    point_to_cell,
    scan_to_cells,
    update_grid_from_scan,
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
# 탐색 중 카메라에 대상이 이 크기 이상으로 보이면 (5,5) 옆이 아니어도 바로 정렬·접근한다 (2026-10-31 팀 결정).
# 너무 작으면(멀면) 경기장 밖 빨간 물체일 수 있어 무시한다. 50 cm에서 약 9000 (TODO_MEASURE)
SEE_TARGET_MIN_AREA = 2000.0
# 탐색 단계에서 대상을 놓치고 이 시간 동안 못 찾으면 다시 격자 탐색으로 돌아간다.
SEARCH_GIVE_UP_S = 10.0

# 격자 가운데에 도착했다고 보는 거리, 가운데에서 벗어났다고 보는 거리
ARRIVE_TOLERANCE_M = 0.03
CELL_CENTER_TOLERANCE_M = 0.1

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
ALIGN_GAIN = 1.5  # 2026-10-31 1.0 -> 1.5. TODO_MEASURE: 정렬이 좌우로 흔들리면 낮춘다

# TODO_MEASURE: 펄스 한 번 회전량(turn_pulse.PULSE_S)보다 범위(±)가 넓어야 멈출 수 있다
TURN_TOLERANCE_RAD = math.radians(5.0)
# x_offset 1.0 = 화면 끝 (약 33도) -> 0.15 = 약 5도
ALIGN_TOLERANCE = 0.15
# 이 안(약 12도)이면 제자리 정렬을 끝까지 하지 않고 전진하면서 방향을 맞춘다 (멈춤 한 번 줄임).
APPROACH_START_OFFSET = 0.35
# 접근 중 이보다 벗어나면 다시 제자리 정렬한다.
APPROACH_MAX_OFFSET = 0.5

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


class Mission1Logic:

    def __init__(self):
        self.grid = GridMap()
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
        # 직전에 내보낸 (linear, angular)와 시각. 가감속 제한에 쓴다.
        self._last_linear = 0.0
        self._last_angular = 0.0
        self._last_command_at = None
        self.stop_reason = None

    def _change_state(self, new_state, obs):
        self.state = new_state
        self.state_started_at = obs.now
        self.turn_pulse.reset()
        self._gripper_reached_at = None

    def step(self, obs):
        self.maze.observe(point_to_cell(obs.x, obs.y), obs.yaw)
        if self.state_started_at is None:
            self.state_started_at = obs.now

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
        if self._sees_target(obs):
            self._change_state(ALIGN, obs)
            return Command()

        update_grid_from_scan(self.grid, obs.scan, (obs.x, obs.y, obs.yaw))
        # 상하좌우 이웃 칸은 방향별 ±15도 레이저 판단으로 덮어쓴다 (사선 오차에 강함).
        for cell, state in neighbor_states(obs.scan, (obs.x, obs.y, obs.yaw)).items():
            if not self.grid.in_bounds(cell):
                continue
            if state == BLOCKED:
                self.grid.mark_blocked(cell)
            else:
                self.grid.mark_free(cell)
        return self._plan(obs)

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

        if self.phase == EXPLORE:
            goals = self.grid.neighbors(CENTER)
        else:
            goals = [MISSION1_START]

        if current in goals:
            return self._arrived_at_goal(obs)

        if self.phase == EXPLORE:
            next_cell = self.maze.next_cell(self.grid)
            path = [current, next_cell] if next_cell is not None else None
        else:
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

        if distance_to(obs, center) > CELL_CENTER_TOLERANCE_M:
            # 격자 가운데에서 벗어나 있으면 먼저 가운데로 돌아간다.
            # (옆 격자로 비스듬히 가다가 장애물 모서리에 걸리지 않게)
            self.target = center
            drive_origin = (obs.x, obs.y)
            self.delivering = False
        elif self.phase == RETURN and path[1] == MISSION1_START:
            self.target = self._release_point(path[0], path[1])
            self.delivering = True
        else:
            self.target = cell_center_m(path[1])
            self.delivering = False
            # 미로 탐색에서 뒤쪽 이웃을 고르면 180도 돌지 않고 후진한다.
            # 복귀 때는 집게를 진행 방향으로 두어 기존 놓기 위치 계산을 유지한다.
            self.reverse = (
                self.phase == EXPLORE
                and abs(heading_error_to(obs, self.target)) > 3 * math.pi / 4)

        self._drive_heading = math.atan2(
            self.target[1] - drive_origin[1], self.target[0] - drive_origin[0])
        self._change_state(TURN, obs)
        return Command()

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
        """탐색 중 대상이 충분히 크게 보이는지 (보이면 위치와 상관없이 잡으러 간다)."""
        return self.phase == EXPLORE and obs.detected and obs.area >= SEE_TARGET_MIN_AREA

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

        if abs(error) < TURN_TOLERANCE_RAD:
            self._change_state(DRIVE, obs)
            return self._drive(obs)

        return Command(angular=self._turn_toward(error, obs))

    def _drive(self, obs):
        if self._sees_target(obs):
            self._change_state(ALIGN, obs)
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
        distance = distance_to(obs, self.target)
        error = self._travel_heading_error(obs)

        # 목표 바로 앞에서는 작은 옆 오차가 큰 회전각으로 증폭된다.
        # 마지막 구간은 계획한 진행축을 유지하고 앞뒤 거리로 도착을 판단한다.
        near_endpoint = distance < CELL_CENTER_TOLERANCE_M and self._drive_heading is not None
        endpoint_reached = False
        remaining = distance
        if near_endpoint:
            dx, dy = self.target[0] - obs.x, self.target[1] - obs.y
            along = dx * math.cos(self._drive_heading) + dy * math.sin(self._drive_heading)
            endpoint_reached = along <= ARRIVE_TOLERANCE_M
            remaining = along
            desired_yaw = self._drive_heading + (math.pi if self.reverse else 0.0)
            error = math.atan2(math.sin(desired_yaw - obs.yaw), math.cos(desired_yaw - obs.yaw))

        # 도착했거나, 가까운 곳에서 목표를 지나쳤으면 (목표가 뒤에 있음) 멈춘다.
        passed = distance < CELL_CENTER_TOLERANCE_M and abs(error) > math.pi / 2
        if distance < ARRIVE_TOLERANCE_M or passed or endpoint_reached:
            if self.delivering:
                self._change_state(RELEASE, obs)
                return Command(gripper=GRIPPER_OPEN)

            self._change_state(SCAN, obs)
            return Command()

        # 목표에 가까워지면 일정한 감속으로 줄여 최저 속도로 도착한다. (출발 가속은 _limit_accel)
        remaining = max(0.0, remaining - ARRIVE_TOLERANCE_M)
        speed = min(DRIVE_SPEED, math.sqrt(MIN_DRIVE_SPEED ** 2 + 2.0 * DRIVE_DECEL * remaining))
        angular = clamp(HEADING_GAIN * error, TURN_SPEED)
        # 방향이 많이 틀어져 있으면 제자리에서 먼저 돈다.
        linear = speed if abs(error) < math.radians(45.0) else 0.0
        return Command(linear=-linear if self.reverse else linear, angular=angular)

    def _travel_heading_error(self, obs):
        """후진할 때 목표 반대쪽으로 차체 정면을 유지한다."""
        error = heading_error_to(obs, self.target)
        if self.reverse:
            error = math.atan2(math.sin(error + math.pi), math.cos(error + math.pi))
        return error

    def _stuck(self, obs):
        # 상대 로봇이 길을 막고 있을 수 있다. 기다렸다가 다시 스캔한다.
        # (상대가 떠났으면 update_grid_from_scan이 지도에서 지운다)
        if obs.now - self.state_started_at < STUCK_RETRY_S:
            return Command()

        self._change_state(SCAN, obs)
        return Command()

    # ------------------------------------------------------------
    # 큐브 잡기 (FACE -> SEARCH / ALIGN -> APPROACH -> PUSH -> GRASP)
    # ------------------------------------------------------------

    def _face(self, obs):
        error = heading_error_to(obs, CENTER_M)

        if abs(error) < TURN_TOLERANCE_RAD:
            self._change_state(ALIGN if obs.detected else SEARCH, obs)
            return Command()

        return Command(angular=self._turn_toward(error, obs))

    def _search(self, obs):
        if obs.detected:
            self._change_state(ALIGN, obs)
            return Command()
        if self.phase == EXPLORE and obs.now - self.state_started_at > SEARCH_GIVE_UP_S:
            self._change_state(SCAN, obs)
            return Command()

        # TODO: 큐브가 중앙에 없을 때 (상대가 먼저 가져감) 대응
        return Command(angular=SEARCH_SPEED)

    def _align(self, obs):
        if not obs.detected:
            self._change_state(SEARCH, obs)
            return Command()

        if abs(obs.x_offset) < APPROACH_START_OFFSET:
            self._change_state(APPROACH, obs)
            return self._approach(obs)

        # x_offset 음수 = 왼쪽. 정렬 오차가 작아지면 감속한다.
        speed = max(MIN_TURN_SPEED, min(TURN_SPEED, ALIGN_GAIN * abs(obs.x_offset)))
        return Command(angular=math.copysign(speed, -obs.x_offset))

    def _approach(self, obs):
        if not obs.detected:
            self._change_state(SEARCH, obs)
            return Command()

        if obs.area >= GRAB_AREA:
            self._push_start = (obs.x, obs.y)
            self._change_state(PUSH, obs)
            return self._push(obs)
        if abs(obs.x_offset) > APPROACH_MAX_OFFSET:
            self._change_state(ALIGN, obs)
            return self._align(obs)

        # 다가가면서 방향도 계속 맞춘다.
        speed = APPROACH_SLOW_SPEED if obs.area >= APPROACH_SLOW_AREA else APPROACH_SPEED
        angular = clamp(-ALIGN_GAIN * obs.x_offset, TURN_SPEED)
        return Command(linear=speed, angular=angular)

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
