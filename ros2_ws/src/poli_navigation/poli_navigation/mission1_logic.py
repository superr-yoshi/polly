"""
임무 1 (자율탐색 및 요구조자 구출) 판단 로직.

ROS2와 무관한 순수 Python 모듈이다.
센서 값(Observation)을 받아 지금 할 동작(Command)을 결정한다.
토픽 연결은 별도 ROS2 노드에서 한다.

전략 (docs/mission_strategy.md 2장):
  [EXPLORE] SCAN -> TURN -> DRIVE -> SCAN ... (중앙 옆 격자까지 한 칸씩)
            FACE -> SEARCH / ALIGN -> APPROACH -> GRASP
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
from poli_navigation.turn_pulse import FINE_TURN_RAD, HW_MIN_WHEEL_SPEED, TurnPulse


CENTER_M = cell_center_m(CENTER)

# TODO_MEASURE: 로봇 중심 ~ 집게로 잡은 큐브 중심 거리
# 큐브는 들지 않고 바닥에 끌고 간다.
# 출발 격자에서 놓을 때 로봇 중심이 아니라 큐브가 격자 가운데에 오게 한다.
GRIPPER_REACH_M = 0.25

# SIM_ONLY: 멈춘 뒤 스캔할 때까지 기다리는 시간 (움직이며 찍힌 스캔을 쓰지 않게)
SCAN_SETTLE_S = 0.6  # 2026-10-31: 0.3이면 멈춘 직후 차체 흔들림이 남은 스캔으로 지도를 갱신했다 (실기)
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
DRIVE_SPEED = 0.25
MIN_DRIVE_SPEED = HW_MIN_WHEEL_SPEED
SLOWDOWN_GAIN = 1.0
# 한 칸 주행 출발 가속 (m/s^2). 0 -> 0.25 m/s에 약 0.6초 (2026-10-31 급출발 방지)
DRIVE_ACCEL = 0.4
APPROACH_SPEED = HW_MIN_WHEEL_SPEED
TURN_SPEED = 0.8
SEARCH_SPEED = 0.6
HEADING_GAIN = 1.5
ALIGN_GAIN = 1.0

# TODO_MEASURE: 펄스 한 번 회전량(turn_pulse.PULSE_S)보다 범위(±)가 넓어야 멈출 수 있다
TURN_TOLERANCE_RAD = math.radians(5.0)
# x_offset 1.0 = 화면 끝 (약 33도) -> 0.15 = 약 5도
ALIGN_TOLERANCE = 0.15

# 큐브(50mm)가 이 크기 이상으로 보이면 집을 수 있는 거리 (카메라 픽셀 면적, 640x480)
# 2026-10-30 실측 (주황 시험 대상, 같은 크기): 열린 집게 사이 잡는 위치에서 area 약 33000
# (아래가 화면 밖으로 잘려 사실상 최댓값) -> 집게 안에 들어온 순간 잡도록 조금 낮춤 (TODO_MEASURE: 경기장에서 재확인)
GRAB_AREA = 27000.0

# 집게가 닫히는 시간, 열리는 시간 (조원 A: 약 1.2초, 서보 40<->120도, 1도/15ms)
# TODO_MEASURE: 서보 각도가 실측 후 바뀌면 다시 확인
GRASP_WAIT_S = 1.5
RELEASE_WAIT_S = 1.5

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
GRASP = 'GRASP'
RELEASE = 'RELEASE'
DONE = 'DONE'
STUCK = 'STUCK'


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
        self._drive_speed = 0.0
        self._drive_time = None
        self.stop_reason = None

    def _change_state(self, new_state, obs):
        self.state = new_state
        self.state_started_at = obs.now
        self.turn_pulse.reset()
        self._drive_speed = 0.0
        self._drive_time = None

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

        if distance_to(obs, center) > CELL_CENTER_TOLERANCE_M:
            # 격자 가운데에서 벗어나 있으면 먼저 가운데로 돌아간다.
            # (옆 격자로 비스듬히 가다가 장애물 모서리에 걸리지 않게)
            self.target = center
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

        self._change_state(TURN, obs)
        return Command()

    def _shortest_path(self, current, goals):
        return_grid = GridMap(use_point_symmetry=False)
        visited = self.maze.visited_cells()
        for cell in return_grid.cells:
            if cell not in visited or self.grid.get(cell) == BLOCKED:
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
        """제자리 회전. 멀면 계속 돌고, 가까우면 끊어서 돈다 (turn_pulse.py)."""
        if abs(error) > FINE_TURN_RAD:
            self.turn_pulse.reset()
            return math.copysign(TURN_SPEED, error)
        return self.turn_pulse.angular(error, TURN_SPEED, obs.now)

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

        # 도착했거나, 가까운 곳에서 목표를 지나쳤으면 (목표가 뒤에 있음) 멈춘다.
        passed = distance < CELL_CENTER_TOLERANCE_M and abs(error) > math.pi / 2
        if distance < ARRIVE_TOLERANCE_M or passed:
            if self.delivering:
                self._change_state(RELEASE, obs)
                return Command(gripper=GRIPPER_OPEN)

            self._change_state(SCAN, obs)
            return Command()

        # 목표에 가까워지면 속도를 줄인다.
        speed = max(MIN_DRIVE_SPEED, min(DRIVE_SPEED, SLOWDOWN_GAIN * distance))
        # 급출발 방지: 직전 속도에서 DRIVE_ACCEL만큼씩만 올린다.
        if self._drive_time is not None:
            elapsed = max(0.0, obs.now - self._drive_time)
            speed = min(speed, self._drive_speed + DRIVE_ACCEL * elapsed)
        else:
            speed = min(speed, MIN_DRIVE_SPEED)
        angular = clamp(HEADING_GAIN * error, TURN_SPEED)
        # 방향이 많이 틀어져 있으면 제자리에서 먼저 돈다.
        linear = speed if abs(error) < math.radians(45.0) else 0.0
        self._drive_speed = linear
        self._drive_time = obs.now
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
    # 큐브 잡기 (FACE -> SEARCH / ALIGN -> APPROACH -> GRASP)
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
        # 끊어서 돌아야 카메라가 멈춘 화면에서 대상을 찾는다.
        return Command(angular=self.turn_pulse.angular(1.0, SEARCH_SPEED, obs.now))

    def _align(self, obs):
        if not obs.detected:
            self._change_state(SEARCH, obs)
            return Command()

        if abs(obs.x_offset) < ALIGN_TOLERANCE:
            self._change_state(APPROACH, obs)
            return Command()

        # 대상이 화면 안에 있으면 오차가 작으므로 항상 끊어서 돈다. x_offset 음수 = 왼쪽.
        return Command(
            angular=self.turn_pulse.angular(-obs.x_offset, TURN_SPEED, obs.now))

    def _approach(self, obs):
        if not obs.detected:
            self._change_state(SEARCH, obs)
            return Command()

        if obs.area >= GRAB_AREA:
            self._change_state(GRASP, obs)
            return Command(gripper=GRIPPER_GRAB)

        # 다가가면서 방향도 계속 맞춘다.
        angular = clamp(-ALIGN_GAIN * obs.x_offset, TURN_SPEED)
        return Command(linear=APPROACH_SPEED, angular=angular)

    def _grasp(self, obs):
        if obs.now - self.state_started_at < GRASP_WAIT_S:
            return Command()

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
        if obs.now - self.state_started_at < RELEASE_WAIT_S:
            return Command()

        self._change_state(DONE, obs)
        return Command()

    def _done(self, obs):
        # 놓은 뒤에는 움직이지 않는다. (상대가 가져가지 못하게 막는 효과도 있다)
        return Command()
