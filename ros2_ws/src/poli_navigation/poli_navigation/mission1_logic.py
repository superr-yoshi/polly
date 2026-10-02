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
  SCAN에서 스캔을 지도에 반영하고 바로 경로를 다시 계산한다 (PLAN).

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
from poli_navigation.mission2_logic import (
    clamp,
    Command,
    distance_to,
    GRIPPER_GRAB,
    GRIPPER_OPEN,
    heading_error_to,
)
from poli_navigation.scan_to_grid import point_to_cell, update_grid_from_scan


CENTER_M = cell_center_m(CENTER)

# TODO_MEASURE: 로봇 중심 ~ 집게로 잡은 큐브 중심 거리
# 큐브는 들지 않고 바닥에 끌고 간다.
# 출발 격자에서 놓을 때 로봇 중심이 아니라 큐브가 격자 가운데에 오게 한다.
GRIPPER_REACH_M = 0.25

# SIM_ONLY: 멈춘 뒤 스캔할 때까지 기다리는 시간 (움직이며 찍힌 스캔을 쓰지 않게)
SCAN_SETTLE_S = 0.3

# 격자 가운데에 도착했다고 보는 거리, 가운데에서 벗어났다고 보는 거리
ARRIVE_TOLERANCE_M = 0.03
CELL_CENTER_TOLERANCE_M = 0.1

# SIM_ONLY: 속도와 제어 값 (임무 2와 같은 값에서 시작, 실제 로봇에서 조정)
DRIVE_SPEED = 0.2
MIN_DRIVE_SPEED = 0.05
SLOWDOWN_GAIN = 1.0
APPROACH_SPEED = 0.08
TURN_SPEED = 0.8
SEARCH_SPEED = 0.6
HEADING_GAIN = 1.5
ALIGN_GAIN = 1.0

TURN_TOLERANCE_RAD = math.radians(5.0)
ALIGN_TOLERANCE = 0.1

# TODO_MEASURE: 큐브(50mm)가 이 크기 이상으로 보이면 집을 수 있는 거리
GRAB_AREA = 15000.0

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

    def _change_state(self, new_state, obs):
        self.state = new_state
        self.state_started_at = obs.now

    def step(self, obs):
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

        update_grid_from_scan(self.grid, obs.scan, (obs.x, obs.y, obs.yaw))
        return self._plan(obs)

    def _plan(self, obs):
        current = point_to_cell(obs.x, obs.y)
        if current is None:
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

        path = self._shortest_path(current, goals)
        if path is None:
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

        self._change_state(TURN, obs)
        return Command()

    def _shortest_path(self, current, goals):
        paths = [self.grid.find_path(current, goal) for goal in goals]
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

    def _arrived_at_goal(self, obs):
        if self.phase == EXPLORE:
            self._change_state(FACE, obs)
            return Command()

        # TODO: 출발 격자 안에서 돌아가기 시작한 경우 (보통은 없다)
        self._change_state(RELEASE, obs)
        return Command(gripper=GRIPPER_OPEN)

    def _turn(self, obs):
        error = heading_error_to(obs, self.target)

        if abs(error) < TURN_TOLERANCE_RAD:
            self._change_state(DRIVE, obs)
            return self._drive(obs)

        return Command(angular=math.copysign(TURN_SPEED, error))

    def _drive(self, obs):
        distance = distance_to(obs, self.target)
        error = heading_error_to(obs, self.target)

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
        angular = clamp(HEADING_GAIN * error, TURN_SPEED)
        # 방향이 많이 틀어져 있으면 제자리에서 먼저 돈다.
        linear = speed if abs(error) < math.radians(45.0) else 0.0
        return Command(linear=linear, angular=angular)

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

        return Command(angular=math.copysign(TURN_SPEED, error))

    def _search(self, obs):
        if obs.detected:
            self._change_state(ALIGN, obs)
            return Command()

        # TODO: 큐브가 중앙에 없을 때 (상대가 먼저 가져감) 대응
        return Command(angular=SEARCH_SPEED)

    def _align(self, obs):
        if not obs.detected:
            self._change_state(SEARCH, obs)
            return Command()

        if abs(obs.x_offset) < ALIGN_TOLERANCE:
            self._change_state(APPROACH, obs)
            return Command()

        return Command(angular=clamp(-ALIGN_GAIN * obs.x_offset, TURN_SPEED))

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
