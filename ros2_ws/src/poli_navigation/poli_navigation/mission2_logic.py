"""
임무 2 (배틀럼블) 판단 로직.

ROS2와 무관한 순수 Python 모듈이다.
센서 값(Observation)을 받아 지금 할 동작(Command)을 결정한다.
토픽 연결은 별도 ROS2 노드에서 한다.

전략 (docs/mission_strategy.md 3장):
  TURN -> DRIVE -> SEARCH / ALIGN -> APPROACH -> GRASP -> HOLD
  + 언제나 탈락선 감시 (RETREAT)

좌표:
- 경기장 좌표(m), 왼쪽 아래 모서리가 (0, 0)
- 네 출발 위치는 중앙 기준 90도씩 돌린 모양이고 장애물이 없으므로
  로봇은 항상 "(8, 2) 격자 중심에서 +y 방향을 보고 출발"한 것으로 계산한다.
"""

from dataclasses import dataclass
import math
from typing import Optional


CELL_SIZE_M = 0.4
ARENA_SIZE_M = 9 * CELL_SIZE_M

START_POSE = (7.5 * CELL_SIZE_M, 1.5 * CELL_SIZE_M, math.pi / 2)
CENTER_M = (4.5 * CELL_SIZE_M, 4.5 * CELL_SIZE_M)

# 탈락 영역: 바깥 테두리 한 줄 (노란 선 바깥)
SAFE_ZONE_MIN_M = CELL_SIZE_M
SAFE_ZONE_MAX_M = ARENA_SIZE_M - CELL_SIZE_M

# TODO_MEASURE: 로봇 중심이 노란 선에서 이 거리 안으로 들어오면 중앙 쪽으로 복귀
# (로봇 반지름 정도. 출발 위치(선에서 0.2m)보다는 작아야 한다.)
DANGER_MARGIN_M = 0.15
RETREAT_EXIT_MARGIN_M = 0.3

# SIM_ONLY: 카메라로 전환하는 중앙까지의 거리
VISION_HANDOFF_M = 0.6

# SIM_ONLY: 속도와 제어 값 (실제 로봇에서 조정)
DRIVE_SPEED = 0.2
APPROACH_SPEED = 0.08
TURN_SPEED = 0.8
SEARCH_SPEED = 0.6
HEADING_GAIN = 1.5
ALIGN_GAIN = 1.0

TURN_TOLERANCE_RAD = math.radians(5.0)
ALIGN_TOLERANCE = 0.1

# TODO_MEASURE: 이 크기 이상으로 보이면 집을 수 있는 거리로 판단 (카메라 픽셀 면적)
GRAB_AREA = 15000.0

# 집게가 닫히는 데 걸리는 시간 (조원 A: 약 1.2초, 서보 40->120도, 1도/15ms)
# TODO_MEASURE: 서보 각도가 실측 후 바뀌면 다시 확인
GRASP_WAIT_S = 1.5

GRIPPER_OPEN = 'open'
GRIPPER_GRAB = 'grab'

TURN = 'TURN'
DRIVE = 'DRIVE'
SEARCH = 'SEARCH'
ALIGN = 'ALIGN'
APPROACH = 'APPROACH'
GRASP = 'GRASP'
HOLD = 'HOLD'
RETREAT = 'RETREAT'


@dataclass
class Observation:
    x: float
    y: float
    yaw: float
    now: float
    # 조원 B Vision 결과
    # TODO: x_offset 부호 확인. 현재 가정: -1 ~ 1, 음수 = 대상이 화면 왼쪽
    detected: bool = False
    x_offset: float = 0.0
    area: float = 0.0
    # 조원 A 집게 상태. 알 수 없으면 None
    holding: Optional[bool] = None


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


def clamp(value, limit):
    return max(-limit, min(limit, value))


class Mission2Logic:

    def __init__(self):
        self.state = TURN
        self.state_started_at = None
        self.gripper_opened = False
        self._state_before_retreat = TURN

    def _change_state(self, new_state, obs):
        self.state = new_state
        self.state_started_at = obs.now

    def step(self, obs):
        if self.state_started_at is None:
            self.state_started_at = obs.now

        command = self._safety_check(obs)
        if command is None:
            command = self._run_state(obs)

        # 시작 시 집게를 한 번 연다.
        if not self.gripper_opened and command.gripper is None:
            command.gripper = GRIPPER_OPEN
            self.gripper_opened = True

        return command

    def _safety_check(self, obs):
        margin = distance_to_danger_line(obs)

        if self.state != RETREAT and margin < DANGER_MARGIN_M:
            self._state_before_retreat = self.state
            self._change_state(RETREAT, obs)

        if self.state != RETREAT:
            return None

        if margin >= RETREAT_EXIT_MARGIN_M:
            self._change_state(self._resume_state_after_retreat(), obs)
            return None

        return self._drive_toward(obs, CENTER_M, DRIVE_SPEED)

    def _resume_state_after_retreat(self):
        if self._state_before_retreat == HOLD:
            return HOLD
        return SEARCH

    def _run_state(self, obs):
        handlers = {
            TURN: self._turn,
            DRIVE: self._drive,
            SEARCH: self._search,
            ALIGN: self._align,
            APPROACH: self._approach,
            GRASP: self._grasp,
            HOLD: self._hold,
        }
        return handlers[self.state](obs)

    def _drive_toward(self, obs, target, speed):
        error = heading_error_to(obs, target)
        angular = clamp(HEADING_GAIN * error, TURN_SPEED)
        # 방향이 많이 틀어져 있으면 제자리에서 먼저 돈다.
        linear = speed if abs(error) < math.radians(45.0) else 0.0
        return Command(linear=linear, angular=angular)

    def _turn(self, obs):
        error = heading_error_to(obs, CENTER_M)

        if abs(error) < TURN_TOLERANCE_RAD:
            self._change_state(DRIVE, obs)
            return self._drive(obs)

        return Command(angular=math.copysign(TURN_SPEED, error))

    def _drive(self, obs):
        if distance_to(obs, CENTER_M) <= VISION_HANDOFF_M:
            self._change_state(ALIGN if obs.detected else SEARCH, obs)
            return Command()

        return self._drive_toward(obs, CENTER_M, DRIVE_SPEED)

    def _search(self, obs):
        if obs.detected:
            self._change_state(ALIGN, obs)
            return Command()

        # TODO: 탐색 회전 방향 (현재 왼쪽)
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
        self._change_state(HOLD, obs)
        return Command()

    def _hold(self, obs):
        # 규정 3.4.7.3: 끝날 때까지 연속으로 오래 확보한 팀이 유리하다.
        # 잡은 뒤에는 움직이지 않고 계속 유지한다.
        if obs.holding is False:
            self._change_state(SEARCH, obs)
            return Command(gripper=GRIPPER_OPEN)

        return Command()
