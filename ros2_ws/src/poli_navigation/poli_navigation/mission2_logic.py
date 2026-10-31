"""
임무 2 (배틀럼블) 판단 로직.

ROS2와 무관한 순수 Python 모듈이다.
센서 값(Observation)을 받아 지금 할 동작(Command)을 결정한다.
토픽 연결은 별도 ROS2 노드에서 한다.

전략 (docs/mission_strategy.md 3장, 2026-10-31 임무 1 개선을 옮김):
  DRIVE(출발하자마자 가속하며 중앙 쪽으로 곡선) -> 카메라에 보이면 바로 APPROACH
  -> PUSH(밀어 넣으며 닫기) -> GRASP -> HOLD
  중앙 근처까지 와도 안 보이면 SEARCH -> ALIGN -> APPROACH
  + 언제나 탈락선 감시 (RETREAT)

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

START_POSE = (7.5 * CELL_SIZE_M, 1.5 * CELL_SIZE_M, math.pi / 2)
CENTER_M = (4.5 * CELL_SIZE_M, 4.5 * CELL_SIZE_M)

# 탈락 영역: 바깥 테두리 한 줄 (노란 선 바깥)
SAFE_ZONE_MIN_M = CELL_SIZE_M
SAFE_ZONE_MAX_M = ARENA_SIZE_M - CELL_SIZE_M

# TODO_MEASURE: 로봇 중심이 노란 선에서 이 거리 안으로 들어오면 중앙 쪽으로 복귀
# (로봇 반지름 정도. 출발 위치(선에서 0.2m)보다는 작아야 한다.)
DANGER_MARGIN_M = 0.15
RETREAT_EXIT_MARGIN_M = 0.3

# 이 거리(중앙까지)에 오면 카메라로 넘긴다 (그 전에라도 보이면 바로 넘긴다).
VISION_HANDOFF_M = 0.6

# 속도·가감속 (2026-10-31 임무 1과 같은 값, mission1_logic.py 참고)
DRIVE_SPEED = 0.8
DRIVE_DECEL = 1.2     # 카메라 전환 지점 앞에서 일정하게 감속
LINEAR_ACCEL = 1.2
LINEAR_DECEL = 2.0
ANGULAR_ACCEL = 15.0
# 이만큼 판단이 끊기면 그동안 노드가 정지 명령을 보냈다고 보고 0에서 다시 가속한다.
ACCEL_RESET_S = 0.15
TURN_SPEED = 3.5
MIN_TURN_SPEED = 0.6  # 바퀴 약 0.084 m/s = 하드웨어 최저 속도 근처
# 출발 곡선: 방향 오차 x 이 값으로 돌면서, 방향 오차의 cos만큼 전진한다 (45도면 70% 속도).
# 가속 구간(약 0.7초)에 회전을 겹쳐 제자리 45도 회전(약 0.5초)을 없앤다. 반지름은 처음 약 7~10 cm.
# TODO_MEASURE: 출발 때 크게 휘청이면 낮춘다
HEADING_GAIN = 4.0
SEARCH_SPEED = 1.0

# 카메라: x_offset 1.0 = 화면 끝 = 약 33도
CAMERA_HALF_FOV_RAD = math.radians(33.0)
# 카메라 결과는 이만큼 늦게 온다. 그 시각의 위치로 큐브 위치를 계산한다. TODO_MEASURE
CAMERA_LAG_S = 0.3
# 큐브 거리 추정: 면적은 거리 제곱에 반비례, 50 cm에서 약 9000 (mission1_logic.py와 같음, TODO_MEASURE)
TARGET_AREA_AT_50CM = 9000.0
# 새 카메라 결과를 큐브 위치 추정에 섞는 비율 (나머지는 이전 추정)
TARGET_SMOOTHING = 0.5
# 제자리 맞추기(ALIGN)는 큐브 방향이 이 안이면 끝내고 전진하며 맞춘다.
ALIGN_DONE_RAD = math.radians(15.0)
# 이 크기 이상으로 보이면 DRIVE 중에도 바로 카메라 접근 (경기장 밖 빨간 물체는 작게 보여 무시)
SEE_TARGET_MIN_AREA = 2000.0
# DRIVE 중에 이 안(화면 가운데 쪽)에 보이면 바로 접근한다.
APPROACH_MAX_OFFSET = 0.6
# 접근 속도: 멀면 빠르게, 큐브 위치 추정까지 남은 거리로 일정하게 감속해
# APPROACH_SLOW_AREA(잡는 위치 약 8 cm 전)에서 APPROACH_SLOW_SPEED가 된다.
# 남은 거리는 마지막 카메라 결과 + 그 뒤 엔코더 이동으로 계산하므로 카메라 지연만큼 지나치지 않는다.
APPROACH_FAST_SPEED = 0.5
APPROACH_SLOW_SPEED = 0.12
APPROACH_DECEL = DRIVE_DECEL
APPROACH_SLOW_AREA = 20000.0  # TODO_MEASURE: 잡는 위치 약 8 cm 전
# 카메라가 대상을 잠깐 놓쳐도 이 시간은 기다린다 (카메라 지연 약 0.3초)
TARGET_LOST_S = 0.5

# 잡기 (임무 1과 같음)
GRAB_AREA = 35000.0
GRAB_PUSH_M = 0.04
GRAB_CLOSE_LEAD_M = 0.02
GRAB_PUSH_SPEED = APPROACH_SLOW_SPEED
GRAB_PUSH_TIMEOUT_S = 1.0
APPROACH_STALL_M = 0.06
AREA_GROW_RATIO = 1.05

# 집게 기다리기: /gripper/state가 closed면 바로, 모르면 최대 GRASP_WAIT_S
GRASP_WAIT_S = 1.5
GRIPPER_MIN_WAIT_S = 0.25
GRIPPER_SETTLE_S = 0.1

GRIPPER_OPEN = 'open'
GRIPPER_GRAB = 'grab'
GRIPPER_CLOSED = 'closed'

TURN = 'TURN'  # 예전 호환용 (지금은 출발하자마자 DRIVE 곡선)
DRIVE = 'DRIVE'
SEARCH = 'SEARCH'
ALIGN = 'ALIGN'
APPROACH = 'APPROACH'
PUSH = 'PUSH'
GRASP = 'GRASP'
HOLD = 'HOLD'
RETREAT = 'RETREAT'


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


def area_to_distance(area):
    """카메라 면적 -> 큐브까지 거리 추정(m). 면적은 거리 제곱에 반비례한다."""
    return 0.5 * math.sqrt(TARGET_AREA_AT_50CM / max(area, 1.0))


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
        self.state = DRIVE
        self.state_started_at = None
        self.gripper_opened = False
        self._state_before_retreat = DRIVE
        self._target_seen_at = None
        # 카메라로 계산한 큐브 위치 (경기장 좌표). 카메라 지연 때문에 화면 값으로 바로 돌지 않고
        # 이 점을 향해 엔코더·IMU 위치로 돈다 (위치는 지연이 작다).
        self.target = None
        self._poses = collections.deque()
        self._area_best = 0.0
        self._area_best_at = None
        self._push_start = None
        self._grab_sent_at = None
        self._gripper_reached_at = None
        self._last_linear = 0.0
        self._last_angular = 0.0
        self._last_command_at = None

    def _change_state(self, new_state, obs):
        self.state = new_state
        self.state_started_at = obs.now
        self._gripper_reached_at = None

    def step(self, obs):
        if self.state_started_at is None:
            self.state_started_at = obs.now
        self._poses.append((obs.now, obs.x, obs.y, obs.yaw))
        while self._poses and obs.now - self._poses[0][0] > 2.0 * CAMERA_LAG_S:
            self._poses.popleft()
        self._update_target(obs)

        command = self._safety_check(obs)
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
            self._last_linear = self._last_angular = 0.0
            dt = ACCEL_RESET_S / 2.0
        command.linear = ramp(command.linear, self._last_linear, LINEAR_ACCEL, LINEAR_DECEL, dt)
        command.angular = ramp(
            command.angular, self._last_angular, ANGULAR_ACCEL, ANGULAR_ACCEL, dt)
        self._last_linear = command.linear
        self._last_angular = command.angular
        self._last_command_at = now
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

        return self._drive_toward(obs, CENTER_M, APPROACH_FAST_SPEED)

    def _resume_state_after_retreat(self):
        if self._state_before_retreat == HOLD:
            return HOLD
        return SEARCH

    def _run_state(self, obs):
        handlers = {
            TURN: self._drive,
            DRIVE: self._drive,
            SEARCH: self._search,
            ALIGN: self._align,
            APPROACH: self._approach,
            PUSH: self._push,
            GRASP: self._grasp,
            HOLD: self._hold,
        }
        return handlers[self.state](obs)

    def _drive_toward(self, obs, target, speed):
        """곡선 주행: 방향 오차만큼 돌면서 cos(오차)만큼 전진한다 (90도 넘게 틀어지면 제자리)."""
        error = heading_error_to(obs, target)
        angular = clamp(HEADING_GAIN * error, TURN_SPEED)
        linear = speed * max(0.0, math.cos(error))
        return Command(linear=linear, angular=angular)

    def _pose_at(self, when):
        """주어진 시각 when(또는 바로 전)의 위치. 기록이 그보다 짧으면 가장 오래된 위치."""
        for pose in reversed(self._poses):
            if pose[0] <= when:
                return pose[1:]
        return self._poses[0][1:]

    def _update_target(self, obs):
        if not obs.detected or obs.area <= 0.0:
            return
        x, y, yaw = self._pose_at(obs.now - CAMERA_LAG_S)
        bearing = yaw - obs.x_offset * CAMERA_HALF_FOV_RAD
        distance = area_to_distance(obs.area)
        point = (x + distance * math.cos(bearing), y + distance * math.sin(bearing))
        if self.target is None:
            self.target = point
        else:
            self.target = tuple(
                old + TARGET_SMOOTHING * (new - old) for old, new in zip(self.target, point))

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

    def _drive(self, obs):
        # 가는 길에 카메라에 보이면 멈추지 않고 바로 카메라 접근으로
        if self._sees_target(obs) and abs(obs.x_offset) < APPROACH_MAX_OFFSET:
            return self._start_approach(obs)
        distance = distance_to(obs, CENTER_M)
        if distance <= VISION_HANDOFF_M:
            self._target_seen_at = obs.now
            self._change_state(ALIGN if obs.detected else SEARCH, obs)
            return self._run_state(obs)
        # 카메라 전환 지점에서 APPROACH_FAST_SPEED가 되도록 일정하게 감속
        remaining = distance - VISION_HANDOFF_M
        speed = min(DRIVE_SPEED,
                    math.sqrt(APPROACH_FAST_SPEED ** 2 + 2.0 * DRIVE_DECEL * remaining))
        return self._drive_toward(obs, CENTER_M, speed)

    def _search(self, obs):
        if obs.detected:
            self._target_seen_at = obs.now
            self._change_state(ALIGN, obs)
            return self._align(obs)

        # 전에 본 큐브가 있으면 그쪽으로, 없으면 왼쪽으로 돈다.
        direction = 1.0
        if self.target is not None:
            direction = math.copysign(1.0, heading_error_to(obs, self.target))
        return Command(angular=direction * SEARCH_SPEED)

    def _turn_speed(self, error):
        speed = max(MIN_TURN_SPEED, min(TURN_SPEED, HEADING_GAIN * abs(error)))
        return math.copysign(speed, error)

    def _align(self, obs):
        if self._target_lost(obs) or self.target is None:
            self._change_state(SEARCH, obs)
            return Command()

        error = heading_error_to(obs, self.target)
        if abs(error) < ALIGN_DONE_RAD:
            return self._start_approach(obs)
        return Command(angular=self._turn_speed(error))

    def _approach(self, obs):
        if self._target_lost(obs) or self.target is None:
            self._change_state(SEARCH, obs)
            return Command()

        if self._area_best_at is None:
            self._area_best_at = (obs.x, obs.y)
        if obs.detected and obs.area > self._area_best * AREA_GROW_RATIO:
            self._area_best = obs.area
            self._area_best_at = (obs.x, obs.y)
        stalled = (obs.detected and obs.area >= APPROACH_SLOW_AREA
                   and math.dist((obs.x, obs.y), self._area_best_at) >= APPROACH_STALL_M)
        if obs.detected and (obs.area >= GRAB_AREA or stalled):
            self._push_start = (obs.x, obs.y)
            self._grab_sent_at = None
            self._change_state(PUSH, obs)
            return self._push(obs)

        # 잠깐 놓쳤을 때나 가까우면 천천히
        speed = APPROACH_SLOW_SPEED
        if obs.detected and self._area_best < APPROACH_SLOW_AREA:
            remaining = distance_to(obs, self.target) - area_to_distance(APPROACH_SLOW_AREA)
            speed = min(APPROACH_FAST_SPEED, math.sqrt(
                APPROACH_SLOW_SPEED ** 2 + 2.0 * APPROACH_DECEL * max(0.0, remaining)))
        return self._drive_toward(obs, self.target, speed)

    def _push(self, obs):
        """GRAB_AREA에 닿은 뒤 GRAB_PUSH_M만큼 곧게 더 들어가며 끝 GRAB_CLOSE_LEAD_M에서 닫는다."""
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
        if not self._gripper_closed(obs):
            return Command()
        self._grab_sent_at = None

        if obs.holding is False:
            self._target_seen_at = obs.now
            self._change_state(SEARCH, obs)
            return Command(gripper=GRIPPER_OPEN)

        # TODO: 집게 상태를 알 수 없으면(None) 잡았다고 가정한다.
        self._change_state(HOLD, obs)
        return Command()

    def _hold(self, obs):
        # 규정 3.4.7.3: 끝날 때까지 연속으로 오래 확보한 팀이 유리하다.
        # 잡은 뒤에는 움직이지 않고 계속 유지한다.
        if obs.holding is False:
            self._target_seen_at = obs.now
            self._change_state(SEARCH, obs)
            return Command(gripper=GRIPPER_OPEN)

        return Command()
