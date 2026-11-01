import collections
import dataclasses
import math

from hw_model import actual_twist
from poli_navigation import mission2_logic
from poli_navigation.mission2_logic import (
    ALIGN,
    APPROACH,
    BACKOFF,
    BACKOFF_M,
    CAMERA_AHEAD_M,
    CENTER_M,
    Command,
    distance_to_danger_line,
    DRIVE,
    GRASP,
    GRASP_WAIT_S,
    GRIPPER_CLOSED,
    GRIPPER_GRAB,
    GRIPPER_OPEN,
    GUARD,
    GUARD_ARRIVE_M,
    GUARD_ESCAPE_SPEED,
    GUARD_HOME_ANGLE,
    guard_point,
    GUARD_RADIUS_M,
    GUARD_SPEED,
    heading_error_to,
    HOLDING_CONFIRM_S,
    Mission2Logic,
    Observation,
    PUSH,
    RETREAT,
    SEARCH,
    START_POSE,
    TARGET_AREA_AT_50CM,
    TURN,
)


DT = 0.05

# 테스트용 가짜 카메라 (SIM_ONLY)
CAMERA_HALF_FOV_RAD = math.radians(30.0)
CAMERA_MAX_RANGE_M = 2.0
# 로직의 거리 추정과 같은 크기 (50 cm에서 면적 TARGET_AREA_AT_50CM)
AREA_SCALE = TARGET_AREA_AT_50CM * 0.5 ** 2


class SimRobot:
    """명령대로 움직이는 이상적인 로봇 + 가짜 카메라 (테스트 전용)."""

    def __init__(self, pose=START_POSE, target=CENTER_M):
        self.x, self.y, self.yaw = pose
        self.target = target
        self.now = 0.0
        self.holding = None
        self.gripper_state = None
        self.gripper_log = []

    def observe(self):
        # 카메라는 로봇 중심보다 CAMERA_AHEAD_M 앞에 있다
        dx = self.target[0] - (self.x + CAMERA_AHEAD_M * math.cos(self.yaw))
        dy = self.target[1] - (self.y + CAMERA_AHEAD_M * math.sin(self.yaw))
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) - self.yaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))

        detected = (
            abs(bearing) < CAMERA_HALF_FOV_RAD
            and distance < CAMERA_MAX_RANGE_M
        )

        return Observation(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            now=self.now,
            detected=detected,
            # 대상이 왼쪽(bearing > 0)이면 x_offset 음수
            x_offset=-bearing / CAMERA_HALF_FOV_RAD if detected else 0.0,
            area=AREA_SCALE / max(distance, 0.01) ** 2 if detected else 0.0,
            holding=self.holding,
            gripper_state=self.gripper_state,
        )

    def apply(self, command):
        if command.gripper is not None:
            self.gripper_log.append(command.gripper)

        # 실제 하드웨어처럼 느린 명령은 0.25 m/s 바퀴 속도로 커진다 (hw_model.py)
        linear, angular = actual_twist(command.linear, command.angular)
        self.yaw += angular * DT
        self.x += linear * math.cos(self.yaw) * DT
        self.y += linear * math.sin(self.yaw) * DT
        self.now += DT


def run(logic, robot, max_seconds=60.0, until=None):
    states = []
    while robot.now < max_seconds:
        command = logic.step(robot.observe())
        robot.apply(command)
        if not states or states[-1] != logic.state:
            states.append(logic.state)
        if until is not None and logic.state == until:
            break
    return states


# ---------------------------------------------------------------
# 전체 흐름
# ---------------------------------------------------------------

def test_full_run_grabs_near_center():
    logic = Mission2Logic()
    robot = SimRobot()

    states = run(logic, robot, until=GUARD)

    assert logic.state == GUARD
    # 출발 곡선 중에 대상이 보이면 서지 않고 바로 카메라 접근
    assert states == [DRIVE, APPROACH, PUSH, GRASP, GUARD]

    # 집는 순간 대상과 가까워야 한다.
    distance = math.hypot(CENTER_M[0] - robot.x, CENTER_M[1] - robot.y)
    assert distance < 0.3

    # 처음에 집게를 열고(Mega 리셋으로 힘이 빠져 조금 닫히므로), 잡을 때 닫는다.
    assert robot.gripper_log == [GRIPPER_OPEN, GRIPPER_GRAB]


def test_start_pivots_in_place_then_drives_straight(monkeypatch):
    """START_PIVOT이면 제자리에서 중앙 방향(왼쪽 45도)으로 돈 뒤 곧게 직진한다 (기본은 곡선)."""
    monkeypatch.setattr(mission2_logic, 'START_PIVOT', True)
    logic = Mission2Logic()
    robot = LaggedSimRobot(START_POSE)
    command = logic.step(robot.observe())
    assert logic.state == TURN and command.linear == 0.0 and command.angular > 0.0
    path = []
    while logic.state != APPROACH and robot.now < 5.0:
        robot.apply(logic.step(robot.observe()))
        path.append((robot.x, robot.y))
    # 출발 -> 중앙 대각선에서 2 cm 넘게 벗어나지 않는다
    sx, sy = START_POSE[:2]
    length = math.dist(START_POSE[:2], CENTER_M)
    ux, uy = (CENTER_M[0] - sx) / length, (CENTER_M[1] - sy) / length
    assert max(abs(-(x - sx) * uy + (y - sy) * ux) for x, y in path) < 0.02


def test_start_moves_forward_while_turning_left():
    logic = Mission2Logic()
    robot = SimRobot()

    # 제자리 45도 회전 없이 첫 명령부터 전진하면서 왼쪽으로 돈다.
    for _ in range(4):
        command = logic.step(robot.observe())
        robot.apply(command)
        assert command.linear > 0.0
        assert command.angular > 0.0
    assert logic.state == DRIVE


def test_start_arc_is_small_and_does_not_swing():
    logic = Mission2Logic()
    # 모터 지연이 있어야 실제처럼 지나쳐 돈다 (회전 예측 TURN_LEAD_S가 없으면 약 19도 넘어감)
    robot = LaggedSimRobot(START_POSE)

    # 15 cm 안에 중앙 방향(왼쪽 45도)으로 거의 맞춘다.
    while math.dist((robot.x, robot.y), START_POSE[:2]) < 0.15:
        robot.apply(logic.step(robot.observe()))
    assert abs(heading_error_to(robot.observe(), CENTER_M)) < math.radians(10.0)

    # 그 뒤로 반대쪽으로 크게 넘어가지 않는다.
    while robot.now < 1.2:
        robot.apply(logic.step(robot.observe()))
        assert heading_error_to(robot.observe(), CENTER_M) > -math.radians(8.0)


def test_approach_does_not_wobble_when_camera_lag_differs():
    # 2026-11-01 실기: 큐브 앞에서 좌우로 흔들리다 잡음. 실제 카메라 지연이 가정(0.3 s)과 달라도 곧게.
    for lag in (0.15, 0.3, 0.45):
        for target in (CENTER_M, (CENTER_M[0] + 0.2, CENTER_M[1] - 0.2)):
            logic = Mission2Logic()
            robot = LaggedSimRobot(START_POSE, camera_lag_s=lag)
            robot.target = target
            angulars = []
            while robot.now < 10.0 and logic.state != PUSH:
                command = logic.step(robot.observe())
                robot.apply(command)
                if logic.state == APPROACH:
                    angulars.append(command.angular)

            assert logic.state == PUSH, (lag, target)
            flips = sum(1 for a, b in zip(angulars, angulars[1:])
                        if a * b < 0 and abs(a - b) > 0.2)
            assert flips == 0, (lag, target)


def test_never_enters_danger_zone_in_normal_run():
    logic = Mission2Logic()
    robot = SimRobot()

    while robot.now < 60.0 and logic.state != GUARD:
        robot.apply(logic.step(robot.observe()))
        assert distance_to_danger_line(robot.observe()) > 0.0


def test_guard_backs_out_to_circle_and_stays():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GUARD)
    grab_yaw = robot.yaw

    # 잡자마자 돌지 않고 후진으로 우리 출발 쪽 원 위 점까지 천천히 빠진다.
    first = logic.step(robot.observe())
    assert first.linear <= 0.0
    commands = []
    while robot.now < 30.0:
        command = logic.step(robot.observe())
        robot.apply(command)
        commands.append(command)
        assert abs(command.linear) <= GUARD_ESCAPE_SPEED + 1e-9
        assert distance_to_danger_line(robot.observe()) > 0.4
    assert logic.state == GUARD
    assert math.dist((robot.x, robot.y), guard_point(GUARD_HOME_ANGLE)) < GUARD_ARRIVE_M + 0.02
    # 크게 돌지 않았다 (큐브가 빠지지 않게)
    assert abs(robot.yaw - grab_yaw) < math.radians(30.0)

    # 처음 빠질 때는 GUARD_SPEED보다 빠르게
    assert max(-c.linear for c in commands) > GUARD_SPEED + 0.1

    # 도착하면 서 있는다.
    assert all(c.linear == 0.0 and c.angular == 0.0 for c in commands[-20:])


# ---------------------------------------------------------------
# 예외 상황
# ---------------------------------------------------------------

def test_search_when_target_not_visible_after_drive():
    logic = Mission2Logic()
    # 대상이 중앙에서 옆으로 밀려나 있는 경우 (출발 곡선에서 안 보이는 쪽)
    robot = SimRobot(target=(CENTER_M[0] + 0.5, CENTER_M[1] + 0.5))

    states = run(logic, robot, until=GUARD)

    assert SEARCH in states
    assert logic.state == GUARD


def test_grasp_failed_backs_off_then_searches():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GRASP)

    robot.holding = False
    states = run(logic, robot, max_seconds=robot.now + GRASP_WAIT_S + 3.0, until=SEARCH)

    # 집게를 다시 열고, 큐브가 카메라에 보이게 물러난 뒤 탐색부터 다시 시작한다.
    assert states[:3] == [GRASP, BACKOFF, SEARCH]
    assert robot.gripper_log[-1] == GRIPPER_OPEN


def test_grasp_failure_backs_off_far_enough_to_see_cube():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GRASP)
    grasp_pose = (robot.x, robot.y)

    robot.holding = False
    run(logic, robot, max_seconds=robot.now + GRASP_WAIT_S + 3.0, until=SEARCH)
    assert math.dist((robot.x, robot.y), grasp_pose) >= BACKOFF_M - 0.02
    assert robot.observe().detected


def test_grasp_waits_for_holding_sensor_before_failing():
    # 집게가 닫힌 직후 ToF가 아직 false여도 HOLDING_CONFIRM_S 안에 true가 되면 잡은 것이다.
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GRASP)
    robot.holding = False
    robot.gripper_state = GRIPPER_CLOSED
    closed = None
    while robot.now < 10.0 and logic.state == GRASP:
        robot.apply(logic.step(robot.observe()))
        if closed is None and logic._grasp_closed_at is not None:
            closed = robot.now
        if closed is not None and robot.now - closed >= HOLDING_CONFIRM_S - 2 * DT:
            robot.holding = True
    assert logic.state == GUARD


def test_retries_after_failed_grab_and_holds():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GRASP)

    # 첫 시도는 빠졌다. 물러나 다시 찾아 잡으면 지킨다.
    robot.holding = False
    run(logic, robot, max_seconds=robot.now + 5.0, until=BACKOFF)
    robot.holding = None
    states = run(logic, robot, max_seconds=robot.now + 20.0, until=GUARD)
    assert logic.state == GUARD
    assert GRASP in states


def test_grasp_ends_as_soon_as_gripper_reports_closed():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GRASP)
    started = robot.now

    robot.gripper_state = GRIPPER_CLOSED
    run(logic, robot, max_seconds=started + GRASP_WAIT_S, until=GUARD)

    # 최대 대기 시간(GRASP_WAIT_S)까지 기다리지 않는다.
    assert logic.state == GUARD
    assert robot.now - started < 0.5


def test_lost_target_during_guard_backs_off():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GUARD)

    robot.holding = False
    command = logic.step(robot.observe())

    assert logic.state == BACKOFF
    assert command.gripper == GRIPPER_OPEN


def test_retreat_when_pushed_near_elimination_zone():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GUARD)

    # 상대에게 밀려서 노란 선 가까이 감
    robot.x, robot.y = 3.15, 1.8
    command = logic.step(robot.observe())

    assert logic.state == RETREAT
    assert command.linear != 0.0 or command.angular != 0.0

    # 큐브를 잡고 있으므로 천천히 돌아온다.
    while robot.now < 60.0 and logic.state == RETREAT:
        command = logic.step(robot.observe())
        robot.apply(command)
        assert abs(command.linear) <= GUARD_SPEED + 1e-9
    assert logic.state == GUARD
    assert distance_to_danger_line(robot.observe()) > 0.15


def test_align_turns_toward_target_on_left():
    logic = Mission2Logic()
    logic.state = ALIGN
    logic.state_started_at = 0.0
    logic.gripper_opened = True

    obs = Observation(x=1.8, y=1.2, yaw=0.0, now=0.0,
                      detected=True, x_offset=-0.5, area=1000.0)
    command = logic.step(obs)

    # 대상이 왼쪽이면 왼쪽(양수)으로 회전
    assert command.angular > 0.0
    assert command.linear == 0.0


def test_approach_lost_target_goes_to_search():
    logic = Mission2Logic()
    logic.state = APPROACH
    logic.state_started_at = 0.0
    logic.gripper_opened = True

    obs = Observation(x=1.8, y=1.2, yaw=0.0, now=0.0, detected=False)
    command = logic.step(obs)

    assert logic.state == SEARCH
    assert command == Command()


# ---------------------------------------------------------------
# 지연이 있는 실제 로봇 (모터 0.2 s, 카메라 0.3 s)
# 하드웨어 최소 속도 때문에 제자리 회전이 약 2.8 rad/s라서
# 계속 돌면 카메라가 대상을 본 순간 이미 지나쳐 있다 (turn_pulse.py).
# ---------------------------------------------------------------

class LaggedSimRobot(SimRobot):
    """명령이 늦게 반영되고 카메라 결과도 늦게 오는 로봇 (테스트 전용, SIM_ONLY)."""

    # 2026-11-01 실기 6회차 로그 기준 (모터 약 0.15 s, 카메라 약 0.1 s)
    def __init__(self, pose, motor_lag_s=0.15, camera_lag_s=0.1):
        super().__init__(pose=pose)
        self.commands = collections.deque([Command()] * round(motor_lag_s / DT))
        self.camera_lag_steps = round(camera_lag_s / DT)
        self.frames = collections.deque()

    def observe(self):
        obs = super().observe()
        self.frames.append(dataclasses.replace(obs))
        old = self.frames[0]
        if len(self.frames) > self.camera_lag_steps:
            self.frames.popleft()
        obs.detected, obs.x_offset, obs.area = old.detected, old.x_offset, old.area
        return obs

    def apply(self, command):
        self.commands.append(command)
        super().apply(self.commands.popleft())


def test_search_finds_target_with_latency_from_any_heading():
    for start_deg in range(0, 360, 30):
        logic = Mission2Logic()
        logic.state = SEARCH
        robot = LaggedSimRobot(
            (CENTER_M[0] - 0.5, CENTER_M[1], math.radians(start_deg)))

        # 2026-10-31 최저 바퀴 속도 0.25 -> 0.08: 제자리 회전이 느려져 20초 -> 40초
        states = run(logic, robot, max_seconds=40.0, until=GUARD)

        assert logic.state == GUARD, (start_deg, states)
        # 한 번 찾은 대상을 놓쳐서 다시 찾는 일(SEARCH로 되돌아감)이 없어야 한다.
        assert SEARCH not in states[1:], (start_deg, states)


def test_full_run_with_latency_is_fast_and_direct():
    logic = Mission2Logic()
    robot = LaggedSimRobot(START_POSE)

    states = run(logic, robot, until=PUSH)

    # 서거나 다시 찾는 일 없이 곡선 -> 카메라 접근 -> 밀어 넣기
    assert states == [DRIVE, APPROACH, PUSH]
    # 시뮬레이션 약 2.35 s (제자리 45도 회전 후 직진은 약 2.8 s, 예전 0.25 m/s 직진은 약 9초)
    assert robot.now < 3.0
    assert math.dist((robot.x, robot.y), CENTER_M) < 0.3


# ---------------------------------------------------------------
# 잡은 뒤 지키기 (GUARD): 상대가 오면 원을 따라 피하기
# ---------------------------------------------------------------

def guarding_robot():
    """원 위 처음 점에 서서 지키는 중인 로봇."""
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GUARD)
    run(logic, robot, max_seconds=robot.now + 15.0)
    assert math.dist((robot.x, robot.y), guard_point(GUARD_HOME_ANGLE)) < GUARD_ARRIVE_M + 0.02
    return logic, robot


def angle_from_center(x, y):
    return math.atan2(y - CENTER_M[1], x - CENTER_M[0])


def test_guard_dodges_along_circle_away_from_opponent():
    for side in (1.0, -1.0):
        logic, robot = guarding_robot()
        # 원 위에서 한쪽(side) 옆으로 0.5 m 떨어진 곳에 상대가 다가와 멈춰 있다.
        opponent = guard_point(GUARD_HOME_ANGLE + side * 0.6)
        start_angle = angle_from_center(robot.x, robot.y)
        moved = False
        while robot.now < 60.0:
            obs = robot.observe()
            obs.opponent = opponent
            command = logic.step(obs)
            robot.apply(command)
            moved = moved or command.linear != 0.0
            # 원에서 크게 벗어나지 않고, 탈락선에 가까이 가지 않는다
            assert abs(math.dist((robot.x, robot.y), CENTER_M) - GUARD_RADIUS_M) < 0.25
            assert distance_to_danger_line(robot.observe()) > 0.3
        assert moved
        # 상대 반대쪽으로 원을 따라 돌았고, 피한 뒤에는 멀리 떨어져 서 있다.
        turned = math.atan2(math.sin(angle_from_center(robot.x, robot.y) - start_angle),
                            math.cos(angle_from_center(robot.x, robot.y) - start_angle))
        assert turned * side < 0.0, side
        assert math.dist((robot.x, robot.y), opponent) > 0.9
        assert command.linear == 0.0


def test_guard_does_not_drive_into_opponent():
    # 가는 쪽(앞이든 뒤든) 바로 앞에 상대가 있으면 전진·후진하지 않는다 (차체가 약하다).
    for goal_dx in (0.5, -0.5):
        logic = Mission2Logic()
        obs = Observation(x=1.8, y=1.0, yaw=0.0, now=0.0)
        free = logic._guard_drive(obs, (1.8 + goal_dx, 1.0), GUARD_SPEED)
        assert free.linear * goal_dx > 0.0

        obs.opponent = (1.8 + 0.28 * math.copysign(1.0, goal_dx), 1.0)
        blocked = logic._guard_drive(obs, (1.8 + goal_dx, 1.0), GUARD_SPEED, (obs.opponent,))
        assert blocked.linear == 0.0

    # 빠르게 가는 중이면 서는 거리만큼 더 일찍 멈춘다 (0.5 m/s, 상대 0.45 m 앞)
    logic = Mission2Logic()
    logic._last_linear = GUARD_ESCAPE_SPEED
    obs = Observation(x=1.8, y=1.0, yaw=0.0, now=0.0, opponent=(2.25, 1.0))
    assert logic._guard_drive(obs, (2.6, 1.0), GUARD_ESCAPE_SPEED, (obs.opponent,)).linear == 0.0
    logic._last_linear = 0.0
    assert logic._guard_drive(obs, (2.6, 1.0), GUARD_ESCAPE_SPEED, (obs.opponent,)).linear > 0.0


def test_guard_ignores_far_opponent():
    logic, robot = guarding_robot()
    for _ in range(40):
        obs = robot.observe()
        obs.opponent = (CENTER_M[0] - 1.0, CENTER_M[1] + 1.0)
        command = logic.step(obs)
        robot.apply(command)
        assert command == Command()


def run_guard_with_opponents(logic, robot, opponents, seconds):
    commands = []
    end = robot.now + seconds
    while robot.now < end:
        obs = robot.observe()
        obs.opponents = tuple(opponents)
        command = logic.step(obs)
        robot.apply(command)
        commands.append(command)
        assert distance_to_danger_line(robot.observe()) > 0.3
    return commands


def test_guard_stands_still_when_sandwiched():
    logic, robot = guarding_robot()
    # 원 위 양쪽에서 상대가 붙어 온다: 어느 쪽으로 피해도 부딪힌다.
    opponents = [guard_point(GUARD_HOME_ANGLE + 0.5), guard_point(GUARD_HOME_ANGLE - 0.5)]
    commands = run_guard_with_opponents(logic, robot, opponents, 3.0)
    assert all(command.linear == 0.0 for command in commands)


def test_guard_dodges_toward_open_side_with_two_opponents():
    logic, robot = guarding_robot()
    start_angle = angle_from_center(robot.x, robot.y)
    # 가까운 상대는 원 안쪽에 비켜 있고(그쪽 길은 지나갈 수 있다), 먼 상대가 반대쪽 원 위를 막고 있다.
    # 예전(가까운 상대 반대쪽으로만)이면 먼 상대 쪽으로 가서 부딪혔다.
    near = (CENTER_M[0] + 0.35 * math.cos(GUARD_HOME_ANGLE + 0.3),
            CENTER_M[1] + 0.35 * math.sin(GUARD_HOME_ANGLE + 0.3))
    far = guard_point(GUARD_HOME_ANGLE - 0.8)
    assert math.dist((robot.x, robot.y), near) < math.dist((robot.x, robot.y), far)
    run_guard_with_opponents(logic, robot, [near, far], 2.0)

    turned = angle_from_center(robot.x, robot.y) - start_angle
    assert turned > 0.05
    assert math.dist((robot.x, robot.y), far) > 0.5


def test_retreat_starts_30cm_from_line_after_start():
    logic, robot = guarding_robot()
    # 밀려서 노란 선(x = 3.2)에서 0.25 m: 예전(0.15)은 그대로 있었다.
    robot.x, robot.y = 2.95, 1.8
    logic.step(robot.observe())
    assert logic.state == RETREAT

    # 출발 위치는 선에서 0.2 m지만 출발 중에는 RETREAT하지 않는다.
    logic = Mission2Logic()
    robot = SimRobot()
    logic.step(robot.observe())
    assert logic.state == DRIVE
