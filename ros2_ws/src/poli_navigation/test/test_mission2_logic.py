import collections
import dataclasses
import math

from hw_model import actual_twist
from poli_navigation import mission2_logic
from poli_navigation.mission2_logic import (
    ALIGN,
    APPROACH,
    APPROACH_SLOW_SPEED,
    ATTACK,
    BACKOFF,
    BACKOFF_M,
    CAMERA_AHEAD_M,
    CENTER_M,
    Command,
    distance_to_danger_line,
    DODGE_START_M,
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
    LURK,
    Mission2Logic,
    Observation,
    PUSH,
    RECENTER,
    RETREAT,
    SAFE_ZONE_MAX_M,
    SEARCH,
    SEARCH_CENTER_M,
    SPIN,
    SPIN_SPEED,
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


def test_guard_backs_out_to_circle_and_stays(monkeypatch):
    monkeypatch.setattr(mission2_logic, 'GUARD_ESCAPE_FIRST', True)
    monkeypatch.setattr(mission2_logic, 'GUARD_ESCAPE_TURN_AROUND', False)
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

    # 처음 빠질 때는 GUARD_ESCAPE_SPEED까지 낸다 (2026-11-02부터 피하기 GUARD_SPEED도 같은 0.65 m/s)
    assert max(-c.linear for c in commands) > GUARD_ESCAPE_SPEED - 0.01

    # 도착하면 서 있는다.
    assert all(c.linear == 0.0 and c.angular == 0.0 for c in commands[-20:])


def test_guard_turns_around_and_drives_forward_to_circle(monkeypatch):
    # 2026-11-03 팀: 후진 대신 180도 돌아 전진으로 빠진다 (중앙 쪽 상대가 큐브를 못 보게).
    monkeypatch.setattr(mission2_logic, 'GUARD_ESCAPE_FIRST', True)
    monkeypatch.setattr(mission2_logic, 'GUARD_ESCAPE_TURN_AROUND', True)
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GUARD)

    commands = []
    while robot.now < 30.0:
        command = logic.step(robot.observe())
        robot.apply(command)
        commands.append(command)
        assert command.linear >= 0.0
        assert command.linear <= GUARD_ESCAPE_SPEED + 1e-9
        assert distance_to_danger_line(robot.observe()) > 0.4
    assert logic.state == GUARD
    goal = guard_point(GUARD_HOME_ANGLE)
    assert math.dist((robot.x, robot.y), goal) < GUARD_ARRIVE_M + 0.02
    # 도착하면 중앙을 등지고(큐브가 바깥 쪽) 서 있다
    away = math.atan2(robot.y - CENTER_M[1], robot.x - CENTER_M[0])
    assert abs(math.atan2(math.sin(robot.yaw - away), math.cos(robot.yaw - away))) \
        < math.radians(30.0)
    assert max(c.linear for c in commands) > GUARD_ESCAPE_SPEED - 0.01
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


def test_grasp_failed_backs_off_then_searches(monkeypatch):
    monkeypatch.setattr(mission2_logic, 'LURK_AFTER_FAIL', False)
    monkeypatch.setattr(mission2_logic, 'SPIN_AFTER_FAIL', False)
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GRASP)

    robot.holding = False
    states = run(logic, robot, max_seconds=robot.now + GRASP_WAIT_S + 3.0, until=SEARCH)

    # 집게를 다시 열고, 큐브가 카메라에 보이게 물러난 뒤 탐색부터 다시 시작한다.
    assert states[:3] == [GRASP, BACKOFF, SEARCH]
    assert robot.gripper_log[-1] == GRIPPER_OPEN


def test_grasp_failure_backs_off_far_enough_to_see_cube(monkeypatch):
    monkeypatch.setattr(mission2_logic, 'LURK_AFTER_FAIL', False)
    monkeypatch.setattr(mission2_logic, 'SPIN_AFTER_FAIL', False)
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


def test_retries_after_failed_grab_and_holds(monkeypatch):
    monkeypatch.setattr(mission2_logic, 'LURK_AFTER_FAIL', False)
    monkeypatch.setattr(mission2_logic, 'SPIN_AFTER_FAIL', False)
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


def test_lost_target_during_guard_backs_off(monkeypatch):
    monkeypatch.setattr(mission2_logic, 'LURK_AFTER_FAIL', False)
    monkeypatch.setattr(mission2_logic, 'SPIN_AFTER_FAIL', False)
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

def guarding_robot(monkeypatch):
    """원 위 처음 점에 서서 지키는 중인 로봇 (GUARD_ESCAPE_FIRST, 먼저 빠지는 방식)."""
    monkeypatch.setattr(mission2_logic, 'GUARD_ESCAPE_FIRST', True)
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GUARD)
    run(logic, robot, max_seconds=robot.now + 15.0)
    assert math.dist((robot.x, robot.y), guard_point(GUARD_HOME_ANGLE)) < GUARD_ARRIVE_M + 0.02
    return logic, robot


def angle_from_center(x, y):
    return math.atan2(y - CENTER_M[1], x - CENTER_M[0])


def test_guard_dodges_along_circle_away_from_opponent(monkeypatch):
    for side in (1.0, -1.0):
        logic, robot = guarding_robot(monkeypatch)
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
        # 상대가 DODGE_CLEAR_M 밖으로 가면 그 자리에서 원 위로 돌아오느라 몇 cm 가까워질 수 있다.
        # 다시 피하기 시작하는 거리(DODGE_START_M)보다 넉넉히 멀면 된다.
        assert math.dist((robot.x, robot.y), opponent) > DODGE_START_M + 0.15
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


def test_guard_ignores_far_opponent(monkeypatch):
    logic, robot = guarding_robot(monkeypatch)
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


def test_guard_stands_still_when_sandwiched(monkeypatch):
    logic, robot = guarding_robot(monkeypatch)
    # 원 위 양쪽에서 상대가 붙어 온다: 어느 쪽으로 피해도 부딪힌다.
    opponents = [guard_point(GUARD_HOME_ANGLE + 0.5), guard_point(GUARD_HOME_ANGLE - 0.5)]
    commands = run_guard_with_opponents(logic, robot, opponents, 3.0)
    assert all(command.linear == 0.0 for command in commands)


def test_guard_moves_away_when_pinned_between_two_opponents(monkeypatch):
    # 2026-11-03 17:33 실기: 0.27 m에서 미는 상대와 반대쪽 물체 사이에서 2초 서 있다가 탈락선 쪽으로 밀렸다.
    # 상대가 이미 붙어 있으면 서 있지 않고 덜 막힌 쪽으로 빠진다 (정면 충돌은 _guard_drive가 막는다).
    logic, robot = guarding_robot(monkeypatch)
    touching = guard_point(GUARD_HOME_ANGLE + 0.3)
    other = guard_point(GUARD_HOME_ANGLE - 0.55)
    start = math.dist((robot.x, robot.y), touching)
    assert start < mission2_logic.DODGE_BLOCKED_M
    commands = run_guard_with_opponents(logic, robot, [touching, other], 2.0)
    assert any(command.linear != 0.0 for command in commands)
    assert math.dist((robot.x, robot.y), touching) > start + 0.05
    assert math.dist((robot.x, robot.y), other) > 0.25


def test_guard_dodges_toward_open_side_with_two_opponents(monkeypatch):
    logic, robot = guarding_robot(monkeypatch)
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


def test_retreat_starts_30cm_from_line_after_start(monkeypatch):
    logic, robot = guarding_robot(monkeypatch)
    # 밀려서 노란 선(x = 3.2)에서 0.25 m: 예전(0.15)은 그대로 있었다.
    robot.x, robot.y = 2.95, 1.8
    logic.step(robot.observe())
    assert logic.state == RETREAT

    # 출발 위치는 선에서 0.2 m지만 출발 중에는 RETREAT하지 않는다.
    logic = Mission2Logic()
    robot = SimRobot()
    logic.step(robot.observe())
    assert logic.state == DRIVE


def test_retreat_pushes_back_when_opponent_squeezes_toward_line(monkeypatch):
    # 2026-11-01 실기: 사방에서 조이면 노란 선 밖으로 나갔다. 중앙 쪽 상대가 차체에 붙어(표면 0.2 m)
    # 0.15 m/s로 밀고 온다.
    # 예전에는 중앙으로 가려다 앞의 상대 때문에 서 버려 그대로 밀려 나갔다.
    logic, robot = guarding_robot(monkeypatch)
    robot.x, robot.y = 2.95, 1.8
    push = 0.15
    for _ in range(round(4.0 / DT)):
        obs = robot.observe()
        obs.opponents = ((robot.x - 0.2, robot.y), (robot.x, robot.y + 0.4),
                         (robot.x, robot.y - 0.4))
        robot.apply(logic.step(obs))
        robot.x += push * DT
        assert distance_to_danger_line(robot.observe()) > 0.1


def test_moves_closer_to_center_when_full_turn_finds_nothing():
    # 구석 쪽에서 큐브를 못 찾고 있다 (큐브는 상대가 가져가 안 보임).
    logic = Mission2Logic()
    logic.state = SEARCH
    robot = SimRobot(pose=(0.75, 0.75, 0.0), target=(100.0, 100.0))
    states = run(logic, robot, max_seconds=25.0)

    # 한 바퀴 돌고 중앙 쪽으로 다가가 다시 돈다. 탈락선에는 가지 않는다.
    assert RECENTER in states
    assert states[-1] == SEARCH
    assert math.dist((robot.x, robot.y), CENTER_M) < SEARCH_CENTER_M + 0.1
    assert distance_to_danger_line(robot.observe()) > 0.3

    # 상대가 탈락해 큐브가 중앙으로 돌아오면 찾아서 잡는다.
    robot.target = CENTER_M
    run(logic, robot, max_seconds=robot.now + 30.0, until=GUARD)
    assert logic.state == GUARD


def test_search_near_center_moves_around_after_full_turn():
    # 2026-11-03 19:42 실기: 중앙에서 1.0 m 자리에서 큐브를 못 찾고 6초 넘게 제자리로만 돌았다.
    # 한 바퀴 돌아도 없으면 중앙 둘레를 따라 옮겨 다시 찾는다.
    logic = Mission2Logic()
    logic.gripper_opened = True
    start = (CENTER_M[0] - 0.9, CENTER_M[1] - 0.3)
    robot = SimRobot(pose=(*start, 0.0), target=(100.0, 100.0))
    logic.state = APPROACH
    logic.state_started_at = 0.0
    run(logic, robot, max_seconds=12.0)
    moved = angle_from_center(robot.x, robot.y) - angle_from_center(*start)
    moved = math.atan2(math.sin(moved), math.cos(moved))
    assert abs(moved) > math.radians(40.0)
    # 옮긴 자리에서 보이는 큐브는 바로 잡으러 간다
    side = robot.yaw + 2.0
    robot.target = (robot.x + 0.6 * math.cos(side), robot.y + 0.6 * math.sin(side))
    states = run(logic, robot, max_seconds=robot.now + 6.0, until=APPROACH)
    assert logic.state == APPROACH, states


def _flicker_then_lose(logic, now):
    """멀리서 큐브가 깜빡 보였다가(면적 400) 사라진다. 다시 SEARCH·RECENTER가 될 때까지 돌린다."""
    logic.step(Observation(x=1.0, y=2.6, yaw=0.0, now=now, detected=True,
                           x_offset=0.0, area=400.0))
    assert logic.state in (ALIGN, APPROACH)
    for i in range(1, 40):
        logic.step(Observation(x=1.0, y=2.6, yaw=0.0, now=now + i * DT))
        if logic.state in (SEARCH, RECENTER):
            return


def test_short_chase_resumes_search_turn_count():
    # 2026-11-03 23:12 실기: 깜빡 보인 큐브를 쫓다 놓칠 때마다 돈 각도가 0이 되어 원을 따라 옮기지 못했다.
    logic = Mission2Logic()
    logic.gripper_opened = True
    logic._left_start = True
    logic.state = SEARCH
    logic.state_started_at = 0.0
    logic._search_step = 2
    logic._search_turned = 5.0
    logic._search_last_yaw = 0.0
    _flicker_then_lose(logic, 0.0)
    assert logic.state == SEARCH
    assert logic._search_turned >= 5.0
    assert logic._search_step == 2


def test_short_chase_resumes_recenter_goal():
    logic = Mission2Logic()
    logic.gripper_opened = True
    logic._left_start = True
    logic.state = RECENTER
    logic.state_started_at = 0.0
    logic._recenter_angle = 3.3
    _flicker_then_lose(logic, 0.0)
    assert logic.state == RECENTER
    assert logic._recenter_angle == 3.3


def test_long_chase_starts_search_fresh():
    logic = Mission2Logic()
    logic.gripper_opened = True
    logic._left_start = True
    logic.state = SEARCH
    logic.state_started_at = 0.0
    logic._search_step = 2
    logic._search_turned = 5.0
    _flicker_then_lose(logic, 0.0)
    logic.state = APPROACH
    logic._search_paused = (SEARCH, 5.0, None, 0.0)
    logic._target_seen_at = 10.0
    logic.step(Observation(x=1.0, y=2.6, yaw=0.0, now=10.6))
    assert logic.state == SEARCH
    assert logic._search_turned < 1.0


def test_close_off_center_cube_slows_and_turns():
    # 2026-11-03 23:12 실기: 40 cm 앞 x_offset +0.58에서 0.77 m/s로 달려 큐브 옆을 지나가며 밀어냈다.
    logic = Mission2Logic()
    logic.gripper_opened = True
    logic._left_start = True
    logic.state = APPROACH
    logic.state_started_at = 0.0
    logic.target = (2.0, 1.4)
    logic._area_best = 2000.0
    logic._area_best_at = (2.27, 1.34)
    logic._target_seen_at = 0.0
    logic._last_linear = 0.77
    command = logic._approach(Observation(x=2.27, y=1.34, yaw=math.radians(130), now=0.05,
                                          detected=True, x_offset=0.58, area=2000.0))
    assert command.linear <= APPROACH_SLOW_SPEED
    assert command.angular < -0.5  # 큐브가 오른쪽 -> 오른쪽으로


def test_no_retreat_when_cube_seen_right_after_start():
    # 2026-11-03 23:12 실기: 출발 0.4초 만에 큐브를 보고 APPROACH가 되자 출발 칸(선에서 0.2~0.3 m)에서 RETREAT.
    logic = Mission2Logic()
    robot = SimRobot()
    logic.step(robot.observe())
    logic.state = APPROACH
    logic.step(robot.observe())
    assert logic.state != RETREAT
    # 선에서 한 번 멀어진 뒤에는 다시 0.3 m 기준
    robot.x, robot.y = robot.x - 0.3, robot.y + 0.3
    logic.step(robot.observe())
    robot.x, robot.y = robot.x + 0.25, robot.y - 0.25
    logic.step(robot.observe())
    assert logic.state == RETREAT


def test_recenter_does_not_drive_into_opponent():
    logic = Mission2Logic()
    logic.state = RECENTER
    logic.gripper_opened = True
    # 중앙 쪽(+x, +y)으로 가야 하는데 바로 앞에 상대가 있다.
    obs = Observation(x=0.8, y=0.8, yaw=math.pi / 4, now=0.0,
                      opponents=((1.0, 1.0),))
    assert logic.wants_opponents()
    command = logic.step(obs)
    assert command.linear == 0.0


# ---------------------------------------------------------------
# 2026-11-01 실기 19:30: 큐브 옆을 집음, 경기장 밖 빨간 물체
# ---------------------------------------------------------------

def _close_approach_logic():
    logic = Mission2Logic()
    logic.state = APPROACH
    logic.state_started_at = 0.0
    logic.target = (1.8, 1.8)
    logic._area_best = 13000.0
    logic._area_best_at = (2.1, 2.0)
    logic._target_seen_at = 0.0
    return logic


def test_close_off_center_cube_turns_to_center_before_going_straight():
    """가까이서 큐브가 화면 가운데가 아니면(x_offset -0.43) 곧게 가지 않고 가면서 카메라 방향으로 돈다."""
    logic = _close_approach_logic()
    command = logic._approach(Observation(x=2.06, y=2.02, yaw=math.radians(154), now=0.1,
                                          detected=True, x_offset=-0.43, area=13500.0))
    assert logic.state == APPROACH
    # 서지 않고(주춤하지 않고) 가면서 왼쪽으로 맞춘다 (큐브를 조금 밀어도 된다, 2026-11-01 팀)
    assert command.linear > 0.0 and command.angular > 0.0
    # 가운데로 들어오면 곧게
    command = logic._approach(Observation(x=2.06, y=2.02, yaw=math.radians(160), now=0.2,
                                          detected=True, x_offset=-0.05, area=14000.0))
    assert command.linear > 0.0 and command.angular == 0.0


def test_grab_waits_until_cube_is_centered():
    """잡을 만큼 커도 큐브가 화면 옆(x_offset -0.54)이면 집게를 닫지 않는다 (헛 집게질)."""
    logic = _close_approach_logic()
    logic._approach(Observation(x=2.0, y=2.05, yaw=math.radians(156), now=0.1,
                                detected=True, x_offset=-0.54, area=36000.0))
    assert logic.state == APPROACH
    logic._approach(Observation(x=2.0, y=2.05, yaw=math.radians(170), now=0.3,
                                detected=True, x_offset=-0.05, area=36000.0))
    assert logic.state == PUSH


def test_red_object_outside_arena_is_not_used_as_cube():
    """출발 전 오른쪽 경기장 밖 빨간 물체(x_offset +0.81, 면적 430)는 큐브 위치로 쓰지 않는다."""
    logic = Mission2Logic()
    x, y, yaw = START_POSE
    logic.step(Observation(x=x, y=y, yaw=yaw, now=0.0, detected=True, x_offset=0.81, area=430.0))
    assert logic.target is None


# ---------------------------------------------------------------
# 2026-11-01 팀: 큐브를 조금 밀어도 되니 주춤하지 말고 적외선으로 확실하게 잡기
# ---------------------------------------------------------------

def test_push_closes_when_tof_sees_cube_inside_gripper():
    """적외선이 있으면 카메라 면적으로 닫지 않고 밀며 들어가다가 큐브가 집게 안(5 cm)에 보이면 닫는다."""
    logic = _close_approach_logic()
    obs = Observation(x=2.0, y=2.05, yaw=math.radians(156), now=0.1, detected=True,
                      x_offset=-0.4, area=36000.0, gripper_range=0.12)
    command = logic._approach(obs)
    assert logic.state == PUSH            # 비켜 있어도 적외선이 있으니 밀며 들어간다
    assert command.gripper is None and command.linear > 0.0
    command = logic._push(Observation(x=2.0, y=2.08, yaw=math.radians(156), now=0.2,
                                      gripper_range=0.08))
    assert logic.state == PUSH and command.gripper is None
    command = logic._push(Observation(x=2.0, y=2.10, yaw=math.radians(156), now=0.3,
                                      gripper_range=0.045))
    assert logic.state == GRASP and command.gripper == GRIPPER_GRAB


def test_push_backs_off_without_closing_when_cube_never_enters(monkeypatch):
    """15 cm를 밀어도 적외선에 안 잡히면(옆으로 비켜 감) 닫지 않고 집게를 연 채 물러난다 (헛 집게질 없음)."""
    monkeypatch.setattr(mission2_logic, 'LURK_AFTER_FAIL', False)
    monkeypatch.setattr(mission2_logic, 'SPIN_AFTER_FAIL', False)
    logic = _close_approach_logic()
    logic._approach(Observation(x=2.0, y=2.0, yaw=math.radians(90), now=0.1, detected=True,
                                x_offset=0.0, area=36000.0, gripper_range=math.inf))
    assert logic.state == PUSH
    command = logic._push(Observation(x=2.0, y=2.16, yaw=math.radians(90), now=0.5,
                                      gripper_range=math.inf))
    assert command.gripper == GRIPPER_OPEN
    assert logic.state != GRASP


def test_tof_inside_during_approach_closes_right_away():
    """카메라에서 사라졌어도 적외선이 집게 안 큐브를 보면 바로 닫는다."""
    logic = _close_approach_logic()
    command = logic._approach(Observation(x=2.0, y=2.05, yaw=0.0, now=0.1, gripper_range=0.03))
    assert logic.state == GRASP and command.gripper == GRIPPER_GRAB


def test_grasp_failed_spins_in_place_forever(monkeypatch):
    """잡기 실패 뒤에는 쫓아가지 않고 끝까지 제자리에서 돌기만 한다 (2026-11-01 팀)."""
    monkeypatch.setattr(mission2_logic, 'LURK_AFTER_FAIL', False)
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GRASP)

    robot.holding = False
    run(logic, robot, max_seconds=robot.now + GRASP_WAIT_S + 1.0, until=SPIN)
    assert logic.state == SPIN
    spot = (robot.x, robot.y)
    gripper_commands = len(robot.gripper_log)
    for _ in range(int(3.0 / DT)):
        command = logic.step(robot.observe())
        robot.apply(command)
        assert logic.state == SPIN
        assert command.linear == 0.0
    assert command.angular == SPIN_SPEED
    assert math.dist((robot.x, robot.y), spot) < 0.02
    assert len(robot.gripper_log) == gripper_commands  # 집게는 그대로


def test_lost_cube_during_guard_spins(monkeypatch):
    monkeypatch.setattr(mission2_logic, 'LURK_AFTER_FAIL', False)
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GUARD)

    robot.holding = False
    command = logic.step(robot.observe())
    assert logic.state == SPIN
    assert command.gripper is None


def test_spin_returns_to_spin_after_retreat(monkeypatch):
    monkeypatch.setattr(mission2_logic, 'LURK_AFTER_FAIL', False)
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GRASP)
    robot.holding = False
    run(logic, robot, max_seconds=robot.now + GRASP_WAIT_S + 1.0, until=SPIN)

    # 밀려서 탈락선 가까이 가면 RETREAT로 안쪽에 돌아온 뒤 다시 돌기만 한다.
    robot.x, robot.y = 0.6, 1.8
    states = run(logic, robot, max_seconds=robot.now + 5.0, until=SPIN)
    assert RETREAT in states
    assert logic.state == SPIN


def test_push_fail_spins_without_closing(monkeypatch):
    """밀어도 큐브가 안 들어오면 닫지 않고(헛 집게질 없음) 그 자리에서 돌기만 한다."""
    monkeypatch.setattr(mission2_logic, 'LURK_AFTER_FAIL', False)
    logic = _close_approach_logic()
    logic._approach(Observation(x=2.0, y=2.0, yaw=math.radians(90), now=0.1, detected=True,
                                x_offset=0.0, area=36000.0, gripper_range=math.inf))
    assert logic.state == PUSH
    command = logic._push(Observation(x=2.0, y=2.16, yaw=math.radians(90), now=0.5,
                                      gripper_range=math.inf))
    assert logic.state == SPIN
    assert command.gripper is None and command.linear == 0.0


# ---------------------------------------------------------------
# 실패·놓침 뒤 LURK: GUARD처럼 피하고, 빨간 대상이 보이면 다시 잡으러 간다 (2026-11-02 팀)
# ---------------------------------------------------------------

FAR_AWAY = (100.0, 100.0)  # 카메라에 안 보이는 곳 (큐브를 상대가 가져갔다)


def lurking_robot(target=FAR_AWAY):
    """잡기에 실패해 LURK로 간 로봇. 그 뒤 큐브는 target에 있다."""
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GRASP)
    robot.holding = False
    run(logic, robot, max_seconds=robot.now + GRASP_WAIT_S + 1.0, until=LURK)
    assert logic.state == LURK
    robot.target = target
    return logic, robot


def test_grasp_failed_retries_when_cube_is_visible():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GRASP)
    robot.holding = False
    run(logic, robot, max_seconds=robot.now + GRASP_WAIT_S + 1.0, until=LURK)
    grabs = robot.gripper_log.count(GRIPPER_GRAB)
    opens = robot.gripper_log.count(GRIPPER_OPEN)

    # 큐브는 그대로 앞에 있다: 물러나 거리를 번 뒤 집게를 열고 다시 잡아 지킨다.
    robot.holding = None
    states = run(logic, robot, max_seconds=robot.now + 10.0, until=GUARD)
    assert logic.state == GUARD
    # (ALIGN은 큐브가 이미 앞에 있으면 같은 판단 안에서 바로 APPROACH로 넘어간다)
    assert states[0] == LURK and APPROACH in states and GRASP in states
    assert robot.gripper_log.count(GRIPPER_OPEN) == opens + 1
    assert robot.gripper_log.count(GRIPPER_GRAB) > grabs


def test_lurk_keeps_gripper_until_retry():
    # ToF가 잘못 '놓침'으로 봐도 큐브를 떨어뜨리지 않게, 다시 잡으러 갈 때까지 집게를 열지 않는다.
    logic, robot = lurking_robot()
    gripper_commands = len(robot.gripper_log)
    run(logic, robot, max_seconds=robot.now + 5.0)
    assert logic.state == LURK
    assert len(robot.gripper_log) == gripper_commands


def test_lurk_backs_out_to_circle_and_spins():
    logic, robot = lurking_robot()
    run(logic, robot, max_seconds=robot.now + 8.0)
    assert logic.state == LURK
    assert abs(math.dist((robot.x, robot.y), CENTER_M) - GUARD_RADIUS_M) < GUARD_ARRIVE_M + 0.03
    spot = (robot.x, robot.y)
    for _ in range(int(2.0 / DT)):
        command = logic.step(robot.observe())
        robot.apply(command)
        assert command.linear == 0.0
    assert command.angular == SPIN_SPEED
    assert math.dist((robot.x, robot.y), spot) < 0.02


def test_lurk_dodges_opponent_along_circle():
    logic, robot = lurking_robot()
    run(logic, robot, max_seconds=robot.now + 8.0)
    start_angle = angle_from_center(robot.x, robot.y)
    opponent = guard_point(start_angle + 0.6)
    commands = run_guard_with_opponents(logic, robot, [opponent], 5.0)
    assert any(command.linear != 0.0 for command in commands)
    turned = angle_from_center(robot.x, robot.y) - start_angle
    assert turned < -0.05
    assert math.dist((robot.x, robot.y), opponent) > DODGE_START_M


def test_lurk_does_not_chase_cube_held_by_opponent(monkeypatch):
    # 가짜 카메라는 지연이 없다: 로직이 실제 지연(CAMERA_LAG_S)을 빼면 도는 동안 큐브 위치가 15 cm 틀어진다
    monkeypatch.setattr(mission2_logic, 'CAMERA_LAG_S', 0.0)
    # 공격(ATTACK_HELD_CUBE)을 끄면 예전처럼 상대가 잡은 큐브에는 가지 않는다
    monkeypatch.setattr(mission2_logic, 'ATTACK_HELD_CUBE', False)
    logic, robot = lurking_robot()
    run(logic, robot, max_seconds=robot.now + 8.0)
    # 원 위에서 돌다 보면 큐브가 보이지만, 상대 로봇(큐브에서 0.15 m) 집게에 있어 가지 않는다.
    robot.target = (CENTER_M[0] + 0.2, CENTER_M[1])
    holder = (CENTER_M[0] + 0.35, CENTER_M[1])
    seen = False
    while robot.now < 40.0:
        obs = robot.observe()
        obs.opponents = (holder,)
        seen = seen or obs.detected
        robot.apply(logic.step(obs))
        assert logic.state == LURK
    assert seen

    # 상대가 큐브를 두고 떠나면 다시 잡으러 간다
    # (ALIGN은 큐브가 앞에 있으면 같은 판단 안에서 바로 APPROACH로 넘어가 기록에 안 남을 수 있다)
    states = run(logic, robot, max_seconds=robot.now + 10.0, until=PUSH)
    assert APPROACH in states and logic.state == PUSH


def test_lurk_retry_dodges_opponent_like_guard():
    # 2026-11-03 실기: 실패 뒤 다시 잡으러 가는 중(ALIGN·APPROACH)에는 상대를 안 봐서 피하지 않았다.
    # 이제는 GUARD·LURK처럼 상대가 DODGE_START_M 안으로 오면 LURK로 돌아가 피한다.
    logic, robot = lurking_robot(target=CENTER_M)
    run(logic, robot, max_seconds=robot.now + 15.0, until=APPROACH)
    assert logic.state == APPROACH
    assert logic.wants_opponents()
    # 큐브 쪽으로 가는 길 옆에 상대가 다가와 있다
    ahead = (robot.x + 0.5 * math.cos(robot.yaw), robot.y + 0.5 * math.sin(robot.yaw))
    opponent = (ahead[0] - 0.15 * math.sin(robot.yaw), ahead[1] + 0.15 * math.cos(robot.yaw))
    states = []
    closest = math.inf
    for _ in range(int(3.0 / DT)):
        obs = robot.observe()
        obs.opponents = (opponent,)
        robot.apply(logic.step(obs))
        states.append(logic.state)
        closest = min(closest, math.dist((robot.x, robot.y), opponent))
    assert states[0] == LURK
    assert PUSH not in states and GRASP not in states
    assert closest > 0.35
    assert math.dist((robot.x, robot.y), opponent) > DODGE_START_M


def test_lurk_waits_for_opponent_to_clear_before_retry():
    # 상대가 DODGE_START_M ~ DODGE_CLEAR_M 사이에 있으면 다시 잡으러 가지 않는다 (가다가 바로 피하게 된다)
    logic, robot = lurking_robot()
    run(logic, robot, max_seconds=robot.now + 8.0)
    robot.target = CENTER_M
    toward_center = math.atan2(CENTER_M[1] - robot.y, CENTER_M[0] - robot.x)
    side = toward_center + math.pi / 2
    opponent = (robot.x + 0.8 * math.cos(side), robot.y + 0.8 * math.sin(side))
    for _ in range(int(5.0 / DT)):
        obs = robot.observe()
        obs.opponents = (opponent,)
        robot.apply(logic.step(obs))
        if math.dist((robot.x, robot.y), opponent) <= mission2_logic.DODGE_CLEAR_M:
            assert logic.state == LURK


def test_lurk_moves_along_circle_after_full_turn_without_cube():
    # 2026-11-03 팀: 한 자리에서 한 바퀴 돌아도 못 찾으면 원을 따라 LURK_MOVE_RAD 옮겨 다시 돈다.
    logic, robot = lurking_robot()
    run(logic, robot, max_seconds=robot.now + 4.0)
    spot = (robot.x, robot.y)
    first_angle = angle_from_center(*spot)
    run(logic, robot, max_seconds=robot.now + 8.0)
    assert logic.state == LURK
    moved = angle_from_center(robot.x, robot.y) - first_angle
    moved = math.atan2(math.sin(moved), math.cos(moved))
    assert abs(moved) > math.radians(40.0)
    assert abs(math.dist((robot.x, robot.y), CENTER_M) - GUARD_RADIUS_M) < 0.15


def test_lurk_finds_far_cube_after_moving():
    # 첫 자리에서는 너무 멀어(약 1.35 m 넘게) 안 보이던 큐브를 옮긴 자리에서 보고 잡으러 간다.
    logic, robot = lurking_robot()
    run(logic, robot, max_seconds=robot.now + 4.0)
    # 원 위 첫 자리에서 약 1.9 m (안 보임), LURK_MOVE_RAD 옮긴 자리에서 약 1.1 m
    away = angle_from_center(robot.x, robot.y) + 2 * mission2_logic.LURK_MOVE_RAD
    robot.target = (CENTER_M[0] + 1.3 * math.cos(away), CENTER_M[1] + 1.3 * math.sin(away))
    states = run(logic, robot, max_seconds=robot.now + 25.0, until=APPROACH)
    assert logic.state == APPROACH, states


class PushableSimRobot(SimRobot):
    """닿으면 상대(victim)가 우리가 미는 만큼 같이 밀려나는 가짜 로봇 (테스트 전용)."""

    victim = None
    contact_m = 0.32  # 로봇 중심 ~ 상대 중심

    def apply(self, command):
        before = (self.x, self.y)
        super().apply(command)
        if self.victim is None:
            return
        gap = math.dist((self.x, self.y), self.victim)
        if gap < self.contact_m:
            # 겹친 만큼 상대를 우리 진행 방향 쪽으로 밀어낸다
            ux, uy = self.victim[0] - self.x, self.victim[1] - self.y
            push = self.contact_m - gap
            self.victim = (self.victim[0] + ux / gap * push, self.victim[1] + uy / gap * push)
            moved = math.dist(before, (self.x, self.y))
            assert moved >= 0.0
        # 상대가 잡은 큐브도 같이 끌려간다
        self.target = (self.victim[0] - 0.15, self.victim[1])


def test_attack_pushes_opponent_holding_cube_to_line(monkeypatch):
    # 2026-11-03 팀: 상대가 잡은 큐브를 보면 그 상대를 가장 가까운 노란 선 쪽으로 민다.
    monkeypatch.setattr(mission2_logic, 'CAMERA_LAG_S', 0.0)
    logic, robot = lurking_robot()
    pushable = PushableSimRobot(pose=(robot.x, robot.y, robot.yaw), target=robot.target)
    pushable.now = robot.now
    pushable.holding = False
    run(logic, pushable, max_seconds=pushable.now + 4.0)
    # 상대가 큐브를 물고 중앙 오른쪽에 서 있다 (가장 가까운 선은 +x 쪽)
    pushable.victim = (2.6, 1.8)
    pushable.target = (2.45, 1.8)
    states = set()
    lowest = math.inf
    while pushable.now < 40.0 and pushable.victim[0] < SAFE_ZONE_MAX_M + 0.05:
        obs = pushable.observe()
        obs.opponents = (pushable.victim,)
        pushable.apply(logic.step(obs))
        states.add(logic.state)
        lowest = min(lowest, distance_to_danger_line(pushable.observe()))
    assert ATTACK in states
    assert logic.attack_dir == (1.0, 0.0)
    # 상대 중심이 노란 선(+x 쪽)까지 밀렸고, 우리는 선 앞에서 RETREAT로 멈췄다
    assert pushable.victim[0] > SAFE_ZONE_MAX_M - 0.05
    assert lowest > 0.15


def test_lurk_returns_to_lurk_after_retreat():
    logic, robot = lurking_robot()
    robot.x, robot.y = 0.6, 1.8
    states = run(logic, robot, max_seconds=robot.now + 5.0)
    assert RETREAT in states
    assert logic.state == LURK


def test_lost_cube_during_guard_lurks_without_opening():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GUARD)

    robot.holding = False
    command = logic.step(robot.observe())
    assert logic.state == LURK
    assert command.gripper is None


def test_regrabbed_cube_is_guarded_after_retreat():
    # 실패 뒤 다시 잡았으면 RETREAT가 끝나고 LURK가 아니라 GUARD로 돌아간다.
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GUARD)
    logic.gave_up = True
    robot.x, robot.y = 0.6, 1.8
    states = run(logic, robot, max_seconds=robot.now + 8.0)
    assert RETREAT in states
    assert logic.state == GUARD


def test_cube_inside_gripper_during_lurk_closes_without_opening():
    logic, robot = lurking_robot()
    obs = robot.observe()
    obs.gripper_range = 0.03
    command = logic.step(obs)
    assert logic.state == GRASP
    assert command.gripper == GRIPPER_GRAB


# ---------------------------------------------------------------
# 잡은 자리에서 지키기 (GUARD_ESCAPE_FIRST False로 켤 때)
# ---------------------------------------------------------------

def test_guard_stays_where_it_grabbed_without_opponents(monkeypatch):
    monkeypatch.setattr(mission2_logic, 'GUARD_ESCAPE_FIRST', False)
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GUARD)
    spot = (robot.x, robot.y)
    for _ in range(int(5.0 / DT)):
        command = logic.step(robot.observe())
        robot.apply(command)
        assert command.linear == 0.0
    assert logic.state == GUARD
    assert math.dist((robot.x, robot.y), spot) < 0.02


def test_guard_dodges_only_when_opponent_comes_then_stands(monkeypatch):
    monkeypatch.setattr(mission2_logic, 'GUARD_ESCAPE_FIRST', False)
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GUARD)
    # 상대가 잡은 자리 0.5 m 옆으로 다가와 멈춰 있다 -> 피한다
    opponent = (robot.x + 0.5, robot.y - 0.1)
    start = (robot.x, robot.y)
    moved = False
    for _ in range(int(6.0 / DT)):
        obs = robot.observe()
        obs.opponent = opponent
        command = logic.step(obs)
        robot.apply(command)
        moved = moved or command.linear != 0.0
        assert distance_to_danger_line(robot.observe()) > 0.3
    assert moved
    assert math.dist((robot.x, robot.y), opponent) > math.dist(start, opponent) + 0.2
    # 상대가 멀어지면 원 위나 처음 자리로 가지 않고 지금 자리에 선다
    spot = (robot.x, robot.y)
    for _ in range(int(3.0 / DT)):
        command = logic.step(robot.observe())
        robot.apply(command)
    assert math.dist((robot.x, robot.y), spot) < 0.1
    assert command.linear == 0.0


# ---------------------------------------------------------------
# 밀림 감지·막힌 RETREAT 비켜 가기 (2026-11-02 21:47 실기: 3초 밀리고 선 앞에서 바퀴만 돌다 밀려 나감)
# ---------------------------------------------------------------

def test_push_while_standing_starts_dodging_right_away(monkeypatch):
    logic, robot = guarding_robot(monkeypatch)
    # 서 있다가 앞에서 밀려 뒤로 밀린다 (엔코더 -0.1 m/s)
    moved_at = None
    for k in range(10):
        obs = robot.observe()
        obs.measured_linear = -0.1
        command = logic.step(obs)
        robot.apply(command)
        if command.linear != 0.0 or command.angular != 0.0:
            moved_at = k
            break
    assert moved_at is not None and moved_at <= 3  # 0.15 s 안에 움직이기 시작
    # 미는 상대는 로봇 앞쪽에 있다고 본다
    px, py, _ = logic.push_opponent
    ahead = (px - robot.x) * math.cos(robot.yaw) + (py - robot.y) * math.sin(robot.yaw)
    assert ahead > 0.2
    # 피하는 동안 선에 가까이 가지 않고, 미는 상대에게서 멀어진다
    start_gap = math.dist((robot.x, robot.y), (px, py))
    for _ in range(int(1.5 / DT)):
        robot.apply(logic.step(robot.observe()))
        assert distance_to_danger_line(robot.observe()) > 0.3
    assert math.dist((robot.x, robot.y), (px, py)) > start_gap + 0.2


def test_no_push_when_encoder_still_or_just_stopped(monkeypatch):
    logic, robot = guarding_robot(monkeypatch)
    for _ in range(20):
        obs = robot.observe()
        obs.measured_linear = 0.0
        assert logic.step(obs) == Command()
    assert logic.push_opponent is None
    # 멈춘 직후(PUSH_SETTLE_S 안)에 바퀴가 아직 도는 것은 밀림이 아니다
    logic._stand_since = None
    for _ in range(3):
        obs = robot.observe()
        obs.measured_linear = 0.2
        logic.step(obs)
    assert logic.push_opponent is None


def test_slide_push_without_encoder_starts_dodging(monkeypatch):
    # 2026-11-03 17:33 실기: 바퀴가 안 돌고(엔코더 0) 미끄러지며 밀려 밀림 감지가 안 됐다.
    # 서 있는 동안 LiDAR 벽 보정 위치가 한쪽으로 계속 움직이면 밀린 것으로 본다.
    logic, robot = guarding_robot(monkeypatch)
    offset = [0.0, 0.0]
    moved_at = None
    for k in range(20):
        if k >= 2:
            # +x 쪽으로 스캔마다 1.5 cm씩 밀린다 (엔코더는 0)
            robot.x += 0.015
            offset[0] += 0.015
        obs = robot.observe()
        obs.measured_linear = 0.0
        obs.wall_offset = tuple(offset)
        command = logic.step(obs)
        robot.apply(command)
        if command.linear != 0.0 or command.angular != 0.0:
            moved_at = k
            break
    assert moved_at is not None
    assert 'LiDAR' in logic.push_reason
    # 미는 상대는 밀려 간 쪽(+x)의 반대쪽에 있다
    assert logic.push_opponent[0] < robot.x - 0.2


def test_single_wall_offset_jump_is_not_a_push(monkeypatch):
    # 빨리 돈 뒤 첫 스캔에서 한 번에 따라잡는 보정은 밀림이 아니다
    logic, robot = guarding_robot(monkeypatch)
    for k in range(20):
        obs = robot.observe()
        obs.measured_linear = 0.0
        obs.wall_offset = (0.06, 0.0) if k >= 5 else (0.0, 0.0)
        assert logic.step(obs) == Command()
    assert logic.push_opponent is None


def test_retreat_is_full_power_even_holding_cube():
    # 2026-11-03 팀: 선 밖은 즉시 탈락이라 RETREAT는 무슨 상황에서든 최대 출력.
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GUARD)
    # 큐브를 잡은 채 선 0.2 m 앞에서 바깥(-y)을 비스듬히(60도) 보고 0.5 m/s로 나가는 중
    robot.x, robot.y, robot.yaw = 1.8, 0.6, -math.pi / 2 + math.radians(60.0)
    logic._last_linear = 0.5
    logic._last_command_at = robot.now - DT
    commands, states = [], set()
    for _ in range(int(1.0 / DT)):
        obs = robot.observe()
        command = logic.step(obs)
        robot.apply(command)
        commands.append(command)
        states.add(logic.state)
    assert RETREAT in states
    assert max(abs(c.linear) for c in commands) >= mission2_logic.RETREAT_SPEED - 0.01
    assert distance_to_danger_line(robot.observe()) > 0.3


def test_stands_still_when_pose_is_outside_arena():
    # 2026-11-03 20:20 실기: 위치가 (0.17, 9.91)로 계산돼 RETREAT가 엉뚱한 쪽으로 후진해 선 밖으로 나갔다.
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GUARD)
    robot.x, robot.y = 0.17, 9.91
    for _ in range(10):
        command = logic.step(robot.observe())
        robot.now += DT
    assert logic.pose_lost
    assert command.linear == 0.0 and command.angular == 0.0
    assert logic.state != RETREAT


def _with_camera(obs, red_area=None):
    """카메라 결과가 살아 있는 관측. red_area를 주면 그 면적의 빨강이 보이는 것으로 바꾼다."""
    obs.vision_stamp = obs.now
    if red_area is not None:
        obs.detected, obs.x_offset, obs.area = red_area > 0, 0.0, red_area
    return obs


def test_releases_grabbed_thing_that_is_not_cube():
    # 2026-11-03 팀: 상대 바퀴 같은 다른 것을 잡아도 큐브로 봤다. 카메라에 큰 빨강이 없으면 놓는다.
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GRASP)
    robot.holding = True  # ToF는 집게 안에 무언가 있다고 한다
    commands = []
    # 닫힘 기다리기(최대 GRASP_WAIT_S) + 빨강 확인(HELD_CUBE_CONFIRM_S)
    for _ in range(int(3.0 / DT)):
        command = logic.step(_with_camera(robot.observe(), red_area=0.0))
        robot.apply(command)
        commands.append(command)
        if logic.state == LURK:
            break
    assert logic.state == LURK
    assert commands[-1].gripper == GRIPPER_OPEN
    assert '큐브가 아님' in logic.notice


def test_keeps_real_cube_and_releases_when_red_disappears():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GRASP)
    robot.holding = True
    # 진짜 큐브: 집게 앞에 큰 빨강이 계속 보이면 지킨다
    for _ in range(int(3.0 / DT)):
        robot.apply(logic.step(_with_camera(robot.observe(), red_area=38000.0)))
    assert logic.state == GUARD
    # 잡고 있는 것에서 빨강이 1초 넘게 안 보이면(큐브가 빠지고 다른 것이 끼었다) 놓는다
    for _ in range(int(1.5 / DT)):
        command = logic.step(_with_camera(robot.observe(), red_area=0.0))
        robot.apply(command)
        if logic.state == LURK:
            break
    assert logic.state == LURK and command.gripper == GRIPPER_OPEN


def test_camera_dropout_does_not_release_cube():
    # 카메라 결과가 끊기면(vision_stamp 없음) 예전처럼 ToF만 믿는다
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GUARD)
    robot.holding = True
    for _ in range(int(3.0 / DT)):
        obs = robot.observe()
        obs.detected, obs.area = False, 0.0
        robot.apply(logic.step(obs))
    assert logic.state == GUARD


def test_tof_shortcut_does_not_close_on_non_red_thing():
    # 다가가는 중 ToF가 집게 안을 봐도 큰 빨강이 아니면 바로 닫지 않는다 (상대 바퀴)
    logic = Mission2Logic()
    logic.state = APPROACH
    logic.state_started_at = 0.0
    logic.gripper_opened = True
    logic.target = (1.8, 1.8)
    logic._target_seen_at = 0.0
    obs = _with_camera(Observation(x=1.8, y=1.2, yaw=math.pi / 2, now=0.1, detected=True,
                                   x_offset=0.0, area=3000.0, gripper_range=0.03))
    logic.step(obs)
    assert logic.state != PUSH


def test_retreat_sidesteps_when_blocked_at_line():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GUARD)
    # 선 0.25 m 앞에서 바깥을 보고 서 있는데, 미는 상대에 막혀 안쪽으로 한 발도 못 간다
    robot.x, robot.y, robot.yaw = 1.8, 2.95, -math.pi / 2
    angles = []
    for _ in range(int(2.5 / DT)):
        command = logic.step(robot.observe())
        robot.now += DT  # 위치는 그대로 (막힘)
        angles.append(command.angular)
    assert logic.state == RETREAT
    assert logic._retreat_side != 0.0
    # 곧장 안쪽(-y, 지금 방향)만 고집하지 않고 비스듬히 틀어 비켜 간다
    assert max(abs(a) for a in angles[int(1.0 / DT):]) > 0.3
