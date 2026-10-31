import collections
import dataclasses
import math

from hw_model import actual_twist
from poli_navigation.mission2_logic import (
    ALIGN,
    APPROACH,
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
    heading_error_to,
    HOLD,
    Mission2Logic,
    Observation,
    PUSH,
    RETREAT,
    SEARCH,
    START_POSE,
    TARGET_AREA_AT_50CM,
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

def test_full_run_reaches_hold_near_center():
    logic = Mission2Logic()
    robot = SimRobot()

    states = run(logic, robot, until=HOLD)

    assert logic.state == HOLD
    # 출발 곡선 중에 대상이 보이면 서지 않고 바로 카메라 접근
    assert states == [DRIVE, APPROACH, PUSH, GRASP, HOLD]

    # 집는 순간 대상과 가까워야 한다.
    distance = math.hypot(CENTER_M[0] - robot.x, CENTER_M[1] - robot.y)
    assert distance < 0.3

    # 시작 때 집게 명령 없이(미리 열어 둠, 2026-11-01) 잡을 때 한 번만 닫는다.
    assert robot.gripper_log == [GRIPPER_GRAB]


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

    while robot.now < 60.0 and logic.state != HOLD:
        robot.apply(logic.step(robot.observe()))
        assert distance_to_danger_line(robot.observe()) > 0.0


def test_hold_does_not_move():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=HOLD)

    for _ in range(100):
        command = logic.step(robot.observe())
        robot.apply(command)
        assert command.linear == 0.0
        assert command.angular == 0.0

    assert logic.state == HOLD


# ---------------------------------------------------------------
# 예외 상황
# ---------------------------------------------------------------

def test_search_when_target_not_visible_after_drive():
    logic = Mission2Logic()
    # 대상이 중앙에서 옆으로 밀려나 있는 경우 (출발 곡선에서 안 보이는 쪽)
    robot = SimRobot(target=(CENTER_M[0] + 0.5, CENTER_M[1] + 0.5))

    states = run(logic, robot, until=HOLD)

    assert SEARCH in states
    assert logic.state == HOLD


def test_grasp_failed_goes_back_to_search():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GRASP)

    robot.holding = False
    for _ in range(int(GRASP_WAIT_S / DT) + 2):
        command = logic.step(robot.observe())
        robot.apply(command)
        if logic.state != GRASP:
            break

    # 집게를 다시 열고 탐색부터 다시 시작한다. (이후 재시도)
    assert logic.state == SEARCH
    assert robot.gripper_log[-1] == GRIPPER_OPEN


def test_grasp_ends_as_soon_as_gripper_reports_closed():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=GRASP)
    started = robot.now

    robot.gripper_state = GRIPPER_CLOSED
    run(logic, robot, max_seconds=started + GRASP_WAIT_S, until=HOLD)

    # 최대 대기 시간(GRASP_WAIT_S)까지 기다리지 않는다.
    assert logic.state == HOLD
    assert robot.now - started < 0.5


def test_lost_target_during_hold_goes_back_to_search():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=HOLD)

    robot.holding = False
    logic.step(robot.observe())

    assert logic.state == SEARCH


def test_retreat_when_pushed_near_elimination_zone():
    logic = Mission2Logic()
    robot = SimRobot()
    run(logic, robot, until=HOLD)

    # 상대에게 밀려서 노란 선 가까이 감
    robot.x, robot.y = 3.15, 1.8
    command = logic.step(robot.observe())

    assert logic.state == RETREAT
    assert command.linear > 0.0 or command.angular != 0.0

    run(logic, robot, max_seconds=robot.now + 20.0, until=HOLD)
    assert logic.state == HOLD
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

    def __init__(self, pose, motor_lag_s=0.2, camera_lag_s=0.3):
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
        states = run(logic, robot, max_seconds=40.0, until=HOLD)

        assert logic.state == HOLD, (start_deg, states)
        # 한 번 찾은 대상을 놓쳐서 다시 찾는 일(SEARCH로 되돌아감)이 없어야 한다.
        assert SEARCH not in states[1:], (start_deg, states)


def test_full_run_with_latency_is_fast_and_direct():
    logic = Mission2Logic()
    robot = LaggedSimRobot(START_POSE)

    states = run(logic, robot, until=PUSH)

    # 서거나 다시 찾는 일 없이 곡선 -> 카메라 접근 -> 밀어 넣기
    assert states == [DRIVE, APPROACH, PUSH]
    # 예전 방식(제자리 45도 회전 + 0.25 m/s 직진)은 집기 시작까지 약 9초 걸렸다.
    assert robot.now < 3.5
    assert math.dist((robot.x, robot.y), CENTER_M) < 0.3
