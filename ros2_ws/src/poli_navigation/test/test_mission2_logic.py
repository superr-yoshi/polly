import math

from poli_navigation.mission2_logic import (
    ALIGN,
    APPROACH,
    CENTER_M,
    Command,
    DRIVE,
    GRASP,
    GRASP_WAIT_S,
    GRIPPER_GRAB,
    GRIPPER_OPEN,
    HOLD,
    Mission2Logic,
    Observation,
    RETREAT,
    SEARCH,
    START_POSE,
    TURN,
    distance_to_danger_line,
)


DT = 0.05

# 테스트용 가짜 카메라 (SIM_ONLY)
CAMERA_HALF_FOV_RAD = math.radians(30.0)
CAMERA_MAX_RANGE_M = 2.0
AREA_SCALE = 937.5  # 거리 0.25m에서 면적 15000


class SimRobot:
    """명령대로 움직이는 이상적인 로봇 + 가짜 카메라 (테스트 전용)."""

    def __init__(self, pose=START_POSE, target=CENTER_M):
        self.x, self.y, self.yaw = pose
        self.target = target
        self.now = 0.0
        self.holding = None
        self.gripper_log = []

    def observe(self):
        dx = self.target[0] - self.x
        dy = self.target[1] - self.y
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
        )

    def apply(self, command):
        if command.gripper is not None:
            self.gripper_log.append(command.gripper)

        self.yaw += command.angular * DT
        self.x += command.linear * math.cos(self.yaw) * DT
        self.y += command.linear * math.sin(self.yaw) * DT
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
    assert states[:2] == [TURN, DRIVE]
    assert GRASP in states

    # 집는 순간 대상과 가까워야 한다.
    distance = math.hypot(CENTER_M[0] - robot.x, CENTER_M[1] - robot.y)
    assert distance < 0.3

    # 처음에 집게를 열고, 마지막에 잡는다.
    assert robot.gripper_log[0] == GRIPPER_OPEN
    assert robot.gripper_log[-1] == GRIPPER_GRAB


def test_first_turn_is_left_45_degrees():
    logic = Mission2Logic()
    robot = SimRobot()

    run(logic, robot, until=DRIVE)

    turned = robot.yaw - START_POSE[2]
    assert abs(turned - math.radians(45.0)) < math.radians(6.0)


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
    # 대상이 중앙에서 옆으로 밀려나 있는 경우
    robot = SimRobot(target=(CENTER_M[0] + 0.5, CENTER_M[1] - 0.5))

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
