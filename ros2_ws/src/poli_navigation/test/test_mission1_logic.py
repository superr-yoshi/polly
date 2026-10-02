import math

from poli_navigation.grid_map import (
    BLOCKED,
    cell_center_m,
    FREE,
    GridMap,
    MISSION1_START,
    RULE_EXAMPLE_BLOCKED,
)
from poli_navigation.mission1_logic import (
    CENTER_M,
    Command,
    DONE,
    DRIVE,
    EXPLORE,
    FACE,
    GRASP,
    GRASP_WAIT_S,
    GRIPPER_GRAB,
    GRIPPER_OPEN,
    GRIPPER_REACH_M,
    Mission1Logic,
    Observation,
    RELEASE,
    RETURN,
    SCAN,
    SEARCH,
    STUCK,
)
from poli_navigation.scan_to_grid import point_to_cell, START_POSE
from poli_navigation.sim_world import sim_camera, simulate_scan


DT = 0.05

# 경기 시간 10분 (3.3)
MATCH_SECONDS = 600.0

# SIM_ONLY: 이 거리 안에서 "grab"하면 잡힌다.
GRAB_REACH_M = 0.3


def example_grid():
    grid = GridMap()
    for cell in RULE_EXAMPLE_BLOCKED:
        grid.mark_blocked(cell)
    return grid


class SimRobot:
    """명령대로 움직이는 이상적인 로봇 + 가짜 LiDAR/카메라/집게 (테스트 전용)."""

    def __init__(self, true_grid, cube=CENTER_M):
        self.true_grid = true_grid
        self.x, self.y, self.yaw = START_POSE
        self.cube = cube
        self.now = 0.0
        self.holding = None
        self.gripper_log = []
        self.visited = set()
        self._scan_pose = None
        self._scan = None

    def scan(self):
        # 광선 계산이 느리므로 로봇이 멈춰 있으면 지난 스캔을 다시 쓴다.
        pose = (round(self.x, 4), round(self.y, 4), round(self.yaw, 4))
        if pose != self._scan_pose:
            self._scan_pose = pose
            self._scan = simulate_scan(self.true_grid, (self.x, self.y, self.yaw))
        return self._scan

    def forget_scan(self):
        # 경기장이 바뀌면 (상대 로봇이 움직임) 지난 스캔을 다시 쓰지 않는다.
        self._scan_pose = None

    def observe(self, with_scan=True):
        detected, x_offset, area = sim_camera(
            self.x, self.y, self.yaw, self.cube
        )
        return Observation(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            now=self.now,
            scan=self.scan() if with_scan else None,
            detected=detected,
            x_offset=x_offset,
            area=area,
            holding=self.holding,
        )

    def gripper_front(self):
        return (
            self.x + GRIPPER_REACH_M * math.cos(self.yaw),
            self.y + GRIPPER_REACH_M * math.sin(self.yaw),
        )

    def apply(self, command):
        if command.gripper == GRIPPER_GRAB:
            distance = math.hypot(
                self.cube[0] - self.x, self.cube[1] - self.y
            )
            self.holding = distance < GRAB_REACH_M
        elif command.gripper == GRIPPER_OPEN:
            self.holding = False
        if command.gripper is not None:
            self.gripper_log.append(command.gripper)

        self.yaw += command.angular * DT
        self.x += command.linear * math.cos(self.yaw) * DT
        self.y += command.linear * math.sin(self.yaw) * DT
        self.now += DT
        self.visited.add(point_to_cell(self.x, self.y))

        # 큐브는 들지 않고 집게 앞에 붙여서 바닥에 끌고 간다.
        if self.holding:
            self.cube = self.gripper_front()


def run(logic, robot, max_seconds=MATCH_SECONDS, until=None):
    states = []
    while robot.now < max_seconds:
        # 스캔은 SCAN 상태에서만 쓰므로 그때만 계산한다.
        obs = robot.observe(with_scan=logic.state == SCAN)
        robot.apply(logic.step(obs))
        if not states or states[-1] != logic.state:
            states.append(logic.state)
        if until is not None and logic.state == until:
            break
    return states


def distance_to_start(point):
    start = cell_center_m(MISSION1_START)
    return math.hypot(point[0] - start[0], point[1] - start[1])


# ---------------------------------------------------------------
# 전체 흐름
# ---------------------------------------------------------------

def test_full_run_delivers_cube_to_start_cell():
    true_grid = example_grid()
    logic = Mission1Logic()
    robot = SimRobot(true_grid)

    states = run(logic, robot, until=DONE)

    assert logic.state == DONE
    assert FACE in states
    assert GRASP in states
    assert RELEASE in states

    # 큐브가 출발 격자 가운데에 놓여 있어야 한다.
    assert distance_to_start(robot.cube) < 0.1
    assert point_to_cell(*robot.cube) == MISSION1_START

    # 처음에 집게를 열고, 잡은 뒤, 마지막에 놓는다.
    assert robot.gripper_log[0] == GRIPPER_OPEN
    assert GRIPPER_GRAB in robot.gripper_log
    assert robot.gripper_log[-1] == GRIPPER_OPEN
    assert robot.holding is False


def test_full_run_never_enters_blocked_cell():
    true_grid = example_grid()
    logic = Mission1Logic()
    robot = SimRobot(true_grid)

    run(logic, robot, until=DONE)

    assert logic.state == DONE
    assert None not in robot.visited
    blocked = [cell for cell in robot.visited
               if not true_grid.is_passable(cell)]
    assert blocked == []


def test_full_run_without_obstacles():
    logic = Mission1Logic()
    robot = SimRobot(GridMap())

    run(logic, robot, until=DONE)

    assert logic.state == DONE
    assert distance_to_start(robot.cube) < 0.1


def test_done_does_not_move():
    logic = Mission1Logic()
    robot = SimRobot(example_grid())
    run(logic, robot, until=DONE)

    for _ in range(100):
        command = logic.step(robot.observe(with_scan=False))
        robot.apply(command)
        assert command.linear == 0.0
        assert command.angular == 0.0

    assert logic.state == DONE


# ---------------------------------------------------------------
# 예외 상황
# ---------------------------------------------------------------

def test_grasp_failed_goes_back_to_search():
    logic = Mission1Logic()
    robot = SimRobot(example_grid())
    run(logic, robot, until=GRASP)

    robot.holding = False
    for _ in range(int(GRASP_WAIT_S / DT) + 2):
        robot.apply(logic.step(robot.observe(with_scan=False)))
        if logic.state != GRASP:
            break

    assert logic.state == SEARCH
    assert logic.phase == EXPLORE
    assert robot.gripper_log[-1] == GRIPPER_OPEN


def test_unknown_holding_assumes_grabbed():
    logic = Mission1Logic()
    robot = SimRobot(example_grid())
    run(logic, robot, until=GRASP)

    robot.holding = None
    for _ in range(int(GRASP_WAIT_S / DT) + 2):
        robot.apply(logic.step(robot.observe(with_scan=False)))
        if logic.state != GRASP:
            break

    assert logic.phase == RETURN
    assert logic.state == SCAN


def test_dropped_while_returning_grabs_again():
    logic = Mission1Logic()
    robot = SimRobot(example_grid())
    run(logic, robot, until=GRASP)
    run(logic, robot, max_seconds=robot.now + 60.0, until=DRIVE)
    assert logic.phase == RETURN

    # 돌아가는 중에 큐브를 놓침
    robot.holding = False
    command = logic.step(robot.observe(with_scan=False))
    robot.apply(command)

    assert logic.phase == EXPLORE
    assert logic.state == SEARCH
    assert command.gripper == GRIPPER_OPEN

    run(logic, robot, until=DONE)
    assert logic.state == DONE
    assert distance_to_start(robot.cube) < 0.1


def test_opponent_blocking_exit_then_leaving():
    # 규정 예시 배치에서 출발 구역의 유일한 출구는 (9, 4)다.
    # 상대 로봇이 거기 서 있으면 장애물로 보이고 갈 길이 없다.
    true_grid = example_grid()
    true_grid.cells[(9, 4)] = BLOCKED
    logic = Mission1Logic()
    robot = SimRobot(true_grid)

    run(logic, robot, max_seconds=5.0, until=STUCK)
    assert logic.state == STUCK
    assert logic.grid.get((9, 4)) == BLOCKED

    # 상대가 떠나면 다시 스캔할 때 지도에서 지우고 출발한다.
    true_grid.cells[(9, 4)] = FREE
    robot.forget_scan()
    run(logic, robot, until=DONE)

    assert logic.state == DONE
    assert logic.grid.get((9, 4)) != BLOCKED
    assert distance_to_start(robot.cube) < 0.1


def test_release_point_puts_cube_on_cell_center():
    logic = Mission1Logic()
    start_x, start_y = cell_center_m(MISSION1_START)

    # (9, 2)에서 -y 방향으로 (9, 1)에 들어가면 로봇은 그만큼 위에서 멈춘다.
    x, y = logic._release_point((9, 2), MISSION1_START)

    assert abs(x - start_x) < 1e-9
    assert abs(y - (start_y + GRIPPER_REACH_M)) < 1e-9


def test_search_turns_in_place_when_cube_not_visible():
    logic = Mission1Logic()
    logic.state = SEARCH
    logic.state_started_at = 0.0
    logic.gripper_opened = True

    obs = Observation(x=1.8, y=1.8, yaw=0.0, now=0.0, detected=False)
    command = logic.step(obs)

    assert command.linear == 0.0
    assert command.angular > 0.0


def test_opens_gripper_once_at_start():
    logic = Mission1Logic()
    x, y, yaw = START_POSE

    first = logic.step(Observation(x=x, y=y, yaw=yaw, now=0.0))
    second = logic.step(Observation(x=x, y=y, yaw=yaw, now=DT))

    assert first == Command(gripper=GRIPPER_OPEN)
    assert second == Command()
