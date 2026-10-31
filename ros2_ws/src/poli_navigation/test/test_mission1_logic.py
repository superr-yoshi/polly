import math

from hw_model import actual_twist
from poli_navigation import mission1_logic
from poli_navigation.grid_map import (
    BLOCKED,
    cell_center_m,
    FREE,
    GridMap,
    MISSION1_START,
    RULE_EXAMPLE_BLOCKED,
)
from poli_navigation.mission1_logic import (
    ALIGN,
    APPROACH,
    CENTER_M,
    Command,
    DONE,
    DRIVE,
    DRIVE_BLOCK_SCANS,
    EXPLORE,
    FACE,
    GRAB_AREA,
    GRAB_CLOSE_LEAD_M,
    GRAB_PUSH_M,
    GRASP,
    GRASP_WAIT_S,
    GRIPPER_GRAB,
    GRIPPER_OPEN,
    GRIPPER_REACH_M,
    Mission1Logic,
    Observation,
    PUSH,
    ramp,
    RELEASE,
    RETURN,
    SCAN,
    SEARCH,
    SEE_TARGET_MIN_AREA,
    STUCK,
    TURN,
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

        # 실제 하드웨어처럼 느린 명령은 0.25 m/s 바퀴 속도로 커진다 (hw_model.py)
        linear, angular = actual_twist(command.linear, command.angular)
        self.yaw += angular * DT
        self.x += linear * math.cos(self.yaw) * DT
        self.y += linear * math.sin(self.yaw) * DT
        self.now += DT
        self.visited.add(point_to_cell(self.x, self.y))

        # 큐브는 들지 않고 집게 앞에 붙여서 바닥에 끌고 간다.
        if self.holding:
            self.cube = self.gripper_front()


def run(logic, robot, max_seconds=MATCH_SECONDS, until=None, rolling=False):
    states = []
    while robot.now < max_seconds:
        # 스캔은 SCAN 상태에서만 쓰므로 그때만 계산한다. (주행 중 판단 시험은 DRIVE에서도)
        obs = robot.observe(with_scan=logic.state == SCAN or (rolling and logic.state == DRIVE))
        if rolling and obs.scan is not None:
            obs.scan_time = robot.now
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
    # 먼 출구가 막혀 있어도 빈 이웃을 탐색한다. 벽 칸에는 진입하지 않는다.
    assert (9, 2) in robot.visited
    assert (9, 4) not in robot.visited
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


def test_fine_turn_continues_between_old_pulses_and_slows_near_goal():
    """작은 회전 오차에서도 정지 구간 없이 감속하고 회전한다."""
    logic = Mission1Logic()
    obs = Observation(x=0.0, y=0.0, yaw=0.0, now=0.0)
    fast = logic._turn_toward(math.pi / 2, obs)
    for now in (0.05, 0.10, 0.20, 0.30):
        obs.now = now
        slow = logic._turn_toward(math.radians(10), obs)
        assert 0 < slow < fast
        assert logic._turn_toward(-math.radians(10), obs) == -slow


def _approaching_logic(origin=(1.0, 1.0), axis=0.0):
    """출발 칸(origin) 가운데에서 axis 방향으로 대상에 다가가는 중인 상태."""
    logic = Mission1Logic()
    logic.state = APPROACH
    logic.state_started_at = 0.0
    logic._approach_axis = axis
    logic._approach_origin = origin
    logic._target_seen_at = 0.0
    return logic


def test_align_turns_to_corridor_axis_not_to_target():
    """대상이 축에서 10도 비껴 보여도 통로 축 방향으로 돈 뒤 접근한다."""
    logic = Mission1Logic()
    logic.state = ALIGN
    logic._approach_axis = 0.0
    logic._approach_origin = (1.0, 1.0)
    obs = Observation(x=1.0, y=1.0, yaw=-0.5, now=0.0, detected=True, x_offset=0.0, area=5000.0)
    assert logic._align(obs).angular > 0.0
    obs.yaw = 0.0
    command = logic._align(obs)
    assert logic.state == APPROACH
    assert command.linear > 0.0


def test_approach_steers_back_to_corridor_center_line():
    """통로 가운데 선에서 옆으로 벗어나면 카메라보다 가운데 선 복귀가 먼저다."""
    logic = _approaching_logic()
    # 왼쪽(+y)으로 3 cm 벗어났는데 큐브는 더 왼쪽에 보인다 -> 오른쪽으로 돌아온다.
    obs = Observation(x=1.2, y=1.03, yaw=0.0, now=0.0, detected=True, x_offset=-0.5, area=5000.0)
    assert logic._approach(obs).angular < 0.0
    # 가운데 선 안이면 큐브 쪽(왼쪽)으로 돌되 축에서 10도까지만.
    obs = Observation(x=1.2, y=1.0, yaw=0.0, now=0.0, detected=True, x_offset=-0.5, area=5000.0)
    assert logic._approach(obs).angular > 0.0
    obs.yaw = math.radians(10.0)
    assert abs(logic._approach(obs).angular) < 1e-9


def test_approach_stops_when_obstacle_ahead_of_gripper():
    true_grid = GridMap()
    true_grid.mark_blocked((7, 1))
    y = cell_center_m((8, 1))[1]
    logic = _approaching_logic(origin=cell_center_m((9, 1)), axis=math.pi)
    # 장애물 칸 (7,1) 면까지 집게 끝이 약 3 cm
    pose = (cell_center_m((7, 1))[0] + 0.2 + 0.17 + 0.03, y, math.pi)
    obs = Observation(x=pose[0], y=pose[1], yaw=math.pi, now=0.0, detected=True, area=5000.0,
                      scan=simulate_scan(true_grid, pose))
    assert logic._approach(obs) == Command()
    assert logic.state == SCAN
    assert (9, 1) in logic._approach_blocked


def test_approach_gives_up_when_target_lost():
    logic = _approaching_logic()
    obs = Observation(x=1.1, y=1.0, yaw=0.0, now=0.3, detected=False)
    assert logic._approach(obs).linear > 0.0  # 잠깐 놓치면 축을 따라 계속
    obs.now = 0.6
    logic._approach(obs)
    assert logic.state == SCAN


def test_near_cell_center_stops_instead_of_turning_towards_lateral_error():
    """좌회전 후 다음 칸에 앞 2 cm·옆 5 cm 남으면 우회전하지 않고 정지한다."""
    logic = Mission1Logic()
    origin = Observation(x=3.4, y=1.4, yaw=math.pi, now=0.0)
    logic.step(origin)
    logic._plan(origin)
    assert logic.target == cell_center_m((8, 4))
    obs = Observation(x=3.02, y=1.35, yaw=math.pi, now=1.0)
    assert logic._drive(obs) == Command()
    assert logic.state == SCAN


def test_final_forward_and_reverse_keep_planned_heading():
    """마지막 구간의 작은 옆 오차로 전진·후진 방향을 크게 꺾지 않는다."""
    for reverse, yaw in ((False, math.pi), (True, 0.0)):
        logic = Mission1Logic()
        logic.target = (3.0, 1.4)
        logic._drive_heading = math.pi
        logic.reverse = reverse
        obs = Observation(x=3.08, y=1.36, yaw=yaw, now=0.0)
        command = logic._drive(obs)
        assert abs(command.angular) < 1e-9
        assert command.linear < 0 if reverse else command.linear > 0


def test_lateral_offset_at_endpoint_stops_and_recenters_without_diagonal():
    """진행선 끝에서 옆으로 15 cm 벗어나 있으면 대각선으로 꺾지 않고 서서, 가운데로 직각 이동한다."""
    logic = Mission1Logic()
    logic.state = DRIVE
    logic.target = (3.0, 1.4)
    logic._drive_heading = math.pi
    obs = Observation(x=3.02, y=1.25, yaw=math.pi, now=0.0)
    logic._drive(obs)
    assert logic.state == SCAN
    logic._plan(obs)
    assert logic.target == cell_center_m((8, 4))
    assert abs(logic._drive_heading - math.atan2(0.15, -0.02)) < 1e-6


def test_drive_follows_planned_line_with_limited_correction():
    """목표 점이 옆으로 옮겨져도 진행선 기준 최대 15도까지만 튼다."""
    logic = Mission1Logic()
    logic.state = DRIVE
    logic.target = (3.4, 0.6)
    logic._drive_heading = math.pi / 2
    # 진행선에서 왼쪽(-x)으로 15 cm 벗어남, 목표까지 30 cm -> 오른쪽으로 최대 15도만
    obs = Observation(x=3.25, y=0.3, yaw=math.pi / 2, now=0.0)
    command = logic._drive(obs)
    assert command.linear > 0.0
    expected = mission1_logic.HEADING_GAIN * -mission1_logic.DRIVE_MAX_CORRECTION_RAD
    assert abs(command.angular - expected) < 1e-9


def test_grab_area_pushes_further_and_closes_near_end_of_push():
    """면적 기준에 닿으면 GRAB_PUSH_M만큼 곧게 더 들어가고, 끝 GRAB_CLOSE_LEAD_M에서 닫기 시작한다."""
    logic = _approaching_logic()
    obs = Observation(x=1.0, y=1.0, yaw=0.0, now=0.0, detected=True, area=GRAB_AREA)
    command = logic._approach(obs)
    assert logic.state == PUSH
    assert command.linear > 0.0 and command.angular == 0.0
    assert command.gripper is None
    # 카메라가 큐브를 놓쳐도 엔코더 거리로 계속 민다.
    obs = Observation(x=1.0 + GRAB_PUSH_M - GRAB_CLOSE_LEAD_M - 0.005, y=1.0, yaw=0.0, now=0.1)
    assert logic._push(obs).gripper is None
    obs = Observation(x=1.0 + GRAB_PUSH_M - GRAB_CLOSE_LEAD_M, y=1.0, yaw=0.0, now=0.2)
    command = logic._push(obs)
    assert command.linear > 0.0 and command.gripper == GRIPPER_GRAB
    obs = Observation(x=1.0 + GRAB_PUSH_M, y=1.0, yaw=0.0, now=0.35)
    assert logic._push(obs) == Command()
    assert logic.state == GRASP
    # 닫힘 대기는 grab을 보낸 시각(0.2)부터 센다.
    logic._grasp(Observation(x=1.04, y=1.0, yaw=0.0, now=0.5, gripper_state='closed'))
    logic._grasp(Observation(x=1.04, y=1.0, yaw=0.0, now=0.65, gripper_state='closed'))
    assert logic.phase == RETURN


def test_grab_after_push_when_close_lead_is_zero(monkeypatch):
    monkeypatch.setattr(mission1_logic, 'GRAB_CLOSE_LEAD_M', 0.0)
    logic = _approaching_logic()
    obs = Observation(x=1.0, y=1.0, yaw=0.0, now=0.0, detected=True, area=GRAB_AREA)
    assert logic._approach(obs).gripper is None
    obs = Observation(x=1.0 + GRAB_PUSH_M, y=1.0, yaw=0.0, now=0.4)
    assert logic._push(obs) == Command(gripper=GRIPPER_GRAB)
    assert logic.state == GRASP


def test_push_closes_after_timeout_when_wheels_slip():
    logic = _approaching_logic()
    logic._approach(Observation(x=1.0, y=1.0, yaw=0.0, now=0.0, detected=True, area=GRAB_AREA))
    logic._push(Observation(x=1.0, y=1.0, yaw=0.0, now=5.0))
    assert logic.state == GRASP


def _grasping_logic():
    logic = Mission1Logic()
    logic.state = GRASP
    logic.state_started_at = 0.0
    return logic


def test_grasp_ends_soon_after_gripper_reports_closed():
    """집게가 다 닫혔다고(/gripper/state closed) 알리면 1.5초를 다 기다리지 않는다."""
    logic = _grasping_logic()
    logic._grasp(Observation(x=1.0, y=1.0, yaw=0.0, now=0.7, gripper_state='closed'))
    assert logic.state == GRASP
    logic._grasp(Observation(x=1.0, y=1.0, yaw=0.0, now=0.8, gripper_state='closed'))
    assert logic.state == SCAN and logic.phase == RETURN


def test_grasp_ignores_old_closed_state_right_after_command():
    logic = _grasping_logic()
    for now in (0.05, 0.1, 0.2):
        logic._grasp(Observation(x=1.0, y=1.0, yaw=0.0, now=now, gripper_state='closed'))
    assert logic.state == GRASP


def test_grasp_waits_full_time_without_gripper_state():
    logic = _grasping_logic()
    logic._grasp(Observation(x=1.0, y=1.0, yaw=0.0, now=1.0, gripper_state='moving'))
    assert logic.state == GRASP
    logic._grasp(Observation(x=1.0, y=1.0, yaw=0.0, now=GRASP_WAIT_S))
    assert logic.state == SCAN


def test_release_ends_soon_after_gripper_reports_open():
    logic = Mission1Logic()
    logic.state = RELEASE
    logic.state_started_at = 0.0
    logic._release(Observation(x=1.0, y=1.0, yaw=0.0, now=0.7, gripper_state='open'))
    logic._release(Observation(x=1.0, y=1.0, yaw=0.0, now=0.8, gripper_state='open'))
    assert logic.state == DONE


def test_ramp_limits_change_and_passes_through_zero():
    assert ramp(0.45, 0.0, 0.5, 1.2, 0.05) == 0.025
    assert abs(ramp(0.0, 0.45, 0.5, 1.2, 0.05) - 0.39) < 1e-9
    # 전진 중 후진 명령은 먼저 0까지 줄인다 (부호가 한 번에 바뀌지 않음).
    assert ramp(-0.2, 0.02, 0.5, 1.2, 0.05) == 0.0


def test_motion_commands_ramp_up_and_down():
    """주행 시작은 천천히 올리고, 정지 명령도 한 번에 0으로 떨어뜨리지 않는다."""
    logic = Mission1Logic()
    logic.gripper_opened = True
    logic.state = DRIVE
    logic.state_started_at = 0.0
    logic.target = (1.0, 3.0)
    logic._drive_heading = math.pi / 2
    speeds = []
    for i in range(20):
        speeds.append(logic.step(Observation(x=1.0, y=1.0, yaw=math.pi / 2, now=i * DT)).linear)
    assert speeds[0] < 0.05
    limit = mission1_logic.LINEAR_ACCEL * DT + 1e-9
    assert all(b - a <= limit for a, b in zip(speeds, speeds[1:]))
    assert speeds[-1] > 0.3
    logic.state = STUCK
    logic.state_started_at = 20 * DT
    stopping = logic.step(Observation(x=1.0, y=1.0, yaw=math.pi / 2, now=20 * DT)).linear
    assert 0.0 < stopping < speeds[-1]


def test_ramp_restarts_from_zero_after_gap():
    """노드가 판단을 건너뛴 동안 정지 명령을 보냈으면 다시 0부터 가속한다."""
    logic = Mission1Logic()
    logic.gripper_opened = True
    logic.state = DRIVE
    logic.state_started_at = 0.0
    logic.target = (1.0, 3.0)
    logic._drive_heading = math.pi / 2
    for i in range(20):
        logic.step(Observation(x=1.0, y=1.0, yaw=math.pi / 2, now=i * DT))
    command = logic.step(Observation(x=1.0, y=1.0, yaw=math.pi / 2, now=5.0))
    assert command.linear < 0.05


def test_drive_arrives_slowly_after_constant_deceleration():
    logic = Mission1Logic()
    logic.target = (1.0, 1.4)
    logic._drive_heading = math.pi / 2
    far = logic._drive(Observation(x=1.0, y=1.0, yaw=math.pi / 2, now=0.0)).linear
    near = logic._drive(Observation(x=1.0, y=1.36, yaw=math.pi / 2, now=0.0)).linear
    assert far == mission1_logic.DRIVE_SPEED
    assert near < 0.15


def test_turn_slows_down_before_target_angle():
    logic = Mission1Logic()
    big = abs(logic._turn_toward(math.radians(90.0), None))
    small = abs(logic._turn_toward(math.radians(6.0), None))
    assert big == mission1_logic.TURN_SPEED
    assert mission1_logic.MIN_TURN_SPEED <= small < 0.8


def test_return_reverses_instead_of_turning_around():
    """잡은 뒤 다음 칸이 뒤쪽이면 돌지 않고 후진한다 (2026-10-31 수아)."""
    logic = Mission1Logic()
    logic.phase = RETURN
    logic.maze.observe((9, 2), math.pi / 2)
    logic.maze.observe((9, 3), math.pi / 2)
    x, y = cell_center_m((9, 3))
    obs = Observation(x=x, y=y, yaw=math.pi / 2, now=0.0)
    logic._plan(obs)
    assert logic.path[1] == (9, 2)
    assert logic.reverse
    assert logic._turn(obs).linear < 0.0


def test_last_cell_into_start_is_forward():
    """출발 격자로 들어가는 칸은 뒤쪽이어도 전진 (큐브를 격자 안에 넣기 위해)."""
    logic = Mission1Logic()
    logic.phase = RETURN
    logic.maze.observe((9, 2), math.pi / 2)
    x, y = cell_center_m((9, 2))
    logic._plan(Observation(x=x, y=y, yaw=math.pi / 2, now=0.0))
    assert logic.delivering
    assert not logic.reverse


def _rolling_logic(true_grid, y):
    """(9,1) -> (9,2)로 ROLL_SPEED로 달리는 중, (9,2) 가운데 y 앞에서 스캔을 받은 상태."""
    logic = Mission1Logic()
    logic.gripper_opened = True
    x, y0, yaw = START_POSE
    logic.step(Observation(x=x, y=y0, yaw=yaw, now=0.0))
    logic.maze.observe((9, 2), yaw)
    logic.state = DRIVE
    logic.state_started_at = 0.0
    logic.target = cell_center_m((9, 2))
    logic._drive_heading = math.pi / 2
    logic._has_scan_time = True
    logic._last_linear = mission1_logic.ROLL_SPEED
    logic._last_command_at = 1.0
    logic._speed_changed_at = 0.0
    pose = (x, logic.target[1] - y, yaw)
    obs = Observation(x=pose[0], y=pose[1], yaw=yaw, now=1.0,
                      scan=simulate_scan(true_grid, pose), scan_time=1.0)
    return logic, obs


def test_rolling_decision_keeps_driving_straight_without_stopping(monkeypatch):
    monkeypatch.setattr(mission1_logic, 'ROLLING_DECISION', True)
    logic, obs = _rolling_logic(GridMap(), 0.06)
    assert logic.rolling_scan_ok(obs)
    command = logic.step(obs)
    assert logic.state == DRIVE
    assert logic.target == cell_center_m((9, 3))
    assert command.linear > 0.0


def test_rolling_decision_with_turn_stops_and_turns_without_scan_wait(monkeypatch):
    monkeypatch.setattr(mission1_logic, 'ROLLING_DECISION', True)
    true_grid = GridMap()
    true_grid.mark_blocked((9, 3))
    logic, obs = _rolling_logic(true_grid, 0.06)
    logic.step(obs)
    assert logic.state == DRIVE
    assert logic.target == cell_center_m((9, 2))
    assert logic.grid.get((9, 3)) == BLOCKED
    # 가운데에 도착하면 SCAN에서 기다리지 않고 바로 회전한다.
    x, y = cell_center_m((9, 2))
    logic.step(Observation(x=x, y=y - 0.02, yaw=math.pi / 2, now=1.05))
    assert logic.state == TURN


def test_rolling_scan_rejected_right_after_speed_change_or_far_from_center(monkeypatch):
    monkeypatch.setattr(mission1_logic, 'ROLLING_DECISION', True)
    logic, obs = _rolling_logic(GridMap(), 0.06)
    logic._speed_changed_at = 0.95
    assert not logic.rolling_scan_ok(obs)
    logic, obs = _rolling_logic(GridMap(), 0.15)
    assert not logic.rolling_scan_ok(obs)
    logic, obs = _rolling_logic(GridMap(), 0.06)
    logic._last_linear = 0.45
    assert not logic.rolling_scan_ok(obs)


def test_scan_pose_goes_back_to_capture_time_when_moving():
    logic = Mission1Logic()
    logic._last_linear = 0.2
    obs = Observation(x=1.0, y=1.0, yaw=math.pi / 2, now=1.15, scan_time=1.0)
    x, y, _ = logic.scan_pose(obs)
    assert abs(x - 1.0) < 1e-9 and abs(y - (1.0 - 0.2 * 0.1)) < 1e-9


def test_drive_slows_to_roll_speed_before_cell_center(monkeypatch):
    monkeypatch.setattr(mission1_logic, 'ROLLING_DECISION', True)
    logic = Mission1Logic()
    logic._has_scan_time = True
    logic.target = (1.0, 1.4)
    logic._drive_heading = math.pi / 2
    near = logic._drive(Observation(x=1.0, y=1.4 - 0.12, yaw=math.pi / 2, now=0.0)).linear
    assert abs(near - mission1_logic.ROLL_SPEED) < 1e-9


def test_turn_stops_early_by_current_turn_speed():
    """빠르게 돌고 있으면 더 돌 각도만큼 일찍 회전을 끝낸다."""
    for angular, expected in ((1.0, DRIVE), (0.0, TURN)):
        logic = Mission1Logic()
        logic.state = TURN
        logic.target = (2.0, 1.0)
        logic._last_angular = angular
        logic._turn(Observation(x=1.0, y=1.0, yaw=-math.radians(10.0), now=0.0))
        assert logic.state == expected


def test_full_run_with_rolling_decisions(monkeypatch):
    monkeypatch.setattr(mission1_logic, 'ROLLING_DECISION', True)
    true_grid = example_grid()
    logic = Mission1Logic()
    robot = SimRobot(true_grid)
    run(logic, robot, until=DONE, rolling=True)
    assert logic.state == DONE
    assert distance_to_start(robot.cube) < 0.1
    blocked = [cell for cell in robot.visited if not true_grid.is_passable(cell)]
    assert blocked == []


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


def test_maze_explores_free_neighbor_even_when_center_is_enclosed():
    logic = Mission1Logic()
    for cell in logic.grid.neighbors((5, 5)):
        logic.grid.mark_blocked(cell)
    x, y, yaw = START_POSE
    obs = Observation(x=x, y=y, yaw=yaw, now=0.0)
    logic.step(obs)
    logic._plan(obs)
    assert logic.path == [(9, 1), (9, 2)]
    assert logic._turn(obs).linear > 0.0


def test_maze_turns_left_when_front_is_blocked():
    logic = Mission1Logic()
    logic.grid.mark_blocked((9, 2))
    x, y, yaw = START_POSE
    obs = Observation(x=x, y=y, yaw=yaw, now=0.0)
    logic.step(obs)
    logic._plan(obs)
    assert logic.path == [(9, 1), (8, 1)]
    command = logic._turn(obs)
    assert command.linear == 0.0
    assert command.angular > 0.0


def test_maze_reverses_without_turning_in_dead_end():
    logic = Mission1Logic()
    for cell in ((9, 3), (8, 2)):
        logic.grid.mark_blocked(cell)
    x, y = cell_center_m((9, 2))
    obs = Observation(x=x, y=y, yaw=math.pi / 2, now=0.0)
    logic.step(obs)
    logic._plan(obs)
    assert logic.path == [(9, 2), (9, 1)]
    command = logic._turn(obs)
    assert command.linear < 0.0
    assert abs(command.angular) < 1e-9


def test_maze_stops_when_all_neighbors_are_blocked():
    logic = Mission1Logic()
    logic.grid.mark_blocked((9, 2))
    logic.grid.mark_blocked((8, 1))
    x, y, yaw = START_POSE
    obs = Observation(x=x, y=y, yaw=yaw, now=0.0)
    logic.step(obs)
    assert logic._plan(obs) == Command()
    assert logic.state == STUCK
    assert logic.stop_reason is not None


def test_maze_counts_actual_cell_entry_once_not_replans():
    logic = Mission1Logic()
    x, y, yaw = START_POSE
    obs = Observation(x=x, y=y, yaw=yaw, now=0.0)
    for _ in range(10):
        logic.step(obs)
        logic._plan(obs)
    assert logic.maze.state.visits == {(9, 1): 1}
    x, y = cell_center_m((9, 2))
    logic.step(Observation(x=x, y=y, yaw=yaw, now=0.1))
    assert logic.maze.state.visits[(9, 2)] == 1


def test_return_uses_only_visited_cells():
    logic = Mission1Logic()
    logic.phase = RETURN
    x, y = cell_center_m((9, 3))
    obs = Observation(x=x, y=y, yaw=0.0, now=0.0)
    logic.step(obs)
    logic._plan(obs)
    assert logic.state == STUCK  # (9,2)를 방문하지 않아 연결된 복귀 경로가 없다.


def test_new_obstacle_in_target_cell_stops_drive(monkeypatch):
    monkeypatch.setattr(mission1_logic, 'DRIVE_BLOCK_CHECK', True)
    logic = Mission1Logic()
    x, y, yaw = START_POSE
    obs = Observation(x=x, y=y, yaw=yaw, now=0.0)
    logic.step(obs)
    logic._plan(obs)
    true_grid = GridMap()
    true_grid.mark_blocked((9, 2))
    # 출발 순간 흔들림으로 한두 번 막혀 보이는 것은 무시하고, 연속으로 막혀 보이면 멈춘다.
    for _ in range(DRIVE_BLOCK_SCANS - 1):
        obs.scan = simulate_scan(true_grid, (x, y, yaw))
        logic._drive(obs)
        assert logic.state != SCAN
    obs.scan = simulate_scan(true_grid, (x, y, yaw))
    assert logic._drive(obs) == Command()
    assert logic.state == SCAN
    # 이동 중 관측은 정지 판단에만 쓰고, 정지 후 스캔까지 지도에 남기지 않는다.
    assert logic.grid.get((9, 2)) != BLOCKED


def test_target_seen_while_driving_does_not_turn_toward_it():
    """주행 중 대상이 보여도 대각선으로 꺾지 않고 칸 가운데까지 간다 (2026-10-31 실기)."""
    logic = Mission1Logic()
    x, y, yaw = START_POSE
    obs = Observation(x=x, y=y, yaw=yaw, now=0.0)
    logic.step(obs)
    logic._plan(obs)
    logic.state = DRIVE
    obs = Observation(x=x, y=y + 0.1, yaw=yaw, now=1.0, detected=True, x_offset=-0.6,
                      area=SEE_TARGET_MIN_AREA + 1.0)
    command = logic._drive(obs)
    assert logic.state == DRIVE
    assert command.linear > 0.0 and abs(command.angular) < 0.1


def test_target_on_axis_at_cell_center_goes_to_align():
    logic = Mission1Logic()
    logic.state = SCAN
    logic.state_started_at = -1.0
    x, y, yaw = START_POSE
    obs = Observation(x=x, y=y, yaw=yaw, now=0.0, scan=simulate_scan(GridMap(), START_POSE),
                      detected=True, x_offset=0.1, area=9000.0)
    logic.step(obs)
    assert logic.state == ALIGN
    assert logic._approach_axis == math.pi / 2


def test_diagonal_or_blocked_target_is_not_approached():
    x, y, yaw = START_POSE
    # 대각선 (축에서 약 26도)
    logic = Mission1Logic()
    obs = Observation(x=x, y=y, yaw=yaw, now=0.0, detected=True, x_offset=0.8, area=9000.0)
    assert not logic._target_approach(obs)
    # 축 방향이지만 사이 칸 (9,2)가 막힘 (대상은 두 칸 앞, 약 80 cm)
    logic = Mission1Logic()
    logic.grid.mark_blocked((9, 2))
    obs = Observation(x=x, y=y, yaw=yaw, now=0.0, detected=True, x_offset=0.0, area=3500.0)
    assert not logic._target_approach(obs)
    # 칸 가운데가 아님
    logic = Mission1Logic()
    obs = Observation(x=x, y=y + 0.15, yaw=yaw, now=0.0, detected=True, area=9000.0)
    assert not logic._target_approach(obs)


def test_small_far_target_is_ignored_while_exploring():
    logic = Mission1Logic()
    x, y, yaw = START_POSE
    obs = Observation(x=x, y=y, yaw=yaw, now=0.0, detected=True, area=SEE_TARGET_MIN_AREA - 1.0)
    logic.step(obs)
    assert logic.state != ALIGN
