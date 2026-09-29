import math
from types import SimpleNamespace

from poli_navigation.grid_map import (
    BLOCKED,
    cell_center_m,
    CENTER,
    FREE,
    GridMap,
    MISSION1_START,
    RULE_EXAMPLE_BLOCKED,
    UNKNOWN,
)
from poli_navigation.scan_to_grid import (
    ARENA_SIZE_M,
    odom_to_arena,
    point_to_cell,
    scan_to_cells,
    START_POSE,
    update_grid_from_scan,
)


NUM_RAYS = 360
RAY_STEP_M = 0.005


def make_true_grid():
    grid = GridMap()
    for cell in RULE_EXAMPLE_BLOCKED:
        grid.mark_blocked(cell)
    return grid


def simulate_scan(true_grid, pose):
    """정답 지도에서 광선을 쏴서 가짜 LaserScan을 만든다 (테스트 전용)."""
    x0, y0, yaw = pose
    increment = 2.0 * math.pi / NUM_RAYS
    angle_min = -math.pi + increment / 2.0
    ranges = []

    for i in range(NUM_RAYS):
        angle = yaw + angle_min + i * increment
        dx, dy = math.cos(angle), math.sin(angle)
        distance = 0.0

        while True:
            distance += RAY_STEP_M
            x, y = x0 + dx * distance, y0 + dy * distance
            cell = point_to_cell(x, y)
            if cell is None or true_grid.get(cell) == BLOCKED:
                break

        ranges.append(distance)

    return SimpleNamespace(
        angle_min=angle_min,
        angle_increment=increment,
        range_min=0.05,
        range_max=12.0,
        ranges=ranges,
    )


# ---------------------------------------------------------------
# 좌표 변환
# ---------------------------------------------------------------

def test_point_to_cell():
    assert point_to_cell(0.2, 0.2) == (1, 1)
    assert point_to_cell(3.4, 0.2) == (9, 1)
    assert point_to_cell(1.8, 1.8) == (5, 5)
    assert point_to_cell(-0.1, 1.0) is None
    assert point_to_cell(ARENA_SIZE_M, 1.0) is None


def test_odom_origin_is_start_pose():
    assert odom_to_arena(0.0, 0.0, 0.0) == START_POSE


def test_odom_forward_moves_up():
    # 출발 시 정면(odom +x)은 경기장 +y 방향
    x, y, _ = odom_to_arena(0.4, 0.0, 0.0)
    assert point_to_cell(x, y) == (9, 2)


def test_odom_left_moves_toward_center():
    # 출발 시 왼쪽(odom +y)은 경기장 -x 방향 (중앙 쪽)
    x, y, _ = odom_to_arena(0.0, 0.4, 0.0)
    assert point_to_cell(x, y) == (8, 1)


# ---------------------------------------------------------------
# 스캔 -> 격자
# ---------------------------------------------------------------

def test_scan_from_start_finds_visible_obstacles():
    true_grid = make_true_grid()
    scan = simulate_scan(true_grid, START_POSE)

    blocked, free = scan_to_cells(scan, START_POSE)

    # 출발 위치에서 바로 보이는 장애물
    assert (7, 1) in blocked
    assert (8, 5) in blocked

    # 잘못 막힘으로 판단한 격자가 없어야 한다.
    assert all(true_grid.get(cell) == BLOCKED for cell in blocked)

    # 빈 격자로 판단한 곳은 실제로 비어 있어야 한다.
    assert all(true_grid.get(cell) != BLOCKED for cell in free)
    assert (9, 2) in free


def test_empty_arena_has_no_obstacles():
    true_grid = GridMap()
    scan = simulate_scan(true_grid, START_POSE)

    blocked, free = scan_to_cells(scan, START_POSE)

    assert blocked == set()
    assert (5, 5) in free


def test_invalid_ranges_are_ignored():
    scan = SimpleNamespace(
        angle_min=0.0,
        angle_increment=0.01,
        range_min=0.05,
        range_max=12.0,
        ranges=[float('inf'), float('nan'), 0.0, 20.0],
    )

    assert scan_to_cells(scan, START_POSE) == (set(), set())


def test_update_grid_uses_symmetry():
    true_grid = make_true_grid()
    scan = simulate_scan(true_grid, START_POSE)
    grid = GridMap()

    update_grid_from_scan(grid, scan, START_POSE)

    # (7, 1)을 봤으면 대칭인 (3, 9)도 막힘
    assert grid.get((7, 1)) == BLOCKED
    assert grid.get((3, 9)) == BLOCKED


def test_update_grid_keeps_blocked_cells():
    true_grid = GridMap()
    scan = simulate_scan(true_grid, START_POSE)
    grid = GridMap()
    grid.mark_blocked((9, 3))

    update_grid_from_scan(grid, scan, START_POSE)

    assert grid.get((9, 3)) == BLOCKED
    assert grid.get((9, 2)) == FREE


def test_hit_near_corner_stays_in_obstacle_cell():
    from poli_navigation.scan_to_grid import hit_to_cell

    # (8, 5) 오른쪽 면(x = 3.2)의 아래 모서리 근처에 비스듬히(왼쪽 아래로) 맞음
    assert hit_to_cell(3.198, 1.62, -0.7, -0.7) == (8, 5)
    # 면 바로 바깥(빈 격자 쪽)에 찍힌 점도 장애물 격자로
    assert hit_to_cell(3.203, 1.8, -1.0, 0.0) == (8, 5)


def explore_to_center(true_grid, max_moves=40):
    """
    스캔 -> 경로 계산 -> 한 칸 이동을 반복해서 중앙까지 간다.

    반환: (지나간 격자 목록, 로봇이 만든 지도)
    """
    grid = GridMap()
    current = MISSION1_START
    trail = [current]

    for _ in range(max_moves):
        pose = (*cell_center_m(current), math.pi / 2)
        update_grid_from_scan(grid, simulate_scan(true_grid, pose), pose)

        wrong = [
            cell for cell, state in grid.cells.items()
            if state == BLOCKED and true_grid.get(cell) != BLOCKED
        ]
        assert not wrong, f'wrongly blocked at {current}: {wrong}'

        if current == CENTER:
            break

        path = grid.find_path(current, CENTER)
        assert path is not None, f'no path from {current}'

        current = path[1]
        assert true_grid.get(current) != BLOCKED, f'drove into {current}'
        trail.append(current)

    return trail, grid


def test_explore_rule_example_reaches_center():
    true_grid = make_true_grid()
    trail, _ = explore_to_center(true_grid)

    assert trail[-1] == CENTER
    # 지도를 모두 알 때의 최단 경로(16칸)와 같아야 한다.
    assert len(trail) - 1 == 16


def test_explore_empty_arena_goes_straight():
    trail, _ = explore_to_center(GridMap())

    assert trail[-1] == CENTER
    assert len(trail) - 1 == 8


def test_scan_then_plan_avoids_known_obstacles():
    true_grid = make_true_grid()
    scan = simulate_scan(true_grid, START_POSE)
    grid = GridMap()

    update_grid_from_scan(grid, scan, START_POSE)
    path = grid.find_path(MISSION1_START, (5, 5))

    assert path is not None
    assert all(grid.get(cell) != BLOCKED for cell in path)
    assert any(state == UNKNOWN for state in grid.cells.values())
