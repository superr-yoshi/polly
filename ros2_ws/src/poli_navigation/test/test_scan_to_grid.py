import math
from types import SimpleNamespace

from poli_navigation import scan_to_grid
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
    corner_offset,
    neighbor_states,
    odom_to_arena,
    point_to_cell,
    scan_to_cells,
    START_POSE,
    update_grid_from_scan,
    wall_offset,
    wall_yaw_offset,
)
from poli_navigation.sim_world import simulate_scan
import pytest


@pytest.fixture
def long_range(monkeypatch):
    """먼 점까지 쓰던 예전 동작으로 시험한다 (MAX_USE_RANGE_M 제한 없음)."""
    monkeypatch.setattr(scan_to_grid, 'MAX_USE_RANGE_M', 100.0)


def make_true_grid():
    grid = GridMap()
    for cell in RULE_EXAMPLE_BLOCKED:
        grid.mark_blocked(cell)
    return grid


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

def test_scan_from_start_finds_visible_obstacles(long_range):
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


def test_empty_arena_has_no_obstacles(long_range):
    true_grid = GridMap()
    scan = simulate_scan(true_grid, START_POSE)

    blocked, free = scan_to_cells(scan, START_POSE)

    assert blocked == set()
    assert (5, 5) in free


def test_far_points_are_not_used():
    # 레이저가 낮은 장애물 위로 넘어가는 먼 곳은 막힘도 빈칸도 아닌 '모름'으로 둔다.
    true_grid = make_true_grid()
    scan = simulate_scan(true_grid, START_POSE)

    blocked, free = scan_to_cells(scan, START_POSE)

    assert (8, 5) not in blocked
    assert (9, 2) in free
    assert (9, 5) not in free


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


def test_update_grid_clears_cell_when_rays_pass_through(long_range):
    # 상대 로봇을 (9, 3)에서 장애물로 봤는데, 지금은 떠나서 광선이 지나간다.
    true_grid = GridMap()
    scan = simulate_scan(true_grid, START_POSE)
    grid = GridMap()
    grid.mark_blocked((9, 3))

    update_grid_from_scan(grid, scan, START_POSE)

    assert grid.get((9, 3)) == FREE
    # 점대칭으로 같이 막혔던 격자도 지운다.
    assert grid.get((1, 7)) == FREE


def test_update_grid_keeps_obstacles_with_small_pose_error():
    # 위치를 5cm, 방향을 3도 틀리게 알고 있어도 진짜 장애물은 지우지 않는다.
    true_grid = make_true_grid()
    error_m = 0.05
    error_rad = math.radians(3.0)

    for cell in [(9, 2), (8, 4), (6, 2), (4, 8), (6, 6)]:
        x, y = cell_center_m(cell)
        scan = simulate_scan(true_grid, (x, y, math.pi / 2))
        for dx, dy, dyaw in [
            (error_m, error_m, error_rad),
            (-error_m, error_m, -error_rad),
            (error_m, -error_m, -error_rad),
            (-error_m, -error_m, error_rad),
        ]:
            grid = make_true_grid()
            update_grid_from_scan(
                grid, scan, (x + dx, y + dy, math.pi / 2 + dyaw)
            )

            lost = [c for c in RULE_EXAMPLE_BLOCKED if grid.get(c) != BLOCKED]
            assert not lost, f'lost {lost} at {cell} error {dx, dy, dyaw}'


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
    found = set()

    for _ in range(max_moves):
        pose = (*cell_center_m(current), math.pi / 2)
        update_grid_from_scan(grid, simulate_scan(true_grid, pose), pose)

        wrong = [
            cell for cell, state in grid.cells.items()
            if state == BLOCKED and true_grid.get(cell) != BLOCKED
        ]
        assert not wrong, f'wrongly blocked at {current}: {wrong}'

        # 한 번 찾은 진짜 장애물은 다시 지워지면 안 된다.
        lost = [cell for cell in found if grid.get(cell) != BLOCKED]
        assert not lost, f'lost obstacles at {current}: {lost}'
        found |= {
            cell for cell, state in grid.cells.items() if state == BLOCKED
        }

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


def test_neighbor_states_uses_direction_cones():
    # 상하좌우 이웃 칸은 그 방향 ±15도 레이저로만 판단한다 (2026-10-31).
    true_grid = GridMap()
    true_grid.mark_blocked((9, 2))
    scan = simulate_scan(true_grid, START_POSE)

    states = neighbor_states(scan, START_POSE)

    assert states.get((9, 2)) == BLOCKED
    assert states.get((8, 1)) != BLOCKED


def test_wall_offset_pulls_back_to_cell_center():
    # 출발 칸에서 실제로는 5 cm 앞(+y)에 있는데 가운데라고 믿으면, 앞 벽으로 +y 보정이 나온다.
    true_grid = GridMap()
    true_grid.mark_blocked((9, 2))
    x, y, yaw = START_POSE
    scan = simulate_scan(true_grid, (x, y + 0.05, yaw))

    dx, dy = wall_offset(scan, START_POSE)

    assert abs(dy - 0.05) < 0.02


def test_wall_yaw_offset_finds_heading_error():
    """실제로 4도 더 돌아 있는데 덜 돌았다고 믿으면, 여러 벽이 같은 기울기라 +2도(절반) 보정."""
    true_grid = GridMap()
    true_grid.mark_blocked((9, 2))
    x, y, yaw = START_POSE
    for error, expected in ((4.0, 2.0), (-4.0, -2.0), (-10.0, -3.0)):
        scan = simulate_scan(true_grid, (x, y, yaw + math.radians(error)))
        assert abs(math.degrees(wall_yaw_offset(scan, START_POSE)) - expected) < 0.6


def test_wall_yaw_offset_ignores_single_or_disagreeing_walls():
    """벽 면이 하나뿐이면 (상자 하나가 비뚤어져 있을 수 있어) 고치지 않는다."""
    x, y = cell_center_m((5, 4))
    pose = (x, y, math.pi / 2)
    one_wall = GridMap()
    one_wall.mark_blocked((5, 5))
    scan = simulate_scan(one_wall, (x, y, math.pi / 2 + math.radians(4.0)))
    assert wall_yaw_offset(scan, pose) == 0.0


def test_wall_yaw_offset_is_zero_when_heading_is_right_or_no_wall():
    true_grid = GridMap()
    true_grid.mark_blocked((9, 2))
    scan = simulate_scan(true_grid, START_POSE)
    assert abs(wall_yaw_offset(scan, START_POSE)) < math.radians(0.5)
    # 가운데 칸, 주변이 모두 비어 있으면 가까운 벽이 없다.
    center = (*cell_center_m((5, 5)), math.pi / 2)
    assert wall_yaw_offset(simulate_scan(GridMap(), center), center) == 0.0


def test_neighbor_states_measured_from_cell_center_when_off_center():
    """가운데 8 cm 앞(동쪽)에서 서쪽을 보고 찍어도, 뒤(동쪽) 칸은 외벽까지 0.6 m라 빈칸."""
    true_grid = GridMap()
    true_grid.mark_blocked((7, 3))
    center = cell_center_m((8, 3))
    pose = (center[0] + 0.08, center[1], math.pi)
    scan = simulate_scan(true_grid, pose)
    states = neighbor_states(scan, pose, center)
    assert states.get((7, 3)) == BLOCKED
    assert states.get((9, 3)) == FREE
    assert neighbor_states(scan, pose).get((9, 3)) is None  # 로봇 기준이면 판단 못 함


def test_corner_offset_finds_along_error_in_corridor():
    """앞뒤 벽 없는 통로에서 실제로 6 cm 더 갔으면, 옆 열린 칸 입구 모서리로 +6 cm 보정."""
    true_grid = GridMap()
    for cell in ((8, 2), (8, 4)):
        true_grid.mark_blocked(cell)
    x, y = cell_center_m((9, 3))
    scan = simulate_scan(true_grid, (x, y + 0.06, math.pi / 2))
    corner_x, corner_y = corner_offset(scan, (x, y, math.pi / 2))
    assert abs(corner_y - 0.06) < 0.015
    # (8,4) 상자 아래면이 x = 가운데 -0.2에서 끝나는 모서리로 x도 잰다 (실제 오차 0)
    assert abs(corner_x) < 0.015


def test_corner_offset_none_along_continuous_walls():
    true_grid = GridMap()
    for cell in ((8, 2), (8, 3), (8, 4)):
        true_grid.mark_blocked(cell)
    x, y = cell_center_m((9, 3))
    scan = simulate_scan(true_grid, (x, y, math.pi / 2))
    assert corner_offset(scan, (x, y, math.pi / 2))[1] is None


def test_corridor_between_boxes_not_blocked_with_small_pose_error():
    """위아래가 상자인 통로 칸 (8,3)이 위치 3 cm·3도 오차로 막힘이 되면 안 된다 (2026-10-31 실기)."""
    import itertools
    true_grid = GridMap(use_point_symmetry=False)
    for cell in ((9, 3), (7, 3), (7, 1), (6, 2), (6, 3)):
        true_grid.mark_blocked(cell)
    for cell, yaw in (((9, 1), math.pi / 2), ((9, 2), math.pi / 2), ((7, 2), math.pi)):
        x, y = cell_center_m(cell)
        for dx, dy, dyaw in itertools.product((-0.03, 0.0, 0.03), (-0.03, 0.0, 0.03), (-3, 0, 3)):
            scan = simulate_scan(true_grid, (x + dx, y + dy, yaw + math.radians(dyaw)))
            blocked, _ = scan_to_cells(scan, (x, y, yaw))
            assert (8, 3) not in blocked, (cell, dx, dy, dyaw)
            assert (9, 3) in blocked or (7, 3) in blocked
