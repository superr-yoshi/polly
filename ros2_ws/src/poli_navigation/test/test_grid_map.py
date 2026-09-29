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


def make_rule_example_grid():
    grid = GridMap()
    for cell in RULE_EXAMPLE_BLOCKED:
        grid.mark_blocked(cell)
    return grid


def is_connected_path(grid, path):
    """경로의 모든 이동이 상하좌우 한 칸이고, 막힌 칸을 지나지 않는지."""
    for current, following in zip(path, path[1:]):
        dx = abs(current[0] - following[0])
        dy = abs(current[1] - following[1])
        if dx + dy != 1:
            return False

    return all(grid.get(cell) != BLOCKED for cell in path)


# ---------------------------------------------------------------
# 점대칭
# ---------------------------------------------------------------

def test_mirror():
    grid = GridMap()
    assert grid.mirror((9, 1)) == (1, 9)
    assert grid.mirror((3, 1)) == (7, 9)
    assert grid.mirror(CENTER) == CENTER


def test_mark_blocked_also_marks_mirror():
    grid = GridMap()
    grid.mark_blocked((7, 2))
    assert grid.get((7, 2)) == BLOCKED
    assert grid.get((3, 8)) == BLOCKED


def test_mark_free_also_marks_mirror():
    grid = GridMap()
    grid.mark_free((8, 1))
    assert grid.get((2, 9)) == FREE


def test_symmetry_can_be_disabled():
    grid = GridMap(use_point_symmetry=False)
    grid.mark_blocked((7, 2))
    assert grid.get((3, 8)) == UNKNOWN


def test_mark_outside_grid_is_ignored():
    grid = GridMap()
    grid.mark_blocked((0, 5))
    grid.mark_blocked((10, 5))
    assert all(state == UNKNOWN for state in grid.cells.values())


# ---------------------------------------------------------------
# 경로 계산
# ---------------------------------------------------------------

def test_path_on_empty_grid_is_shortest():
    grid = GridMap()
    path = grid.find_path(MISSION1_START, CENTER)

    # (9,1) -> (5,5): 가로 4칸 + 세로 4칸
    assert len(path) - 1 == 8
    assert path[0] == MISSION1_START
    assert path[-1] == CENTER
    assert is_connected_path(grid, path)


def test_path_on_rule_example():
    grid = make_rule_example_grid()
    path = grid.find_path(MISSION1_START, CENTER)

    assert len(path) - 1 == 16
    assert path[0] == MISSION1_START
    assert path[-1] == CENTER
    assert is_connected_path(grid, path)


def test_path_start_equals_goal():
    grid = GridMap()
    assert grid.find_path(CENTER, CENTER) == [CENTER]


def test_no_path_when_goal_surrounded():
    grid = GridMap()
    for cell in [(5, 6), (6, 5), (5, 4), (4, 5)]:
        grid.mark_blocked(cell)

    assert grid.find_path(MISSION1_START, CENTER) is None


def test_no_path_when_goal_blocked():
    grid = GridMap()
    grid.mark_blocked(CENTER)
    assert grid.find_path(MISSION1_START, CENTER) is None


def test_replan_after_new_obstacle():
    grid = GridMap()
    first_path = grid.find_path(MISSION1_START, CENTER)

    # 이동 중 첫 경로의 한 칸이 막힌 것을 발견했다고 가정
    discovered = first_path[3]
    grid.mark_blocked(discovered)
    new_path = grid.find_path(MISSION1_START, CENTER)

    assert discovered not in new_path
    assert is_connected_path(grid, new_path)


# ---------------------------------------------------------------
# 좌표 변환
# ---------------------------------------------------------------

def test_cell_center_m():
    assert cell_center_m((1, 1)) == (0.2, 0.2)

    x, y = cell_center_m(CENTER)
    assert abs(x - 1.8) < 1e-9
    assert abs(y - 1.8) < 1e-9
