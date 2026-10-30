"""알고리즘이 시뮬레이터와 같은 결과를 내는지 확인 (ROS 없이 pytest 로 실행)."""

import math

from poli_maze import maze_logic as ml
from poli_maze.maze_mapper import GridFrame, WallMapper

# 처음 손으로 그린 7x7 경기장 (세모 = 벽). 시뮬레이터와 결과를 맞춰 보는 기준.
N7, START7, GOAL7 = 7, (6, 6), (3, 3)
DRAWN_WALLS = {(0, 1), (0, 4), (1, 0), (2, 1), (2, 3), (2, 4), (2, 5),
               (3, 1), (3, 5), (4, 5), (5, 5), (6, 5)}


def explore(walls, n=N7, start=START7, goal=GOAL7):
    """라이다가 사방 한 칸을 완벽하게 본다고 치고 목표까지 간다."""
    maze = ml.MazeState(n=n, start=start, goal=goal)
    for _ in range(500):
        if maze.at_goal():
            return maze
        for d in range(4):
            cell = ml.step(maze.cell, d)
            if cell in walls:
                maze.add_wall(cell)
        d = maze.choose_next()
        assert d is not None
        maze.move(d)
    raise AssertionError('목표에 못 감')


def test_drawn_map_reaches_goal_in_12_moves():
    maze = explore(DRAWN_WALLS)
    assert len(maze.path) - 1 == 12
    assert maze.path[:3] == [(6, 6), (5, 6), (4, 6)]


def test_return_map_and_path():
    maze = explore(DRAWN_WALLS)
    rmap = ml.build_return_map(maze.visited_cells(), maze.walls, start=START7, n=N7)
    assert rmap[START7] == 1
    assert rmap[GOAL7] == 13
    dirs = ml.plan_return(maze.cell, maze.facing, rmap, start=START7)
    assert len(dirs) == 12
    # 목표에서 오른쪽을 보고 있으므로 첫 칸(왼쪽)은 후진
    assert maze.facing == ml.RIGHT
    assert ml.how_to_move(maze.facing, dirs[0]) == ml.REVERSE
    assert ml.group_segments(dirs) == [(ml.LEFT, 1), (ml.UP, 2), (ml.RIGHT, 4), (ml.DOWN, 5)]


def test_return_skips_dead_end():
    # 막다른 길에 들어갔다 나와도 복귀는 그 길을 건너뛴다
    visited = {(6, 6), (5, 6), (4, 6), (4, 5), (4, 4), (3, 6), (2, 6)}
    rmap = ml.build_return_map(visited, set(), start=START7, n=N7)
    dirs = ml.plan_return((2, 6), ml.UP, rmap, start=START7)
    assert dirs == [ml.DOWN] * 4


def test_tie_prefers_straight_then_reverse():
    assert ml.choice_order(ml.UP) == [ml.UP, ml.DOWN, ml.RIGHT, ml.LEFT]


def test_wall_never_on_visited_cell():
    maze = ml.MazeState()
    assert not maze.add_wall(ml.START)
    assert not maze.add_wall(ml.GOAL)
    assert maze.add_wall((8, 7))


def test_9x9_settings():
    # 대회 경기장: 9x9, 목표 정가운데(사람 기준 5,5), 시작 오른쪽 아래 구석
    assert ml.N == 9 and ml.GOAL == (4, 4) and ml.START == (8, 8)
    maze = ml.MazeState()
    assert maze.base_value(ml.GOAL) == 1
    assert maze.base_value(ml.START) == 9


def test_9x9_open_field_goes_straight_to_goal():
    # 벽이 없으면 최단 8칸, 직진 우선이라 위로 4칸 -> 왼쪽 4칸
    maze = explore(set(), n=ml.N, start=ml.START, goal=ml.GOAL)
    assert len(maze.path) - 1 == 8
    assert maze.path[4] == (4, 8)


def test_9x9_wall_column_and_return():
    # 시작 왼쪽 세로줄이 막혀 있으면 오른쪽 끝 줄로 올라가서 돌아 들어간다
    walls = {(r, 7) for r in range(3, 9)} | {(3, 4), (3, 5), (3, 6)}
    maze = explore(walls, n=ml.N, start=ml.START, goal=ml.GOAL)
    assert maze.at_goal()
    rmap = ml.build_return_map(maze.visited_cells(), maze.walls)
    dirs = ml.plan_return(maze.cell, maze.facing, rmap)
    assert ml.step(maze.cell, dirs[0]) in rmap
    assert len(dirs) == rmap[ml.GOAL] - 1


def test_grid_frame_round_trip():
    f = GridFrame()
    assert f.cell_center(ml.START) == (0.0, 0.0)
    x, y = f.cell_center((7, 8))   # 위로 한 칸 = +x
    assert math.isclose(x, 0.4) and math.isclose(y, 0.0, abs_tol=1e-9)
    x, y = f.cell_center((8, 7))   # 왼쪽 한 칸 = +y
    assert math.isclose(x, 0.0, abs_tol=1e-9) and math.isclose(y, 0.4)
    for cell in [(0, 0), (4, 4), (8, 0), (2, 7)]:
        assert f.world_to_cell(*f.cell_center(cell)) == cell


def test_mapper_finds_left_wall_after_confirm_scans():
    # 시작 칸 중앙, 위를 보고 있을 때 왼쪽 칸 (8,7) 앞면(0.2 m)에 점이 찍힘
    f = GridFrame()
    m = WallMapper(f)
    n = 360
    inc = 2 * math.pi / n
    ranges = [math.inf] * n
    for i in range(n):
        a = -math.pi + i * inc
        if abs(a - math.pi / 2) < math.radians(20):
            ranges[i] = 0.2 / math.cos(a - math.pi / 2)
    found = []
    for _ in range(5):
        found += m.add_scan((0.0, 0.0, 0.0), ranges, -math.pi, inc)
    assert found == [(8, 7)]
