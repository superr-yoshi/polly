"""
poli_maze의 탐색 규칙을 임무 1에 연결한다.

navigation과 maze 모두 칸을 (x, y)로, 왼쪽 아래부터 1부터 센다 (2026-10-31 통일).
그래서 칸 번호를 바꾸지 않고 그대로 넘긴다.
주행 완료는 명령 발행이 아니라 엔코더 위치 변화로 기록한다.
"""

import copy
import math

from poli_maze.maze_logic import MazeState, step
from poli_navigation.grid_map import BLOCKED, CENTER, GRID_SIZE, MISSION1_START


class Mission1Maze:
    """방문 횟수와 목표 거리로 바로 이웃한 다음 칸을 선택한다."""

    def __init__(self):
        self.state = MazeState(n=GRID_SIZE, start=MISSION1_START, goal=CENTER)

    def observe(self, cell, yaw):
        """실제로 들어간 칸을 한 번 기록하고 현재 방향을 맞춘다."""
        if cell is None:
            return
        if cell != self.state.cell:
            self.state.cell = cell
            self.state.visits[cell] = self.state.visits.get(cell, 0) + 1
            self.state.path.append(cell)
        # yaw pi/2(+y) = 위(0), 0(+x) = 오른쪽(1). maze 방향 번호와 같다
        self.state.facing = round((math.pi / 2 - yaw) / (math.pi / 2)) % 4

    def next_cell(self, grid):
        """현재 지도에서 막히지 않은 이웃 중 maze 규칙으로 선택한다."""
        self.state.walls = {cell for cell, value in grid.cells.items() if value == BLOCKED}
        direction = self.state.choose_next()
        if direction is None:
            return None
        return step(self.state.cell, direction)

    def next_cell_from(self, grid, cell, yaw):
        """cell에 yaw 방향으로 들어갔다고 가정했을 때 고를 다음 칸 (상태는 바꾸지 않는다)."""
        state = copy.copy(self.state)
        state.visits = dict(self.state.visits)
        state.walls = {c for c, value in grid.cells.items() if value == BLOCKED}
        state.cell = cell
        state.visits[cell] = state.visits.get(cell, 0) + 1
        state.facing = round((math.pi / 2 - yaw) / (math.pi / 2)) % 4
        direction = state.choose_next()
        if direction is None:
            return None
        return step(cell, direction)

    def visited_cells(self):
        """복귀 경로에 사용할 실제 방문 칸을 반환한다."""
        return self.state.visited_cells()
