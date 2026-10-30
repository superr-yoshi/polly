"""
poli_maze의 탐색 규칙을 임무 1의 좌표와 연결한다.

navigation은 왼쪽 아래부터 (x, y)를 1부터 세고,
maze는 왼쪽 위부터 (행, 열)을 0부터 센다.
주행 완료는 명령 발행이 아니라 엔코더 위치 변화로 기록한다.
"""

import math

from poli_maze.maze_logic import MazeState, step
from poli_navigation.grid_map import BLOCKED, CENTER, GRID_SIZE, MISSION1_START


def to_maze(cell):
    """임무 격자를 미로의 행·열로 바꾼다."""
    return GRID_SIZE - cell[1], cell[0] - 1


def from_maze(cell):
    """미로의 행·열을 임무 격자로 바꾼다."""
    return cell[1] + 1, GRID_SIZE - cell[0]


class Mission1Maze:
    """방문 횟수와 목표 거리로 바로 이웃한 다음 칸을 선택한다."""

    def __init__(self):
        self.state = MazeState(
            n=GRID_SIZE, start=to_maze(MISSION1_START), goal=to_maze(CENTER))

    def observe(self, cell, yaw):
        """실제로 들어간 칸을 한 번 기록하고 현재 방향을 맞춘다."""
        if cell is None:
            return
        current = to_maze(cell)
        if current != self.state.cell:
            self.state.cell = current
            self.state.visits[current] = self.state.visits.get(current, 0) + 1
            self.state.path.append(current)
        self.state.facing = round((math.pi / 2 - yaw) / (math.pi / 2)) % 4

    def next_cell(self, grid):
        """현재 지도에서 막히지 않은 이웃 중 maze 규칙으로 선택한다."""
        self.state.walls = {
            to_maze(cell) for cell, value in grid.cells.items() if value == BLOCKED
        }
        direction = self.state.choose_next()
        if direction is None:
            return None
        return from_maze(step(self.state.cell, direction))

    def visited_cells(self):
        """복귀 경로에 사용할 실제 방문 칸을 반환한다."""
        return {from_maze(cell) for cell in self.state.visited_cells()}
