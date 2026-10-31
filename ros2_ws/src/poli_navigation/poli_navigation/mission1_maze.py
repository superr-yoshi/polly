"""
poli_maze의 탐색 규칙을 임무 1에 연결한다.

navigation과 maze 모두 칸을 (x, y)로, 왼쪽 아래부터 1부터 센다 (2026-10-31 통일).
그래서 칸 번호를 바꾸지 않고 그대로 넘긴다.
주행 완료는 명령 발행이 아니라 엔코더 위치 변화로 기록한다.
"""

import math

from poli_maze.maze_logic import choice_order, inside, MazeState, step
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
        """
        현재 지도에서 막히지 않은 이웃 중 점수(목표 거리 + 방문 x 10)가 가장 작은 칸.

        점수가 같으면 안쪽에서 경기장 가장자리(맨 바깥 줄)로 새로 들어가는 칸은 뒤로 미루고,
        그것도 같으면 maze 순서(직진 먼저). 이미 가장자리를 따라가는 중이면 그대로 직진한다.
        2026-10-31 실기: (8,5)에서 동쪽 (9,5)와 북쪽 (8,6)이 같은 점수라 직진(가장자리)을 골랐다.
        (같으면 중앙 직선거리로 고르면 계단식으로 매 칸 꺾어 시뮬레이션이 65.9 -> 80.6 s로 느려졌다)
        maze의 choose_next는 같으면 직진만 보므로 여기서 처리한다 (poli_maze는 그대로).
        """
        state = self.state
        state.walls = {cell for cell, value in grid.cells.items() if value == BLOCKED}

        def on_edge(cell):
            return cell[0] in (1, state.n) or cell[1] in (1, state.n)

        inside_now = not on_edge(state.cell)
        best, best_key = None, None
        for order, direction in enumerate(choice_order(state.facing)):
            nxt = step(state.cell, direction)
            if not inside(nxt, state.n) or nxt in state.walls:
                continue
            key = (state.score(nxt), inside_now and on_edge(nxt), order)
            if best_key is None or key < best_key:
                best, best_key = nxt, key
        return best

    def visited_cells(self):
        """복귀 경로에 사용할 실제 방문 칸을 반환한다."""
        return self.state.visited_cells()
