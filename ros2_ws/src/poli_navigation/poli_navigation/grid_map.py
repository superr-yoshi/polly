"""
9 x 9 경기장 격자 지도와 최단 경로 계산.

ROS2와 무관한 순수 Python 모듈이다. (하드웨어 없이 테스트 가능)

좌표 규칙 (docs/mission_strategy.md 0장):
- 격자 좌표 (x, y), x는 왼쪽->오른쪽 1~9, y는 아래->위 1~9
- 장애물은 중앙 (5, 5) 기준 점대칭이므로 (1,9) 출발과 (9,1) 출발은
  로봇 입장에서 똑같이 보인다.
  따라서 로봇은 항상 "(9, 1)에서 +y 방향을 보고 출발"한 것으로 계산한다.
"""

from collections import deque


GRID_SIZE = 9
CELL_SIZE_M = 0.4

UNKNOWN = 0
FREE = 1
BLOCKED = 2

MISSION1_START = (9, 1)
CENTER = (5, 5)

# 규정 그림 1 예시 장애물 (한쪽 절반만, 나머지는 점대칭으로 자동 표시)
# 실제 경기장은 당일 공개되므로 테스트/확인용으로만 사용한다.
RULE_EXAMPLE_BLOCKED = [
    (7, 1), (7, 2), (7, 3), (7, 4), (7, 5), (8, 5), (7, 6),
    (3, 1), (3, 9), (5, 7), (6, 7),
]


class GridMap:

    def __init__(self, size=GRID_SIZE, use_point_symmetry=True):
        self.size = size
        self.use_point_symmetry = use_point_symmetry
        self.cells = {
            (x, y): UNKNOWN
            for x in range(1, size + 1)
            for y in range(1, size + 1)
        }

    def in_bounds(self, cell):
        x, y = cell
        return 1 <= x <= self.size and 1 <= y <= self.size

    def mirror(self, cell):
        """중앙 기준 점대칭 격자를 반환한다. 예: (3, 1) -> (7, 9)."""
        x, y = cell
        return (self.size + 1 - x, self.size + 1 - y)

    def get(self, cell):
        return self.cells[cell]

    def mark_blocked(self, cell):
        self._mark(cell, BLOCKED)

    def mark_free(self, cell):
        self._mark(cell, FREE)

    def _mark(self, cell, state):
        if not self.in_bounds(cell):
            return

        self.cells[cell] = state

        if self.use_point_symmetry:
            self.cells[self.mirror(cell)] = state

    def is_passable(self, cell):
        # 아직 모르는 격자는 지나갈 수 있다고 보고 계획한다.
        # 이동 중 막힌 것을 발견하면 경로를 다시 계산한다.
        return self.in_bounds(cell) and self.cells[cell] != BLOCKED

    def neighbors(self, cell):
        """상하좌우로 이동 가능한 인접 격자 목록."""
        x, y = cell
        candidates = [(x, y + 1), (x + 1, y), (x, y - 1), (x - 1, y)]
        return [c for c in candidates if self.is_passable(c)]

    def find_path(self, start, goal):
        """
        BFS로 start -> goal 최단 경로를 찾는다.

        경로는 start와 goal을 포함한 격자 목록이다.
        갈 수 없으면 None을 반환한다.
        """
        if not self.is_passable(start) or not self.is_passable(goal):
            return None

        came_from = {start: None}
        queue = deque([start])

        while queue:
            current = queue.popleft()

            if current == goal:
                return self._build_path(came_from, goal)

            for neighbor in self.neighbors(current):
                if neighbor not in came_from:
                    came_from[neighbor] = current
                    queue.append(neighbor)

        return None

    def _build_path(self, came_from, goal):
        path = []
        cell = goal

        while cell is not None:
            path.append(cell)
            cell = came_from[cell]

        path.reverse()
        return path

    def to_text(self, path=None):
        """
        터미널 확인용 지도 문자열. 위쪽이 y = 9.

        # = 막힘, . = 비어 있음, ? = 모름, * = 경로
        """
        path_cells = set(path or [])
        symbols = {UNKNOWN: '?', FREE: '.', BLOCKED: '#'}
        lines = []

        for y in range(self.size, 0, -1):
            row = []
            for x in range(1, self.size + 1):
                if (x, y) in path_cells:
                    row.append('*')
                else:
                    row.append(symbols[self.cells[(x, y)]])
            lines.append(f'{y} ' + ' '.join(row))

        lines.append('  ' + ' '.join(str(x) for x in range(1, self.size + 1)))
        return '\n'.join(lines)


def cell_center_m(cell):
    """격자 중심 좌표 (m). 경기장 왼쪽 아래 모서리가 (0, 0)."""
    x, y = cell
    return ((x - 0.5) * CELL_SIZE_M, (y - 0.5) * CELL_SIZE_M)


if __name__ == '__main__':
    grid = GridMap()
    for blocked in RULE_EXAMPLE_BLOCKED:
        grid.mark_blocked(blocked)

    path = grid.find_path(MISSION1_START, CENTER)

    print(grid.to_text(path))
    print()
    print(f'Path ({len(path) - 1} moves): {path}')
