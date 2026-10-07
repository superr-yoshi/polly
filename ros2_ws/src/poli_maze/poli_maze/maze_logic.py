"""미로 탐색 알고리즘 (ROS 없이 동작하는 순수 로직).

시뮬레이터(폴리 미로 시뮬레이터)와 같은 규칙을 쓴다.

갈 때 (목표 찾기)
  1. 칸 숫자 = 목표까지 맨해튼 거리 + 1 (목표 칸 = 1)
  2. 점수 = 칸 숫자 + 방문 횟수 x 10. 지날 때마다 +10 누적
  3. 라이다로 찾은 벽 칸은 50 으로 표시하고 절대 가지 않는다
  4. 점수가 가장 작은 칸으로 간다. 같으면 직진 -> 후진 -> 오른쪽 -> 왼쪽
  5. 뒤쪽 칸은 돌지 않고 후진, 옆 칸은 90도 돌고 전진

돌아올 때 (복귀)
  1. 목표에서 물건을 집는다
  2. 올 때 지나간 칸에만, 시작점 1 부터 물결처럼(BFS) 새 숫자를 매긴다
  3. 숫자가 가장 작은 칸으로 간다 (동점 규칙은 갈 때와 같음)

좌표: 칸은 (행, 열). 행 0 이 경기장 맨 위, 열 0 이 맨 왼쪽 (0 부터 셈).
방향: 0 = 위, 1 = 오른쪽, 2 = 아래, 3 = 왼쪽 (경기장 그림 기준).
"""

from collections import deque

# ---------------------------------------------------------------------------
# 경기장 설정
# ---------------------------------------------------------------------------
# TODO_HW: 대회 규정의 목표 칸, 시작 칸, 시작 방향 확인.
#          지금은 목표 = 정가운데, 시작 = 오른쪽 아래 구석에서 위를 보고 출발로 가정.
# 칸 번호는 0 부터 센다. 사람이 1 부터 세는 (5,5) 가 코드의 (4,4).
N = 9
GOAL = (4, 4)    # 사람 기준 (5, 5)
START = (8, 8)   # 사람 기준 (9, 9)
START_DIR = 0  # 위쪽을 보고 출발

VISIT_COST = 10
WALL_VALUE = 50  # 표시용. 벽 칸은 점수와 상관없이 후보에서 뺀다.

UP, RIGHT, DOWN, LEFT = 0, 1, 2, 3
DIRS = [(-1, 0), (0, 1), (1, 0), (0, -1)]
DIR_NAME = ['위', '오른쪽', '아래', '왼쪽']

FORWARD = 'forward'
REVERSE = 'reverse'
TURN_RIGHT = 'turn_right'
TURN_LEFT = 'turn_left'


def opposite(d):
    """반대 방향."""
    return (d + 2) % 4


def step(cell, d):
    """cell 에서 d 방향으로 한 칸 간 칸."""
    return (cell[0] + DIRS[d][0], cell[1] + DIRS[d][1])


def inside(cell, n=N):
    """경기장 안인지."""
    return 0 <= cell[0] < n and 0 <= cell[1] < n


def choice_order(facing):
    """점수가 같을 때 고르는 순서: 직진, 후진, 오른쪽, 왼쪽."""
    return [facing, opposite(facing), (facing + 1) % 4, (facing + 3) % 4]


def how_to_move(facing, d):
    """지금 보는 방향에서 d 방향 칸으로 가는 방법."""
    if d == facing:
        return FORWARD
    if d == opposite(facing):
        return REVERSE
    return TURN_RIGHT if d == (facing + 1) % 4 else TURN_LEFT


def facing_after(facing, d):
    """d 방향으로 한 칸 간 뒤 보는 방향. 후진이면 그대로."""
    return facing if d == opposite(facing) else d


class MazeState:
    """갈 때 쓰는 상태: 찾은 벽, 방문 횟수, 현재 칸, 보는 방향."""

    def __init__(self, n=N, start=START, goal=GOAL, start_dir=START_DIR):
        self.n = n
        self.start = start
        self.goal = goal
        self.cell = start
        self.facing = start_dir
        self.walls = set()
        self.visits = {start: 1}  # 시작 칸에 도착한 것으로 친다
        self.path = [start]

    # ----- 벽 -----
    def add_wall(self, cell):
        """벽 칸을 기록한다. 새로 찾았으면 True.

        지나간 칸, 시작 칸, 목표 칸은 벽일 수 없으므로 무시한다
        (라이다 노이즈로 잘못 찍힌 경우를 걸러낸다).
        """
        if not inside(cell, self.n) or cell in self.walls:
            return False
        if cell in self.visits or cell in (self.start, self.goal):
            return False
        self.walls.add(cell)
        return True

    # ----- 점수 -----
    def base_value(self, cell):
        """목표까지 맨해튼 거리 + 1."""
        return abs(cell[0] - self.goal[0]) + abs(cell[1] - self.goal[1]) + 1

    def score(self, cell):
        """칸 숫자 + 방문 x 10."""
        return self.base_value(cell) + self.visits.get(cell, 0) * VISIT_COST

    def display_value(self, cell):
        """화면/로그용 값. 벽은 50."""
        return WALL_VALUE if cell in self.walls else self.score(cell)

    # ----- 판단 -----
    def at_goal(self):
        """목표 칸인지."""
        return self.cell == self.goal

    def choose_next(self):
        """다음에 갈 방향. 갈 곳이 없으면 None."""
        best_d, best_s = None, None
        for d in choice_order(self.facing):
            nxt = step(self.cell, d)
            if not inside(nxt, self.n) or nxt in self.walls:
                continue
            s = self.score(nxt)
            if best_s is None or s < best_s:  # 같으면 먼저 본 쪽(직진 우선)
                best_d, best_s = d, s
        return best_d

    def move(self, d):
        """d 방향으로 한 칸 이동했다고 기록하고, 도착 칸 방문 +10."""
        how = how_to_move(self.facing, d)
        self.facing = facing_after(self.facing, d)
        self.cell = step(self.cell, d)
        self.visits[self.cell] = self.visits.get(self.cell, 0) + 1
        self.path.append(self.cell)
        return how

    def visited_cells(self):
        """지금까지 지나간 칸."""
        return set(self.visits)

    def text_map(self):
        """디버그용 숫자표 문자열."""
        rows = []
        for r in range(self.n):
            row = []
            for c in range(self.n):
                if (r, c) == self.cell:
                    row.append('  R')
                else:
                    row.append('%3d' % self.display_value((r, c)))
            rows.append(' '.join(row))
        return '\n'.join(rows)


# ---------------------------------------------------------------------------
# 복귀
# ---------------------------------------------------------------------------
def build_return_map(visited, walls, start=START, n=N):
    """올 때 지나간 칸에만, 시작점 1 부터 물결처럼 숫자를 매긴다.

    대기 줄(BFS): 맨 앞 칸을 꺼내서, 옆 칸이 지나간 칸이고 아직 숫자가
    없으면 '꺼낸 칸 숫자 + 1' 을 적고 줄 맨 뒤에 세운다.
    """
    num = {start: 1}
    queue = deque([start])
    while queue:
        cell = queue.popleft()
        for d in range(4):
            nxt = step(cell, d)
            if (inside(nxt, n) and nxt in visited
                    and nxt not in walls and nxt not in num):
                num[nxt] = num[cell] + 1
                queue.append(nxt)
    return num


def plan_return(cell, facing, return_map, start=START):
    """목표에서 시작점까지 갈 방향 목록. 가장 작은 숫자를 따라간다."""
    dirs = []
    while cell != start:
        best_d, best_n = None, None
        for d in choice_order(facing):
            nxt = step(cell, d)
            if nxt in return_map and (best_n is None or return_map[nxt] < best_n):
                best_d, best_n = d, return_map[nxt]
        if best_d is None:
            raise RuntimeError('복귀 길을 찾지 못함: %s' % (cell,))
        dirs.append(best_d)
        facing = facing_after(facing, best_d)
        cell = step(cell, best_d)
    return dirs


def group_segments(dirs):
    """같은 방향이 이어지는 구간으로 묶는다. [(방향, 칸 수), ...]"""
    segments = []
    for d in dirs:
        if segments and segments[-1][0] == d:
            segments[-1][1] += 1
        else:
            segments.append([d, 1])
    return [tuple(s) for s in segments]
