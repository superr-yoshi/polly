"""경기장 좌표 변환과 라이다 벽 맵핑 (ROS 없이 동작하는 순수 로직).

좌표계 ("출발 좌표계")
  - 원점: 시작 칸 중앙
  - +x: 출발할 때 로봇이 보는 방향 (경기장 그림의 위쪽)
  - +y: 로봇 왼쪽 (경기장 그림의 왼쪽)
  - yaw: +x 기준, 반시계 방향이 + (ROS 규칙)

맵핑 방법
  라이다 점 하나 = "로봇 기준 어느 각도로 몇 m". 여기에 로봇 위치와 방향을
  더해 출발 좌표계 점으로 바꾸고, 칸 크기로 나눠 몇 번째 칸인지 구한다.
  한 스캔에서 점이 HIT_MIN_PER_SCAN 개 이상 찍힌 칸을 '이번 스캔에서 봤다'
  로 치고, 그런 스캔이 CONFIRM_SCANS 번 쌓이면 벽으로 확정한다.
"""

import math

from poli_maze import maze_logic as ml

# ---------------------------------------------------------------------------
# 경기장 크기
# ---------------------------------------------------------------------------
CELL_SIZE = 0.40  # TODO_MEASURE: 실제 경기장 칸 한 변 (m)

# ---------------------------------------------------------------------------
# 라이다 장착 위치 (로봇 회전 중심 기준)
# ---------------------------------------------------------------------------
# TODO_HW: 하드웨어 완성 후 라이다가 달린 위치를 재서 넣는다.
#          URDF 에 고정된 값과 같아야 한다 (조원 A: LiDAR 위치는 URDF 만 고치면 됨).
LASER_X = 0.0    # 회전 중심에서 앞쪽으로 (m)
LASER_Y = 0.0    # 회전 중심에서 왼쪽으로 (m)
LASER_YAW = 0.0  # 라이다 0도가 로봇 정면에서 반시계로 몇 rad 돌아가 있는지
# TODO_HW: 라이다가 거꾸로(뒤집혀) 달리면 각도 방향이 반대가 된다. 그 경우 True.
LASER_FLIPPED = False

# ---------------------------------------------------------------------------
# 벽 판단 기준
# ---------------------------------------------------------------------------
MAX_RANGE = 1.3          # TODO_MEASURE: 이보다 먼 점은 위치 오차가 커서 버린다 (약 3칸)
MIN_RANGE = 0.05         # TODO_MEASURE: 로봇 몸체/집게에 맞은 점 거르기
PUSH = 0.03              # 점을 레이저 방향으로 더 밀어 벽 물체 안쪽 칸에 들어가게 함 (m)
HIT_MIN_PER_SCAN = 3     # TODO_MEASURE: 한 스캔에서 이만큼 찍혀야 '봤다'
CONFIRM_SCANS = 3        # TODO_MEASURE: '봤다' 가 이만큼 쌓이면 벽 확정


def wrap_angle(a):
    """각도를 -pi ~ pi 로."""
    return math.atan2(math.sin(a), math.cos(a))


class GridFrame:
    """칸 번호 <-> 출발 좌표계 (m) 변환."""

    def __init__(self, start=ml.START, start_dir=ml.START_DIR, cell=CELL_SIZE):
        self.start = start
        self.start_dir = start_dir
        self.cell = cell

    def dir_yaw(self, d):
        """경기장 방향 d 를 출발 좌표계 yaw 로. 출발 방향 = 0."""
        return wrap_angle(-(d - self.start_dir) * math.pi / 2)

    def dir_vec(self, d):
        """경기장 방향 d 의 단위 벡터 (x, y)."""
        yaw = self.dir_yaw(d)
        return (math.cos(yaw), math.sin(yaw))

    def cell_center(self, cell):
        """칸 중앙의 (x, y)."""
        up = self.start[0] - cell[0]      # 위로 몇 칸
        right = cell[1] - self.start[1]   # 오른쪽으로 몇 칸
        ux, uy = self.dir_vec(ml.UP)
        rx, ry = self.dir_vec(ml.RIGHT)
        return ((up * ux + right * rx) * self.cell,
                (up * uy + right * ry) * self.cell)

    def world_to_cell(self, x, y):
        """(x, y) 가 들어 있는 칸."""
        ux, uy = self.dir_vec(ml.UP)
        rx, ry = self.dir_vec(ml.RIGHT)
        up = math.floor((x * ux + y * uy) / self.cell + 0.5)
        right = math.floor((x * rx + y * ry) / self.cell + 0.5)
        return (self.start[0] - up, self.start[1] + right)


class WallMapper:
    """라이다 스캔을 받아 벽 칸을 찾는다."""

    def __init__(self, frame, n=ml.N):
        self.frame = frame
        self.n = n
        self.seen = {}  # 칸 -> '봤다' 횟수

    def scan_to_cells(self, pose, ranges, angle_min, angle_inc,
                      range_min=0.0, range_max=float('inf')):
        """스캔 한 번에서 칸별 점 개수를 센다. {칸: 개수}"""
        x, y, yaw = pose
        lyaw = yaw + LASER_YAW
        lx = x + LASER_X * math.cos(yaw) - LASER_Y * math.sin(yaw)
        ly = y + LASER_X * math.sin(yaw) + LASER_Y * math.cos(yaw)
        counts = {}
        for i, rng in enumerate(ranges):
            if not math.isfinite(rng):
                continue
            if rng < max(range_min, MIN_RANGE) or rng > min(range_max, MAX_RANGE):
                continue
            beam = angle_min + i * angle_inc
            if LASER_FLIPPED:
                beam = -beam
            a = lyaw + beam
            d = rng + PUSH
            cell = self.frame.world_to_cell(lx + d * math.cos(a), ly + d * math.sin(a))
            if 0 <= cell[0] < self.n and 0 <= cell[1] < self.n:
                counts[cell] = counts.get(cell, 0) + 1
        return counts

    def add_scan(self, pose, ranges, angle_min, angle_inc,
                 range_min=0.0, range_max=float('inf'), free_cells=()):
        """스캔을 반영하고, 이번에 새로 확정된 벽 칸 목록을 돌려준다.

        free_cells: 이미 지나가서 벽이 아닌 게 확실한 칸 (무시).
        경기장 밖(바깥 테두리 벽)에 찍힌 점은 칸 번호가 범위 밖이라 자동으로 빠진다.
        """
        counts = self.scan_to_cells(pose, ranges, angle_min, angle_inc,
                                    range_min, range_max)
        confirmed = []
        for cell, cnt in counts.items():
            if cell in free_cells or cnt < HIT_MIN_PER_SCAN:
                continue
            self.seen[cell] = self.seen.get(cell, 0) + 1
            if self.seen[cell] == CONFIRM_SCANS:
                confirmed.append(cell)
        return confirmed
