"""
임무 2: LiDAR로 외벽까지 거리를 재서 경기장 안 위치(x, y)를 추정한다.

ROS2와 무관한 순수 Python 모듈이다.

왜 필요한가:
- 임무 2는 밀어내기가 허용된다. 밀리면 바퀴가 돌지 않아서 odom(엔코더)은
  로봇이 움직인 것을 모른다. 그러면 탈락선에 가까워져도 알 수 없다.
- 임무 2 경기장은 장애물이 없는 3.6m 정사각형이므로 외벽까지 거리로 위치를 알 수 있다.

방법:
- 방향(yaw)은 odom/IMU 값을 믿는다. (밀려서 돌아가도 IMU는 회전을 잰다)
  yaw 오차 3도 이내면 위치 오차 약 4cm 이내, 5도 이상이면 10cm 넘게 틀어질 수 있다.
  TODO: IMU 방향이 많이 틀어지면 벽의 기울기로 yaw도 보정
- 스캔 점을 경기장 방향으로 돌려서 +x, -x, +y, -y 네 방향의 벽을 찾는다.
- 한 방향에서 "가장 먼 곳에 점이 여러 개 모인 거리"를 벽으로 본다.
  상대 로봇은 항상 벽보다 가까이 있으므로, 벽 일부를 가려도 벽은 찾을 수 있다.
- 마주 보는 두 벽까지 거리의 합이 경기장 크기와 맞으면 그대로 쓴다.
  맞지 않으면 한쪽이 가려진 것이므로 바로 전 위치(guess)에 가까운 쪽을 쓴다.

좌표: 경기장 좌표(m), 왼쪽 아래 모서리가 (0, 0) (mission2_logic.py와 같다)
"""

import math

from poli_navigation.mission2_logic import ARENA_SIZE_M, normalize_angle
from poli_navigation.scan_to_grid import LASER_OFFSET_X_M, LASER_OFFSET_Y_M


# 벽을 찾을 때 쓰는 방향 범위: 벽에 수직인 방향에서 좌우 이 각도까지
WALL_SECTOR_HALF_RAD = math.radians(30.0)

# TODO_MEASURE: 같은 벽으로 볼 점들의 거리 차이 (LiDAR 오차, 방향 오차 포함)
WALL_TOLERANCE_M = 0.03

# 이 개수 이상 점이 모여 있어야 벽으로 본다. (TODO: 실측 후 조정)
MIN_WALL_POINTS = 5

# 마주 보는 두 벽 거리의 합이 경기장 크기와 이 이내로 맞으면 둘 다 믿는다.
WALL_PAIR_TOLERANCE_M = 0.1

# 한쪽 벽만 믿을 때, 바로 전 위치에서 이보다 멀리 떨어진 값은 버린다.
# (스캔은 0.1초마다 오므로 그 사이에 이만큼 밀리기는 어렵다)
MAX_CORRECTION_M = 0.3

PLUS_X = '+x'
MINUS_X = '-x'
PLUS_Y = '+y'
MINUS_Y = '-y'

# 경기장 기준 방향
AXIS_ANGLES = {
    PLUS_X: 0.0,
    PLUS_Y: math.pi / 2,
    MINUS_X: math.pi,
    MINUS_Y: -math.pi / 2,
}


def farthest_cluster(distances):
    """가장 먼 곳에서 MIN_WALL_POINTS개 이상 모인 거리들의 중앙값. 없으면 None."""
    values = sorted(distances, reverse=True)

    for i, farthest in enumerate(values):
        cluster = [
            value for value in values[i:]
            if farthest - value <= WALL_TOLERANCE_M
        ]
        if len(cluster) >= MIN_WALL_POINTS:
            return cluster[len(cluster) // 2]

    return None


def find_wall_distances(scan, yaw):
    """
    LiDAR에서 네 방향 벽까지의 수직 거리.

    yaw: 경기장 기준 로봇 방향
    반환: {PLUS_X: 거리 또는 None, ...}
    """
    projections = {axis: [] for axis in AXIS_ANGLES}

    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance):
            continue
        if not scan.range_min <= distance <= scan.range_max:
            continue

        ray_angle = yaw + scan.angle_min + i * scan.angle_increment

        for axis, axis_angle in AXIS_ANGLES.items():
            diff = normalize_angle(ray_angle - axis_angle)
            if abs(diff) < WALL_SECTOR_HALF_RAD:
                # 벽에 수직인 방향으로 잰 거리
                projections[axis].append(distance * math.cos(diff))

    return {
        axis: farthest_cluster(values)
        for axis, values in projections.items()
    }


def axis_position(plus_distance, minus_distance, guess):
    """
    한 축(x 또는 y)의 위치. 벽을 못 찾았거나 믿을 수 없으면 None.

    plus_distance: + 방향 벽까지 거리, minus_distance: - 방향 벽까지 거리
    """
    candidates = []
    if plus_distance is not None:
        candidates.append(ARENA_SIZE_M - plus_distance)
    if minus_distance is not None:
        candidates.append(minus_distance)

    if not candidates:
        return None

    # 두 벽이 모두 보이고 거리가 맞으면 확실하다.
    if len(candidates) == 2:
        if abs(candidates[0] - candidates[1]) <= WALL_PAIR_TOLERANCE_M:
            return (candidates[0] + candidates[1]) / 2.0

    # 한쪽이 가려졌다 (상대 로봇). 바로 전 위치에 가까운 쪽을 믿는다.
    best = min(candidates, key=lambda value: abs(value - guess))
    if abs(best - guess) > MAX_CORRECTION_M:
        return None
    return best


def estimate_position(scan, guess_pose):
    """
    스캔으로 로봇 중심의 경기장 좌표 (x, y)를 추정한다.

    guess_pose: 바로 전 위치 (x, y, yaw). yaw는 그대로 믿는다.
    반환: (x 또는 None, y 또는 None)
    """
    guess_x, guess_y, yaw = guess_pose
    cos_r = math.cos(yaw)
    sin_r = math.sin(yaw)

    # 벽까지 거리는 LiDAR 위치 기준이다.
    offset_x = LASER_OFFSET_X_M * cos_r - LASER_OFFSET_Y_M * sin_r
    offset_y = LASER_OFFSET_X_M * sin_r + LASER_OFFSET_Y_M * cos_r

    walls = find_wall_distances(scan, yaw)

    laser_x = axis_position(
        walls[PLUS_X], walls[MINUS_X], guess_x + offset_x
    )
    laser_y = axis_position(
        walls[PLUS_Y], walls[MINUS_Y], guess_y + offset_y
    )

    x = None if laser_x is None else laser_x - offset_x
    y = None if laser_y is None else laser_y - offset_y
    return x, y
