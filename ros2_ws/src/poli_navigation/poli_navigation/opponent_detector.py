"""
임무 2: LiDAR로 가장 가까운 상대 로봇 위치를 찾는다.

ROS2와 무관한 순수 Python 모듈이다.

방법:
- 임무 2 경기장에는 장애물이 없다. 외벽도 아니고 우리 로봇(잡은 큐브 포함)도 아닌 점은 상대 로봇이다.
- 스캔 점을 경기장 좌표로 바꿔 벽 근처 점과 우리 차체 안 점을 버리고,
  로봇에서 가까운 점부터 주변에 점이 MIN_POINTS개 이상 모인 곳을 찾아 그 평균 위치를 돌려준다.
  (점 하나짜리는 잡음으로 본다)

좌표: 경기장 좌표(m), 왼쪽 아래 모서리가 (0, 0) (mission2_logic.py와 같다)
"""

import math

# LiDAR 장착값은 노드가 실행 중에 TF로 바꾸므로 모듈째 가져와서 매번 읽는다.
from poli_navigation import scan_to_grid
from poli_navigation.mission2_logic import ARENA_SIZE_M


# 외벽에서 이 안쪽 점은 벽으로 본다 (위치 추정 오차 + 벽 두께). TODO_MEASURE
WALL_IGNORE_M = 0.15

# 우리 차체(잡은 큐브 포함)로 보고 버리는 범위. 로봇 기준(바퀴 축 중앙, x = 앞).
# 집게 끝 0.17 m(mission1_logic.BODY_FRONT_M), 잡은 큐브 중심 0.13 m(hardware_reference.md).
BODY_FRONT_M = 0.22  # 집게 끝 + 여유. TODO_MEASURE
BODY_REAR_M = 0.15  # TODO_MEASURE: 축 ~ 차체 뒤끝
BODY_HALF_WIDTH_M = 0.18  # 바퀴 포함 폭 30.5 cm의 절반 + 여유. TODO_MEASURE

# 상대 로봇으로 볼 점 묶음: 기준 점에서 이 거리 안에 MIN_POINTS개 이상
CLUSTER_M = 0.12
MIN_POINTS = 3
# 가까운 점부터 이 개수까지만 묶음 기준으로 본다 (스캔 콜백이 판단을 늦추지 않게)
MAX_CANDIDATES = 40


def inside_body(x, y):
    """로봇 기준 점 (x, y)가 우리 차체·잡은 큐브 범위 안인지."""
    return -BODY_REAR_M <= x <= BODY_FRONT_M and abs(y) <= BODY_HALF_WIDTH_M


def opponent_points(scan, pose):
    """벽·우리 차체를 뺀 스캔 점들의 경기장 좌표 목록."""
    robot_x, robot_y, yaw = pose
    cos_r = math.cos(yaw)
    sin_r = math.sin(yaw)
    points = []

    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance):
            continue
        if not scan.range_min <= distance <= scan.range_max:
            continue

        # 로봇 기준 좌표
        ray = scan_to_grid.LASER_YAW_RAD + scan.angle_min + i * scan.angle_increment
        local_x = scan_to_grid.LASER_OFFSET_X_M + distance * math.cos(ray)
        local_y = scan_to_grid.LASER_OFFSET_Y_M + distance * math.sin(ray)
        if inside_body(local_x, local_y):
            continue

        x = robot_x + local_x * cos_r - local_y * sin_r
        y = robot_y + local_x * sin_r + local_y * cos_r
        if not (WALL_IGNORE_M < x < ARENA_SIZE_M - WALL_IGNORE_M
                and WALL_IGNORE_M < y < ARENA_SIZE_M - WALL_IGNORE_M):
            continue
        points.append((x, y))

    return points


def find_opponent(scan, pose):
    """
    가장 가까운 상대 로봇의 경기장 좌표 (x, y). 없으면 None.

    pose: 우리 로봇 위치 (x, y, yaw), 경기장 좌표
    """
    points = opponent_points(scan, pose)
    points.sort(key=lambda point: math.dist(point, pose[:2]))

    for point in points[:MAX_CANDIDATES]:
        cluster = [other for other in points if math.dist(other, point) <= CLUSTER_M]
        if len(cluster) >= MIN_POINTS:
            return (
                sum(other[0] for other in cluster) / len(cluster),
                sum(other[1] for other in cluster) / len(cluster),
            )

    return None
