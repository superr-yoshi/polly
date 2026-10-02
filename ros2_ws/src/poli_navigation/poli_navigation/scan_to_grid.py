"""
LiDAR 스캔 -> 막힌 격자 / 빈 격자 판단.

ROS2와 무관한 순수 Python 모듈이다.
scan 인자는 sensor_msgs/LaserScan 또는 같은 속성
(angle_min, angle_increment, range_min, range_max, ranges)을 가진 객체면 된다.

좌표 (docs/mission_strategy.md 0장):
- 경기장 좌표(arena): 경기장 왼쪽 아래 모서리가 (0, 0), 단위 m
- 로봇은 항상 (9, 1) 격자 중심에서 +y 방향을 보고 출발한 것으로 계산한다.
"""

import math

from poli_navigation.grid_map import (
    BLOCKED,
    cell_center_m,
    CELL_SIZE_M,
    GRID_SIZE,
    MISSION1_START,
)


ARENA_SIZE_M = GRID_SIZE * CELL_SIZE_M

# 출발 자세: (9, 1) 격자 중심, +y 방향
# TODO_MEASURE: 실제로 로봇이 출발 격자 중심에 정확히 놓이는지 확인
START_POSE = (*cell_center_m(MISSION1_START), math.pi / 2)

# TODO_MEASURE: 로봇 중심(base_link) 기준 LiDAR 장착 위치. 현재 중심에 있다고 가정.
LASER_OFFSET_X_M = 0.0
LASER_OFFSET_Y_M = 0.0

# 외벽에서 이 거리 안쪽의 점은 벽으로 보고 무시한다. (TODO: 실측 후 조정)
WALL_MARGIN_M = 0.05

# 장애물 표면에 맞은 점은 격자 경계에 걸리므로
# 맞은 면을 가로질러 조금 밀어서 장애물 격자 안쪽으로 넣는다. (TODO: 실측 후 조정)
HIT_PUSH_M = 0.05

# 한 격자에 이 개수 이상 점이 맞아야 막힘으로 판단한다. (TODO: 실측 후 조정)
MIN_HITS_PER_CELL = 2

# 빈 격자 판단 시 광선을 따라 확인하는 간격, 맞은 지점 앞에서 멈추는 여유
FREE_STEP_M = 0.1
FREE_STOP_BEFORE_HIT_M = 0.1

# 막힘으로 표시된 격자를 광선 여러 개가 뚫고 지나가면 장애물이 없는 것이다.
# (상대 로봇을 장애물로 잘못 표시한 경우. 상대가 떠나면 지도에서 지운다.)
# 격자 가장자리에서 CORE_MARGIN_M 안쪽을 지나간 광선만 센다.
# 위치 오차로 장애물 면을 스치는 광선을 잘못 세지 않기 위해서다.
# TODO_MEASURE: 실제 위치 오차를 보고 조정
CORE_MARGIN_M = 0.1
MIN_SEE_THROUGH_RAYS = 3


def odom_to_arena(odom_x, odom_y, odom_yaw):
    """Odom 좌표(출발 지점 기준, x = 출발 시 정면) -> 경기장 좌표."""
    start_x, start_y, start_yaw = START_POSE
    cos_s = math.cos(start_yaw)
    sin_s = math.sin(start_yaw)

    arena_x = start_x + odom_x * cos_s - odom_y * sin_s
    arena_y = start_y + odom_x * sin_s + odom_y * cos_s
    arena_yaw = start_yaw + odom_yaw

    return arena_x, arena_y, arena_yaw


def point_to_cell(x, y):
    """경기장 좌표(m) -> 격자 (x, y). 경기장 밖이면 None."""
    if not (0.0 <= x < ARENA_SIZE_M and 0.0 <= y < ARENA_SIZE_M):
        return None

    return (int(x // CELL_SIZE_M) + 1, int(y // CELL_SIZE_M) + 1)


def is_near_wall(x, y):
    return (
        x < WALL_MARGIN_M
        or y < WALL_MARGIN_M
        or x > ARENA_SIZE_M - WALL_MARGIN_M
        or y > ARENA_SIZE_M - WALL_MARGIN_M
    )


def hit_to_cell(hit_x, hit_y, dir_x, dir_y):
    """
    광선이 맞은 점 -> 맞은 장애물 격자.

    장애물은 격자 1칸 크기이므로 맞은 점에서 가장 가까운 격자 선이 맞은 면이다.
    그 면을 가로지르는 방향(광선 진행 방향)으로만 밀어서 장애물 격자를 찾는다.
    (광선 방향 그대로 밀면 모서리 근처에서 옆 격자로 넘어간다.)
    """
    dist_to_x_line = abs(hit_x - round(hit_x / CELL_SIZE_M) * CELL_SIZE_M)
    dist_to_y_line = abs(hit_y - round(hit_y / CELL_SIZE_M) * CELL_SIZE_M)

    if dist_to_x_line <= dist_to_y_line:
        hit_x += math.copysign(HIT_PUSH_M, dir_x)
    else:
        hit_y += math.copysign(HIT_PUSH_M, dir_y)

    return point_to_cell(hit_x, hit_y)


def scan_to_cells(scan, robot_pose):
    """
    스캔 한 번으로 막힌 격자와 빈 격자를 판단한다.

    robot_pose: 경기장 좌표 (x, y, yaw)
    반환: (blocked_cells, free_cells) 두 개의 set
    """
    blocked_cells, free_cells, _ = _trace_scan(scan, robot_pose)
    return blocked_cells, free_cells


def is_in_cell_core(x, y):
    """격자 가장자리에서 CORE_MARGIN_M보다 안쪽인지."""
    in_x = x % CELL_SIZE_M
    in_y = y % CELL_SIZE_M
    return (
        CORE_MARGIN_M <= in_x <= CELL_SIZE_M - CORE_MARGIN_M
        and CORE_MARGIN_M <= in_y <= CELL_SIZE_M - CORE_MARGIN_M
    )


def _trace_scan(scan, robot_pose):
    """
    스캔 광선을 따라가며 막힌 격자, 빈 격자, 격자별 관통 광선 수를 구한다.

    반환: (blocked_cells, free_cells, see_through_counts)
    """
    robot_x, robot_y, robot_yaw = robot_pose
    cos_r = math.cos(robot_yaw)
    sin_r = math.sin(robot_yaw)

    laser_x = robot_x + LASER_OFFSET_X_M * cos_r - LASER_OFFSET_Y_M * sin_r
    laser_y = robot_y + LASER_OFFSET_X_M * sin_r + LASER_OFFSET_Y_M * cos_r
    robot_cell = point_to_cell(robot_x, robot_y)

    hit_counts = {}
    free_cells = set()
    see_through_counts = {}

    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance):
            continue
        if not scan.range_min <= distance <= scan.range_max:
            continue

        ray_angle = robot_yaw + scan.angle_min + i * scan.angle_increment
        dir_x = math.cos(ray_angle)
        dir_y = math.sin(ray_angle)

        # 광선이 지나간 곳은 비어 있다.
        step = FREE_STEP_M
        see_through = set()
        while step < distance - FREE_STOP_BEFORE_HIT_M:
            x = laser_x + dir_x * step
            y = laser_y + dir_y * step
            cell = point_to_cell(x, y)
            if cell is not None:
                free_cells.add(cell)
                if is_in_cell_core(x, y):
                    see_through.add(cell)
            step += FREE_STEP_M

        for cell in see_through:
            see_through_counts[cell] = see_through_counts.get(cell, 0) + 1

        # 광선이 맞은 곳
        hit_x = laser_x + dir_x * distance
        hit_y = laser_y + dir_y * distance

        if is_near_wall(hit_x, hit_y):
            continue

        cell = hit_to_cell(hit_x, hit_y, dir_x, dir_y)
        if cell is None or cell == robot_cell:
            continue

        hit_counts[cell] = hit_counts.get(cell, 0) + 1

    blocked_cells = {
        cell for cell, count in hit_counts.items()
        if count >= MIN_HITS_PER_CELL
    }
    free_cells -= blocked_cells

    return blocked_cells, free_cells, see_through_counts


def update_grid_from_scan(grid, scan, robot_pose):
    """
    스캔 결과를 격자 지도에 반영한다.

    막힘으로 표시된 격자는 광선이 가장자리를 스치는 정도로는 바꾸지 않는다.
    광선 여러 개가 격자 안쪽을 뚫고 지나가면 장애물이 없는 것이므로 지운다.
    (상대 로봇을 장애물로 잘못 표시했다가 상대가 떠난 경우)
    지울 때는 점대칭인 격자도 같이 지운다.
    """
    blocked_cells, free_cells, see_through_counts = _trace_scan(
        scan, robot_pose
    )

    for cell in blocked_cells:
        grid.mark_blocked(cell)

    for cell, count in see_through_counts.items():
        if count < MIN_SEE_THROUGH_RAYS or grid.get(cell) != BLOCKED:
            continue
        if cell in blocked_cells or grid.mirror(cell) in blocked_cells:
            continue
        grid.mark_free(cell)

    for cell in free_cells:
        if grid.get(cell) != BLOCKED and grid.get(grid.mirror(cell)) != BLOCKED:
            grid.mark_free(cell)

    return blocked_cells, free_cells
