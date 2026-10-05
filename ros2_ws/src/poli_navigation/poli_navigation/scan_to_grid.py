"""LiDAR 스캔 -> 막힌 격자 / 빈 격자 판단.

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
    CELL_SIZE_M,
    GRID_SIZE,
    MISSION1_START,
    cell_center_m,
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


def odom_to_arena(odom_x, odom_y, odom_yaw):
    """odom 좌표(출발 지점 기준, x = 출발 시 정면) -> 경기장 좌표."""
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
    """광선이 맞은 점 -> 맞은 장애물 격자.

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
    """스캔 한 번으로 막힌 격자와 빈 격자를 판단한다.

    robot_pose: 경기장 좌표 (x, y, yaw)
    반환: (blocked_cells, free_cells) 두 개의 set
    """
    robot_x, robot_y, robot_yaw = robot_pose
    cos_r = math.cos(robot_yaw)
    sin_r = math.sin(robot_yaw)

    laser_x = robot_x + LASER_OFFSET_X_M * cos_r - LASER_OFFSET_Y_M * sin_r
    laser_y = robot_y + LASER_OFFSET_X_M * sin_r + LASER_OFFSET_Y_M * cos_r
    robot_cell = point_to_cell(robot_x, robot_y)

    hit_counts = {}
    free_cells = set()

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
        while step < distance - FREE_STOP_BEFORE_HIT_M:
            cell = point_to_cell(laser_x + dir_x * step, laser_y + dir_y * step)
            if cell is not None:
                free_cells.add(cell)
            step += FREE_STEP_M

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

    return blocked_cells, free_cells


def update_grid_from_scan(grid, scan, robot_pose):
    """스캔 결과를 격자 지도에 반영한다.

    이미 막힘으로 표시된 격자는 빈 격자로 바꾸지 않는다.
    (장애물은 고정되어 있으므로, 한 번 막힘이면 계속 막힘으로 본다.)

    TODO: 상대 로봇도 막힘으로 인식될 수 있다. (움직이는 장애물 처리 필요)
    """
    blocked_cells, free_cells = scan_to_cells(scan, robot_pose)

    for cell in blocked_cells:
        grid.mark_blocked(cell)

    for cell in free_cells:
        if grid.get(cell) != BLOCKED and grid.get(grid.mirror(cell)) != BLOCKED:
            grid.mark_free(cell)

    return blocked_cells, free_cells
