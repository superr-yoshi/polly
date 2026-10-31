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
    FREE,
    GRID_SIZE,
    MISSION1_START,
)


ARENA_SIZE_M = GRID_SIZE * CELL_SIZE_M

# 출발 자세: (9, 1) 격자 중심, +y 방향
# TODO_MEASURE: 실제로 로봇이 출발 격자 중심에 정확히 놓이는지 확인
START_POSE = (*cell_center_m(MISSION1_START), math.pi / 2)

# 로봇 중심(base_link) 기준 LiDAR 장착 위치와 방향.
# 임무 노드가 켜지면 TF(base_link -> laser, poli_description URDF)에서 읽어
# set_laser_mount()로 바꾼다. TF가 없을 때(시뮬레이션)는 아래 기본값을 쓴다.
# 실측값은 URDF(lidar_x, lidar_y, lidar_yaw) 한 곳에서만 고친다.
LASER_OFFSET_X_M = 0.0
LASER_OFFSET_Y_M = 0.0
LASER_YAW_RAD = 0.0

# 외벽에서 이 거리 안쪽의 점은 벽으로 보고 무시한다. (TODO: 실측 후 조정)
WALL_MARGIN_M = 0.05

# 장애물 표면에 맞은 점은 격자 경계에 걸리므로
# 맞은 면을 가로질러 조금 밀어서 장애물 격자 안쪽으로 넣는다. (TODO: 실측 후 조정)
HIT_PUSH_M = 0.10  # 2026-10-31: 0.05면 로봇 놓는 위치·방향이 몇 cm·몇 도만 어긋나도 벽 점이 통로 칸으로 넘어와 막힘 (실기 확인)

# 한 격자에 이 개수 이상 점이 맞아야 막힘으로 판단한다. (TODO: 실측 후 조정)
MIN_HITS_PER_CELL = 2
# 이보다 먼 점은 지도에 쓰지 않는다 (막힘·빈칸 모두 '모름'으로 둔다).
# 2026-10-31 실기: 장애물(높이 약 20 cm) 윗변과 레이저 높이가 비슷해 약 0.55 m 너머 벽은 레이저가 위로 넘어간다.
# 임무는 한 칸씩 멈춰 옆 칸만 고르므로 가까운 점(옆 칸의 면은 약 0.3 m)만으로 충분하다. (TODO_MEASURE)
MAX_USE_RANGE_M = 0.7
# 칸 가운데에서 상하좌우 이웃 칸은 그 방향 ±CONE_HALF_RAD 레이저로만 판단한다 (2026-10-31 팀 요청).
# 로봇이 몇 도 비뚤어져 있어도 45도 쪽 옆 벽이 앞칸 판단에 섞이지 않게 한다.
CONE_HALF_RAD = math.radians(15.0)
CONE_BLOCK_M = 0.40      # 로봇 중심에서 이 거리 안(이웃 칸 앞쪽 절반)에 맞으면 그 칸에 장애물
CONE_FREE_M = 0.55       # 이보다 멀리 맞은 레이저는 이웃 칸을 지나간 것
CONE_SKIP_M = 0.12       # 이보다 가까운 점은 로봇 자기 부품으로 보고 버린다
CONE_MIN_RAYS = 3
# 칸 가운데에 멈췄을 때 가까운 벽(칸 경계, 중심에서 0.2 m)까지 거리로 위치를 바로잡는다 (2026-10-31 팀 요청).
WALL_FIT_MIN_M = 0.10
WALL_FIT_MAX_M = 0.32
MAX_POSE_FIX_M = 0.10    # 한 번에 고치는 최대 거리 (잘못 잰 값에 크게 끌려가지 않게)
# 같은 가까운 벽 면의 기울기로 방향(yaw)도 바로잡는다 (2026-10-31: IMU 방향 오차가 쌓여 벽에 박음).
# 벽은 모두 경기장 축과 나란하므로, 추정 방향이 e만큼 틀리면 벽 점들이 e만큼 기울어 보인다.
YAW_FIT_CONE_RAD = math.radians(30.0)  # 방향마다 이 범위 레이저로 벽 직선을 맞춘다
YAW_FIT_MIN_POINTS = 8
YAW_FIT_MIN_SPAN_M = 0.08    # 벽 점이 옆으로 이만큼은 퍼져 있어야 기울기를 믿는다
YAW_FIT_OUTLIER_M = 0.02     # 첫 직선에서 이보다 먼 점(모서리·다른 면)은 빼고 다시 맞춘다
MAX_YAW_FIX_RAD = math.radians(8.0)  # 한 번에 고치는 최대 각도. TODO_MEASURE
# 차체 바로 앞 장애물 감시 (대상에 다가갈 때. 2026-10-31: 대각선 접근 중 옆 장애물에 박음).
# 치수는 바퀴 축(base_link) 기준, 2026-10-30 사용자 실측 (집게 포함 길이 33 cm, 축은 뒤에서 16 cm).
# 큐브(5 cm)는 LiDAR 높이보다 낮아 여기에 걸리지 않는다.
BODY_FRONT_M = 0.17       # 축 ~ 집게 끝. TODO_MEASURE
BODY_HALF_WIDTH_M = 0.153  # 바퀴 포함 폭 30.5 cm의 절반. TODO_MEASURE
GUARD_AHEAD_M = 0.06      # 집게 끝 앞 이 거리 안에 장애물 점이 있으면 멈춘다
GUARD_SIDE_M = 0.01       # 차체 폭보다 이만큼 넓게 본다 (통로 여유가 한쪽 약 4.7 cm뿐)
GUARD_MIN_POINTS = 3

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


def set_laser_mount(offset_x, offset_y, yaw):
    """로봇 기준 LiDAR 장착 위치(m)와 방향(rad)을 바꾼다. wall_localizer.py도 이 값을 쓴다."""
    global LASER_OFFSET_X_M, LASER_OFFSET_Y_M, LASER_YAW_RAD
    LASER_OFFSET_X_M = offset_x
    LASER_OFFSET_Y_M = offset_y
    LASER_YAW_RAD = yaw


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

        ray_angle = robot_yaw + LASER_YAW_RAD + scan.angle_min + i * scan.angle_increment
        dir_x = math.cos(ray_angle)
        dir_y = math.sin(ray_angle)

        # 광선이 지나간 곳은 비어 있다.
        step = FREE_STEP_M
        see_through = set()
        while step < min(distance - FREE_STOP_BEFORE_HIT_M, MAX_USE_RANGE_M):
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

        if distance > MAX_USE_RANGE_M:
            continue

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


def neighbor_states(scan, robot_pose):
    """
    칸 가운데에서 상하좌우 이웃 칸을 각 방향 ±15도 레이저로만 판단한다.

    반환: {이웃 격자: BLOCKED 또는 FREE}. 판단이 애매한 방향은 넣지 않는다.
    """
    robot_x, robot_y, robot_yaw = robot_pose
    cell = point_to_cell(robot_x, robot_y)
    if cell is None:
        return {}
    cos_r = math.cos(robot_yaw)
    sin_r = math.sin(robot_yaw)
    laser_x = robot_x + LASER_OFFSET_X_M * cos_r - LASER_OFFSET_Y_M * sin_r
    laser_y = robot_y + LASER_OFFSET_X_M * sin_r + LASER_OFFSET_Y_M * cos_r
    states = {}
    for phi, (dx, dy) in ((0.0, (1, 0)), (math.pi / 2, (0, 1)),
                          (math.pi, (-1, 0)), (-math.pi / 2, (0, -1))):
        near = far = 0
        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if not scan.range_min <= distance <= scan.range_max:
                continue
            angle = robot_yaw + LASER_YAW_RAD + scan.angle_min + i * scan.angle_increment
            if abs(math.atan2(math.sin(angle - phi), math.cos(angle - phi))) > CONE_HALF_RAD:
                continue
            hit_x = laser_x + distance * math.cos(angle)
            hit_y = laser_y + distance * math.sin(angle)
            along = (hit_x - robot_x) * math.cos(phi) + (hit_y - robot_y) * math.sin(phi)
            if along < CONE_SKIP_M:
                continue
            if along < CONE_BLOCK_M:
                near += 1
            elif along >= CONE_FREE_M:
                far += 1
        neighbor = (cell[0] + dx, cell[1] + dy)
        if near >= CONE_MIN_RAYS:
            states[neighbor] = BLOCKED
        elif near == 0 and far >= CONE_MIN_RAYS:
            states[neighbor] = FREE
    return states


def wall_offset(scan, robot_pose):
    """
    칸 가운데에서 가까운 벽까지 거리로 위치 오차를 잰다.

    장애물·벽 면은 칸 경계(칸 중심에서 0.2 m)에 있다. 상하좌우 ±15도 레이저 중
    0.10~0.32 m에 맞은 점의 중간값으로 실제 위치를 구한다.
    반환: 경기장 좌표 보정량 (dx, dy). 벽이 안 보이는 축은 0.
    """
    robot_x, robot_y, robot_yaw = robot_pose
    cell = point_to_cell(robot_x, robot_y)
    if cell is None:
        return 0.0, 0.0
    center = cell_center_m(cell)
    cos_r = math.cos(robot_yaw)
    sin_r = math.sin(robot_yaw)
    laser_x = robot_x + LASER_OFFSET_X_M * cos_r - LASER_OFFSET_Y_M * sin_r
    laser_y = robot_y + LASER_OFFSET_X_M * sin_r + LASER_OFFSET_Y_M * cos_r
    fixes = ([], [])
    for phi, axis, sign in ((0.0, 0, 1.0), (math.pi / 2, 1, 1.0),
                            (math.pi, 0, -1.0), (-math.pi / 2, 1, -1.0)):
        alongs = []
        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if not scan.range_min <= distance <= scan.range_max:
                continue
            angle = robot_yaw + LASER_YAW_RAD + scan.angle_min + i * scan.angle_increment
            if abs(math.atan2(math.sin(angle - phi), math.cos(angle - phi))) > CONE_HALF_RAD:
                continue
            hit_x = laser_x + distance * math.cos(angle)
            hit_y = laser_y + distance * math.sin(angle)
            along = (hit_x - robot_x) * math.cos(phi) + (hit_y - robot_y) * math.sin(phi)
            if WALL_FIT_MIN_M <= along <= WALL_FIT_MAX_M:
                alongs.append(along)
        if len(alongs) >= CONE_MIN_RAYS:
            alongs.sort()
            measured = alongs[len(alongs) // 2]
            fixes[axis].append(center[axis] + sign * (CELL_SIZE_M / 2 - measured))
    current = (robot_x, robot_y)
    offset = []
    for axis in (0, 1):
        if not fixes[axis]:
            offset.append(0.0)
            continue
        delta = sum(fixes[axis]) / len(fixes[axis]) - current[axis]
        offset.append(max(-MAX_POSE_FIX_M, min(MAX_POSE_FIX_M, delta)))
    return offset[0], offset[1]


def _fit_slope(points):
    """(lateral, along) 점들의 직선 기울기 d(along)/d(lateral). 점이 부족하면 None."""
    if len(points) < YAW_FIT_MIN_POINTS:
        return None
    lats = [p[0] for p in points]
    if max(lats) - min(lats) < YAW_FIT_MIN_SPAN_M:
        return None
    mean_l = sum(lats) / len(lats)
    mean_a = sum(p[1] for p in points) / len(points)
    var = sum((lat - mean_l) ** 2 for lat in lats)
    cov = sum((lat - mean_l) * (a - mean_a) for lat, a in points)
    slope = cov / var
    return slope, mean_a - slope * mean_l


def wall_yaw_offset(scan, robot_pose):
    """
    칸 가운데에서 가까운 벽 면의 기울기로 방향 오차를 잰다.

    상하좌우 ±30도 레이저 중 0.10~0.32 m에 맞은 점(칸 경계의 벽 면)에 직선을 맞춘다.
    추정 방향이 실제보다 e 작으면 벽 직선의 기울기가 tan(e)로 보인다.
    반환: 방향 보정량 (rad, 실제 = 추정 + 보정). 벽이 안 보이면 0.
    """
    robot_x, robot_y, robot_yaw = robot_pose
    if point_to_cell(robot_x, robot_y) is None:
        return 0.0
    cos_r = math.cos(robot_yaw)
    sin_r = math.sin(robot_yaw)
    laser_x = robot_x + LASER_OFFSET_X_M * cos_r - LASER_OFFSET_Y_M * sin_r
    laser_y = robot_y + LASER_OFFSET_X_M * sin_r + LASER_OFFSET_Y_M * cos_r
    total = 0.0
    weight = 0
    for phi in (0.0, math.pi / 2, math.pi, -math.pi / 2):
        cos_p = math.cos(phi)
        sin_p = math.sin(phi)
        points = []
        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if not scan.range_min <= distance <= scan.range_max:
                continue
            angle = robot_yaw + LASER_YAW_RAD + scan.angle_min + i * scan.angle_increment
            if abs(math.atan2(math.sin(angle - phi), math.cos(angle - phi))) > YAW_FIT_CONE_RAD:
                continue
            rel_x = laser_x + distance * math.cos(angle) - robot_x
            rel_y = laser_y + distance * math.sin(angle) - robot_y
            along = rel_x * cos_p + rel_y * sin_p
            lateral = -rel_x * sin_p + rel_y * cos_p
            if WALL_FIT_MIN_M <= along <= WALL_FIT_MAX_M:
                points.append((lateral, along))
        fit = _fit_slope(points)
        if fit is None:
            continue
        slope, intercept = fit
        points = [(lat, a) for lat, a in points
                  if abs(a - (slope * lat + intercept)) <= YAW_FIT_OUTLIER_M]
        fit = _fit_slope(points)
        if fit is None:
            continue
        total += math.atan(fit[0]) * len(points)
        weight += len(points)
    if weight == 0:
        return 0.0
    return max(-MAX_YAW_FIX_RAD, min(MAX_YAW_FIX_RAD, total / weight))


def obstacle_ahead(scan, robot_pose):
    """집게 끝 바로 앞(GUARD_AHEAD_M, 차체 폭 + GUARD_SIDE_M)에 장애물 점이 있는지."""
    robot_x, robot_y, robot_yaw = robot_pose
    cos_r = math.cos(robot_yaw)
    sin_r = math.sin(robot_yaw)
    laser_x = robot_x + LASER_OFFSET_X_M * cos_r - LASER_OFFSET_Y_M * sin_r
    laser_y = robot_y + LASER_OFFSET_X_M * sin_r + LASER_OFFSET_Y_M * cos_r
    hits = 0
    for i, distance in enumerate(scan.ranges):
        if not math.isfinite(distance):
            continue
        if not scan.range_min <= distance <= scan.range_max:
            continue
        angle = robot_yaw + LASER_YAW_RAD + scan.angle_min + i * scan.angle_increment
        rel_x = laser_x + distance * math.cos(angle) - robot_x
        rel_y = laser_y + distance * math.sin(angle) - robot_y
        along = rel_x * cos_r + rel_y * sin_r
        lateral = -rel_x * sin_r + rel_y * cos_r
        if (BODY_FRONT_M <= along <= BODY_FRONT_M + GUARD_AHEAD_M
                and abs(lateral) <= BODY_HALF_WIDTH_M + GUARD_SIDE_M):
            hits += 1
    return hits >= GUARD_MIN_POINTS


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
