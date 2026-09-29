"""
SIM_ONLY: 시뮬레이션용 가짜 LiDAR와 가짜 카메라.

ROS2와 무관한 순수 Python 모듈이다.
fake_robot 노드와 테스트에서 같이 사용한다. 실제 로봇에서는 사용하지 않는다.
실제 RPLIDAR C1, AI Camera의 사양이나 측정값이 아니다.
"""

import math
from types import SimpleNamespace

from poli_navigation.grid_map import BLOCKED
from poli_navigation.scan_to_grid import point_to_cell


# SIM_ONLY: 가짜 LiDAR
SIM_ONLY_NUM_RAYS = 360
SIM_ONLY_RAY_STEP_M = 0.005
SIM_ONLY_RANGE_MIN = 0.05
SIM_ONLY_RANGE_MAX = 12.0

# SIM_ONLY: 가짜 카메라 (TODO_MEASURE: 실제 화각, 면적 값)
SIM_ONLY_CAMERA_HALF_FOV_RAD = math.radians(30.0)
SIM_ONLY_CAMERA_MAX_RANGE_M = 2.0
SIM_ONLY_AREA_SCALE = 937.5  # 거리 0.25m에서 면적 15000


def simulate_scan(true_grid, pose):
    """
    정답 지도에서 광선을 쏴서 가짜 LaserScan을 만든다.

    pose: 경기장 좌표 (x, y, yaw). 외벽과 막힌 격자에서 광선이 멈춘다.
    """
    x0, y0, yaw = pose
    increment = 2.0 * math.pi / SIM_ONLY_NUM_RAYS
    angle_min = -math.pi + increment / 2.0
    ranges = []

    for i in range(SIM_ONLY_NUM_RAYS):
        angle = yaw + angle_min + i * increment
        dx, dy = math.cos(angle), math.sin(angle)
        distance = 0.0

        while True:
            distance += SIM_ONLY_RAY_STEP_M
            x, y = x0 + dx * distance, y0 + dy * distance
            cell = point_to_cell(x, y)
            if cell is None or true_grid.get(cell) == BLOCKED:
                break

        ranges.append(distance)

    return SimpleNamespace(
        angle_min=angle_min,
        angle_increment=increment,
        range_min=SIM_ONLY_RANGE_MIN,
        range_max=SIM_ONLY_RANGE_MAX,
        ranges=ranges,
    )


def sim_camera(x, y, yaw, target):
    """로봇 위치에서 대상을 봤을 때의 (detected, x_offset, area)."""
    dx = target[0] - x
    dy = target[1] - y
    distance = math.hypot(dx, dy)
    bearing = math.atan2(dy, dx) - yaw
    bearing = math.atan2(math.sin(bearing), math.cos(bearing))

    detected = (
        abs(bearing) < SIM_ONLY_CAMERA_HALF_FOV_RAD
        and distance < SIM_ONLY_CAMERA_MAX_RANGE_M
    )
    if not detected:
        return False, 0.0, 0.0

    # 대상이 왼쪽(bearing > 0)이면 x_offset 음수
    x_offset = -bearing / SIM_ONLY_CAMERA_HALF_FOV_RAD
    area = SIM_ONLY_AREA_SCALE / max(distance, 0.01) ** 2
    return True, x_offset, area
