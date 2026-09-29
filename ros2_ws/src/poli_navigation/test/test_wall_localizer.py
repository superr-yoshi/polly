import math
from types import SimpleNamespace

from poli_navigation.grid_map import GridMap
from poli_navigation.mission2_logic import (
    ARENA_SIZE_M,
    CENTER_M,
    distance_to_danger_line,
    Observation,
    START_POSE,
)
from poli_navigation.scan_to_grid import point_to_cell
from poli_navigation.sim_world import simulate_scan
from poli_navigation.wall_localizer import (
    estimate_position,
    farthest_cluster,
    find_wall_distances,
    MINUS_X,
    PLUS_X,
    PLUS_Y,
)
import pytest


# 가짜 LiDAR 광선 간격(5mm)보다 조금 크게
TOLERANCE_M = 0.02


def arena(opponents=()):
    """임무 2 경기장. 상대 로봇은 막힌 격자 한 칸으로 흉내 낸다."""
    grid = GridMap(use_point_symmetry=False)
    for cell in opponents:
        grid.mark_blocked(cell)
    return grid


def test_farthest_cluster_ignores_closer_points():
    # 상대 로봇(0.5m) 뒤로 벽(2.0m)이 보인다.
    distances = [0.5] * 20 + [2.0] * 6

    assert farthest_cluster(distances) == pytest.approx(2.0)


def test_farthest_cluster_needs_enough_points():
    assert farthest_cluster([3.0, 3.0, 1.0, 1.0, 1.0, 1.0, 1.0]) == 1.0
    assert farthest_cluster([3.0, 3.0]) is None


def test_wall_distances_at_start():
    x, y, yaw = START_POSE
    scan = simulate_scan(arena(), START_POSE)

    walls = find_wall_distances(scan, yaw)

    assert walls[PLUS_X] == pytest.approx(ARENA_SIZE_M - x, abs=TOLERANCE_M)
    assert walls[MINUS_X] == pytest.approx(x, abs=TOLERANCE_M)
    assert walls[PLUS_Y] == pytest.approx(ARENA_SIZE_M - y, abs=TOLERANCE_M)


def test_estimate_at_start():
    scan = simulate_scan(arena(), START_POSE)

    x, y = estimate_position(scan, START_POSE)

    assert x == pytest.approx(START_POSE[0], abs=TOLERANCE_M)
    assert y == pytest.approx(START_POSE[1], abs=TOLERANCE_M)


def test_estimate_works_when_robot_is_rotated():
    pose = (1.3, 2.1, math.radians(37.0))
    scan = simulate_scan(arena(), pose)

    x, y = estimate_position(scan, pose)

    assert x == pytest.approx(1.3, abs=TOLERANCE_M)
    assert y == pytest.approx(2.1, abs=TOLERANCE_M)


def test_estimate_finds_position_after_being_pushed():
    # odom은 중앙이라고 알고 있지만 실제로는 오른쪽으로 0.25m 밀렸다.
    true_pose = (CENTER_M[0] + 0.25, CENTER_M[1], 0.0)
    odom_pose = (CENTER_M[0], CENTER_M[1], 0.0)
    scan = simulate_scan(arena(), true_pose)

    x, y = estimate_position(scan, odom_pose)

    assert x == pytest.approx(true_pose[0], abs=TOLERANCE_M)
    assert y == pytest.approx(true_pose[1], abs=TOLERANCE_M)


def test_estimate_with_opponent_next_to_robot():
    # 상대 로봇이 바로 왼쪽(-x) 격자에 붙어 있어 왼쪽 벽이 가려진다.
    pose = (*CENTER_M, 0.0)
    robot_cell = point_to_cell(*CENTER_M)
    opponent = (robot_cell[0] - 1, robot_cell[1])
    scan = simulate_scan(arena([opponent]), pose)

    x, y = estimate_position(scan, pose)

    assert x == pytest.approx(CENTER_M[0], abs=TOLERANCE_M)
    assert y == pytest.approx(CENTER_M[1], abs=TOLERANCE_M)


def test_pushed_toward_wall_with_opponent_behind():
    # 상대가 왼쪽에서 밀어서 오른쪽 탈락선 근처(x = 3.1)까지 왔다.
    # odom은 0.1m 전 위치를 알고 있다. 상대 로봇 때문에 왼쪽 벽은 안 보인다.
    true_pose = (3.1, CENTER_M[1], 0.0)
    odom_pose = (3.0, CENTER_M[1], 0.0)
    opponent = (7, 5)  # 로봇 바로 왼쪽 격자 (x 2.4 ~ 2.8m)
    scan = simulate_scan(arena([opponent]), true_pose)

    x, _ = estimate_position(scan, odom_pose)

    assert x == pytest.approx(3.1, abs=TOLERANCE_M)
    # 탈락선(3.2m)에 가깝다는 것을 알 수 있어야 한다.
    observation = Observation(x=x, y=CENTER_M[1], yaw=0.0, now=0.0)
    assert distance_to_danger_line(observation) < 0.15


def test_no_valid_ranges_gives_no_estimate():
    scan = SimpleNamespace(
        angle_min=-math.pi,
        angle_increment=2.0 * math.pi / 360,
        range_min=0.05,
        range_max=12.0,
        ranges=[float('inf')] * 360,
    )

    assert estimate_position(scan, START_POSE) == (None, None)


def test_far_jump_from_one_wall_is_ignored():
    # 한쪽 벽만 보일 때, 바로 전 위치에서 너무 먼 값은 믿지 않는다.
    true_pose = (*CENTER_M, 0.0)
    guess = (CENTER_M[0] + 0.8, CENTER_M[1], 0.0)
    scan = simulate_scan(arena(), true_pose)
    # 오른쪽 벽 쪽 점만 남긴다.
    scan.ranges = [
        distance if abs(scan.angle_min + i * scan.angle_increment) < 0.5
        else float('inf')
        for i, distance in enumerate(scan.ranges)
    ]

    x, _ = estimate_position(scan, guess)

    assert x is None


def test_small_yaw_error_gives_small_position_error():
    # IMU 방향이 3도 틀려도 위치 오차는 5cm 이내
    for true_pose in [(*CENTER_M, 0.3), START_POSE, (3.1, CENTER_M[1], 0.0)]:
        scan = simulate_scan(arena(), true_pose)
        guess = (true_pose[0], true_pose[1], true_pose[2] + math.radians(3.0))

        x, y = estimate_position(scan, guess)

        assert x == pytest.approx(true_pose[0], abs=0.05)
        assert y == pytest.approx(true_pose[1], abs=0.05)
