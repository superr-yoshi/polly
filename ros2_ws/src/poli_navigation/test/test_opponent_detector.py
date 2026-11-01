import math
from types import SimpleNamespace

from poli_navigation.mission2_logic import ARENA_SIZE_M
from poli_navigation.opponent_detector import find_opponent, find_opponents


RAYS = 720
OPPONENT_RADIUS_M = 0.12


def ray_box_distance(px, py, angle):
    """(px, py)에서 angle 방향으로 경기장 외벽까지 거리."""
    best = math.inf
    dx, dy = math.cos(angle), math.sin(angle)
    if dx > 1e-9:
        best = min(best, (ARENA_SIZE_M - px) / dx)
    if dx < -1e-9:
        best = min(best, -px / dx)
    if dy > 1e-9:
        best = min(best, (ARENA_SIZE_M - py) / dy)
    if dy < -1e-9:
        best = min(best, -py / dy)
    return best


def ray_circle_distance(px, py, angle, center, radius):
    dx, dy = math.cos(angle), math.sin(angle)
    fx, fy = px - center[0], py - center[1]
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - c
    if disc < 0.0:
        return math.inf
    hit = -b - math.sqrt(disc)
    return hit if hit > 0.0 else math.inf


def make_scan(pose, opponents=(), held_cube=False):
    """외벽 + 원기둥 상대 로봇 + (잡은 큐브) 가짜 스캔. LiDAR는 로봇 중심, 정면 기준 (SIM_ONLY)."""
    x, y, yaw = pose
    ranges = []
    for i in range(RAYS):
        local = -math.pi + i * 2.0 * math.pi / RAYS
        angle = yaw + local
        distance = ray_box_distance(x, y, angle)
        for center in opponents:
            distance = min(distance, ray_circle_distance(x, y, angle, center, OPPONENT_RADIUS_M))
        if held_cube and abs(local) < math.radians(12.0):
            distance = min(distance, 0.12)  # 집게 안 큐브
        ranges.append(distance)
    return SimpleNamespace(
        ranges=ranges, angle_min=-math.pi, angle_increment=2.0 * math.pi / RAYS,
        range_min=0.05, range_max=12.0)


def test_no_opponent_only_walls_and_held_cube():
    pose = (1.0, 2.5, 0.7)
    assert find_opponent(make_scan(pose, held_cube=True), pose) is None


def test_finds_opponent_position():
    pose = (2.37, 1.23, math.radians(135.0))
    opponent = (1.6, 1.5)  # 집게 안 큐브에 가리지 않는 방향 (왼쪽 약 25도)
    found = find_opponent(make_scan(pose, [opponent], held_cube=True), pose)
    assert found is not None
    # 보이는 면의 평균이라 중심보다 우리 쪽으로 조금 당겨진다
    assert math.dist(found, opponent) < OPPONENT_RADIUS_M


def test_returns_nearest_of_two_opponents():
    pose = (1.8, 1.0, 0.0)
    near, far = (2.4, 1.4), (1.0, 2.8)
    found = find_opponent(make_scan(pose, [near, far]), pose)
    assert math.dist(found, near) < OPPONENT_RADIUS_M


def test_finds_all_opponents_nearest_first():
    pose = (1.8, 1.0, 0.0)
    near, middle, far = (2.4, 1.4), (1.0, 1.9), (2.6, 3.0)
    found = find_opponents(make_scan(pose, [far, near, middle]), pose)
    assert len(found) == 3
    for spot, expected in zip(found, (near, middle, far)):
        assert math.dist(spot, expected) < OPPONENT_RADIUS_M
