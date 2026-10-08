"""diff_drive 단위 테스트. ROS 없이 pytest로 실행."""
import math

from poli_hardware.diff_drive import (
    CmdWatchdog, DiffDriveParams, OdomIntegrator, rad_s_to_rpm,
    twist_to_wheels, wheels_to_twist)
import pytest

P = DiffDriveParams(wheel_radius=0.0325, wheel_separation=0.18,
                    max_linear=0.25, max_angular=1.0, max_wheel_rpm=330.0)


def test_forward_both_wheels_same():
    left, right = twist_to_wheels(P, 0.2, 0.0)
    assert left == pytest.approx(right)
    assert left == pytest.approx(0.2 / 0.0325)


def test_left_turn_right_wheel_faster():
    left, right = twist_to_wheels(P, 0.0, 0.5)
    assert right > 0 > left
    assert left == pytest.approx(-right)


def test_round_trip():
    for v, w in [(0.1, 0.3), (-0.2, 0.0), (0.0, -0.8), (0.25, 1.0)]:
        assert wheels_to_twist(P, *twist_to_wheels(P, v, w)) == pytest.approx((v, w))


def test_speed_limits():
    v, w = wheels_to_twist(P, *twist_to_wheels(P, 5.0, -9.0))
    assert v == pytest.approx(0.25)
    assert w == pytest.approx(-1.0)


def test_rpm_limit_keeps_curvature():
    fast = DiffDriveParams(0.0325, 0.18, 10.0, 50.0, 330.0)
    left, right = twist_to_wheels(fast, 2.0, 5.0)
    assert max(abs(rad_s_to_rpm(left)), abs(rad_s_to_rpm(right))) == pytest.approx(330.0)
    v, w = wheels_to_twist(fast, left, right)
    assert w / v == pytest.approx(5.0 / 2.0)


def test_odom_straight_and_spin():
    o = OdomIntegrator()
    for _ in range(100):
        o.step(0.2, 0.0, 0.04)          # 4초 x 0.2 m/s
    assert (o.x, o.y) == pytest.approx((0.8, 0.0))
    for _ in range(100):
        o.step(0.0, math.pi / 2, 0.04)  # 4초 x 90도/s = 360도
    assert o.yaw == pytest.approx(0.0, abs=1e-9)
    assert o.x == pytest.approx(0.8)


def test_odom_quarter_circle():
    o = OdomIntegrator()
    r, v = 0.5, 0.2
    steps, dt = 1000, (math.pi / 2 * r / v) / 1000
    for _ in range(steps):
        o.step(v, v / r, dt)
    assert (o.x, o.y, o.yaw) == pytest.approx((r, r, math.pi / 2), abs=1e-4)


def test_watchdog():
    wd = CmdWatchdog(0.3)
    assert wd.command(0.0) == (0.0, 0.0)          # 명령 받기 전에는 정지
    wd.update(1.0, 0.2, 0.1)
    assert wd.command(1.2) == (0.2, 0.1)
    assert wd.command(1.31) == (0.0, 0.0)         # timeout 후 정지
    assert wd.expired(1.31)


PMIN = DiffDriveParams(wheel_radius=0.0325, wheel_separation=0.18,
                       max_linear=0.25, max_angular=1.0, max_wheel_rpm=330.0,
                       min_wheel_speed=0.25)


def test_min_speed_raises_slow_forward():
    left, right = twist_to_wheels(PMIN, 0.1, 0.0)
    assert left * PMIN.wheel_radius == pytest.approx(0.25)
    assert right == pytest.approx(left)


def test_min_speed_keeps_curvature_and_sign():
    left, right = twist_to_wheels(PMIN, -0.05, 0.2)
    l0, r0 = twist_to_wheels(P, -0.05, 0.2)
    assert left / right == pytest.approx(l0 / r0)
    assert max(abs(left), abs(right)) * PMIN.wheel_radius == pytest.approx(0.25)


def test_min_speed_zero_stays_zero():
    assert twist_to_wheels(PMIN, 0.0, 0.0) == (0.0, 0.0)
    assert twist_to_wheels(PMIN, 0.001, 0.0) == (0.0, 0.0)


def test_min_speed_off_by_default():
    left, _ = twist_to_wheels(P, 0.1, 0.0)
    assert left * P.wheel_radius == pytest.approx(0.1)


def test_min_speed_does_not_touch_fast_commands():
    assert twist_to_wheels(PMIN, 0.25, 0.0) == twist_to_wheels(P, 0.25, 0.0)
