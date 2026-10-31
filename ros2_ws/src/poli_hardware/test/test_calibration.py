"""calibration 단위 테스트."""
from poli_hardware.calibration import (
    corrected_ticks_per_rev, corrected_wheel_radius, corrected_wheel_separation, trapezoid,
    trapezoid_scale)
import pytest


def test_wheel_radius():
    # 1.0 m 명령했는데 0.95 m 갔으면 실제 바퀴가 5% 작다
    assert corrected_wheel_radius(0.0325, 1.0, 0.95) == pytest.approx(0.030875)
    assert corrected_wheel_radius(0.0325, 1.0, 1.0) == pytest.approx(0.0325)


def test_wheel_separation():
    # 360도 명령했는데 330도만 돌았으면 실제 간격이 더 넓다
    assert corrected_wheel_separation(0.18, 360.0, 330.0) == pytest.approx(0.18 * 360 / 330)
    assert corrected_wheel_separation(0.18, -360.0, 360.0) == pytest.approx(0.18)


def test_ticks_per_rev():
    # 10바퀴 명령에 약 9.17바퀴 -> PPR 12 모터 (1320 -> 1440)
    assert corrected_ticks_per_rev(1320.0, 10.0, 1320 / 144) == pytest.approx(1440.0)


def test_rejects_zero():
    with pytest.raises(ValueError):
        corrected_wheel_radius(0.0325, 0.0, 1.0)
    with pytest.raises(ValueError):
        corrected_ticks_per_rev(1320.0, 10.0, 0.0)


def _integrate(peak, total, ramp_time, dt=0.0005):
    steps = int(total / dt) + 1
    return sum(peak * trapezoid_scale(i * dt, total, ramp_time) * dt for i in range(steps))


def test_trapezoid_covers_distance():
    # 1.2 m를 최고 0.8 m/s, 1.2 m/s^2로: 0.67 s 올리고 0.67 s 내린다
    peak, total, ramp_time = trapezoid(1.2, 0.8, 1.2)
    assert peak == pytest.approx(0.8)
    assert ramp_time == pytest.approx(0.8 / 1.2)
    assert _integrate(peak, total, ramp_time) == pytest.approx(1.2, abs=1e-3)


def test_trapezoid_short_is_triangle():
    peak, total, ramp_time = trapezoid(0.2, 0.8, 1.2)
    assert peak < 0.8
    assert total == pytest.approx(2.0 * ramp_time)
    assert _integrate(peak, total, ramp_time) == pytest.approx(0.2, abs=1e-3)


def test_trapezoid_scale_starts_and_ends_at_zero():
    assert trapezoid_scale(0.0, 2.0, 0.5) == 0.0
    assert trapezoid_scale(0.25, 2.0, 0.5) == pytest.approx(0.5)
    assert trapezoid_scale(1.0, 2.0, 0.5) == 1.0
    assert trapezoid_scale(1.75, 2.0, 0.5) == pytest.approx(0.5)
    assert trapezoid_scale(2.0, 2.0, 0.5) == 0.0
