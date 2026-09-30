"""calibration 단위 테스트."""
from poli_hardware.calibration import (
    corrected_ticks_per_rev, corrected_wheel_radius, corrected_wheel_separation)
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
