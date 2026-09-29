import math

import pytest

from poli_navigation.fake_robot import (
    integrate_pose,
    is_holding,
    SIM_ONLY_GRAB_TIME,
)
from poli_navigation.mission2_logic import GRASP_WAIT_S


DT = 0.05


def test_stopped_robot_does_not_move():
    assert integrate_pose(1.0, 2.0, 0.5, 0.0, 0.0, DT) == (1.0, 2.0, 0.5)


def test_forward_moves_along_heading():
    # 1초 동안 0.2m/s로 정면(+x) 전진
    x, y, yaw = 0.0, 0.0, 0.0
    for _ in range(20):
        x, y, yaw = integrate_pose(x, y, yaw, 0.2, 0.0, DT)

    assert x == pytest.approx(0.2)
    assert y == pytest.approx(0.0)


def test_forward_while_facing_left():
    # 왼쪽(+90도)을 보고 전진하면 +y로 간다.
    x, y, _ = integrate_pose(0.0, 0.0, math.pi / 2, 1.0, 0.0, 1.0)

    assert x == pytest.approx(0.0)
    assert y == pytest.approx(1.0)


def test_positive_angular_turns_left():
    _, _, yaw = integrate_pose(0.0, 0.0, 0.0, 0.0, 0.5, 1.0)

    assert yaw == pytest.approx(0.5)


def test_yaw_stays_between_minus_pi_and_pi():
    _, _, yaw = integrate_pose(0.0, 0.0, math.radians(170), 0.0, 1.0, 1.0)

    assert -math.pi <= yaw <= math.pi
    assert yaw == pytest.approx(math.radians(170) + 1.0 - 2 * math.pi)


def test_open_gripper_is_not_holding():
    assert is_holding(None, 10.0) is False


def test_grab_takes_time():
    assert is_holding(10.0, 10.0 + SIM_ONLY_GRAB_TIME / 2) is False
    assert is_holding(10.0, 10.0 + SIM_ONLY_GRAB_TIME) is True


def test_grab_finishes_before_mission2_checks():
    # mission2는 grab 후 GRASP_WAIT_S 뒤에 holding을 확인한다.
    assert SIM_ONLY_GRAB_TIME < GRASP_WAIT_S
