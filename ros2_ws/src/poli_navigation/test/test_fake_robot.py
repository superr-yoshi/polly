import math

from poli_navigation.fake_robot import (
    arena_to_odom,
    integrate_pose,
    is_holding,
    sim_camera,
    SIM_ONLY_AREA_SCALE,
    SIM_ONLY_GRAB_REACH_M,
    SIM_ONLY_GRAB_TIME,
)
from poli_navigation.mission2_logic import (
    CENTER_M,
    GRAB_AREA,
    GRASP_WAIT_S,
    START_POSE,
)
from poli_navigation.mission2_node import odom_to_arena
import pytest


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


def test_center_is_3_cells_forward_3_cells_left():
    # docs/mission_strategy.md: 임무 2 중앙은 앞 3칸, 왼쪽 3칸
    x, y = arena_to_odom(*CENTER_M)

    assert x == pytest.approx(1.2)
    assert y == pytest.approx(1.2)


def test_arena_to_odom_is_reverse_of_odom_to_arena():
    x, y = arena_to_odom(1.0, 2.5)
    arena_x, arena_y, _ = odom_to_arena(x, y, 0.0)

    assert arena_x == pytest.approx(1.0)
    assert arena_y == pytest.approx(2.5)
    assert START_POSE[:2] == pytest.approx(odom_to_arena(0.0, 0.0, 0.0)[:2])


def test_camera_sees_target_in_front():
    detected, x_offset, _ = sim_camera(0.0, 0.0, 0.0, (1.0, 0.0))

    assert detected is True
    assert x_offset == pytest.approx(0.0)


def test_camera_target_on_left_is_negative_offset():
    detected, x_offset, _ = sim_camera(0.0, 0.0, 0.0, (1.0, 0.3))

    assert detected is True
    assert x_offset < 0.0


def test_camera_does_not_see_target_behind_or_far():
    assert sim_camera(0.0, 0.0, 0.0, (-1.0, 0.0))[0] is False
    assert sim_camera(0.0, 0.0, 0.0, (3.0, 0.0))[0] is False


def test_grab_area_is_within_grab_reach():
    # mission2는 area >= GRAB_AREA일 때 grab을 보낸다.
    # 그 거리에서 가짜 집게가 잡을 수 있어야 한다.
    grab_distance = math.sqrt(SIM_ONLY_AREA_SCALE / GRAB_AREA)

    assert grab_distance < SIM_ONLY_GRAB_REACH_M
