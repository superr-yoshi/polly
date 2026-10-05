"""
TF(base_link -> laser)에서 LiDAR 장착 위치를 읽어 scan_to_grid에 넣는다.

장착값은 poli_description URDF 한 곳에서만 고치고, 임무 노드는 TF로 받아 쓴다.
URDF는 poli_hardware/launch/hardware.launch.py가 robot_state_publisher로 발행한다.
TF가 없으면(시뮬레이션) scan_to_grid.py의 기본값을 그대로 쓴다.
"""

import math

from poli_navigation.scan_to_grid import set_laser_mount
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


BASE_FRAME = 'base_link'
LASER_FRAME = 'laser'

# TF를 다시 찾아보는 주기 (초)
LOOKUP_PERIOD = 0.5

# 이 시간(초) 안에 TF를 못 찾으면 기본값을 쓴다고 한 번 경고한다. (계속 찾기는 한다)
WARN_AFTER = 10.0


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class LaserMountFromTf:
    """노드에 붙여서 TF를 찾을 때까지 주기적으로 확인하고, 찾으면 한 번만 적용한다."""

    def __init__(self, node):
        self.node = node
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.started_at = node.get_clock().now()
        self.warned = False
        self.timer = node.create_timer(LOOKUP_PERIOD, self.try_lookup)

    def try_lookup(self):
        try:
            transform = self.buffer.lookup_transform(
                BASE_FRAME, LASER_FRAME, Time()
            )
        except TransformException:
            elapsed = (
                self.node.get_clock().now() - self.started_at
            ).nanoseconds / 1e9
            if elapsed > WARN_AFTER and not self.warned:
                self.node.get_logger().warn(
                    f'No TF {BASE_FRAME} -> {LASER_FRAME}, '
                    'using default LiDAR mount (scan_to_grid.py)'
                )
                self.warned = True
            return

        translation = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        set_laser_mount(translation.x, translation.y, yaw)
        self.node.get_logger().info(
            f'LiDAR mount from TF: x={translation.x:.3f} m, '
            f'y={translation.y:.3f} m, yaw={math.degrees(yaw):.1f} deg'
        )
        self.timer.cancel()
