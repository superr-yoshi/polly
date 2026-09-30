import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# 방향 구역 (도, ROS 기준: 0 = 앞, +90 = 왼쪽, -90 = 오른쪽)
# TODO: 실제 주행 테스트 후 구역 폭 조정
FRONT_SECTOR = (-45.0, 45.0)
LEFT_SECTOR = (45.0, 135.0)
RIGHT_SECTOR = (-135.0, -45.0)


def is_valid_range(msg, distance):
    return (
        math.isfinite(distance)
        and msg.range_min <= distance <= msg.range_max
    )


def min_distance_in_sector(msg, sector):
    sector_min, sector_max = sector
    distances = []

    for i, distance in enumerate(msg.ranges):
        angle_deg = math.degrees(msg.angle_min + i * msg.angle_increment)

        # 경계 각도는 제외해 인접 구역 값이 섞이지 않게 한다.
        if sector_min < angle_deg < sector_max \
                and is_valid_range(msg, distance):
            distances.append(distance)

    if not distances:
        return None

    return min(distances)


def format_distance(distance):
    if distance is None:
        return '---'

    return f'{distance:.2f} m'


class ScanMonitor(Node):

    def __init__(self):
        super().__init__('scan_monitor')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('POLI Scan Monitor Started')

    def scan_callback(self, msg):
        valid_ranges = [
            distance
            for distance in msg.ranges
            if is_valid_range(msg, distance)
        ]

        if not valid_ranges:
            self.get_logger().warn('No valid LiDAR data')
            return

        min_distance = min(valid_ranges)
        front = min_distance_in_sector(msg, FRONT_SECTOR)
        left = min_distance_in_sector(msg, LEFT_SECTOR)
        right = min_distance_in_sector(msg, RIGHT_SECTOR)

        self.get_logger().info(
            f'Nearest: {min_distance:.2f} m | '
            f'Front: {format_distance(front)} | '
            f'Left: {format_distance(left)} | '
            f'Right: {format_distance(right)}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = ScanMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
