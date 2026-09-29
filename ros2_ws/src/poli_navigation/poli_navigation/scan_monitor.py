import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


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
            if math.isfinite(distance)
            and msg.range_min <= distance <= msg.range_max
        ]

        if not valid_ranges:
            self.get_logger().warn('No valid LiDAR data')
            return

        min_distance = min(valid_ranges)

        self.get_logger().info(
            f'Nearest obstacle: {min_distance:.2f} m'
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
