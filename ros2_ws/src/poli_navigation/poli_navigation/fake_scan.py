import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# SIM_ONLY: 실제 LiDAR 없이 테스트하기 위한 가짜 값이다.
# 실제 RPLIDAR C1 사양이나 측정값이 아니다.
SIM_ONLY_FRAME_ID = 'laser'
SIM_ONLY_NUM_SAMPLES = 360
SIM_ONLY_RANGE_MIN = 0.15
SIM_ONLY_RANGE_MAX = 12.0
SIM_ONLY_PUBLISH_PERIOD = 0.1

# SIM_ONLY: 방향별 가짜 거리 (m)
# ROS 기준 각도: 0 = 앞, +90도 = 왼쪽, -90도 = 오른쪽, ±180도 = 뒤
SIM_ONLY_FRONT_DISTANCE = 0.5
SIM_ONLY_LEFT_DISTANCE = 1.0
SIM_ONLY_RIGHT_DISTANCE = 2.0
SIM_ONLY_REAR_DISTANCE = 1.5


def sim_only_distance_at(angle):
    angle_deg = math.degrees(angle)

    if -45.0 <= angle_deg <= 45.0:
        return SIM_ONLY_FRONT_DISTANCE
    if 45.0 < angle_deg <= 135.0:
        return SIM_ONLY_LEFT_DISTANCE
    if -135.0 <= angle_deg < -45.0:
        return SIM_ONLY_RIGHT_DISTANCE
    return SIM_ONLY_REAR_DISTANCE


class FakeScan(Node):

    def __init__(self):
        super().__init__('fake_scan')

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.timer = self.create_timer(
            SIM_ONLY_PUBLISH_PERIOD,
            self.publish_scan
        )

        self.get_logger().info('POLI Fake Scan Started (SIM_ONLY)')

    def publish_scan(self):
        msg = LaserScan()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = SIM_ONLY_FRAME_ID

        # 측정 방향을 반 칸씩 옮겨 구역 경계(±45, ±135도)에 걸리는 값이 없게 한다.
        # 경계에 걸리면 float32 전송 오차로 옆 구역 값이 섞인다.
        msg.angle_increment = 2.0 * math.pi / SIM_ONLY_NUM_SAMPLES
        msg.angle_min = -math.pi + msg.angle_increment / 2.0
        msg.angle_max = math.pi - msg.angle_increment / 2.0
        msg.scan_time = SIM_ONLY_PUBLISH_PERIOD
        msg.time_increment = SIM_ONLY_PUBLISH_PERIOD / SIM_ONLY_NUM_SAMPLES

        msg.range_min = SIM_ONLY_RANGE_MIN
        msg.range_max = SIM_ONLY_RANGE_MAX
        msg.ranges = [
            sim_only_distance_at(msg.angle_min + i * msg.angle_increment)
            for i in range(SIM_ONLY_NUM_SAMPLES)
        ]

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = FakeScan()

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
