from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


# SIM_ONLY: 실제 엔코더 없이 테스트하기 위한 가짜 값이다.
# 로봇이 움직이지 않고 출발점(0, 0)에 계속 있다고 가정한다.
# 실제 /odom_raw는 조원 A의 RRC Lite 엔코더 드라이버가 제공한다.
SIM_ONLY_ODOM_FRAME_ID = 'odom'
SIM_ONLY_BASE_FRAME_ID = 'base_link'
SIM_ONLY_PUBLISH_PERIOD = 0.05


class FakeOdom(Node):

    def __init__(self):
        super().__init__('fake_odom')

        self.odom_publisher = self.create_publisher(Odometry, '/odom_raw', 10)

        # SIM_ONLY: 실제 로봇에서는 odom -> base_link TF를
        # robot_localization이 보낸다. 이 노드와 동시에 실행하지 않는다.
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(
            SIM_ONLY_PUBLISH_PERIOD,
            self.publish_odom
        )

        self.get_logger().info('POLI Fake Odom Started (SIM_ONLY)')

    def publish_odom(self):
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = SIM_ONLY_ODOM_FRAME_ID
        odom.child_frame_id = SIM_ONLY_BASE_FRAME_ID

        # 위치 (0, 0, 0), 방향 회전 없음, 속도 0
        odom.pose.pose.orientation.w = 1.0

        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = SIM_ONLY_ODOM_FRAME_ID
        transform.child_frame_id = SIM_ONLY_BASE_FRAME_ID
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)

    node = FakeOdom()

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
