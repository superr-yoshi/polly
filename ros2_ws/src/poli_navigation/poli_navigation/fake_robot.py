import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


# SIM_ONLY: 실제 로봇 없이 임무 노드를 테스트하기 위한 가짜 로봇이다.
# /cmd_vel 속도대로 움직였다고 가정하고 위치를 계산해 /odom_raw로 보낸다.
# 바퀴 미끄러짐, 가속 시간 등은 없다. 실제 /odom_raw는 조원 A가 제공한다.
# fake_odom과 같은 토픽을 보내므로 동시에 실행하지 않는다.
SIM_ONLY_ODOM_FRAME_ID = 'odom'
SIM_ONLY_BASE_FRAME_ID = 'base_link'
SIM_ONLY_PUBLISH_PERIOD = 0.05


def integrate_pose(x, y, yaw, linear, angular, dt):
    """속도(linear m/s, angular rad/s)로 dt초 움직인 뒤의 위치를 계산한다.

    odom 좌표: 출발 지점이 (0, 0), 출발 시 정면이 +x, 왼쪽이 +y.
    """
    new_x = x + linear * math.cos(yaw) * dt
    new_y = y + linear * math.sin(yaw) * dt
    new_yaw = math.atan2(
        math.sin(yaw + angular * dt),
        math.cos(yaw + angular * dt)
    )
    return new_x, new_y, new_yaw


class FakeRobot(Node):

    def __init__(self):
        super().__init__('fake_robot')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear = 0.0
        self.angular = 0.0

        self.odom_publisher = self.create_publisher(Odometry, '/odom_raw', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # SIM_ONLY: 실제 로봇에서는 odom -> base_link TF를
        # robot_localization이 보낸다. 이 노드와 동시에 실행하지 않는다.
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(
            SIM_ONLY_PUBLISH_PERIOD,
            self.update
        )

        self.get_logger().info('POLI Fake Robot Started (SIM_ONLY)')

    def cmd_vel_callback(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z

    def update(self):
        self.x, self.y, self.yaw = integrate_pose(
            self.x, self.y, self.yaw,
            self.linear, self.angular,
            SIM_ONLY_PUBLISH_PERIOD
        )
        self.publish_odom()

    def publish_odom(self):
        now = self.get_clock().now().to_msg()

        # yaw(z축 회전)만 있는 쿼터니언
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = SIM_ONLY_ODOM_FRAME_ID
        odom.child_frame_id = SIM_ONLY_BASE_FRAME_ID
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.linear
        odom.twist.twist.angular.z = self.angular

        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = SIM_ONLY_ODOM_FRAME_ID
        transform.child_frame_id = SIM_ONLY_BASE_FRAME_ID
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)

    node = FakeRobot()

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
