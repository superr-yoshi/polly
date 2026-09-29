import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

from poli_interfaces.msg import TargetDetection
from poli_navigation.mission2_logic import (
    CENTER_M,
    GRIPPER_GRAB,
    GRIPPER_OPEN,
    START_POSE,
)


# SIM_ONLY: 실제 로봇 없이 임무 노드를 테스트하기 위한 가짜 로봇이다.
# /cmd_vel 속도대로 움직였다고 가정하고 위치를 계산해 /odom_raw로 보낸다.
# 바퀴 미끄러짐, 가속 시간 등은 없다. 실제 /odom_raw는 조원 A가 제공한다.
# 집게도 흉내 낸다: /gripper/command를 받아 /gripper/holding을 보낸다.
# 카메라도 흉내 낸다: 경기장 중앙의 빨간 대상을 보고 /vision/target을 보낸다.
# fake_odom과 같은 토픽을 보내므로 동시에 실행하지 않는다.
SIM_ONLY_ODOM_FRAME_ID = 'odom'
SIM_ONLY_BASE_FRAME_ID = 'base_link'
SIM_ONLY_PUBLISH_PERIOD = 0.05

# SIM_ONLY: "grab"을 받고 잡았다고 알릴 때까지 걸리는 시간 (초)
# mission2_logic.GRASP_WAIT_S(1.0초)보다 짧아야 GRASP -> HOLD로 넘어간다.
# 실제 집게 시간이 아니다. (TODO_MEASURE)
SIM_ONLY_GRAB_TIME = 0.5

# SIM_ONLY: 대상이 이 거리 안에 있을 때 "grab"을 받아야 잡는 데 성공한다 (m)
SIM_ONLY_GRAB_REACH_M = 0.3

# SIM_ONLY: 가짜 카메라 (test_mission2_logic.py와 같은 값)
# 실제 카메라 화각, 면적 값이 아니다. (TODO_MEASURE)
SIM_ONLY_CAMERA_HALF_FOV_RAD = math.radians(30.0)
SIM_ONLY_CAMERA_MAX_RANGE_M = 2.0
SIM_ONLY_AREA_SCALE = 937.5  # 거리 0.25m에서 면적 15000


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


def arena_to_odom(x, y):
    """임무 2 경기장 좌표 -> odom 좌표 (mission2_node.odom_to_arena의 반대)."""
    start_x, start_y, start_yaw = START_POSE
    dx = x - start_x
    dy = y - start_y
    cos_s = math.cos(start_yaw)
    sin_s = math.sin(start_yaw)

    return (
        dx * cos_s + dy * sin_s,
        -dx * sin_s + dy * cos_s,
    )


def sim_camera(x, y, yaw, target):
    """로봇 위치에서 대상을 봤을 때의 (detected, x_offset, area)."""
    dx = target[0] - x
    dy = target[1] - y
    distance = math.hypot(dx, dy)
    bearing = math.atan2(dy, dx) - yaw
    bearing = math.atan2(math.sin(bearing), math.cos(bearing))

    detected = (
        abs(bearing) < SIM_ONLY_CAMERA_HALF_FOV_RAD
        and distance < SIM_ONLY_CAMERA_MAX_RANGE_M
    )
    if not detected:
        return False, 0.0, 0.0

    # 대상이 왼쪽(bearing > 0)이면 x_offset 음수
    x_offset = -bearing / SIM_ONLY_CAMERA_HALF_FOV_RAD
    area = SIM_ONLY_AREA_SCALE / max(distance, 0.01) ** 2
    return True, x_offset, area


def is_holding(grab_started_at, now):
    """집게가 물체를 잡고 있는지. grab_started_at이 None이면 열린 상태."""
    if grab_started_at is None:
        return False
    return now - grab_started_at >= SIM_ONLY_GRAB_TIME


class FakeRobot(Node):

    def __init__(self):
        super().__init__('fake_robot')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear = 0.0
        self.angular = 0.0
        self.grab_started_at = None

        # SIM_ONLY: 빨간 대상은 경기장 중앙에 가만히 있다고 가정한다.
        self.target = arena_to_odom(*CENTER_M)

        self.odom_publisher = self.create_publisher(Odometry, '/odom_raw', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.holding_publisher = self.create_publisher(
            Bool, '/gripper/holding', 10
        )
        self.create_subscription(
            String, '/gripper/command', self.gripper_callback, 10
        )

        self.vision_publisher = self.create_publisher(
            TargetDetection, '/vision/target', 10
        )

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

    def gripper_callback(self, msg):
        if msg.data == GRIPPER_GRAB:
            distance = math.hypot(
                self.target[0] - self.x, self.target[1] - self.y
            )
            if distance > SIM_ONLY_GRAB_REACH_M:
                # 집게는 닫히지만 잡은 것이 없다.
                self.grab_started_at = None
                self.get_logger().info(
                    f'Gripper: grab missed (target {distance:.2f} m away)'
                )
                return
            self.grab_started_at = self.now_seconds()
        elif msg.data == GRIPPER_OPEN:
            self.grab_started_at = None
        else:
            self.get_logger().warn(f'Unknown gripper command: {msg.data}')
            return

        self.get_logger().info(f'Gripper: {msg.data}')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def update(self):
        self.x, self.y, self.yaw = integrate_pose(
            self.x, self.y, self.yaw,
            self.linear, self.angular,
            SIM_ONLY_PUBLISH_PERIOD
        )
        self.publish_odom()

        holding = is_holding(self.grab_started_at, self.now_seconds())
        self.holding_publisher.publish(Bool(data=holding))

        detected, x_offset, area = sim_camera(
            self.x, self.y, self.yaw, self.target
        )
        self.vision_publisher.publish(
            TargetDetection(detected=detected, x_offset=x_offset, area=area)
        )

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
