import math

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from poli_interfaces.msg import TargetDetection
from poli_navigation.grid_map import GridMap, RULE_EXAMPLE_BLOCKED
from poli_navigation.mission2_logic import (
    CENTER_M,
    GRIPPER_GRAB,
    GRIPPER_OPEN,
    START_POSE,
)
from poli_navigation.scan_to_grid import START_POSE as MISSION1_START_POSE
from poli_navigation.sim_world import sim_camera, simulate_scan
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster


# SIM_ONLY: 실제 로봇 없이 임무 노드를 테스트하기 위한 가짜 로봇이다.
# /cmd_vel 속도대로 움직였다고 가정하고 위치를 계산해 /odom_raw로 보낸다.
# 바퀴 미끄러짐, 가속 시간 등은 없다. 실제 /odom_raw는 조원 A가 제공한다.
# 집게도 흉내 낸다: /gripper/command를 받아 /gripper/holding을 보낸다.
# 카메라도 흉내 낸다: 경기장 중앙의 빨간 대상을 보고 /vision/target을 보낸다.
# LiDAR도 흉내 낸다: 외벽(임무 1은 장애물도)까지 거리를 /scan으로 보낸다.
# 파라미터 mission(1 또는 2)으로 출발 위치와 경기장을 고른다.
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

# SIM_ONLY: 잡은 대상은 들지 않고 로봇 중심에서 이 거리 앞에 붙여 끌고 간다 (m)
SIM_ONLY_DRAG_OFFSET_M = 0.25

SIM_ONLY_SCAN_PERIOD = 0.1
SIM_ONLY_SCAN_FRAME_ID = 'base_link'


def integrate_pose(x, y, yaw, linear, angular, dt):
    """
    속도(linear m/s, angular rad/s)로 dt초 움직인 뒤의 위치를 계산한다.

    odom 좌표: 출발 지점이 (0, 0), 출발 시 정면이 +x, 왼쪽이 +y.
    """
    new_x = x + linear * math.cos(yaw) * dt
    new_y = y + linear * math.sin(yaw) * dt
    new_yaw = math.atan2(
        math.sin(yaw + angular * dt),
        math.cos(yaw + angular * dt)
    )
    return new_x, new_y, new_yaw


def arena_to_odom(x, y, start_pose=START_POSE):
    """경기장 좌표 -> odom 좌표 (mission2_node.odom_to_arena의 반대)."""
    start_x, start_y, start_yaw = start_pose
    dx = x - start_x
    dy = y - start_y
    cos_s = math.cos(start_yaw)
    sin_s = math.sin(start_yaw)

    return (
        dx * cos_s + dy * sin_s,
        -dx * sin_s + dy * cos_s,
    )


def odom_to_arena_pose(x, y, yaw, start_pose):
    """Odom 좌표 -> 경기장 좌표 (arena_to_odom의 반대, 방향 포함)."""
    start_x, start_y, start_yaw = start_pose
    cos_s = math.cos(start_yaw)
    sin_s = math.sin(start_yaw)

    return (
        start_x + x * cos_s - y * sin_s,
        start_y + x * sin_s + y * cos_s,
        start_yaw + yaw,
    )


def sim_world_grid(mission):
    """정답 장애물 지도. 임무 1은 규정 그림 1 예시, 임무 2는 장애물 없음."""
    grid = GridMap()
    if mission == 1:
        for cell in RULE_EXAMPLE_BLOCKED:
            grid.mark_blocked(cell)
    return grid


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

        self.mission = self.declare_parameter('mission', 2).value
        if self.mission not in (1, 2):
            raise ValueError(f'mission must be 1 or 2: {self.mission}')
        self.start_pose = (
            MISSION1_START_POSE if self.mission == 1 else START_POSE
        )
        self.true_grid = sim_world_grid(self.mission)

        # SIM_ONLY: 빨간 대상은 처음에 경기장 중앙에 있다.
        # (임무 1, 2 모두 중앙 (5, 5) 격자 가운데)
        self.target = arena_to_odom(*CENTER_M, self.start_pose)

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

        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.create_timer(SIM_ONLY_SCAN_PERIOD, self.publish_scan)

        # SIM_ONLY: 실제 로봇에서는 odom -> base_link TF를
        # robot_localization이 보낸다. 이 노드와 동시에 실행하지 않는다.
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(
            SIM_ONLY_PUBLISH_PERIOD,
            self.update
        )

        self.get_logger().info(
            f'POLI Fake Robot Started (SIM_ONLY, mission {self.mission})'
        )

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

        if holding:
            self.target = (
                self.x + SIM_ONLY_DRAG_OFFSET_M * math.cos(self.yaw),
                self.y + SIM_ONLY_DRAG_OFFSET_M * math.sin(self.yaw),
            )

        detected, x_offset, area = sim_camera(
            self.x, self.y, self.yaw, self.target
        )
        self.vision_publisher.publish(
            TargetDetection(detected=detected, x_offset=x_offset, area=area)
        )

    def publish_scan(self):
        pose = odom_to_arena_pose(self.x, self.y, self.yaw, self.start_pose)
        scan = simulate_scan(self.true_grid, pose)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = SIM_ONLY_SCAN_FRAME_ID
        msg.angle_min = scan.angle_min
        msg.angle_increment = scan.angle_increment
        msg.angle_max = (
            scan.angle_min + (len(scan.ranges) - 1) * scan.angle_increment
        )
        msg.scan_time = SIM_ONLY_SCAN_PERIOD
        msg.range_min = scan.range_min
        msg.range_max = scan.range_max
        msg.ranges = scan.ranges

        self.scan_publisher.publish(msg)

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
