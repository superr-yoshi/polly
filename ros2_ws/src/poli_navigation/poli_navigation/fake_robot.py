import math
import signal

from geometry_msgs.msg import TransformStamped, Twist, Vector3
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
from poli_navigation.sim_world import (
    sim_camera,
    simulate_empty_arena_scan,
    simulate_scan,
)
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
# 밀림도 흉내 낸다: /sim/push(경기장 좌표 dx, dy m)를 받으면 실제 위치만
# 옮기고 odom은 그대로 둔다. (바퀴가 돌지 않고 밀린 경우)
#   예: ros2 topic pub --once /sim/push geometry_msgs/msg/Vector3 "{x: 0.5}"
# fake_odom과 같은 토픽을 보내므로 동시에 실행하지 않는다.
SIM_ONLY_ODOM_FRAME_ID = 'odom'
SIM_ONLY_BASE_FRAME_ID = 'base_link'
SIM_ONLY_PUBLISH_PERIOD = 0.05

# SIM_ONLY: "grab"을 받고 잡았다고 알릴 때까지 걸리는 시간 (초)
# 조원 A 집게가 완전히 닫히는 시간(약 1.2초)에 맞춘다.
# GRASP_WAIT_S(1.5초)보다 짧아야 GRASP -> HOLD로 넘어간다.
SIM_ONLY_GRAB_TIME = 1.2

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


def arena_vector_to_odom(dx, dy, start_pose=START_POSE):
    """경기장 좌표의 이동량 -> odom 좌표의 이동량 (방향만 바꾼다)."""
    start_yaw = start_pose[2]
    cos_s = math.cos(start_yaw)
    sin_s = math.sin(start_yaw)
    return dx * cos_s + dy * sin_s, -dx * sin_s + dy * cos_s


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
        # SIM_ONLY: 밀려서 생긴 odom과 실제 위치의 차이 (odom 좌표)
        self.push_x = 0.0
        self.push_y = 0.0
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

        self.create_subscription(Vector3, '/sim/push', self.push_callback, 10)

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

    def push_callback(self, msg):
        push_x, push_y = arena_vector_to_odom(msg.x, msg.y, self.start_pose)
        self.push_x += push_x
        self.push_y += push_y
        self.get_logger().info(
            f'Pushed ({msg.x:.2f}, {msg.y:.2f}) m, odom does not know'
        )

    def true_xy(self):
        """밀린 것까지 포함한 실제 위치 (odom 좌표)."""
        return self.x + self.push_x, self.y + self.push_y

    def gripper_callback(self, msg):
        if msg.data == GRIPPER_GRAB:
            if self.grab_started_at is not None:
                # 이미 닫는 중이거나 닫혀 있다. (임무 노드가 명령을 다시 보냄)
                # Mega도 같은 명령은 목표 각도만 다시 정하므로 아무 일도 없다.
                return
            x, y = self.true_xy()
            distance = math.hypot(self.target[0] - x, self.target[1] - y)
            if distance > SIM_ONLY_GRAB_REACH_M:
                # 집게는 닫히지만 잡은 것이 없다.
                self.grab_started_at = None
                self.get_logger().info(
                    f'Gripper: grab missed (target {distance:.2f} m away)'
                )
                return
            self.grab_started_at = self.now_seconds()
        elif msg.data == GRIPPER_OPEN:
            if self.grab_started_at is None:
                return
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

        x, y = self.true_xy()
        if holding:
            self.target = (
                x + SIM_ONLY_DRAG_OFFSET_M * math.cos(self.yaw),
                y + SIM_ONLY_DRAG_OFFSET_M * math.sin(self.yaw),
            )

        detected, x_offset, area = sim_camera(
            x, y, self.yaw, self.target
        )
        self.vision_publisher.publish(
            TargetDetection(detected=detected, x_offset=x_offset, area=area)
        )

    def publish_scan(self):
        pose = odom_to_arena_pose(*self.true_xy(), self.yaw, self.start_pose)
        if self.mission == 1:
            scan = simulate_scan(self.true_grid, pose)
        else:
            # 임무 2는 장애물이 없으므로 빠른 계산을 쓴다.
            # (광선 계산은 무거워서 odom, 집게 타이머를 늦춘다)
            scan = simulate_empty_arena_scan(pose)

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
        # Ctrl+C는 터미널과 launch에서 두 번 올 수 있다.
        # 정리하는 도중에 두 번째 신호로 끊기지 않게 이후 신호는 무시한다.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
