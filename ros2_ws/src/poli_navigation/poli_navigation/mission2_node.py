import math
import signal

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from poli_interfaces.msg import TargetDetection
from poli_navigation.mission2_logic import (
    Mission2Logic,
    Observation,
    START_POSE,
)


# 판단 주기 (초)
CONTROL_PERIOD = 0.05

# 카메라 결과가 이 시간(초) 넘게 오지 않으면 "대상을 못 찾음"으로 본다.
# 카메라가 멈췄는데 예전 값을 믿고 움직이지 않기 위해서다.
# TODO: 조원 B의 전송 주기가 정해지면 조정
VISION_TIMEOUT = 0.5


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def odom_to_arena(odom_x, odom_y, odom_yaw):
    """odom 좌표(출발 지점 기준, x = 출발 시 정면) -> 임무 2 경기장 좌표."""
    start_x, start_y, start_yaw = START_POSE
    cos_s = math.cos(start_yaw)
    sin_s = math.sin(start_yaw)

    return (
        start_x + odom_x * cos_s - odom_y * sin_s,
        start_y + odom_x * sin_s + odom_y * cos_s,
        start_yaw + odom_yaw,
    )


class Mission2Node(Node):

    def __init__(self):
        super().__init__('mission2')

        self.logic = Mission2Logic()
        self.pose = None
        self.holding = None
        self.vision = None
        self.vision_received_at = None
        self.last_state = None

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_publisher = self.create_publisher(
            String, '/gripper/command', 10
        )

        # TODO: 실제 로봇에서는 /odometry/filtered (엔코더 + IMU) 사용 검토
        self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)
        self.create_subscription(
            Bool, '/gripper/holding', self.holding_callback, 10
        )

        self.create_subscription(
            TargetDetection, '/vision/target', self.vision_callback, 10
        )

        self.create_timer(CONTROL_PERIOD, self.control_step)

        self.get_logger().info('POLI Mission 2 Started')

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose = odom_to_arena(position.x, position.y, yaw)

    def holding_callback(self, msg):
        self.holding = msg.data

    def vision_callback(self, msg):
        self.vision = msg
        self.vision_received_at = self.now_seconds()

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def fresh_vision(self, now):
        """최근 카메라 결과. 없거나 오래됐으면 None."""
        if self.vision_received_at is None:
            return None
        if now - self.vision_received_at > VISION_TIMEOUT:
            return None
        return self.vision

    def control_step(self):
        if self.pose is None:
            # 위치를 아직 모르면 움직이지 않는다.
            return

        x, y, yaw = self.pose
        now = self.now_seconds()

        observation = Observation(
            x=x, y=y, yaw=yaw, now=now, holding=self.holding
        )

        vision = self.fresh_vision(now)
        if vision is not None:
            observation.detected = vision.detected
            observation.x_offset = vision.x_offset
            observation.area = vision.area

        command = self.logic.step(observation)

        twist = Twist()
        twist.linear.x = command.linear
        twist.angular.z = command.angular
        self.cmd_vel_publisher.publish(twist)

        if command.gripper is not None:
            self.gripper_publisher.publish(String(data=command.gripper))
            self.get_logger().info(f'Gripper: {command.gripper}')

        if self.logic.state != self.last_state:
            self.get_logger().info(f'State: {self.logic.state}')
            self.last_state = self.logic.state

    def stop(self):
        self.cmd_vel_publisher.publish(Twist())


def raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # 종료 신호(Ctrl+C, kill)를 rclpy가 받으면 통신을 먼저 끊어서
    # 아래 finally에서 정지 명령을 보낼 수 없다.
    # 그래서 신호는 직접 받고, 정지 명령을 보낸 뒤에 종료한다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, raise_keyboard_interrupt)

    node = Mission2Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료할 때 로봇을 멈춘다.
        node.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
