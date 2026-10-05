import signal

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from poli_interfaces.msg import TargetDetection
from poli_navigation.laser_tf import LaserMountFromTf
from poli_navigation.mission1_logic import Mission1Logic, Observation
from poli_navigation.mission2_node import (
    raise_keyboard_interrupt,
    VISION_TIMEOUT,
    yaw_from_quaternion,
)
from poli_navigation.scan_to_grid import odom_to_arena
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


# 판단 주기 (초)
CONTROL_PERIOD = 0.05

# 스캔이 이 시간(초) 넘게 오지 않으면 "스캔 없음"으로 보고 SCAN에서 기다린다.
# 판단 로직은 멈추고 SCAN_SETTLE_S(0.3초) 뒤의 스캔을 쓰므로,
# 이보다 짧게 두어야 멈추기 전에 찍힌 스캔을 쓰지 않는다.
# TODO_MEASURE: RPLIDAR C1 실제 전송 주기 (보통 10Hz)
SCAN_TIMEOUT = 0.2

# 마지막 집게 명령을 이 주기(초)로 다시 보낸다.
# 조원 A 노드가 임무 노드보다 늦게 켜지면 처음 보낸 명령을 받지 못하기 때문이다.
# Mega는 같은 명령을 다시 받아도 목표 각도만 다시 정하므로 안전하다.
GRIPPER_RESEND_PERIOD = 1.0


class Mission1Node(Node):

    def __init__(self):
        super().__init__('mission1')

        self.logic = Mission1Logic()
        self.pose = None
        self.holding = None
        self.vision = None
        self.vision_received_at = None
        self.scan = None
        self.scan_received_at = None
        self.last_state = None
        self.last_phase = None
        self.last_path = None

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_publisher = self.create_publisher(
            String, '/gripper/command', 10
        )
        self.gripper_command = None
        self.create_timer(GRIPPER_RESEND_PERIOD, self.resend_gripper)

        # TODO: 실제 로봇에서는 /odometry/filtered (엔코더 + IMU) 사용 검토
        self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(
            Bool, '/gripper/holding', self.holding_callback, 10
        )
        self.create_subscription(
            TargetDetection, '/vision/target', self.vision_callback, 10
        )

        # LiDAR 장착 위치는 TF(base_link -> laser)에서 읽는다. (laser_tf.py)
        self.laser_mount = LaserMountFromTf(self)

        self.create_timer(CONTROL_PERIOD, self.control_step)

        self.get_logger().info('POLI Mission 1 Started')

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose = odom_to_arena(position.x, position.y, yaw)

    def scan_callback(self, msg):
        self.scan = msg
        self.scan_received_at = self.now_seconds()

    def holding_callback(self, msg):
        self.holding = msg.data

    def vision_callback(self, msg):
        self.vision = msg
        self.vision_received_at = self.now_seconds()

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def fresh_scan(self, now):
        """최근 LiDAR 스캔. 없거나 오래됐으면 None."""
        if self.scan_received_at is None:
            return None
        if now - self.scan_received_at > SCAN_TIMEOUT:
            return None
        return self.scan

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
            x=x, y=y, yaw=yaw, now=now,
            scan=self.fresh_scan(now),
            holding=self.holding,
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
            self.send_gripper(command.gripper)

        if self.logic.phase != self.last_phase:
            self.get_logger().info(f'Phase: {self.logic.phase}')
            self.last_phase = self.logic.phase

        if self.logic.state != self.last_state:
            self.get_logger().info(f'State: {self.logic.state}')
            self.last_state = self.logic.state

        if self.logic.path != self.last_path:
            self.get_logger().info(f'Path: {self.logic.path}')
            self.last_path = self.logic.path

    def send_gripper(self, command):
        self.gripper_command = command
        self.gripper_publisher.publish(String(data=command))
        self.get_logger().info(f'Gripper: {command}')

    def resend_gripper(self):
        if self.gripper_command is not None:
            self.gripper_publisher.publish(String(data=self.gripper_command))

    def stop(self):
        self.cmd_vel_publisher.publish(Twist())


def main(args=None):
    # 종료할 때 정지 명령을 보내기 위해 신호를 직접 받는다. (mission2_node와 같다)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, raise_keyboard_interrupt)

    node = Mission1Node()

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
