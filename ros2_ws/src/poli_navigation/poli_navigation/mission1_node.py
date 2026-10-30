import math
import signal

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from poli_interfaces.msg import TargetDetection
from poli_navigation.laser_tf import LaserMountFromTf
from poli_navigation.mission1_logic import (
    Mission1Logic,
    Observation,
    SCAN,
    SCAN_SETTLE_S,
    STUCK,
)
from poli_navigation.mission2_node import (
    raise_keyboard_interrupt,
    VISION_TIMEOUT,
    yaw_from_quaternion,
)
from poli_navigation.scan_to_grid import odom_to_arena, START_POSE, wall_offset
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String


# 판단 주기 (초)
CONTROL_PERIOD = 0.05

# 스캔이 이 시간(초) 넘게 오지 않으면 "스캔 없음"으로 보고 SCAN에서 기다린다.
# 판단 로직은 멈추고 SCAN_SETTLE_S(0.3초) 뒤의 스캔을 쓰므로,
# 이보다 짧게 두어야 멈추기 전에 찍힌 스캔을 쓰지 않는다.
# TODO_MEASURE: RPLIDAR C1 실제 전송 주기 (보통 10Hz)
SCAN_TIMEOUT = 0.2
ODOM_TIMEOUT = 0.3  # s, 엔코더 위치 갱신이 끊기면 주행을 멈춘다.

# 마지막 집게 명령을 이 주기(초)로 다시 보낸다.
# 조원 A 노드가 임무 노드보다 늦게 켜지면 처음 보낸 명령을 받지 못하기 때문이다.
# Mega는 같은 명령을 다시 받아도 목표 각도만 다시 정하므로 안전하다.
GRIPPER_RESEND_PERIOD = 1.0


class Mission1Node(Node):

    def __init__(self):
        super().__init__('mission1')

        self.logic = Mission1Logic()
        self.pose = None
        self.odom_received_at = None
        self.require_laser_tf = self.declare_parameter('require_laser_tf', False).value
        # 방향은 IMU 자이로(제자리 회전 때 바퀴가 미끄러져도 맞음), 거리는 엔코더로 위치를 계산한다.
        # 2026-10-31 실기: 엔코더만 쓰면 90도 회전 때 실제로 더 돌아 사선으로 주행했다.
        # IMU가 0.3초 넘게 안 오면 엔코더 방향을 쓴다 (시뮬레이션·fake도 그대로 동작).
        self.use_imu_yaw = self.declare_parameter('use_imu_yaw', True).value
        # 2026-10-31 실측: 바닥에서 손으로 반시계 약 45도 -> /imu/data z 적분 -51도 -> 부호 반대(-1).
        # (앞선 손 시험의 +98도는 시계 방향으로 돌린 것으로 판단). 보드를 다시 달면 확인할 것
        self.imu_yaw_sign = self.declare_parameter('imu_yaw_sign', -1.0).value
        self.imu_dyaw = 0.0
        self.imu_stamp = None
        self.imu_received_at = None
        self.prev_odom = None
        self.fused = None
        self.wall_fixed_for = None
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
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 50)
        self.create_subscription(
            Bool, '/gripper/holding', self.holding_callback, 10
        )
        self.create_subscription(
            TargetDetection, '/vision/target', self.vision_callback, 10
        )

        # LiDAR 장착 위치는 TF(base_link -> laser)에서 읽는다. (laser_tf.py)
        self.laser_mount = LaserMountFromTf(self)

        self.create_timer(CONTROL_PERIOD, self.control_step)

        self.get_logger().info('POLI Mission 1 Started: poli_maze 한 칸 탐색 + 카메라 집기')

    def imu_callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.imu_stamp is not None and 0.0 < stamp - self.imu_stamp < 0.2:
            self.imu_dyaw += self.imu_yaw_sign * msg.angular_velocity.z * (stamp - self.imu_stamp)
        self.imu_stamp = stamp
        self.imu_received_at = self.now_seconds()

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        now = self.now_seconds()
        if self.fused is None:
            self.fused = [position.x, position.y, odom_yaw]
        else:
            prev_x, prev_y, prev_yaw = self.prev_odom
            # 엔코더가 잰 앞뒤 이동 거리 (후진이면 음수)
            ds = ((position.x - prev_x) * math.cos(prev_yaw)
                  + (position.y - prev_y) * math.sin(prev_yaw))
            imu_ok = (self.use_imu_yaw and self.imu_received_at is not None
                      and now - self.imu_received_at < 0.3)
            if imu_ok:
                dyaw = self.imu_dyaw
            else:
                dyaw = math.atan2(math.sin(odom_yaw - prev_yaw), math.cos(odom_yaw - prev_yaw))
            mid_yaw = self.fused[2] + dyaw / 2.0
            self.fused[0] += ds * math.cos(mid_yaw)
            self.fused[1] += ds * math.sin(mid_yaw)
            self.fused[2] += dyaw
        self.imu_dyaw = 0.0
        self.prev_odom = (position.x, position.y, odom_yaw)
        self.pose = odom_to_arena(*self.fused)
        self.odom_received_at = now

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

    def fix_pose_with_walls(self, scan, now):
        """칸 가운데에 멈춰 스캔할 때(SCAN) 가까운 벽까지 거리로 위치를 한 번 바로잡는다."""
        logic = self.logic
        if logic.state != SCAN or self.fused is None:
            return
        if self.wall_fixed_for == logic.state_started_at:
            return
        if now - logic.state_started_at < SCAN_SETTLE_S:
            return
        self.wall_fixed_for = logic.state_started_at
        dx, dy = wall_offset(scan, self.pose)
        if dx == 0.0 and dy == 0.0:
            return
        # 경기장 좌표 보정량 -> odom 좌표 (odom_to_arena의 반대 회전)
        start_yaw = START_POSE[2]
        self.fused[0] += dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
        self.fused[1] += -dx * math.sin(start_yaw) + dy * math.cos(start_yaw)
        self.pose = odom_to_arena(*self.fused)
        self.get_logger().info(f'벽 보정: dx={dx:+.3f} m, dy={dy:+.3f} m')

    def control_step(self):
        now = self.now_seconds()
        scan = self.fresh_scan(now)
        if (self.pose is None or self.odom_received_at is None
                or now - self.odom_received_at > ODOM_TIMEOUT
                or scan is None
                or (self.require_laser_tf and not self.laser_mount.ready)):
            self.stop()
            self.get_logger().warn(
                '출발/주행 대기: 최신 odom·scan 및 LiDAR TF 필요',
                throttle_duration_sec=3.0)
            return

        self.fix_pose_with_walls(scan, now)
        x, y, yaw = self.pose

        observation = Observation(
            x=x, y=y, yaw=yaw, now=now,
            scan=scan,
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
            if self.logic.state == STUCK and self.logic.stop_reason is not None:
                self.get_logger().warn(
                    f'{self.logic.stop_reason}; pose=({x:.2f}, {y:.2f}, {yaw:.2f})\n'
                    + self.logic.grid.to_text())

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
        # Ctrl+C는 터미널과 launch에서 두 번 올 수 있다.
        # 정리하는 도중에 두 번째 신호로 끊기지 않게 이후 신호는 무시한다.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)

        # 종료할 때 로봇을 멈춘다.
        node.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
