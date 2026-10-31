import math
import os
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
    DEFAULT_START_FILE,
    raise_keyboard_interrupt,
    VISION_TIMEOUT,
    yaw_from_quaternion,
)
from poli_navigation.scan_to_grid import (
    corner_offset,
    odom_to_arena,
    START_POSE,
    wall_offset,
    wall_yaw_offset,
)
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String


# 판단 주기 (초)
CONTROL_PERIOD = 0.05

# 스캔이 이 시간(초) 넘게 오지 않으면 "스캔 없음"으로 보고 SCAN에서 기다린다.
# 수신 시각 외에 첫 광선의 촬영 시각도 검사해 정지 전 스캔을 배제한다.
# TODO_MEASURE: RPLIDAR C1 실제 전송 주기 (보통 10Hz)
SCAN_TIMEOUT = 0.2
# 지도 갱신에 쓰는 스캔의 최대 나이 (첫 광선 촬영 시각 기준).
# sllidar는 스캔 시작 시각을 찍어 받는 순간 이미 약 0.1초(한 바퀴)다. 0.2초로 두면 전송이
# 조금만 늦어도 정지 후 찍은 멀쩡한 스캔을 버리고 다음 스캔을 기다렸다 (2026-10-31 판단 지연).
# 정지 전 스캔은 촬영 시각 조건(정지 + SCAN_SETTLE_S 이후)으로 따로 거른다.
SCAN_MAX_AGE = 0.35
ODOM_TIMEOUT = 0.3  # s, 엔코더 위치 갱신이 끊기면 주행을 멈춘다.
# TODO_MEASURE: 정지 판정 잡음 허용치. 흔들림이 남으면 최소 대기 시간을 다시 센다.
STILL_LINEAR_MPS = 0.02
# 2026-10-31 0.1 -> 0.2: 감속을 세게 한 뒤 작은 흔들림 때문에 정지 확인을 다시 세지 않게
STILL_ANGULAR_RPS = 0.2


def mapping_scan_ready(scan_started_at, now, stop_started_at):
    """정지 안정 시간이 지난 뒤 촬영한 최신 스캔인지 확인한다."""
    return (0.0 <= now - scan_started_at <= SCAN_MAX_AGE
            and scan_started_at >= stop_started_at + SCAN_SETTLE_S)


# 달리면서 찍은 스캔의 위치 보정은 이만큼만 믿는다 (2026-10-31 실기: 연속 두 보정이 옆으로
# -7.6 cm, +9.4 cm로 반대여서 대각선 주행). 서서 찍은 스캔은 다 반영한다. TODO_MEASURE
MOVING_FIX_GAIN = 0.5
MOVING_FIX_MAX_M = 0.03


# 마지막 집게 명령을 이 주기(초)로 다시 보낸다.
# 조원 A 노드가 임무 노드보다 늦게 켜지면 처음 보낸 명령을 받지 못하기 때문이다.
# Mega는 같은 명령을 다시 받아도 목표 각도만 다시 정하므로 안전하다.
GRIPPER_RESEND_PERIOD = 1.0


class Mission1Node(Node):

    def __init__(self):
        super().__init__('mission1')

        # 대회 경기장은 장애물이 점대칭(규정)이라 기본 True. 대칭이 아닌 연습 경기장에서만 false.
        # rolling_decision: 칸마다 서지 않고 주행 중 판단 (기본 끔, 연습 --test에서 켬)
        self.logic = Mission1Logic(
            point_symmetry=self.declare_parameter('point_symmetry', True).value,
            rolling_decision=self.declare_parameter('rolling_decision', False).value)
        # 시작 신호 대기 (mission2_node.py DEFAULT_START_FILE 참고)
        self.wait_for_start = self.declare_parameter('wait_for_start', False).value
        self.start_file = self.declare_parameter('start_file', DEFAULT_START_FILE).value
        self.started = not self.wait_for_start
        self.ready_logged = False
        self.pose = None
        self.odom_received_at = None
        self.last_motion_at = self.now_seconds()
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
        self.gripper_state = None
        self.vision = None
        self.vision_received_at = None
        self.scan = None
        self.scan_received_at = None
        self.last_state = None
        # 칸마다 판단(SCAN)에 걸린 시간을 나눠 보기 위한 SCAN 시작·정지 확인 시각
        self.scan_entered_at = None
        self.scan_stop_started_at = None
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
            String, '/gripper/state', self.gripper_state_callback, 10
        )
        self.create_subscription(
            TargetDetection, '/vision/target', self.vision_callback, 10
        )

        # LiDAR 장착 위치는 TF(base_link -> laser)에서 읽는다. (laser_tf.py)
        self.laser_mount = LaserMountFromTf(self)

        self.create_timer(CONTROL_PERIOD, self.control_step)

        self.get_logger().info('POLI Mission 1 Started: poli_maze 한 칸 탐색 + 카메라 집기')

    def imu_callback(self, msg):
        if max(abs(msg.angular_velocity.x), abs(msg.angular_velocity.y),
               abs(msg.angular_velocity.z)) > STILL_ANGULAR_RPS:
            self.last_motion_at = self.now_seconds()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.imu_stamp is not None and 0.0 < stamp - self.imu_stamp < 0.2:
            self.imu_dyaw += self.imu_yaw_sign * msg.angular_velocity.z * (stamp - self.imu_stamp)
        self.imu_stamp = stamp
        self.imu_received_at = self.now_seconds()

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        now = self.now_seconds()
        if (abs(msg.twist.twist.linear.x) > STILL_LINEAR_MPS
                or abs(msg.twist.twist.angular.z) > STILL_ANGULAR_RPS):
            self.last_motion_at = now
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

    def gripper_state_callback(self, msg):
        self.gripper_state = msg.data

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

    def fix_pose_with_walls(self, scan, pose, moving=False):
        """
        가까운 벽으로 방향(yaw)과 위치를 바로잡는다.

        pose = 스캔을 찍던 순간의 경기장 위치 (서 있으면 지금 위치).
        2026-10-31: 위치만 고치고 방향은 IMU 적분 그대로라 오차가 쌓여 벽에 박았다.
        """
        dyaw = wall_yaw_offset(scan, pose)
        dx, dy = wall_offset(scan, (pose[0], pose[1], pose[2] + dyaw))
        # 앞뒤 벽이 없는 축(직진 통로)은 옆 상자 모서리로 고친다 (서지 않고 여러 칸 직진할 때)
        corner_x, corner_y = corner_offset(scan, (pose[0] + dx, pose[1] + dy, pose[2] + dyaw))
        source = '벽'
        if dx == 0.0 and corner_x is not None:
            dx, source = corner_x, '벽+모서리'
        if dy == 0.0 and corner_y is not None:
            dy, source = corner_y, '벽+모서리'
        if moving:
            dx, dy, dyaw = (max(-MOVING_FIX_MAX_M, min(MOVING_FIX_MAX_M, MOVING_FIX_GAIN * dx)),
                            max(-MOVING_FIX_MAX_M, min(MOVING_FIX_MAX_M, MOVING_FIX_GAIN * dy)),
                            MOVING_FIX_GAIN * dyaw)
            source = '주행 중 ' + source
        if dx == 0.0 and dy == 0.0 and dyaw == 0.0:
            return
        # 경기장 좌표 보정량 -> odom 좌표 (odom_to_arena의 반대 회전)
        start_yaw = START_POSE[2]
        self.fused[0] += dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
        self.fused[1] += -dx * math.sin(start_yaw) + dy * math.cos(start_yaw)
        self.fused[2] += dyaw
        self.pose = odom_to_arena(*self.fused)
        self.get_logger().info(
            f'{source} 보정: dx={dx:+.3f} m, dy={dy:+.3f} m, dyaw={math.degrees(dyaw):+.1f}도')

    def make_observation(self, now, scan):
        x, y, yaw = self.pose
        observation = Observation(
            x=x, y=y, yaw=yaw, now=now,
            scan=scan,
            holding=self.holding,
            gripper_state=self.gripper_state,
        )
        if scan is not None:
            observation.scan_time = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        vision = self.fresh_vision(now)
        if vision is not None:
            observation.detected = vision.detected
            observation.x_offset = vision.x_offset
            observation.area = vision.area
        return observation

    def check_start(self):
        """시작 신호를 기다리는 중이면 False. 신호가 오면 지금 자리를 출발 위치로 다시 잡는다."""
        if self.started:
            return True
        if not self.ready_logged and self.pose is not None and self.vision is not None:
            self.get_logger().info('Ready: 시작 신호 대기 중 (start_mission.sh go)')
            self.ready_logged = True
        if not os.path.exists(self.start_file):
            return False
        if self.fused is not None:
            self.fused = [0.0, 0.0, 0.0]
            self.pose = odom_to_arena(*self.fused)
        self.started = True
        self.get_logger().info('Start signal')
        return True

    def control_step(self):
        if not self.check_start():
            return
        now = self.now_seconds()
        scan = self.fresh_scan(now)
        if (self.pose is None or self.odom_received_at is None
                or now - self.odom_received_at > ODOM_TIMEOUT
                or (self.require_laser_tf and not self.laser_mount.ready)):
            self.stop()
            self.get_logger().warn(
                '출발/주행 대기: 최신 odom 및 LiDAR TF 필요',
                throttle_duration_sec=3.0)
            return

        # 스캔은 멈춰서 지도를 갱신하는 SCAN에서만 꼭 필요하다.
        # 회전·주행 중에 스캔이 조금 늦었다고 멈추면 급정지 후 급출발하며 끊겨 움직였다 (2026-10-31).
        if scan is None and self.logic.state == SCAN:
            self.stop()
            self.get_logger().warn('스캔 대기: 최신 LiDAR 스캔 없음', throttle_duration_sec=3.0)
            return

        if self.logic.state == SCAN and self.logic.state_started_at is not None:
            scan_started_at = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
            stop_started_at = max(self.logic.state_started_at, self.last_motion_at)
            self.scan_entered_at = self.logic.state_started_at
            self.scan_stop_started_at = stop_started_at
            if not mapping_scan_ready(scan_started_at, now, stop_started_at):
                self.stop()
                if now - self.logic.state_started_at > 1.0:
                    self.get_logger().warn(
                        '정지 후 스캔 대기: '
                        f'정지 경과={now - stop_started_at:.2f}s, '
                        f'스캔 나이={now - scan_started_at:.2f}s',
                        throttle_duration_sec=3.0)
                return

        logic = self.logic
        observation = self.make_observation(now, scan)
        if logic.state == SCAN:
            # 칸 가운데에 서서 판단할 때 한 번
            if (self.wall_fixed_for != logic.state_started_at
                    and now - logic.state_started_at >= SCAN_SETTLE_S):
                self.wall_fixed_for = logic.state_started_at
                self.fix_pose_with_walls(scan, self.pose)
                observation = self.make_observation(now, scan)
        elif logic.rolling_scan_ok(observation):
            # 달리면서 판단할 때: 스캔을 찍던 순간 위치로 보정 (판단도 같은 스캔으로 한다)
            self.fix_pose_with_walls(scan, logic.scan_pose(observation), moving=True)
            observation = self.make_observation(now, scan)
        x, y, yaw = self.pose

        command = self.logic.step(observation)
        if self.logic.event is not None:
            self.get_logger().info(self.logic.event)
            self.logic.event = None

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
            if self.last_state == SCAN and self.scan_entered_at is not None:
                # 정지 확인이 늦으면 차체 흔들림, 그 뒤가 길면 스캔 대기가 원인이다.
                self.get_logger().info(
                    f'판단 시간 {now - self.scan_entered_at:.2f}s '
                    f'(정지 확인까지 {self.scan_stop_started_at - self.scan_entered_at:.2f}s)')
            self.get_logger().info(
                f'State: {self.logic.state}; pose=({x:.3f}, {y:.3f}, '
                f'{math.degrees(yaw):.1f}deg); target={self.logic.target}; '
                f'reverse={self.logic.reverse}')
            self.last_state = self.logic.state
            if self.logic.state == STUCK and self.logic.stop_reason is not None:
                self.get_logger().warn(
                    f'{self.logic.stop_reason}; pose=({x:.2f}, {y:.2f}, {yaw:.2f})\n'
                    + self.logic.grid.to_text())

        if self.logic.path != self.last_path:
            self.get_logger().info(f'Path: {self.logic.path}')
            self.get_logger().info(
                '탐색 지도 (# 막힘, . 빈칸, ? 미관측):\n' + self.logic.grid.to_text()
                + '\n방문 횟수: ' + str(self.logic.maze.state.visits))
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
