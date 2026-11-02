import math
import os
import signal
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from poli_interfaces.msg import TargetDetection
from poli_navigation.laser_tf import LaserMountFromTf, yaw_from_quaternion
from poli_navigation.mission2_logic import (
    Mission2Logic,
    Observation,
    START_POSE,
)
from poli_navigation.opponent_detector import find_opponents
from poli_navigation.wall_localizer import WallCorrection
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, LaserScan, Range
from std_msgs.msg import Bool, String


# 판단 주기 (초)
CONTROL_PERIOD = 0.05

# 카메라 결과가 이 시간(초) 넘게 오지 않으면 "대상을 못 찾음"으로 본다.
# 카메라가 멈췄는데 예전 값을 믿고 움직이지 않기 위해서다.
# TODO: 조원 B의 전송 주기가 정해지면 조정
VISION_TIMEOUT = 0.5
# s, /gripper/holding(tof_node 약 30 Hz)이 이만큼 끊기면 모름(None)으로 본다 (2026-11-01)
HOLDING_TIMEOUT = 0.5

# 상대 로봇 위치(LiDAR)가 이 시간(초) 넘게 갱신되지 않으면 모른다고 본다.
OPPONENT_TIMEOUT = 0.5

# 마지막 집게 명령을 이 주기(초)로 다시 보낸다.
# 조원 A 노드가 임무 노드보다 늦게 켜지면 처음 보낸 명령을 받지 못하기 때문이다.
# Mega는 같은 명령을 다시 받아도 목표 각도만 다시 정하므로 안전하다.
GRIPPER_RESEND_PERIOD = 1.0

# 움직이는 동안 이 주기(초)로 속도·카메라·추정 거리를 로그에 남긴다 (실기 분석용)
MOTION_LOG_PERIOD = 0.2
# 잡은 뒤(GUARD)·실패 뒤(LURK) 서 있는 동안 밀린 흔적이 보이면 이 주기로 'Push:' 로그를 남긴다
# (2026-11-02 밀림 감지 기준을 정하려는 실측용). 엔코더 속도·IMU 회전·수평 가속·벽 보정 변화 중 하나라도 넘으면.
PUSH_LOG_PERIOD = 0.1
PUSH_LOG_ENC_V = 0.02      # m/s
PUSH_LOG_IMU_W = 0.1       # rad/s
PUSH_LOG_ACCEL = 0.8       # m/s^2 (수평, 중력 뺀 값이 아니라 x·y축 그대로)
PUSH_LOG_WALL_JUMP = 0.01  # m, 직전 로그 뒤 벽 보정 변화
# 판단 주기가 이보다 벌어지면 경고 (Mega는 0.3 s 명령이 끊기면 멈춘다)
CONTROL_GAP_WARN_S = 0.15
# 콜백 하나가 이보다 오래 걸리면 경고 (판단 멈춤 원인 확인용)
CALLBACK_WARN_S = 0.05

# 시작 신호 파일 (start_mission.sh go가 만든다). wait_for_start일 때 이 파일이 생기기 전에는
# /cmd_vel·/gripper/command를 하나도 내지 않는다 (규정 3.5.6, 2026-11-01 팀 결정:
# 노드를 미리 켜 두고 시작 신호만 보내 엔터 -> 출발 지연을 약 4 s -> 0.1 s로).
DEFAULT_START_FILE = '/tmp/poli_mission_start'
MOTION_LOG_STATES = (
    'TURN', 'DRIVE', 'SEARCH', 'ALIGN', 'APPROACH', 'PUSH', 'RETREAT', 'GUARD', 'BACKOFF',
    'RECENTER', 'SPIN', 'LURK')


def odom_to_arena(odom_x, odom_y, odom_yaw):
    """Odom 좌표(출발 지점 기준, x = 출발 시 정면) -> 임무 2 경기장 좌표."""
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
        self.wait_for_start = self.declare_parameter('wait_for_start', False).value
        self.start_file = self.declare_parameter('start_file', DEFAULT_START_FILE).value
        self.started = not self.wait_for_start
        self.ready_logged = False
        # odom으로 계산한 위치와 회전 속도. 밀리면 실제 위치와 달라진다.
        self.odom_pose = None
        self.angular_speed = 0.0
        # 방향은 IMU 자이로, 거리는 엔코더 (mission1_node.py와 같은 방식).
        # IMU가 0.3초 넘게 안 오면 엔코더 방향을 쓴다 (시뮬레이션·fake도 그대로 동작).
        self.use_imu_yaw = self.declare_parameter('use_imu_yaw', True).value
        # 2026-10-31 실측 부호 (mission1_node.py 참고)
        self.imu_yaw_sign = self.declare_parameter('imu_yaw_sign', -1.0).value
        self.imu_dyaw = 0.0
        self.imu_stamp = None
        self.imu_received_at = None
        # 밀림 실측 로그용: 마지막 IMU 회전 속도·수평 가속, 엔코더 앞뒤 속도
        self.imu_w = 0.0
        self.imu_accel = (0.0, 0.0)
        self.linear_speed = 0.0
        self.last_push_log = 0.0
        self.last_push_offset = None
        self.prev_odom = None
        self.fused = None
        # 외벽까지 거리(LiDAR)로 odom 위치를 보정한다. (wall_localizer.py)
        self.wall_correction = WallCorrection()
        self.wall_corrected = False
        # LiDAR로 찾은 상대 로봇들, 가까운 순 (opponent_detector.py)
        self.opponents = []
        self.opponent_received_at = None
        self.holding = None
        self.holding_received_at = None
        # 집게 안 적외선 거리 (tof_node /range/gripper, m, 범위 밖 inf)
        self.gripper_range = None
        self.gripper_range_received_at = None
        self.gripper_state = None
        self.vision = None
        self.vision_received_at = None
        self.last_state = None
        self.last_motion_log = 0.0
        self.last_control_at = None

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_publisher = self.create_publisher(
            String, '/gripper/command', 10
        )
        self.gripper_command = None
        self.create_timer(GRIPPER_RESEND_PERIOD, self.resend_gripper)

        # TODO: 실제 로봇에서는 /odometry/filtered (엔코더 + IMU) 사용 검토
        self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Range, '/range/gripper', self.gripper_range_callback, 10)
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

        self.get_logger().info('POLI Mission 2 Started')

    def imu_callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.imu_stamp is not None and 0.0 < stamp - self.imu_stamp < 0.2:
            self.imu_dyaw += self.imu_yaw_sign * msg.angular_velocity.z * (stamp - self.imu_stamp)
        self.imu_stamp = stamp
        self.imu_received_at = self.now_seconds()
        self.imu_w = self.imu_yaw_sign * msg.angular_velocity.z
        self.imu_accel = (msg.linear_acceleration.x, msg.linear_acceleration.y)

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
        self.odom_pose = odom_to_arena(*self.fused)
        self.angular_speed = msg.twist.twist.angular.z
        self.linear_speed = msg.twist.twist.linear.x

    def scan_callback(self, msg):
        if self.odom_pose is None:
            return
        started = time.monotonic()

        corrected = self.wall_correction.update(
            msg, self.odom_pose, self.angular_speed
        )
        if corrected and not self.wall_corrected:
            self.get_logger().info('Wall correction active')
            self.wall_corrected = True
        # 상대 위치는 큐브를 잡은 뒤(GUARD)와 중앙으로 다가갈 때(RECENTER)만 쓴다.
        if self.logic.wants_opponents():
            self.opponents = find_opponents(msg, self.wall_correction.apply(self.odom_pose))
            self.opponent_received_at = self.now_seconds()
        self.warn_if_slow('scan', started)

    def warn_if_slow(self, name, started):
        took = time.monotonic() - started
        if took > CALLBACK_WARN_S:
            self.get_logger().warn(f'{name} 콜백 {took:.2f} s')

    def gripper_range_callback(self, msg):
        self.gripper_range = msg.range
        self.gripper_range_received_at = self.now_seconds()

    def holding_callback(self, msg):
        self.holding = msg.data
        self.holding_received_at = self.now_seconds()

    def gripper_state_callback(self, msg):
        self.gripper_state = msg.data

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

    def check_start(self):
        """시작 신호를 기다리는 중이면 False. 신호가 오면 지금 자리를 출발 위치로 다시 잡는다."""
        if self.started:
            return True
        if not self.ready_logged and self.odom_pose is not None and self.vision is not None:
            self.get_logger().info('Ready: 시작 신호 대기 중 (start_mission.sh go)')
            self.ready_logged = True
        if not os.path.exists(self.start_file):
            return False
        # 준비 뒤 로봇을 옮겨 놓았어도 되도록 지금 자리를 START_POSE로 본다.
        self.fused = [0.0, 0.0, 0.0] if self.prev_odom is not None else None
        if self.fused is not None:
            self.odom_pose = odom_to_arena(*self.fused)
        self.wall_correction = WallCorrection()
        self.started = True
        self.get_logger().info('Start signal')
        return True

    def control_step(self):
        if not self.check_start():
            return
        if self.odom_pose is None:
            # 위치를 아직 모르면 움직이지 않는다.
            return

        x, y, yaw = self.wall_correction.apply(self.odom_pose)
        now = self.now_seconds()
        if self.last_control_at is not None and now - self.last_control_at > CONTROL_GAP_WARN_S:
            # 2026-11-01 실기: 큐브를 본 직후 0.46 s 판단이 멈춰 Mega가 바퀴를 세웠다. 원인 확인용
            self.get_logger().warn(f'판단 주기 지연 {now - self.last_control_at:.2f} s')
        self.last_control_at = now

        observation = Observation(
            x=x, y=y, yaw=yaw, now=now,
            # 집게 센서(tof_node)가 HOLDING_TIMEOUT 넘게 조용하면 모름(None) = 잡았다고 가정 (예전과 같음)
            holding=(self.holding if self.holding_received_at is not None
                     and now - self.holding_received_at <= HOLDING_TIMEOUT else None),
            gripper_state=self.gripper_state,
            gripper_range=(self.gripper_range if self.gripper_range_received_at is not None
                           and now - self.gripper_range_received_at <= HOLDING_TIMEOUT else None),
        )
        if (self.opponent_received_at is not None
                and now - self.opponent_received_at <= OPPONENT_TIMEOUT):
            observation.opponents = tuple(self.opponents)
            observation.opponent = self.opponents[0] if self.opponents else None

        vision = self.fresh_vision(now)
        if vision is not None:
            observation.detected = vision.detected
            observation.x_offset = vision.x_offset
            observation.area = vision.area
            observation.vision_stamp = self.vision_received_at

        started = time.monotonic()
        command = self.logic.step(observation)
        self.warn_if_slow('판단', started)

        twist = Twist()
        twist.linear.x = command.linear
        twist.angular.z = command.angular
        self.cmd_vel_publisher.publish(twist)

        if command.gripper is not None:
            self.send_gripper(command.gripper)

        if self.logic.state != self.last_state:
            self.get_logger().info(f'State: {self.logic.state}')
            self.last_state = self.logic.state

        # GUARD는 서 있는 동안 남기지 않는다 (5분 동안 로그가 너무 많아진다)
        moving = command.linear != 0.0 or command.angular != 0.0
        if (self.logic.state in MOTION_LOG_STATES
                and (moving or self.logic.state != 'GUARD')
                and now - self.last_motion_log >= MOTION_LOG_PERIOD):
            self.last_motion_log = now
            target = self.logic.target
            cam_d = '-' if target is None else f'{self.logic._camera_distance(observation):.2f}'
            opp_text = ' '.join(f'({o[0]:.2f},{o[1]:.2f})' for o in observation.opponents) or '-'
            tof = observation.gripper_range
            tof_text = '-' if tof is None else (
                'inf' if math.isinf(tof) else f'{tof * 1000:.0f}mm')
            self.get_logger().info(
                f'Motion: {self.logic.state} pos=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.0f} '
                f'cmd=({command.linear:.2f},{command.angular:+.2f}) '
                f'odom_w={self.angular_speed:+.2f} '
                f'cam={observation.detected:d} off={observation.x_offset:+.2f} '
                f'area={observation.area:.0f} cube_d={cam_d} opp={opp_text} '
                f'tof={tof_text}')

        self.log_push(now, command, x, y, yaw)

    def log_push(self, now, command, x, y, yaw):
        """GUARD·LURK에서 서 있는데 밀린 흔적(엔코더·IMU·가속·벽 보정)이 보이면 남긴다 (실측용)."""
        if self.logic.state not in ('GUARD', 'LURK'):
            self.last_push_offset = None
            return
        if command.linear != 0.0 or command.angular != 0.0:
            return
        offset = (self.wall_correction.offset_x, self.wall_correction.offset_y)
        jump = (0.0 if self.last_push_offset is None
                else math.dist(offset, self.last_push_offset))
        accel = math.hypot(*self.imu_accel)
        pushed = (abs(self.linear_speed) > PUSH_LOG_ENC_V or abs(self.imu_w) > PUSH_LOG_IMU_W
                  or accel > PUSH_LOG_ACCEL or jump > PUSH_LOG_WALL_JUMP)
        if not pushed or now - self.last_push_log < PUSH_LOG_PERIOD:
            return
        self.last_push_log = now
        self.last_push_offset = offset
        raw_x, raw_y, _ = self.odom_pose
        self.get_logger().info(
            f'Push: {self.logic.state} pos=({x:.3f},{y:.3f}) yaw={math.degrees(yaw):.1f} '
            f'raw=({raw_x:.3f},{raw_y:.3f}) wall_off=({offset[0]:+.3f},{offset[1]:+.3f}) '
            f'enc_v={self.linear_speed:+.3f} enc_w={self.angular_speed:+.2f} '
            f'imu_w={self.imu_w:+.2f} acc=({self.imu_accel[0]:+.2f},{self.imu_accel[1]:+.2f})')

    def send_gripper(self, command):
        self.gripper_command = command
        self.gripper_publisher.publish(String(data=command))
        self.get_logger().info(f'Gripper: {command}')

    def resend_gripper(self):
        if self.gripper_command is not None:
            self.gripper_publisher.publish(String(data=self.gripper_command))

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
