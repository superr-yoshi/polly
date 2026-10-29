"""
하드웨어 계층 공통 노드: 주행·오도메트리·IMU·초음파·집게.

/cmd_vel -> 바퀴, /odom_raw, /imu/data, /range/*, /gripper/command -> 집게, /gripper/state.

- fake_mega_node  : SimTransport (부품 없이, 바퀴가 명령을 그대로 따른다고 가정)
- mega_bridge_node: MegaTransport (Arduino Mega USB Serial, docs/serial_protocol.md v2.0)
  Mega가 MDD10 모터·엔코더·BNO085·초음파·집게를 모두 맡는다 (2026-10-29 RRC Lite 사용 중단).

두 노드의 Topic·frame_id·parameter 계약은 같다. launch에서 노드만 바꿔 끼운다.
odom -> base_link TF는 발행하지 않는다 (robot_localization EKF 소유).

transport 인터페이스:
  set_wheel_speeds(left, right)  바퀴 각속도 rad/s (전진 = +)
  read_wheel_speeds()            (left, right) rad/s
  read_imu()                     None 또는 (quat 또는 None, (gx, gy, gz), (ax, ay, az))
  set_gripper(word)              "open" / "grab"
  gripper_state()                "open" / "closed" / "moving"
  close()
초음파는 transport가 node.publish_ranges(mm 4개)를 부른다.
"""
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from poli_hardware.diff_drive import (
    CmdWatchdog, DiffDriveParams, OdomIntegrator, twist_to_wheels,
    wheels_to_twist, yaw_to_quaternion)
from poli_hardware.mega_protocol import mm_to_range_m, RANGE_ORDER, RangeMedianFilter
from poli_hardware.node_runner import run_node
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import String

GRIPPER_WORDS = ('open', 'grab')   # 들어 올리기 없음 (AGENTS.md 4장)

# RANGE_ORDER 이름 -> URDF frame 이름
RANGE_FRAMES = {
    'front': 'ultrasonic_front_link',
    'left': 'ultrasonic_left_link',
    'right': 'ultrasonic_right_link',
    'rear': 'ultrasonic_rear_link',
}


class DriveNode(Node):

    def __init__(self, name, transport_cls):
        super().__init__(name)
        p = self.declare_parameter
        p('wheel_radius', 0.0325)
        p('wheel_separation', 0.18)
        p('max_linear', 0.25)
        p('max_angular', 1.0)
        p('max_wheel_rpm', 250.0)
        p('min_wheel_speed', 0.0)        # m/s, 0 = 끔 (hardware.yaml)
        p('cmd_timeout', 0.3)
        p('odom_rate', 25.0)
        p('imu_rate', 25.0)
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('imu_frame', 'imu_link')
        p('median_window', 3)            # 초음파 중간값 필터 (1 = 끔)
        p('field_of_view', 0.26)         # HC-SR04 약 15도
        p('min_range', 0.02)
        p('max_range', 4.0)

        g = self.get_parameter
        self.drive = DiffDriveParams(
            wheel_radius=g('wheel_radius').value,
            wheel_separation=g('wheel_separation').value,
            max_linear=g('max_linear').value,
            max_angular=g('max_angular').value,
            max_wheel_rpm=g('max_wheel_rpm').value,
            min_wheel_speed=g('min_wheel_speed').value)
        self.odom_frame = g('odom_frame').value
        self.base_frame = g('base_frame').value
        self.imu_frame = g('imu_frame').value
        self.range_filter = RangeMedianFilter(g('median_window').value)

        self.watchdog = CmdWatchdog(g('cmd_timeout').value)
        self.integrator = OdomIntegrator()
        self._last_t = None
        self._angular = 0.0
        self._was_expired = True
        self._gripper_state = None

        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.range_pubs = {n: self.create_publisher(Range, f'/range/{n}', 10)
                           for n in RANGE_ORDER}
        self.gripper_pub = self.create_publisher(String, '/gripper/state', 10)

        self.transport = transport_cls(self)

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(String, '/gripper/command', self._on_gripper_command, 10)
        self.create_timer(1.0 / g('odom_rate').value, self._on_control)
        self.create_timer(1.0 / g('imu_rate').value, self._on_imu)
        self.create_timer(0.1, self._on_gripper_poll)
        self.create_timer(0.5, self._publish_gripper_state)

        self.get_logger().info(
            f'{name} started: wheel_radius={self.drive.wheel_radius} '
            f'wheel_separation={self.drive.wheel_separation} '
            f'cmd_timeout={self.watchdog.timeout}s')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- 주행 -----------------------------------------------------------
    def _on_cmd_vel(self, msg):
        self.watchdog.update(self._now(), msg.linear.x, msg.angular.z)

    def _on_control(self):
        now = self._now()
        expired = self.watchdog.expired(now)
        if expired and not self._was_expired:
            self.get_logger().warn('/cmd_vel timeout -> 정지')
        self._was_expired = expired

        # 0 속도도 매 주기 보낸다: Mega는 0.3초 넘게 명령이 없으면 멈추므로 연결 확인 역할도 한다
        linear, angular = self.watchdog.command(now)
        self.transport.set_wheel_speeds(*twist_to_wheels(self.drive, linear, angular))

        v, w = wheels_to_twist(self.drive, *self.transport.read_wheel_speeds())
        if self._last_t is not None:
            self.integrator.step(v, w, now - self._last_t)
        self._last_t = now
        self._angular = w
        self._publish_odom(v, w)

    def _publish_odom(self, v, w):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.integrator.x
        msg.pose.pose.position.y = self.integrator.y
        q = yaw_to_quaternion(self.integrator.yaw)
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        # SIM_ONLY: 테스트용 covariance. 실제 센서 노이즈 측정 후 교체
        for i, var in ((0, 1e-3), (7, 1e-3), (35, 1e-2)):
            msg.pose.covariance[i] = var
            msg.twist.covariance[i] = var
        for i in (14, 21, 28):  # z, roll, pitch: 2D 로봇이라 크게
            msg.pose.covariance[i] = 1e6
            msg.twist.covariance[i] = 1e6
        self.odom_pub.publish(msg)

    # ---- IMU ------------------------------------------------------------
    def _on_imu(self):
        imu = self.transport.read_imu()
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        # SIM_ONLY: 테스트용 covariance. 실제 IMU 정지 상태 분산 측정 후 교체
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = 1e-3
        if imu is None:
            # IMU 없음 (fake, BNO085 미연결): 휠 각속도로 z축만 채운다
            msg.orientation_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0
            msg.angular_velocity.z = self._angular
        else:
            quat, gyro, accel = imu
            if quat is None:
                msg.orientation_covariance[0] = -1.0
            else:
                o = msg.orientation
                o.x, o.y, o.z, o.w = quat
                for i in (0, 4, 8):
                    msg.orientation_covariance[i] = 1e-3   # SIM_ONLY
            w, a = msg.angular_velocity, msg.linear_acceleration
            w.x, w.y, w.z = gyro
            a.x, a.y, a.z = accel
            for i in (0, 4, 8):
                msg.linear_acceleration_covariance[i] = 1e-2
        self.imu_pub.publish(msg)

    # ---- 초음파 (transport가 부름, 읽기 스레드에서 올 수 있음) -----------------
    def publish_ranges(self, mm):
        lo = self.get_parameter('min_range').value
        hi = self.get_parameter('max_range').value
        fov = self.get_parameter('field_of_view').value
        stamp = self.get_clock().now().to_msg()
        for name, v in zip(RANGE_ORDER, self.range_filter.update(mm)):
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = RANGE_FRAMES[name]
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = fov
            msg.min_range = lo
            msg.max_range = hi
            msg.range = mm_to_range_m(v, lo, hi)   # 0(실패) -> +inf (REP-117)
            self.range_pubs[name].publish(msg)

    # ---- 집게 -----------------------------------------------------------
    def _on_gripper_command(self, msg):
        word = msg.data.strip().lower()
        if word not in GRIPPER_WORDS:
            self.get_logger().warn(f'알 수 없는 집게 명령 "{msg.data}" (open / grab만 가능)')
            return
        # 같은 명령도 transport에 넘긴다 (임무 노드는 1초마다 다시 보냄, Mega 리셋 복구)
        self.transport.set_gripper(word)

    def _on_gripper_poll(self):
        if self.transport.gripper_state() != self._gripper_state:
            self._publish_gripper_state()

    def _publish_gripper_state(self):
        self._gripper_state = self.transport.gripper_state()
        self.gripper_pub.publish(String(data=self._gripper_state))

    def stop(self):
        self.transport.set_wheel_speeds(0.0, 0.0)
        self.transport.close()


def run(name, transport_cls, args=None):
    run_node(lambda: DriveNode(name, transport_cls), cleanup=DriveNode.stop, args=args)
