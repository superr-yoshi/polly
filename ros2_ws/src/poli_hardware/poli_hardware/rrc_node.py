"""RRC Lite 주행 계층: /cmd_vel -> 바퀴, 엔코더 -> /odom_raw, 내장 IMU -> /imu/data.

- fake_rrc_node   : SimTransport (부품 도착 전, 바퀴가 명령을 그대로 따른다고 가정)
- rrc_adapter_node: RrcTransport (RRC Lite 도착 후 통신 부분만 구현)

두 노드의 Topic·frame_id·parameter 계약은 같다. launch에서 노드만 바꿔 끼운다.
odom -> base_link TF는 발행하지 않는다 (robot_localization EKF 소유).
"""
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from poli_hardware.diff_drive import (
    CmdWatchdog, DiffDriveParams, OdomIntegrator, twist_to_wheels,
    wheels_to_twist, yaw_to_quaternion)


class SimTransport:
    """SIM_ONLY: 모터·엔코더·IMU 없이 바퀴가 명령 속도를 즉시 따른다고 가정."""

    def __init__(self, node):
        self._left = 0.0
        self._right = 0.0

    def set_wheel_speeds(self, left, right):
        self._left, self._right = left, right

    def read_wheel_speeds(self):
        return self._left, self._right

    def read_gyro_z(self):
        return None  # None이면 노드가 휠 오도메트리 각속도로 대신 채운다

    def close(self):
        pass


class RrcTransport:
    """RRC Lite USB Serial 통신. TODO: 보드 도착 후 제조사 SDK/프로토콜 확인 후 구현.

    구현할 것 (docs/rrc_adapter_plan.md 참고):
      set_wheel_speeds(left, right)  바퀴 각속도 rad/s -> RRC 모터 속도 명령
      read_wheel_speeds()            엔코더 -> 바퀴 각속도 rad/s (부호: 전진 +)
      read_gyro_z()                  내장 IMU z축 각속도 rad/s (ROS 축 기준)
    """

    def __init__(self, node):
        port = node.get_parameter('port').value
        raise NotImplementedError(
            f'RRC Lite 통신이 아직 구현되지 않았습니다 (port={port}). '
            '부품 도착 전에는 use_fake_hardware:=true 로 실행하세요.')


class RrcNode(Node):

    def __init__(self, name, transport_cls):
        super().__init__(name)
        p = self.declare_parameter
        p('wheel_radius', 0.0325)
        p('wheel_separation', 0.18)
        p('max_linear', 0.25)
        p('max_angular', 1.0)
        p('max_wheel_rpm', 330.0)
        p('cmd_timeout', 0.3)
        p('odom_rate', 25.0)
        p('imu_rate', 50.0)
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('imu_frame', 'imu_link')
        p('port', '/dev/robot_rrc')
        p('baud', 1000000)

        g = self.get_parameter
        self.drive = DiffDriveParams(
            wheel_radius=g('wheel_radius').value,
            wheel_separation=g('wheel_separation').value,
            max_linear=g('max_linear').value,
            max_angular=g('max_angular').value,
            max_wheel_rpm=g('max_wheel_rpm').value)
        self.odom_frame = g('odom_frame').value
        self.base_frame = g('base_frame').value
        self.imu_frame = g('imu_frame').value

        self.watchdog = CmdWatchdog(g('cmd_timeout').value)
        self.integrator = OdomIntegrator()
        self.transport = transport_cls(self)
        self._last_t = None
        self._angular = 0.0
        self._was_expired = True

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.create_timer(1.0 / g('odom_rate').value, self._on_control)
        self.create_timer(1.0 / g('imu_rate').value, self._on_imu)

        self.get_logger().info(
            f'{name} started: wheel_radius={self.drive.wheel_radius} '
            f'wheel_separation={self.drive.wheel_separation} '
            f'cmd_timeout={self.watchdog.timeout}s')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd_vel(self, msg):
        self.watchdog.update(self._now(), msg.linear.x, msg.angular.z)

    def _on_control(self):
        now = self._now()
        expired = self.watchdog.expired(now)
        if expired and not self._was_expired:
            self.get_logger().warn('/cmd_vel timeout -> 정지')
        self._was_expired = expired

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

    def _on_imu(self):
        gz = self.transport.read_gyro_z()
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        msg.orientation_covariance[0] = -1.0  # orientation 미제공
        msg.angular_velocity.z = self._angular if gz is None else gz
        # SIM_ONLY: 테스트용 covariance
        msg.angular_velocity_covariance[8] = 1e-3
        msg.linear_acceleration_covariance[0] = -1.0  # 가속도 미제공
        self.imu_pub.publish(msg)

    def stop(self):
        self.transport.set_wheel_speeds(0.0, 0.0)
        self.transport.close()


def _run(name, transport_cls, args):
    rclpy.init(args=args)
    node = RrcNode(name, transport_cls)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_fake(args=None):
    _run('fake_rrc_node', SimTransport, args)


def main_real(args=None):
    _run('rrc_adapter_node', RrcTransport, args)


if __name__ == '__main__':
    main_fake()
