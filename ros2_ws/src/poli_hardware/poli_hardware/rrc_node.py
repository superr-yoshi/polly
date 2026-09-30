"""
RRC Lite 주행 계층: /cmd_vel -> 바퀴, /odom_raw, 내장 IMU -> /imu/data, /battery_state.

- fake_rrc_node   : SimTransport (부품 도착 전, 바퀴가 명령을 그대로 따른다고 가정)
- rrc_adapter_node: RrcTransport (RRC Lite USB Serial, docs/rrc_protocol.md)

두 노드의 Topic·frame_id·parameter 계약은 같다. launch에서 노드만 바꿔 끼운다.
odom -> base_link TF는 발행하지 않는다 (robot_localization EKF 소유).
"""
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from poli_hardware import rrc_protocol
from poli_hardware.diff_drive import (
    CmdWatchdog, DiffDriveParams, OdomIntegrator, twist_to_wheels,
    wheels_to_twist, yaw_to_quaternion)
from poli_hardware.node_runner import run_node
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu


class SimTransport:
    """SIM_ONLY: 모터·엔코더·IMU 없이 바퀴가 명령 속도를 즉시 따른다고 가정."""

    def __init__(self, node):
        self._left = 0.0
        self._right = 0.0

    def set_wheel_speeds(self, left, right):
        self._left, self._right = left, right

    def read_wheel_speeds(self):
        return self._left, self._right

    def read_imu(self):
        return None  # None이면 노드가 휠 오도메트리 각속도로 z축만 채운다

    def read_supply_voltage(self):
        return 12.0  # SIM_ONLY

    def close(self):
        pass


class RrcTransport:
    """
    RRC Lite USB Serial (docs/rrc_protocol.md).

    제조사 펌웨어는 엔코더 값을 Pi로 보내지 않는다. 그래서 read_wheel_speeds()는
    마지막으로 보낸 명령(=STM32 PID 목표값)을 돌려준다. /odom_raw는 명령 기반 추정이고,
    회전은 EKF에서 IMU gyro로 보정한다.
    """

    def __init__(self, node):
        self._log = node.get_logger()
        p = node.declare_parameter
        p('motor_type', rrc_protocol.MOTOR_TYPE_JGB37)
        p('motor_ticks_per_rev', 1980.0)   # TODO_MEASURE: 실제 JGB37-520 출력축 1회전 tick
        p('max_motor_rps', 3.0)             # 펌웨어 JGB37 rps 제한과 동일
        p('left_motor_id', 0)               # M1
        p('right_motor_id', 1)              # M2
        p('left_sign', -1.0)                # TODO_MEASURE: 전진 명령에 바퀴가 뒤로 돌면 부호 반전
        p('right_sign', 1.0)                # TODO_MEASURE
        p('imu_timeout', 0.5)
        g = node.get_parameter
        self._ids = (g('left_motor_id').value, g('right_motor_id').value)
        self._signs = (g('left_sign').value, g('right_sign').value)
        self._ticks = g('motor_ticks_per_rev').value
        self._limit = g('max_motor_rps').value
        self._imu_timeout = g('imu_timeout').value

        import serial  # pyserial (python3-serial)
        port = g('port').value
        self._ser = serial.Serial(port, g('baud').value, timeout=0.05)
        self._lock = threading.Lock()
        self._parser = rrc_protocol.FrameParser()
        self._imu = None
        self._imu_t = 0.0
        self._imu_warned = False
        self._sent = (0.0, 0.0)
        self._supply_mv = None
        self._supply_t = 0.0
        self._stop = False
        self._write(rrc_protocol.motor_type_frame(g('motor_type').value))
        self._write(rrc_protocol.motor_stop_frame())
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self._log.info(f'RRC Lite connected: {port}')
        self._log.warn('RRC 펌웨어는 엔코더를 보내지 않음 -> /odom_raw는 명령 기반 추정')

    def _write(self, frame):
        try:
            with self._lock:
                self._ser.write(frame)
        except Exception as e:  # noqa: BLE001 - USB 분리 등
            self._log.error(f'RRC write error: {e}', throttle_duration_sec=2.0)

    def _read_loop(self):
        while not self._stop:
            try:
                chunk = self._ser.read(self._ser.in_waiting or 1)
            except Exception as e:  # noqa: BLE001
                self._log.error(f'RRC read error: {e}', throttle_duration_sec=2.0)
                time.sleep(0.5)
                continue
            for func, data in self._parser.feed(chunk):
                if func == rrc_protocol.FUNC_IMU:
                    imu = rrc_protocol.parse_imu(data)
                    if imu is not None:
                        self._imu, self._imu_t = imu, time.monotonic()
                elif func == rrc_protocol.FUNC_SYS:
                    mv = rrc_protocol.parse_battery_mv(data)
                    if mv is not None:
                        self._supply_mv, self._supply_t = mv, time.monotonic()

    def set_wheel_speeds(self, left, right):
        rps = [rrc_protocol.wheel_to_motor_rps(w, s, self._ticks, self._limit)
               for w, s in zip((left, right), self._signs)]
        self._write(rrc_protocol.motor_speeds_frame(list(zip(self._ids, rps))))
        self._sent = tuple(rrc_protocol.motor_rps_to_wheel(r, s, self._ticks)
                           for r, s in zip(rps, self._signs))

    def read_wheel_speeds(self):
        return self._sent

    def read_imu(self):
        if self._imu is None or time.monotonic() - self._imu_t > self._imu_timeout:
            if not self._imu_warned:
                self._log.warn(f'RRC IMU 수신 없음 ({self._imu_timeout}s)')
                self._imu_warned = True
            return None
        self._imu_warned = False
        return self._imu

    def read_supply_voltage(self):
        """RRC 입력 전압 V (약 1 Hz 보고). 3초 넘게 안 오면 None."""
        if self._supply_mv is None or time.monotonic() - self._supply_t > 3.0:
            return None
        return self._supply_mv / 1000.0

    def close(self):
        self._write(rrc_protocol.motor_stop_frame())
        self._stop = True
        self._reader.join(timeout=0.5)
        self._ser.close()


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
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.create_timer(1.0 / g('odom_rate').value, self._on_control)
        self.create_timer(1.0 / g('imu_rate').value, self._on_imu)
        self.create_timer(1.0, self._on_battery)

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
        imu = self.transport.read_imu()
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        msg.orientation_covariance[0] = -1.0  # orientation 미제공
        # SIM_ONLY: 테스트용 covariance. 실제 IMU 정지 상태 분산 측정 후 교체
        msg.angular_velocity_covariance[0] = 1e-3
        msg.angular_velocity_covariance[4] = 1e-3
        msg.angular_velocity_covariance[8] = 1e-3
        if imu is None:
            msg.angular_velocity.z = self._angular
            msg.linear_acceleration_covariance[0] = -1.0  # 가속도 미제공
        else:
            a, w = msg.linear_acceleration, msg.angular_velocity
            a.x, a.y, a.z, w.x, w.y, w.z = imu
            for i in (0, 4, 8):
                msg.linear_acceleration_covariance[i] = 1e-2
        self.imu_pub.publish(msg)

    def _on_battery(self):
        """
        RRC 입력 전압을 발행한다.

        주의: RRC는 12V 컨버터 뒤에 있어 LiPo 잔량이 아니라 컨버터 출력(약 12V)이 나온다.
        percentage 등 모르는 값은 NaN.
        """
        volts = self.transport.read_supply_voltage()
        if volts is None:
            return
        nan = float('nan')
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = volts
        msg.current = nan
        msg.charge = nan
        msg.capacity = nan
        msg.design_capacity = nan
        msg.percentage = nan
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.present = True
        msg.location = 'rrc_lite_input (12V converter output, not LiPo)'
        self.battery_pub.publish(msg)

    def stop(self):
        self.transport.set_wheel_speeds(0.0, 0.0)
        self.transport.close()


def _run(name, transport_cls, args):
    run_node(lambda: RrcNode(name, transport_cls), cleanup=RrcNode.stop, args=args)


def main_fake(args=None):
    _run('fake_rrc_node', SimTransport, args)


def main_real(args=None):
    _run('rrc_adapter_node', RrcTransport, args)


if __name__ == '__main__':
    main_fake()
