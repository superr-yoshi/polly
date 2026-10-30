"""
MPU6050 (GY-521) -> /imu/data. 파이 I2C에 직접 연결 (RRC Lite 내장 IMU 대체, 2026-10-30).

- frame_id imu_link (URDF poli_description이 base_link -> imu_link 위치·방향을 가진다)
- orientation 미제공 (orientation_covariance[0] = -1). rrc_adapter_node가 내던 것과 같은 계약
- 시작할 때 가만히 있는 동안 자이로 영점을 잡고, 그 전에는 발행하지 않는다
- 로봇을 움직이는 장치가 아니다 (I2C 읽기만)

필요: python3-smbus (또는 pip smbus2), 사용자가 i2c 그룹 (/dev/i2c-1 읽기 권한).
"""
from poli_hardware import mpu6050
from poli_hardware.node_runner import run_node
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    from smbus2 import SMBus
except ImportError:
    try:
        from smbus import SMBus  # Ubuntu 패키지 python3-smbus
    except ImportError:
        SMBus = None


class ImuNode(Node):

    REINIT_AFTER = 10   # 연속 읽기 실패가 이 횟수면 센서를 다시 설정한다

    def __init__(self):
        super().__init__('imu_node')
        p = self.declare_parameter
        p('i2c_bus', 1)
        p('i2c_address', mpu6050.ADDRESS)
        p('imu_rate', 50.0)
        p('imu_frame', 'imu_link')
        p('gyro_range_dps', 500)        # 제자리 회전 약 2.8 rad/s = 160 deg/s -> 500이면 여유
        p('accel_range_g', 4)
        p('dlpf', 3)                    # 내장 필터 약 44 Hz (50 Hz 발행에 맞춤)
        p('calib_samples', 100)         # 50 Hz x 2 s
        p('calib_still_rad_s', 0.05)    # 영점 잡는 동안 이보다 크게 변하면 다시 (약 3 deg/s)
        p('gyro_variance', 1e-3)        # SIM_ONLY: 정지 상태 분산 실측 후 교체 (TODO_MEASURE)
        p('accel_variance', 1e-2)       # SIM_ONLY (TODO_MEASURE)

        g = self.get_parameter
        self.addr = g('i2c_address').value
        self.frame = g('imu_frame').value
        self.gyro_range = g('gyro_range_dps').value
        self.accel_range = g('accel_range_g').value
        self.dlpf = g('dlpf').value
        self.gyro_var = g('gyro_variance').value
        self.accel_var = g('accel_variance').value
        rate = g('imu_rate').value
        # 내부 출력 주기를 발행 주기 근처로 (1 kHz / (1 + div))
        self.sample_div = max(0, min(255, int(round(1000.0 / rate)) - 1))
        self.bias = mpu6050.GyroBias(g('calib_samples').value, g('calib_still_rad_s').value)

        self.bus_id = g('i2c_bus').value
        self.bus = None
        self.ready = False
        self.failures = 0
        self._setup()

        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            f'imu_node started: bus {self.bus_id}, address 0x{self.addr:02x}, '
            f'{rate:g} Hz, gyro ±{self.gyro_range} deg/s, accel ±{self.accel_range} g. '
            '로봇을 가만히 두면 자이로 영점을 잡은 뒤 발행을 시작한다')

    def _setup(self):
        try:
            if SMBus is None:
                raise OSError('python3-smbus(또는 smbus2) 없음: sudo apt install python3-smbus')
            if self.bus is None:
                self.bus = SMBus(self.bus_id)   # /dev/i2c-N 이 없으면 OSError
            who = self.bus.read_byte_data(self.addr, mpu6050.REG_WHO_AM_I)
            if who != mpu6050.WHO_AM_I_MPU6050:
                self.get_logger().warn(
                    f'WHO_AM_I = 0x{who:02x} (MPU6050은 0x68). 호환 칩일 수 있음, 계속 진행')
            for reg, val in mpu6050.init_writes(self.gyro_range, self.accel_range, self.dlpf,
                                                self.sample_div):
                self.bus.write_byte_data(self.addr, reg, val)
            self.ready = True
            self.failures = 0
        except OSError as e:
            self.ready = False
            self.get_logger().error(
                f'MPU6050 설정 실패 (배선·주소·i2c 권한 확인): {e}', throttle_duration_sec=2.0)

    def _on_timer(self):
        if not self.ready:
            self._setup()
            return
        try:
            data = self.bus.read_i2c_block_data(self.addr, mpu6050.REG_ACCEL_XOUT_H, 14)
        except OSError as e:
            self.failures += 1
            self.get_logger().warn(f'MPU6050 읽기 실패 {self.failures}회: {e}',
                                   throttle_duration_sec=2.0)
            if self.failures >= self.REINIT_AFTER:
                self.ready = False
            return
        self.failures = 0
        ax, ay, az, gx, gy, gz = mpu6050.parse_block(data, self.gyro_range, self.accel_range)

        if not self.bias.done:
            if self.bias.add(gx, gy, gz):
                bx, by, bz = self.bias.bias
                self.get_logger().info(
                    f'자이로 영점 완료 (rad/s): {bx:+.4f} {by:+.4f} {bz:+.4f}, '
                    f'다시 시작 {self.bias.restarts}회 -> /imu/data 발행 시작')
            elif self.bias.restarts and self.bias.restarts % 50 == 0:
                self.get_logger().warn('로봇이 움직여서 자이로 영점을 못 잡는 중. 가만히 두세요',
                                       throttle_duration_sec=5.0)
            return
        gx, gy, gz = self.bias.apply(gx, gy, gz)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.orientation_covariance[0] = -1.0  # orientation 미제공
        a, w = msg.linear_acceleration, msg.angular_velocity
        a.x, a.y, a.z, w.x, w.y, w.z = ax, ay, az, gx, gy, gz
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = self.gyro_var
            msg.linear_acceleration_covariance[i] = self.accel_var
        self.pub.publish(msg)

    def stop(self):
        if self.bus is None:
            return
        try:
            self.bus.close()
        except OSError:
            pass


def main(args=None):
    run_node(ImuNode, cleanup=ImuNode.stop, args=args)


if __name__ == '__main__':
    main()
