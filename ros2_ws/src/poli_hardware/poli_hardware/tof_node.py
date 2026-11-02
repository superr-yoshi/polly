"""
VL53L0X (집게 ToF 센서) -> /range/gripper, /gripper/holding (2026-11-01).

- /range/gripper: sensor_msgs/Range (m), frame gripper_tof_link (URDF poli_description)
- /gripper/holding: std_msgs/Bool. 거리가 hold_max_mm 이하로 on_samples번 이어지면 true,
  넘는 값이 off_samples번 이어지면 false (깜빡임 방지). 판단 전에는 보내지 않는다 (= 모름).
- 센서가 없거나 읽기가 계속 실패하면 보내지 않고 다시 설정을 시도한다 (임무는 None으로 보고 카메라로 판단)
- 로봇을 움직이는 장치가 아니다 (I2C 읽기만)

필요: python3-smbus (또는 smbus2), i2c 그룹, 소프트웨어 I2C 버스 4 (scripts/pi_setup/vl53l0x_i2c.sh).
"""
import math

from poli_hardware import vl53l0x
from poli_hardware.node_runner import run_node
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

try:
    from smbus2 import SMBus
except ImportError:
    try:
        from smbus import SMBus  # Ubuntu 패키지 python3-smbus
    except ImportError:
        SMBus = None


class TofNode(Node):

    REINIT_AFTER = 10   # 연속 읽기 실패가 이 횟수면 센서를 다시 설정한다
    RETRY_S = 2.0       # 센서가 없을 때 다시 설정을 시도하는 간격 (버스를 계속 두드리지 않게)

    def __init__(self):
        super().__init__('tof_node')
        p = self.declare_parameter
        p('i2c_bus', 4)
        p('i2c_address', vl53l0x.ADDRESS)
        p('rate', 30.0)                 # Hz. 센서 연속 측정 약 33 ms
        p('frame', 'gripper_tof_link')
        p('hold_max_mm', 50)            # 큐브 28~38 mm, 빈 집게 닫힘 65~108 mm (hardware.yaml)
        p('min_valid_mm', 15)           # 이보다 작으면 센서 오류값으로 버린다
        p('on_samples', 3)
        p('off_samples', 5)
        p('min_range_m', 0.03)
        p('max_range_m', 1.2)
        g = self.get_parameter
        self.bus_id = g('i2c_bus').value
        self.addr = g('i2c_address').value
        self.frame = g('frame').value
        self.min_range = g('min_range_m').value
        self.min_valid_mm = g('min_valid_mm').value
        self.max_range = g('max_range_m').value
        self.detector = vl53l0x.HoldDetector(
            g('hold_max_mm').value, g('on_samples').value, g('off_samples').value)
        self.bus = None
        self.sensor = None
        self.failures = 0
        self.last_try = None
        self.range_pub = self.create_publisher(Range, '/range/gripper', 10)
        self.holding_pub = self.create_publisher(Bool, '/gripper/holding', 10)
        self._setup()
        rate = g('rate').value
        self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            f'tof_node started: bus {self.bus_id}, address 0x{self.addr:02x}, {rate:g} Hz, '
            f'holding <= {self.detector.hold_max_mm} mm')

    def _setup(self):
        self.last_try = self.get_clock().now()
        try:
            if SMBus is None:
                raise OSError('python3-smbus(또는 smbus2) 없음: sudo apt install python3-smbus')
            if self.bus is None:
                self.bus = SMBus(self.bus_id)   # /dev/i2c-N 이 없으면 OSError
            sensor = vl53l0x.VL53L0X(self.bus, self.addr)
            sensor.init()
            self.sensor = sensor
            self.failures = 0
            self.get_logger().info('VL53L0X 설정 완료, 측정 시작')
        except OSError as e:
            self.sensor = None
            self.get_logger().error(
                f'VL53L0X 설정 실패 (배선·버스 {self.bus_id}·i2c 권한 확인): {e}',
                throttle_duration_sec=5.0)

    def _on_timer(self):
        if self.sensor is None:
            waited = (self.get_clock().now() - self.last_try).nanoseconds * 1e-9
            if waited >= self.RETRY_S:
                self._setup()
            return
        try:
            mm = self.sensor.read_mm()
        except OSError as e:
            self.failures += 1
            self.get_logger().warn(f'VL53L0X 읽기 실패 {self.failures}회: {e}',
                                   throttle_duration_sec=2.0)
            if self.failures >= self.REINIT_AFTER:
                self.sensor = None
            return
        self.failures = 0
        if mm is None:
            return   # 새 측정값 아직 없음
        if mm < self.min_valid_mm:
            # 실제로 나올 수 없는 값 (큐브가 끝까지 들어와도 28 mm). 거리·holding 둘 다 보내지 않는다.
            # 2026-11-02 19:32 실기: 빈 집게에 1 mm가 한 번 찍혀 임무 1이 바로 닫았다
            self.get_logger().warn(f'VL53L0X 오류값 {mm} mm 버림', throttle_duration_sec=1.0)
            return
        far = mm >= vl53l0x.OUT_OF_RANGE_MM
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = math.radians(25.0)
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = math.inf if far else mm / 1000.0
        self.range_pub.publish(msg)
        holding = self.detector.add(None if far else mm)
        if holding is not None:
            self.holding_pub.publish(Bool(data=holding))

    def stop(self):
        if self.bus is None:
            return
        try:
            self.bus.close()
        except OSError:
            pass


def main(args=None):
    run_node(TofNode, cleanup=TofNode.stop, args=args)


if __name__ == '__main__':
    main()
