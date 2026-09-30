"""
Arduino Mega <-> ROS 2 브리지 (프로토콜 v1.1, docs/serial_protocol.md).

발행:  /range/front_left, /range/front_right, /range/rear_left, /range/rear_right
       (sensor_msgs/Range), /gripper/state (std_msgs/String: open|closed|moving)
구독:  /gripper/command (std_msgs/String: "open" = 열기, "grab" = 닫아서 잡기)
       들어 올리는 동작은 없다 (조원 A 결정).
TODO: 잡힘 감지 센서가 추가되면 /gripper/holding (std_msgs/Bool)을 여기서 발행한다.

시리얼 읽기는 별도 스레드, packet 해석은 mega_protocol(순수 모듈)이 담당한다.
"""
import threading
import time

from poli_hardware.mega_protocol import (
    gripper_action, GstPacket, make_grip_command, MegaParser,
    mm_to_range_m, RANGE_ORDER, RangeMedianFilter, RngPacket)
from poli_hardware.node_runner import run_node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String

MAX_GRIP_TRIES = 3

# RANGE_ORDER 이름 -> URDF frame 이름
RANGE_FRAMES = {
    'front_left': 'ultrasonic_fl_link',
    'front_right': 'ultrasonic_fr_link',
    'rear_left': 'ultrasonic_rl_link',
    'rear_right': 'ultrasonic_rr_link',
}


def declare_range_params(node):
    node.declare_parameter('field_of_view', 0.26)  # HC-SR04 약 15도
    node.declare_parameter('min_range', 0.02)
    node.declare_parameter('max_range', 4.0)


def make_range_msg(node, name, range_m):
    msg = Range()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = RANGE_FRAMES[name]
    msg.radiation_type = Range.ULTRASOUND
    msg.field_of_view = node.get_parameter('field_of_view').value
    msg.min_range = node.get_parameter('min_range').value
    msg.max_range = node.get_parameter('max_range').value
    msg.range = range_m
    return msg


def create_range_publishers(node):
    return {n: node.create_publisher(Range, f'/range/{n}', 10) for n in RANGE_ORDER}


class MegaBridgeNode(Node):

    def __init__(self):
        super().__init__('mega_bridge_node')
        self.declare_parameter('port', '/dev/robot_mega')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('open_delay', 2.0)   # 포트를 열면 Mega가 자동 리셋된다
        self.declare_parameter('stale_timeout', 1.0)
        self.declare_parameter('ack_timeout', 0.5)
        self.declare_parameter('median_window', 3)  # 1 = 필터 없음
        declare_range_params(self)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.ack_timeout = self.get_parameter('ack_timeout').value
        self.stale_timeout = self.get_parameter('stale_timeout').value

        self.parser = MegaParser()
        self.range_filter = RangeMedianFilter(self.get_parameter('median_window').value)
        self.range_pubs = create_range_publishers(self)
        self.state_pub = self.create_publisher(String, '/gripper/state', 10)
        self.create_subscription(String, '/gripper/command', self._on_gripper_command, 10)
        self.create_timer(0.1, self._on_gripper_retry)
        self.create_timer(1.0, self._on_health)

        self._ser = None
        self._write_lock = threading.Lock()
        self._cmd_id = 0
        self._last_ack = 0      # 읽기 스레드가 GST의 last_id로 갱신
        self._pending = None    # [action, cmd_id, 보낸 시각, 보낸 횟수]
        self._last_rng = None
        self._stale_warned = False
        self._stop = False
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    # ---- serial ---------------------------------------------------------
    def _open(self):
        import serial  # pyserial (python3-serial)
        ser = serial.Serial(self.port, self.baud, timeout=0.2)
        time.sleep(self.get_parameter('open_delay').value)
        ser.reset_input_buffer()
        self.get_logger().info(f'Mega connected: {self.port} @ {self.baud}')
        return ser

    def _read_loop(self):
        while not self._stop:
            if self._ser is None:
                try:
                    self._ser = self._open()
                except Exception as e:  # noqa: BLE001 - 포트 없음/권한 등 모두 재시도
                    self.get_logger().error(
                        f'Mega open failed ({self.port}): {e}; 2초 후 재시도',
                        throttle_duration_sec=10.0)
                    time.sleep(2.0)
                    continue
            try:
                raw = self._ser.readline()
            except Exception as e:  # noqa: BLE001 - USB 분리 등
                self.get_logger().error(f'Mega read error: {e}; 재연결')
                self._close()
                continue
            if raw:
                try:
                    self._handle(self.parser.feed(raw))
                except Exception:  # noqa: BLE001
                    if not rclpy.ok():  # Ctrl+C로 ROS가 먼저 종료됨 -> 조용히 끝낸다
                        break
                    raise

    def _close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
        self._ser = None

    def _handle(self, pkt):
        if isinstance(pkt, RngPacket):
            self._last_rng = time.monotonic()
            self._stale_warned = False
            lo = self.get_parameter('min_range').value
            hi = self.get_parameter('max_range').value
            for name, mm in zip(RANGE_ORDER, self.range_filter.update(pkt.mm)):
                self.range_pubs[name].publish(
                    make_range_msg(self, name, mm_to_range_m(mm, lo, hi)))
        elif isinstance(pkt, GstPacket):
            self.state_pub.publish(String(data=pkt.state_name))
            self._last_ack = pkt.last_id

    # ---- gripper --------------------------------------------------------
    def _on_gripper_command(self, msg):
        action = gripper_action(msg.data)
        if action is None:
            self.get_logger().warn(
                f'알 수 없는 집게 명령 "{msg.data}" (open / grab만 가능)')
            return
        self.get_logger().info(f'gripper command: {msg.data}')
        self._send_grip(action, tries=1)

    def _send_grip(self, action, tries):
        self._cmd_id += 1
        if not self._write(make_grip_command(self._cmd_id, action)):
            self.get_logger().warn('집게 명령 실패: Mega 연결 안 됨')
            self._pending = None
            return
        self._pending = [action, self._cmd_id, time.monotonic(), tries]

    def _on_gripper_retry(self):
        """Mega가 GST로 받았다고 답할 때까지 최대 MAX_GRIP_TRIES번 보낸다."""
        if self._pending is None:
            return
        action, cmd_id, sent_at, tries = self._pending
        if self._last_ack == cmd_id:
            self._pending = None
        elif time.monotonic() - sent_at > self.ack_timeout:
            if tries < MAX_GRIP_TRIES:
                self._send_grip(action, tries + 1)
            else:
                self.get_logger().error('집게 명령: Mega 응답 없음')
                self._pending = None

    def _write(self, data):
        ser = self._ser
        if ser is None:
            return False
        try:
            with self._write_lock:
                ser.write(data)
            return True
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Mega write error: {e}')
            return False

    # ---- 진단 -----------------------------------------------------------
    def _on_health(self):
        if self._ser is None or self._last_rng is None:
            return
        if time.monotonic() - self._last_rng > self.stale_timeout and not self._stale_warned:
            self.get_logger().warn(f'RNG 수신 없음 {self.stale_timeout}s 이상')
            self._stale_warned = True
        p = self.parser
        self.get_logger().debug(f'packets ok={p.ok} bad={p.bad} gap={p.gap}')

    def shutdown(self):
        self._stop = True
        self._reader.join(timeout=1.0)
        self._close()


def _cleanup(node):
    node.shutdown()
    p = node.parser
    print(f'[mega_bridge_node] packets ok={p.ok} bad={p.bad} gap={p.gap}')


def main(args=None):
    run_node(MegaBridgeNode, cleanup=_cleanup, args=args)


if __name__ == '__main__':
    main()
