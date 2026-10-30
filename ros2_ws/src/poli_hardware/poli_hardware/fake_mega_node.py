"""
SIM_ONLY: Mega 없이 mega_bridge_node와 같은 Topic 계약을 제공한다.

바퀴는 명령 속도를 즉시 따른다고 가정, IMU 없음(휠 각속도로 z축만), 초음파는 고정 거리 8 Hz,
집게는 gripper_move_s 동안 moving 후 open/closed.
"""
import time

from poli_hardware import drive_node


class SimTransport:
    """SIM_ONLY: 모터·엔코더·IMU·초음파·집게 없이 명령을 그대로 따른다고 가정."""

    def __init__(self, node):
        node.declare_parameter('fake_range', 2.0)        # SIM_ONLY: m
        node.declare_parameter('range_rate', 8.0)
        node.declare_parameter('fake_gripper_move_s', 1.0)
        g = node.get_parameter
        self._left = 0.0
        self._right = 0.0
        self._grip_word = None
        self._grip_t = 0.0
        self._grip_move = g('fake_gripper_move_s').value
        mm = int(g('fake_range').value * 1000)
        node.create_timer(1.0 / g('range_rate').value,
                          lambda: node.publish_ranges((mm,) * 4))
        node.get_logger().info('fake_mega_node started (SIM_ONLY)')

    def set_wheel_speeds(self, left, right):
        self._left, self._right = left, right

    def read_wheel_speeds(self):
        return self._left, self._right

    def read_imu(self):
        return None

    def set_gripper(self, word):
        if word != self._grip_word:
            self._grip_word = word
            self._grip_t = time.monotonic()

    def gripper_state(self):
        if self._grip_word is None:
            return 'open'
        if time.monotonic() - self._grip_t < self._grip_move:
            return 'moving'
        return 'closed' if self._grip_word == 'grab' else 'open'

    def close(self):
        pass


def main(args=None):
    drive_node.run('fake_mega_node', SimTransport, args)


if __name__ == '__main__':
    main()
