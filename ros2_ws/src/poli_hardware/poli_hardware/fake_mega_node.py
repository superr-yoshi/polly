"""
SIM_ONLY: Mega 없이 mega_bridge_node와 같은 Topic·Service 계약을 제공한다.

발행:  /range/* 8 Hz (고정 거리), /gripper/state 2 Hz
구독:  /gripper/command ("open" / "grab") -> move_time 동안 moving 후 완료
"""
from poli_hardware.mega_bridge_node import (
    create_range_publishers, declare_range_params, make_range_msg)
from poli_hardware.mega_protocol import GRIP_CLOSE, gripper_action
from poli_hardware.node_runner import run_node
from rclpy.node import Node
from std_msgs.msg import String


class FakeMegaNode(Node):

    def __init__(self):
        super().__init__('fake_mega_node')
        self.declare_parameter('fake_range', 2.0)   # SIM_ONLY: m
        self.declare_parameter('range_rate', 8.0)
        self.declare_parameter('move_time', 1.2)    # SIM_ONLY: 집게 이동 시간 s
        declare_range_params(self)

        self.range_pubs = create_range_publishers(self)
        self.state_pub = self.create_publisher(String, '/gripper/state', 10)
        self.create_subscription(String, '/gripper/command', self._on_gripper_command, 10)
        self.create_timer(1.0 / self.get_parameter('range_rate').value, self._on_range)
        self.create_timer(0.5, self._publish_state)

        self._state = 'open'
        self._target = 'open'
        self._done_timer = None
        self.get_logger().info('fake_mega_node started (SIM_ONLY)')

    def _on_range(self):
        r = self.get_parameter('fake_range').value
        for name, pub in self.range_pubs.items():
            pub.publish(make_range_msg(self, name, r))

    def _publish_state(self):
        self.state_pub.publish(String(data=self._state))

    def _on_gripper_command(self, msg):
        action = gripper_action(msg.data)
        if action is None:
            self.get_logger().warn(f'알 수 없는 집게 명령 "{msg.data}" (open / grab만 가능)')
            return
        target = 'closed' if action == GRIP_CLOSE else 'open'
        if target == self._target:
            # mission 노드는 같은 명령을 1초마다 다시 보낸다. 진행 중인 이동을 다시 시작하지 않는다.
            self._publish_state()
            return
        self.get_logger().info(f'gripper command: {msg.data} (fake)')
        self._target = target
        if self._done_timer is not None:
            self._done_timer.cancel()
        if self._state != self._target:
            self._state = 'moving'
            self._done_timer = self.create_timer(
                self.get_parameter('move_time').value, self._on_move_done)
        self._publish_state()

    def _on_move_done(self):
        self._done_timer.cancel()
        self._done_timer = None
        self._state = self._target
        self._publish_state()


def main(args=None):
    run_node(FakeMegaNode, args=args)


if __name__ == '__main__':
    main()
