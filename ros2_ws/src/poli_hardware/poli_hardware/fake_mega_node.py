"""SIM_ONLY: Mega 없이 mega_bridge_node와 같은 Topic·Service 계약을 제공한다.

발행:  /range/* 8 Hz (고정 거리), /gripper/state 2 Hz
서비스: /gripper/set (std_srvs/SetBool: true = 닫기) -> move_time 동안 moving 후 완료
"""
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from poli_hardware.mega_bridge_node import (
    create_range_publishers, declare_range_params, make_range_msg)
from poli_hardware.node_runner import run_node


class FakeMegaNode(Node):

    def __init__(self):
        super().__init__('fake_mega_node')
        self.declare_parameter('fake_range', 2.0)   # SIM_ONLY: m
        self.declare_parameter('range_rate', 8.0)
        self.declare_parameter('move_time', 1.2)    # SIM_ONLY: 집게 이동 시간 s
        declare_range_params(self)

        self.range_pubs = create_range_publishers(self)
        self.state_pub = self.create_publisher(String, '/gripper/state', 10)
        self.create_service(SetBool, '/gripper/set', self._on_gripper_set)
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

    def _on_gripper_set(self, request, response):
        self._target = 'closed' if request.data else 'open'
        if self._done_timer is not None:
            self._done_timer.cancel()
        if self._state != self._target:
            self._state = 'moving'
            self._done_timer = self.create_timer(
                self.get_parameter('move_time').value, self._on_move_done)
        self._publish_state()
        response.success = True
        response.message = f'gripper {"close" if request.data else "open"} accepted (fake)'
        return response

    def _on_move_done(self):
        self._done_timer.cancel()
        self._done_timer = None
        self._state = self._target
        self._publish_state()


def main(args=None):
    run_node(FakeMegaNode, args=args)


if __name__ == '__main__':
    main()
