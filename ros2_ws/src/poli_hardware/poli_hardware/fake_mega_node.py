"""
SIM_ONLY: Mega 없이 mega_bridge_node와 같은 Topic·Service 계약을 제공한다.

발행:  /range/* 8 Hz (고정 거리). 집게는 fake_rrc_node가 담당한다 (RRC PWM 서보로 이전).
"""
from poli_hardware.mega_bridge_node import (
    create_range_publishers, declare_range_params, make_range_msg)
from poli_hardware.node_runner import run_node
from rclpy.node import Node


class FakeMegaNode(Node):

    def __init__(self):
        super().__init__('fake_mega_node')
        self.declare_parameter('fake_range', 2.0)   # SIM_ONLY: m
        self.declare_parameter('range_rate', 8.0)
        declare_range_params(self)

        self.range_pubs = create_range_publishers(self)
        self.create_timer(1.0 / self.get_parameter('range_rate').value, self._on_range)

        self.get_logger().info('fake_mega_node started (SIM_ONLY)')

    def _on_range(self):
        r = self.get_parameter('fake_range').value
        for name, pub in self.range_pubs.items():
            pub.publish(make_range_msg(self, name, r))


def main(args=None):
    run_node(FakeMegaNode, args=args)


if __name__ == '__main__':
    main()
