#!/usr/bin/env python3
"""
집게 + VL53L0X 확인 (Pi에서 실행): 키로 집게를 열고 닫으며 거리와 잡힘 판단을 본다.

  o = 열기, g = 닫기(잡기), q = 끝
hardware.launch.py(use_fake_hardware:=false)가 켜져 있어야 한다 (mega_bridge_node, tof_node).
노트북에서는 scripts/laptop/gripper_tof.sh가 하드웨어를 켜고 이 프로그램을 실행한 뒤 끈다.
바퀴 명령은 보내지 않는다.
"""
import math
import select
import sys
import termios
import time
import tty

import rclpy
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String


def main():
    rclpy.init()
    node = rclpy.create_node('gripper_tof_console')
    state = {'range': None, 'holding': None, 'grip': None}
    node.create_subscription(Range, '/range/gripper',
                             lambda m: state.__setitem__('range', m.range), 10)
    node.create_subscription(Bool, '/gripper/holding',
                             lambda m: state.__setitem__('holding', m.data), 10)
    node.create_subscription(String, '/gripper/state',
                             lambda m: state.__setitem__('grip', m.data), 10)
    pub = node.create_publisher(String, '/gripper/command', 10)
    print('o = 열기, g = 닫기(잡기), q = 끝.  ⚠ 닫을 때 손가락을 집게 사이에 두지 마세요')
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).lower()
                if key == 'q':
                    break
                if key in ('o', 'g'):
                    command = 'open' if key == 'o' else 'grab'
                    for _ in range(3):
                        pub.publish(String(data=command))
                        time.sleep(0.05)
            r = state['range']
            if r is None:
                dist = '거리 없음 (tof_node 확인)'
            elif math.isinf(r):
                dist = '범위 밖'
            else:
                dist = f'{r * 1000:4.0f} mm'
            holding = {None: '판단 전', True: '잡음', False: '안 잡음'}[state['holding']]
            print(f'\r집게 {state["grip"] or "?":7s} | 거리 {dist:14s} | holding {holding:6s}',
                  end='', flush=True)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
