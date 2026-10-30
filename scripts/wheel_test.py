"""
바퀴 시험 (Pi에서 실행): /cmd_vel로 정해진 짧은 동작만 보내고 멈춘다.

mega_bridge_node가 켜져 있어야 한다. ⚠️ 바퀴를 바닥에서 띄우고 한다.
Ctrl+C나 SSH 끊김으로 이 프로그램이 멈추면 /cmd_vel이 끊기고,
mega_bridge_node가 0.3초 뒤 바퀴를 세운다 (cmd_timeout). 노드가 죽어도 Mega 펌웨어가 0.3초 뒤 멈춘다.
"""
import signal
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.signals import SignalHandlerOptions

RATE_HZ = 20.0
# (설명, linear m/s, angular rad/s, 초). 0.25 m/s = 조원 A가 정한 주행 속도 (RRC 때 기준)
STEPS = [
    ('앞으로', 0.25, 0.0, 2.0),
    ('정지', 0.0, 0.0, 1.0),
    ('뒤로', -0.25, 0.0, 2.0),
    ('정지', 0.0, 0.0, 1.0),
    ('제자리 왼쪽 회전 (오른쪽 바퀴가 앞으로)', 0.0, 1.0, 1.5),
    ('정지', 0.0, 0.0, 1.0),
    ('제자리 오른쪽 회전 (왼쪽 바퀴가 앞으로)', 0.0, -1.0, 1.5),
    ('정지', 0.0, 0.0, 1.0),
]


def raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    # rclpy 기본 Ctrl+C 처리는 통신을 먼저 닫아 버려 정지 명령을 못 보낸다. 그래서 끄고,
    # Ctrl+C·SSH 끊김(SIGHUP)·SIGTERM을 모두 KeyboardInterrupt로 받아 정지 명령부터 보낸다.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGHUP, raise_interrupt)
    signal.signal(signal.SIGTERM, raise_interrupt)
    node = rclpy.create_node('wheel_test')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    # mega_bridge_node와 연결될 시간
    time.sleep(1.0)
    try:
        for name, linear, angular, seconds in STEPS:
            print(f'  {name} ({seconds:.1f}초)', flush=True)
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                pub.publish(msg)
                time.sleep(1.0 / RATE_HZ)
        print('  끝', flush=True)
    except KeyboardInterrupt:
        print('  중단', flush=True)
    finally:
        # 정지 명령을 보내는 동안 또 Ctrl+C가 와도 끊기지 않게 한다.
        for sig in (signal.SIGINT, signal.SIGHUP, signal.SIGTERM):
            signal.signal(sig, signal.SIG_IGN)
        for _ in range(10):
            pub.publish(Twist())
            time.sleep(0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
