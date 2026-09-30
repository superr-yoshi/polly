"""
노드 실행/종료 공통 처리.

터미널에서 launch를 Ctrl+C로 끄면 SIGINT가 두 번 온다 (터미널 + launch 전달).
두 번째 신호가 정리 도중에 KeyboardInterrupt를 일으키면 RRC 모터 정지 명령이
빠질 수 있다 (RRC는 명령이 끊겨도 스스로 멈추지 않음). 그래서 정리를 시작하면
추가 SIGINT를 무시하고 cleanup을 끝까지 실행한다.

SIGTERM(systemd stop, launch가 5초 뒤 보내는 신호)은 rclpy가 처리하지 않아
정리 없이 죽는다 -> Ctrl+C와 같은 경로로 정리하도록 바꾼다.
"""
import signal

import rclpy
from rclpy.executors import ExternalShutdownException


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def run_node(node_factory, cleanup=None, args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = None
    try:
        node = node_factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            if cleanup is not None:
                cleanup(node)
            node.destroy_node()
        rclpy.try_shutdown()
