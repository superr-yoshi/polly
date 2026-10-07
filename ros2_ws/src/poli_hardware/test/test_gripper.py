"""gripper 상태 로직 테스트."""
from poli_hardware.gripper import GripperController
import pytest


def test_first_state_and_commands():
    g = GripperController(1500, 1833, 1000)
    assert g.state(0.0) == 'open'                 # 첫 명령 전
    assert g.command('grab', 1.0) == 1833
    assert g.state(1.2) == 'moving'
    g.report(1700)
    assert g.state(1.5) == 'moving'
    g.report(1833)
    assert g.state(1.6) == 'closed'


def test_repeat_same_command_is_ignored():
    # 임무 노드는 같은 명령을 1초마다 다시 보낸다
    g = GripperController(1500, 1833, 1000)
    assert g.command('grab', 0.0) == 1833
    assert g.command(' GRAB ', 0.5) is None
    assert g.state(0.6) == 'moving'


def test_timeout_without_report_and_open():
    g = GripperController(1500, 1833, 1000)
    g.command('grab', 0.0)
    assert g.state(1.4) == 'moving'
    assert g.state(1.6) == 'closed'               # 이동 1.0 s + 여유 0.5 s
    assert g.command('open', 2.0) == 1500
    assert g.state(3.6) == 'open'


def test_unknown_command():
    with pytest.raises(ValueError):
        GripperController(1500, 1833, 1000).command('lift', 0.0)
