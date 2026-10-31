"""정지 후 스캔 선택을 하드웨어 없이 검증한다."""

from poli_navigation.mission1_node import mapping_scan_ready


def test_rejects_scan_captured_while_robot_was_settling():
    """늦게 수신됐어도 안정 시간 중 찍힌 스캔은 쓰지 않는다."""
    assert not mapping_scan_ready(10.03, 10.30, 10.0)


def test_accepts_first_fresh_scan_after_settling():
    """0.6초를 고정 대기하지 않고 안정 후 첫 스캔으로 진행한다."""
    assert mapping_scan_ready(10.11, 10.21, 10.0)


def test_accepts_post_stop_scan_delivered_late():
    """정지 후 찍은 스캔은 전송이 조금 늦어도(나이 0.3초) 버리지 않는다."""
    assert mapping_scan_ready(10.11, 10.41, 10.0)


def test_motion_restarts_settling_time():
    """차체가 계속 움직였으면 안정 대기를 다시 센다."""
    assert not mapping_scan_ready(10.18, 10.31, 10.15)


def test_rejects_stale_or_future_scan():
    """촬영 시각이 오래됐거나 미래이면 지도 갱신에 쓰지 않는다."""
    assert not mapping_scan_ready(10.21, 10.70, 10.0)
    assert not mapping_scan_ready(10.50, 10.31, 10.0)
