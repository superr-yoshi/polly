"""주행 보정 계산 (drive_test가 사용). rclpy에 의존하지 않는 순수 모듈.

/odom_raw는 명령 기반 추정이라 "명령한 양"과 거의 같다. 그래서 보정은
"명령(odom)한 양 vs 실제로 잰 양"의 비율로 파라미터를 고친다.
"""


def corrected_wheel_radius(current: float, odom_distance: float,
                           measured_distance: float) -> float:
    """직진 시험. 실제로 덜 갔으면 실제 바퀴가 설정보다 작다는 뜻 -> radius를 줄인다."""
    _check_positive(odom_distance, measured_distance)
    return current * measured_distance / odom_distance


def corrected_wheel_separation(current: float, odom_angle: float,
                               measured_angle: float) -> float:
    """제자리 회전 시험. 실제로 덜 돌았으면 실제 바퀴 간격이 설정보다 넓다 -> separation을 늘린다.

    wheel_radius를 먼저 보정한 뒤에 한다 (radius 오차가 섞이면 separation이 틀어진다).
    """
    _check_positive(abs(odom_angle), abs(measured_angle))
    return current * abs(odom_angle) / abs(measured_angle)


def corrected_ticks_per_rev(current: float, expected_revs: float, counted_revs: float) -> float:
    """바퀴 띄우고 N바퀴 명령 시험. 덜 돌았으면 실제 ticks/rev가 설정보다 크다."""
    _check_positive(expected_revs, counted_revs)
    return current * expected_revs / counted_revs


def _check_positive(*values):
    if any(v <= 0 for v in values):
        raise ValueError('측정값은 0보다 커야 합니다')
