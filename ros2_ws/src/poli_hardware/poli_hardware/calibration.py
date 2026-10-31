"""
주행 보정 계산 (drive_test가 사용). rclpy에 의존하지 않는 순수 모듈.

/odom_raw는 엔코더 tick을 wheel_radius·wheel_separation·motor_ticks_per_rev로 환산한 값이다.
설정값이 틀리면 그 비율만큼 odom이 틀리므로 "odom 값 vs 실제로 잰 양"의 비율로 파라미터를 고친다.
"""


def corrected_wheel_radius(current: float, odom_distance: float,
                           measured_distance: float) -> float:
    """직진 시험. 실제로 덜 갔으면 실제 바퀴가 설정보다 작다는 뜻 -> radius를 줄인다."""
    _check_positive(odom_distance, measured_distance)
    return current * measured_distance / odom_distance


def corrected_wheel_separation(current: float, odom_angle: float,
                               measured_angle: float) -> float:
    """
    제자리 회전 시험. 실제로 덜 돌았으면 실제 바퀴 간격이 설정보다 넓다 -> separation을 늘린다.

    wheel_radius를 먼저 보정한 뒤에 한다 (radius 오차가 섞이면 separation이 틀어진다).
    """
    _check_positive(abs(odom_angle), abs(measured_angle))
    return current * abs(odom_angle) / abs(measured_angle)


def corrected_ticks_per_rev(current: float, expected_revs: float, counted_revs: float) -> float:
    """바퀴 띄우고 N바퀴 명령 시험. 덜 돌았으면 실제 ticks/rev가 설정보다 크다 (Mega가 목표 tick/s를 맞추므로)."""
    _check_positive(expected_revs, counted_revs)
    return current * expected_revs / counted_revs


def trapezoid(amount: float, speed: float, accel: float):
    """
    급출발·급정지 없이 amount(거리 m 또는 각도 rad)만큼 가는 사다리꼴 속도.

    accel로 올렸다가 같은 accel로 내린다. 짧아서 speed까지 못 올리면 삼각형.
    (최고 속도, 전체 시간, 가속 시간)을 돌려준다. 속도 x 시간 넓이 = amount.
    """
    _check_positive(amount, speed, accel)
    peak = min(speed, (amount * accel) ** 0.5)
    ramp_time = peak / accel
    return peak, amount / peak + ramp_time, ramp_time


def trapezoid_scale(t: float, total: float, ramp_time: float) -> float:
    """사다리꼴 속도에서 시각 t의 속도 / 최고 속도 (0~1)."""
    if t <= 0.0 or t >= total:
        return 0.0
    if ramp_time <= 0.0:
        return 1.0
    return min(1.0, t / ramp_time, (total - t) / ramp_time)


def _check_positive(*values):
    if any(v <= 0 for v in values):
        raise ValueError('측정값은 0보다 커야 합니다')
