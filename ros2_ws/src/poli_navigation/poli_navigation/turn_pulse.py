"""
제자리 회전을 짧게 끊어 가는 도우미 (ROS2와 무관한 순수 Python).

조원 A 하드웨어 노드는 바퀴 속도가 min_wheel_speed(0.25 m/s)보다 느린 명령을
두 바퀴 같은 비율로 키워 0.25 m/s로 낸다 (docs/interfaces.md 4장, 2026-10-08).
그래서 제자리 회전은 명령 크기와 상관없이 약 2.8 rad/s가 되고,
제어 주기(0.05 s)마다 약 8도씩 돌아 목표 각도를 지나치기 쉽다.

목표에 가까우면 "PULSE_S 동안 돌고, SETTLE_S 동안 멈춰 센서(IMU·카메라)가
따라오게" 한다. 멈춘 동안에는 LiDAR 벽 보정(wall_localizer)도 동작한다.
"""

import math


# 조원 A hardware.yaml의 min_wheel_speed (m/s). 저쪽 값이 바뀌면 같이 고친다.
HW_MIN_WHEEL_SPEED = 0.25

# TODO_MEASURE: drive_test로 펄스 한 번에 실제로 몇 도 도는지 재고 조정.
# 펄스 한 번 회전량이 회전 허용 오차 범위(±TURN_TOLERANCE)보다 작아야 멈출 수 있다.
PULSE_S = 0.05
SETTLE_S = 0.25

# 오차가 이보다 크면 끊지 않고 계속 돈다 (TODO_MEASURE: 멈출 때 미끄러지는 각도보다 크게)
FINE_TURN_RAD = math.radians(30.0)


class TurnPulse:
    """돌기(pulse) -> 멈추기(settle)를 반복한다. 상태가 바뀌면 reset()."""

    def __init__(self, pulse_s=PULSE_S, settle_s=SETTLE_S):
        self.pulse_s = pulse_s
        self.settle_s = settle_s
        self._started_at = None

    def reset(self):
        self._started_at = None

    def angular(self, direction, speed, now):
        """
        이번 주기의 각속도 명령.

        direction: 양수 = 왼쪽(반시계), 음수 = 오른쪽. speed: 돌 때 각속도 크기.
        """
        if self._started_at is None or now - self._started_at >= self.pulse_s + self.settle_s:
            self._started_at = now
        if now - self._started_at < self.pulse_s:
            return math.copysign(speed, direction)
        return 0.0
