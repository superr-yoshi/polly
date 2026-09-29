"""차동구동 계산: /cmd_vel <-> 바퀴 속도, 휠 오도메트리 적분, cmd_vel watchdog.

rclpy에 의존하지 않는 순수 모듈이다. fake_rrc_node와 실제 RRC 어댑터가 같이 쓴다.
"""
import math
from dataclasses import dataclass


@dataclass
class DiffDriveParams:
    wheel_radius: float        # m
    wheel_separation: float    # m, 좌우 바퀴 중심 간 거리
    max_linear: float          # m/s
    max_angular: float         # rad/s
    max_wheel_rpm: float       # 모터 정격 (JGB37-520 330 RPM)


def clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


def twist_to_wheels(p: DiffDriveParams, linear: float, angular: float):
    """(linear m/s, angular rad/s) -> (left, right) 바퀴 각속도 rad/s.

    속도 제한 후, 한쪽 바퀴가 정격 RPM을 넘으면 두 바퀴를 같은 비율로 줄여
    회전 반경(곡률)을 유지한다.
    """
    v = clamp(linear, p.max_linear)
    w = clamp(angular, p.max_angular)
    half = p.wheel_separation / 2.0
    left = (v - w * half) / p.wheel_radius
    right = (v + w * half) / p.wheel_radius
    max_w = p.max_wheel_rpm * 2.0 * math.pi / 60.0
    peak = max(abs(left), abs(right))
    if peak > max_w:
        scale = max_w / peak
        left *= scale
        right *= scale
    return left, right


def wheels_to_twist(p: DiffDriveParams, left: float, right: float):
    """(left, right) 바퀴 각속도 rad/s -> (linear m/s, angular rad/s)."""
    vl = left * p.wheel_radius
    vr = right * p.wheel_radius
    return (vr + vl) / 2.0, (vr - vl) / p.wheel_separation


def rad_s_to_rpm(w: float) -> float:
    return w * 60.0 / (2.0 * math.pi)


class OdomIntegrator:
    """바퀴 속도로 odom 좌표계의 x, y, yaw를 적분한다 (2D)."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def step(self, linear: float, angular: float, dt: float):
        if dt <= 0.0:
            return
        # 중점 yaw로 적분하면 회전 중 오차가 줄어든다
        mid = self.yaw + angular * dt / 2.0
        self.x += linear * math.cos(mid) * dt
        self.y += linear * math.sin(mid) * dt
        self.yaw = math.atan2(math.sin(self.yaw + angular * dt),
                              math.cos(self.yaw + angular * dt))


def yaw_to_quaternion(yaw: float):
    """(x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class CmdWatchdog:
    """마지막 명령 후 timeout이 지나면 0 속도를 돌려준다.

    Ctrl+C·네트워크 단절이 '마지막 속도 유지'로 이어지지 않게 한다.
    """

    def __init__(self, timeout: float):
        self.timeout = timeout
        self._t = None
        self._cmd = (0.0, 0.0)

    def update(self, now: float, linear: float, angular: float):
        self._t = now
        self._cmd = (linear, angular)

    def command(self, now: float):
        if self._t is None or now - self._t > self.timeout:
            return 0.0, 0.0
        return self._cmd

    def expired(self, now: float) -> bool:
        return self._t is None or now - self._t > self.timeout
