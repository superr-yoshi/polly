"""
테스트용 하드웨어 속도 모델 (SIM_ONLY).

poli_hardware diff_drive.twist_to_wheels의 최소 바퀴 속도 보정을 흉내 낸다:
빠른 쪽 바퀴가 HW_MIN_WHEEL_SPEED보다 느리면 두 바퀴를 같은 비율로 키운다.
그래서 제자리 회전은 명령과 상관없이 2 * 0.25 / 0.18 = 약 2.8 rad/s가 된다.
"""

from poli_navigation.turn_pulse import HW_MIN_WHEEL_SPEED

# SIM_ONLY: 조원 A hardware.yaml wheel_separation (실측 전 값)
WHEEL_SEPARATION_M = 0.18
STOP_SPEED = 0.005


def actual_twist(linear, angular):
    """보낸 (linear, angular) 명령 -> 실제로 나오는 (linear, angular)."""
    half = WHEEL_SEPARATION_M / 2.0
    left = linear - angular * half
    right = linear + angular * half
    peak = max(abs(left), abs(right))
    if peak <= STOP_SPEED:
        return 0.0, 0.0
    if peak < HW_MIN_WHEEL_SPEED:
        scale = HW_MIN_WHEEL_SPEED / peak
        left *= scale
        right *= scale
    return (left + right) / 2.0, (right - left) / WHEEL_SEPARATION_M
