import math

from poli_navigation.turn_pulse import TurnPulse


def test_pulse_turns_then_settles_then_repeats():
    pulse = TurnPulse(pulse_s=0.05, settle_s=0.25)
    commands = [pulse.angular(1.0, 0.8, step * 0.05) for step in range(13)]

    # 0.05 s 돌고 0.25 s 멈춘 뒤 다시 돈다 (주기 0.3 s)
    assert commands[0] == 0.8
    assert all(c == 0.0 for c in commands[1:6])
    assert commands[6] == 0.8
    assert commands[12] == 0.8


def test_pulse_direction_follows_sign():
    assert TurnPulse().angular(-0.2, 0.8, 0.0) == -0.8
    assert TurnPulse().angular(math.radians(10.0), 0.8, 0.0) == 0.8


def test_reset_starts_new_pulse_immediately():
    pulse = TurnPulse(pulse_s=0.05, settle_s=0.25)
    pulse.angular(1.0, 0.8, 0.0)
    assert pulse.angular(1.0, 0.8, 0.1) == 0.0

    pulse.reset()
    assert pulse.angular(1.0, 0.8, 0.1) == 0.8
