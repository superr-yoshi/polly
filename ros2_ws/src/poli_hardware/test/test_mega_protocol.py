"""mega_protocol 단위 테스트 (매뉴얼 22장 항목 + fuzz). ROS 없이 pytest로 실행."""
import math
import random

from poli_hardware.mega_protocol import (
    checksum, EncPacket, gripper_action, GstPacket, ImuPacket, make_drive_command,
    make_grip_command, make_packet, make_pwm_command, MegaParser, mm_to_range_m, parse_line,
    RANGE_ORDER, RangeMedianFilter, RngPacket, ticks_to_wheel, wheel_to_ticks,
    WheelSpeedEstimator)
import pytest


# docs/serial_protocol.md의 예시 (펌웨어와 같은 체크섬)
@pytest.mark.parametrize('line', [
    '$RNG,5,640,1000,1500,2000,800*63',
    '$GST,3,1500,1,1,120*68',
    '$GRIP,1,1*0C',
    '$GRIP,2,0*0E',
    '$ENC,10,200,1320,-660,1*7B',
    '$IMU,4,160,0,0,0,10000,0,0,12,0,0,9806*57',
    '$DRV,1616,1616*40',
    '$DRV,0,0*40',
    '$PWM,80,-80*67',
])
def test_doc_examples_checksum(line):
    body, cs = line[1:].split('*')
    assert checksum(body) == int(cs, 16)


def test_make_grip_command_matches_doc():
    assert make_grip_command(1, 1) == b'$GRIP,1,1*0C\r\n'
    assert make_grip_command(2, 0) == b'$GRIP,2,0*0E\r\n'
    with pytest.raises(ValueError):
        make_grip_command(3, 2)


# 1. RNG 4개 순서가 front/left/right/rear (배선 D22~D29 순서)와 일치
def test_rng_order():
    assert RANGE_ORDER == ('front', 'left', 'right', 'rear')
    pkt = parse_line('$RNG,5,640,1000,1500,2000,800*63\r\n')
    assert pkt == RngPacket(5, 640, (1000, 1500, 2000, 800))
    assert dict(zip(RANGE_ORDER, pkt.mm))['right'] == 2000


def test_gst_parse():
    pkt = parse_line(b'$GST,3,1500,1,1,120*68\r\n')
    assert pkt == GstPacket(3, 1500, 1, 1, 120)
    assert pkt.state_name == 'closed'


# 2. checksum 오류는 폐기
def test_bad_checksum_dropped():
    assert parse_line('$RNG,5,640,1000,1500,2000,800*64') is None
    assert parse_line('$RNG,5,640,1000,1500,2000,800*ZZ') is None
    assert parse_line('$RNG,5,640,1000,1500,2000,800') is None


# 3. 필드 개수가 틀리면 예외 없이 폐기
@pytest.mark.parametrize('body', [
    'RNG,5,640,1000,1500,2000',
    'RNG,5,640,1000,1500,2000,800,1',
    'GST,3,1500,1,1',
    'IMU,1,2,3',          # v1 형식 (v2.0은 필드 12개)
    'BAT,1,2,24000',      # v1.1에서 삭제된 packet
    'ENC,1,2,3,4',
    'ENC,-1,2,3,4,0',     # seq·millis는 음수 불가
    'GST,3,1500,1,-1,120',  # ENC/IMU 말고는 음수 불가
    'RNG,5,640,a,1500,2000,800',
    'RNG,5,640,1.5,1500,2000,800',
    'RNG,5,640,-1,1500,2000,800',
    '',
])
def test_wrong_fields_dropped(body):
    assert parse_line(make_packet(body)) is None


def test_garbage_dropped():
    for line in ['', '\r\n', 'hello', '$', '*', '$*', '$*00', b'\xff\xfe$RNG*00', 'x' * 500]:
        assert parse_line(line) is None


# 4. timeout/invalid(0) 규칙: REP-117 +inf, 범위 안은 m로 변환
def test_invalid_range_rule():
    assert math.isinf(mm_to_range_m(0, 0.02, 4.0))
    assert mm_to_range_m(1500, 0.02, 4.0) == pytest.approx(1.5)
    assert mm_to_range_m(5000, 0.02, 4.0) == pytest.approx(4.0)


# 5. seq가 건너뛰면 gap 카운터 증가, Mega 리셋(seq 감소)은 gap 아님
def test_seq_gap_counter():
    p = MegaParser()
    for seq in (0, 1, 2, 5):
        p.feed(make_packet(f'RNG,{seq},0,1,2,3,4'))
    assert p.gap == 2
    p.feed(make_packet('RNG,0,0,1,2,3,4'))   # 리셋
    p.feed(make_packet('GST,0,0,0,0,40'))    # 다른 종류는 seq 별도
    p.feed(make_packet('RNG,1,0,1,2,3,4'))
    assert p.gap == 2
    assert p.ok == 7 and p.bad == 0
    p.feed('$RNG,2,0,1,2,3,4*00')
    assert p.bad == 1


# 통과 기준: 샘플 1000개로 crash 없음, 잘못된 packet은 올라가지 않음
def test_fuzz_1000_packets():
    rng = random.Random(1234)
    p = MegaParser()
    valid = 0
    for i in range(1000):
        mm = [rng.randint(0, 4000) for _ in range(4)]
        line = make_packet(f'RNG,{i},{i * 125},{mm[0]},{mm[1]},{mm[2]},{mm[3]}')
        corrupt = rng.random() < 0.3
        if corrupt:
            b = bytearray(line)
            pos = rng.randrange(1, len(b) - 2)
            b[pos] = (b[pos] + rng.randint(1, 254)) % 256
            line = bytes(b)
        pkt = p.feed(line)
        if corrupt:
            # XOR 체크섬은 1바이트 손상을 항상 검출한다
            assert pkt is None
        else:
            valid += 1
            assert pkt == RngPacket(i, i * 125, tuple(mm))
    assert p.ok == valid
    assert p.ok + p.bad == 1000


def test_median_filter_removes_spike_and_dropout():
    f = RangeMedianFilter(3, count=1)
    out = [f.update((v,))[0] for v in (1870, 1870, 1434, 1870, 0, 1870, 1870)]
    assert out[2:] == [1870, 1870, 1870, 1870, 1870]   # 1434 튐, 0 실패 한 번 -> 무시


def test_median_filter_follows_real_change():
    f = RangeMedianFilter(3, count=1)
    out = [f.update((v,))[0] for v in (1870, 1870, 30, 30, 30, 0, 0)]
    assert out[3] == 30          # 두 번 연속이면 반영 (1패킷 지연)
    assert out[-1] == 0          # 실패가 계속되면 0(감지 없음)


def test_median_filter_window_1_is_passthrough_and_validates():
    f = RangeMedianFilter(1)
    assert f.update((1, 2, 0, 4)) == (1, 2, 0, 4)
    with pytest.raises(ValueError):
        RangeMedianFilter(2)


def test_gripper_command_words():
    # docs/interfaces.md: "open" = 열기, "grab" = 닫아서 잡기 (들어 올리지 않음)
    assert gripper_action('open') == 0
    assert gripper_action('grab') == 1
    assert gripper_action(' GRAB\n') == 1
    assert gripper_action('lift') is None
    assert gripper_action('close') is None
    assert gripper_action('') is None


def test_enc_imu_parse_with_negative_values():
    assert parse_line('$ENC,10,200,1320,-660,1*7B') == EncPacket(10, 200, 1320, -660, 1)
    assert parse_line(make_packet('ENC,0,0,0,0,3')).status_name == 'fault_reverse'
    pkt = parse_line('$IMU,4,160,0,0,0,10000,0,0,12,0,0,9806*57')
    assert pkt == ImuPacket(4, 160, (0, 0, 0, 10000), (0, 0, 12), (0, 0, 9806))
    quat, gyro, accel = pkt.to_si()
    assert quat == (0.0, 0.0, 0.0, 1.0)
    assert gyro[2] == pytest.approx(0.012)
    assert accel[2] == pytest.approx(9.806)


def test_drive_and_pwm_commands_match_doc():
    assert make_drive_command(1616, 1616) == b'$DRV,1616,1616*40\r\n'
    assert make_drive_command(0, 0) == b'$DRV,0,0*40\r\n'
    assert make_pwm_command(80, -80) == b'$PWM,80,-80*67\r\n'
    with pytest.raises(ValueError):
        make_pwm_command(256, 0)


def test_wheel_ticks_conversion():
    # 0.25 m/s, 바퀴 반지름 0.0325 m -> 7.69 rad/s -> 1320 tick/rev 기준 약 1616 tick/s
    w = 0.25 / 0.0325
    assert wheel_to_ticks(w, 1320.0) == 1616
    assert wheel_to_ticks(-w, 1320.0) == -1616
    assert ticks_to_wheel(1320.0, 1320.0) == pytest.approx(2 * math.pi)


def test_wheel_speed_estimator():
    est = WheelSpeedEstimator(1320.0)
    assert est.read() == (0.0, 0.0)                 # packet 전
    est.update(EncPacket(0, 1000, 0, 0, 1))
    assert est.read() == (0.0, 0.0)                 # 첫 packet = 기준
    est.update(EncPacket(1, 1020, 13, -13, 1))
    est.update(EncPacket(3, 1500, 660, -330, 1))    # seq 2 누락해도 이동량은 그대로
    left, right = est.read()
    assert left == pytest.approx(math.pi * 2)       # 0.5 s에 반 바퀴 = 1 rev/s
    assert right == pytest.approx(-math.pi)
    assert est.read() == (left, right)              # 새 packet 없으면 지난 값
    est.update(EncPacket(0, 20, 5, 5, 0))           # Mega 리셋 (millis 감소)
    assert est.read() == (0.0, 0.0)
    est.update(EncPacket(1, 520, 665, 5, 1))
    assert est.read()[0] == pytest.approx(math.pi * 2)
