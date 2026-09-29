"""mega_protocol 단위 테스트 (매뉴얼 22장 항목 + fuzz). ROS 없이 pytest로 실행."""
import math
import random

from poli_hardware.mega_protocol import (
    checksum, GstPacket, make_grip_command, make_packet, MegaParser,
    mm_to_range_m, parse_line, RANGE_ORDER, RngPacket)
import pytest


# docs/serial_protocol.md의 예시 (펌웨어와 같은 체크섬)
@pytest.mark.parametrize('line', [
    '$RNG,5,640,1000,1500,2000,800*63',
    '$GST,3,1500,1,1,120*68',
    '$GRIP,1,1*0C',
    '$GRIP,2,0*0E',
])
def test_doc_examples_checksum(line):
    body, cs = line[1:].split('*')
    assert checksum(body) == int(cs, 16)


def test_make_grip_command_matches_doc():
    assert make_grip_command(1, 1) == b'$GRIP,1,1*0C\r\n'
    assert make_grip_command(2, 0) == b'$GRIP,2,0*0E\r\n'
    with pytest.raises(ValueError):
        make_grip_command(3, 2)


# 1. RNG 4개 순서가 front_left/front_right/rear_left/rear_right와 일치
def test_rng_order():
    assert RANGE_ORDER == ('front_left', 'front_right', 'rear_left', 'rear_right')
    pkt = parse_line('$RNG,5,640,1000,1500,2000,800*63\r\n')
    assert pkt == RngPacket(5, 640, (1000, 1500, 2000, 800))
    assert dict(zip(RANGE_ORDER, pkt.mm))['rear_left'] == 2000


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
    'IMU,1,2,3',          # v1.1에서 삭제된 packet
    'BAT,1,2,24000',      # v1.1에서 삭제된 packet
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
