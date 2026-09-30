"""rrc_protocol 단위 테스트. ROS·보드 없이 pytest로 실행."""
import math
import struct

from poli_hardware import rrc_protocol as rp
import pytest


def test_crc8_maxim_check_value():
    # CRC-8/MAXIM 표준 검증값. 펌웨어 crc8_table과 256개 값 일치 확인함 (2026-09-29)
    assert rp.crc8(b'123456789') == 0xA1


def test_motor_type_frame():
    f = rp.motor_type_frame(rp.MOTOR_TYPE_JGB37)
    assert f[:6] == bytes([0xAA, 0x55, 0x03, 0x02, 0x05, 0x01])
    assert f[6] == rp.crc8(f[2:6])
    assert len(f) == 7


def test_motor_speeds_frame_layout():
    f = rp.motor_speeds_frame([(0, -1.5), (1, 2.0)])
    func, length, data = f[2], f[3], f[4:-1]
    assert (func, length) == (rp.FUNC_MOTOR, 2 + 5 * 2)
    assert data[:2] == bytes([rp.MOTOR_SET_MULTI, 2])
    assert struct.unpack('<BfBf', data[2:]) == (0, -1.5, 1, 2.0)
    with pytest.raises(ValueError):
        rp.motor_speeds_frame([(4, 1.0)])   # 펌웨어는 범위 검사가 없어 메모리 오류 위험


def test_motor_stop_frame():
    assert rp.motor_stop_frame()[2:6] == bytes([0x03, 0x02, 0x03, 0x0F])


def test_parser_roundtrip_split_and_garbage():
    imu = struct.pack('<6f', 0.0, 0.0, 1.0, 0.0, 0.0, 90.0)
    stream = (b'\x00\xaa\x12' + rp.build_frame(rp.FUNC_IMU, imu)
              + rp.build_frame(rp.FUNC_SYS, struct.pack('<BH', 0x04, 12100)) + b'\xaa')
    p = rp.FrameParser()
    frames = []
    for i in range(len(stream)):          # 1바이트씩 들어와도 동작
        frames += p.feed(stream[i:i + 1])
    assert [f for f, _ in frames] == [rp.FUNC_IMU, rp.FUNC_SYS]
    ax, ay, az, gx, gy, gz = rp.parse_imu(frames[0][1])
    assert az == pytest.approx(9.80665)
    assert gz == pytest.approx(math.pi / 2)
    assert rp.parse_battery_mv(frames[1][1]) == 12100
    assert p.ok == 2 and p.bad == 0


def test_parser_drops_bad_crc_and_recovers():
    good = rp.build_frame(rp.FUNC_IMU, bytes(24))
    bad = bytearray(good)
    bad[-1] ^= 0xFF
    p = rp.FrameParser()
    frames = p.feed(bytes(bad) + good)
    assert len(frames) == 1 and p.bad == 1


def test_parser_rejects_unknown_func():
    p = rp.FrameParser()
    raw = bytes([0xAA, 0x55, 20, 0, 0]) + rp.build_frame(rp.FUNC_MOTOR, b'\x03\x0f')
    assert p.feed(raw) == [(rp.FUNC_MOTOR, b'\x03\x0f')]


def test_parse_imu_wrong_length():
    assert rp.parse_imu(bytes(23)) is None
    assert rp.parse_battery_mv(bytes([0x05, 0, 0])) is None


def test_wheel_to_motor_rps():
    one_rev = 2 * math.pi
    assert rp.wheel_to_motor_rps(one_rev, 1.0, 1980.0, 3.0) == pytest.approx(1.0)
    assert rp.wheel_to_motor_rps(one_rev, -1.0, 1980.0, 3.0) == pytest.approx(-1.0)
    # 실제 모터가 1320 ticks/rev면 펌웨어 목표값을 줄여야 실제 1 rps
    assert rp.wheel_to_motor_rps(one_rev, 1.0, 1320.0, 3.0) == pytest.approx(1320 / 1980)
    assert rp.wheel_to_motor_rps(10 * one_rev, 1.0, 1980.0, 3.0) == pytest.approx(3.0)
    for w, s, t in [(3.0, -1.0, 1980.0), (-5.0, 1.0, 1320.0)]:
        rps = rp.wheel_to_motor_rps(w, s, t, 99.0)
        assert rp.motor_rps_to_wheel(rps, s, t) == pytest.approx(w)
