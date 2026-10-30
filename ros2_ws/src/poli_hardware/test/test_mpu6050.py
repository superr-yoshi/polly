"""MPU6050 레지스터 설정·값 변환·자이로 영점 테스트 (센서 없이)."""
import math
import struct

from poli_hardware import mpu6050
import pytest


def block(ax=0, ay=0, az=0, gx=0, gy=0, gz=0, temp=0):
    return struct.pack('>7h', ax, ay, az, temp, gx, gy, gz)


def test_init_writes_wakes_and_sets_ranges():
    writes = dict(mpu6050.init_writes(500, 4, 3, 19))
    assert writes[mpu6050.REG_PWR_MGMT_1] == 0x01        # 잠자기 해제
    assert writes[mpu6050.REG_GYRO_CONFIG] == 1 << 3     # ±500 deg/s
    assert writes[mpu6050.REG_ACCEL_CONFIG] == 1 << 3    # ±4 g
    assert writes[mpu6050.REG_CONFIG] == 3
    assert writes[mpu6050.REG_SMPLRT_DIV] == 19          # 1 kHz / 20 = 50 Hz


def test_init_writes_rejects_bad_ranges():
    with pytest.raises(ValueError):
        mpu6050.init_writes(300, 4, 3, 19)
    with pytest.raises(ValueError):
        mpu6050.init_writes(500, 3, 3, 19)
    with pytest.raises(ValueError):
        mpu6050.init_writes(500, 4, 7, 19)


def test_parse_block_units():
    # ±4 g: 8192 LSB/g, ±500 deg/s: 65.5 LSB/(deg/s)
    ax, ay, az, gx, gy, gz = mpu6050.parse_block(
        block(az=8192, gz=int(65.5 * 90), gx=-655), 500, 4)
    assert az == pytest.approx(mpu6050.G)
    assert ax == 0.0 and ay == 0.0
    assert gz == pytest.approx(math.radians(90), rel=1e-3)
    assert gx == pytest.approx(math.radians(-10), rel=1e-3)
    assert gy == 0.0


def test_parse_block_big_endian_and_length():
    data = bytes([0x01, 0x00]) + bytes(12)            # ax = 0x0100 = 256 (빅엔디언)
    ax = mpu6050.parse_block(data, 250, 2)[0]
    assert ax == pytest.approx(256 / 16384.0 * mpu6050.G)
    with pytest.raises(ValueError):
        mpu6050.parse_block(bytes(13), 250, 2)


def test_gyro_bias_average_when_still():
    b = mpu6050.GyroBias(samples=4, still_rad_s=0.05)
    for gz in (0.010, 0.012, 0.011, 0.011):
        done = b.add(0.0, 0.0, gz)
    assert done and b.done
    assert b.bias[2] == pytest.approx(0.011)
    assert b.apply(0.0, 0.0, 0.511)[2] == pytest.approx(0.5)


def test_gyro_bias_restarts_when_moved():
    b = mpu6050.GyroBias(samples=3, still_rad_s=0.05)
    assert not b.add(0.0, 0.0, 0.0)
    assert not b.add(0.0, 0.0, 0.5)                   # 움직임 -> 처음부터
    assert b.restarts == 1 and not b.done
    b.add(0.0, 0.0, 0.5)
    assert b.add(0.0, 0.0, 0.5)
    assert b.bias[2] == pytest.approx(0.5)


def test_apply_before_calibration_is_identity():
    b = mpu6050.GyroBias(samples=10, still_rad_s=0.05)
    assert b.apply(0.1, 0.2, 0.3) == (0.1, 0.2, 0.3)
