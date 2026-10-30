"""
MPU6050 (GY-521 모듈) 레지스터와 값 변환. rclpy / smbus에 의존하지 않는 순수 모듈.

파이 I2C 버스 1 (GPIO 2 SDA, GPIO 3 SCL), VCC는 파이 3.3V 핀 (docs/pin_map.md).
출력 단위: 가속도 m/s^2, 각속도 rad/s (sensor_msgs/Imu 규칙), 축은 칩에 인쇄된 X·Y·Z 그대로.
로봇에 붙인 방향은 URDF imu_link(rpy)로 맞춘다. 코드에서 축을 돌리지 않는다.
"""
import math
import struct
from typing import List, Optional, Tuple

ADDRESS = 0x68          # AD0 핀이 GND(기본). AD0이 3.3V면 0x69
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B  # 여기서 14바이트: 가속도 xyz, 온도, 자이로 xyz (빅엔디언 int16)
REG_PWR_MGMT_1 = 0x6B
REG_WHO_AM_I = 0x75
WHO_AM_I_MPU6050 = 0x68  # 호환 칩(MPU6500 등)은 다른 값을 낸다 -> 경고만 하고 계속

G = 9.80665

# 측정 범위 -> (설정값, LSB 당 단위) (MPU6050 데이터시트)
GYRO_RANGES = {250: (0, 131.0), 500: (1, 65.5), 1000: (2, 32.8), 2000: (3, 16.4)}   # LSB/(deg/s)
ACCEL_RANGES = {2: (0, 16384.0), 4: (1, 8192.0), 8: (2, 4096.0), 16: (3, 2048.0)}   # LSB/g


def init_writes(gyro_range_dps: int, accel_range_g: int, dlpf: int,
                sample_div: int) -> List[Tuple[int, int]]:
    """
    켜기와 설정에 필요한 (레지스터, 값) 목록.

    dlpf 0~6: 내장 저역 필터 (3 = 약 44 Hz). 출력 주기 = 1 kHz / (1 + sample_div) (dlpf 1~6).
    """
    if gyro_range_dps not in GYRO_RANGES:
        raise ValueError(f'gyro_range_dps must be one of {sorted(GYRO_RANGES)}')
    if accel_range_g not in ACCEL_RANGES:
        raise ValueError(f'accel_range_g must be one of {sorted(ACCEL_RANGES)}')
    if not 0 <= dlpf <= 6 or not 0 <= sample_div <= 255:
        raise ValueError('dlpf 0..6, sample_div 0..255')
    return [
        (REG_PWR_MGMT_1, 0x01),   # 잠자기 해제, 클럭 = 자이로 X PLL (내부 8 MHz보다 안정)
        (REG_SMPLRT_DIV, sample_div),
        (REG_CONFIG, dlpf),
        (REG_GYRO_CONFIG, GYRO_RANGES[gyro_range_dps][0] << 3),
        (REG_ACCEL_CONFIG, ACCEL_RANGES[accel_range_g][0] << 3),
    ]


def parse_block(data: bytes, gyro_range_dps: int,
                accel_range_g: int) -> Tuple[float, float, float, float, float, float]:
    """REG_ACCEL_XOUT_H부터 읽은 14바이트 -> (ax, ay, az [m/s^2], gx, gy, gz [rad/s])."""
    if len(data) != 14:
        raise ValueError(f'need 14 bytes, got {len(data)}')
    ax, ay, az, _temp, gx, gy, gz = struct.unpack('>7h', bytes(data))
    a_lsb = ACCEL_RANGES[accel_range_g][1]
    g_lsb = GYRO_RANGES[gyro_range_dps][1]
    d2r = math.pi / 180.0
    return (ax / a_lsb * G, ay / a_lsb * G, az / a_lsb * G,
            gx / g_lsb * d2r, gy / g_lsb * d2r, gz / g_lsb * d2r)


class GyroBias:
    """
    시작할 때 로봇이 가만히 있는 동안 자이로 영점(치우침)을 잰다.

    samples개를 모아 평균을 영점으로 쓴다. 그동안 각속도 변화가 still_rad_s를 넘으면
    (로봇이 움직였으면) 버리고 처음부터 다시 모은다. 센서 초기화는 시작 선언 전에 해도 된다 (Q7·Q17).
    """

    def __init__(self, samples: int, still_rad_s: float):
        if samples < 1:
            raise ValueError('samples >= 1')
        self.samples = samples
        self.still_rad_s = still_rad_s
        self.bias: Optional[Tuple[float, float, float]] = None
        self.restarts = 0
        self._buf: List[Tuple[float, float, float]] = []

    @property
    def done(self) -> bool:
        return self.bias is not None

    def add(self, gx: float, gy: float, gz: float) -> bool:
        """표본 하나 추가. 영점이 정해지면 True."""
        if self.done:
            return True
        self._buf.append((gx, gy, gz))
        for axis in range(3):
            vals = [s[axis] for s in self._buf]
            if max(vals) - min(vals) > self.still_rad_s:
                self._buf = [(gx, gy, gz)]
                self.restarts += 1
                return False
        if len(self._buf) >= self.samples:
            n = len(self._buf)
            self.bias = tuple(sum(s[i] for s in self._buf) / n for i in range(3))
        return self.done

    def apply(self, gx: float, gy: float, gz: float) -> Tuple[float, float, float]:
        if self.bias is None:
            return gx, gy, gz
        bx, by, bz = self.bias
        return gx - bx, gy - by, gz - bz
