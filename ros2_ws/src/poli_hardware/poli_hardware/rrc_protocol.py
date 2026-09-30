"""
Hiwonder RRC Lite (STM32F407VET6) USB Serial 프로토콜.

출처: 제조사 펌웨어 RosRobotControllerLite_ros_250811 (Hiwonder/Misc/packet.c,
System/packet_handle.c, Portings/imu_porting.c). 요약은 docs/rrc_protocol.md.

프레임: AA 55 | func | len | data[len] | crc8(func, len, data)   (1,000,000 baud)
다중 바이트 값은 little-endian, 실수는 float32.
rclpy / pyserial에 의존하지 않는 순수 모듈이다.
"""
import math
import struct
from typing import List, Optional, Tuple

BAUD = 1000000
START = b'\xaa\x55'

FUNC_SYS = 0
FUNC_MOTOR = 3
FUNC_IMU = 7
FUNC_NONE = 12  # 펌웨어는 이 값 이상의 func를 버린다

# 모터 서브 명령 (packet_handle.c packet_motor_handle)
MOTOR_SET_MULTI = 1
MOTOR_STOP_MULTI = 3
MOTOR_SET_TYPE = 5
# RRC -> Pi 엔코더 보고. 제조사 펌웨어에는 없고 POLI 패치 펌웨어(firmware/rrc_lite_patch)만 보낸다.
MOTOR_REPORT_ENCODER = 0x10

# 모터 종류 (motors_param.h). 부팅 직후 기본값은 JGA27 파라미터라서 반드시 설정해야 한다.
MOTOR_TYPE_JGB520 = 0
MOTOR_TYPE_JGB37 = 1   # 45:1 기어, 출력축 1980 ticks/rev, rps 제한 3.0 기준으로 튜닝됨
MOTOR_TYPE_JGA27 = 2
MOTOR_TYPE_JGB528 = 3

FW_JGB37_TICKS = 1980.0  # 펌웨어 MOTOR_JGB37_TICKS_PER_CIRCLE

NUM_MOTORS = 4  # 펌웨어는 motor_id 범위를 검사하지 않는다 -> 여기서 막는다

G = 9.80665


def _make_crc8_table():
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0x8C if c & 1 else c >> 1
        table.append(c)
    return table


_CRC8_TABLE = _make_crc8_table()


def crc8(data: bytes) -> int:
    """CRC-8/MAXIM (펌웨어 checksum_crc8와 동일한 표)."""
    c = 0
    for b in data:
        c = _CRC8_TABLE[c ^ b]
    return c


def build_frame(func: int, data: bytes = b'') -> bytes:
    if not 0 <= func < FUNC_NONE or len(data) > 255:
        raise ValueError('bad func or data length')
    body = bytes([func, len(data)]) + data
    return START + body + bytes([crc8(body)])


def motor_type_frame(motor_type: int) -> bytes:
    return build_frame(FUNC_MOTOR, struct.pack('<BB', MOTOR_SET_TYPE, motor_type))


def motor_speeds_frame(speeds: List[Tuple[int, float]]) -> bytes:
    """[(motor_id 0~3, rps), ...]. rps = 출력축 초당 회전수, 부호는 모터 기준."""
    data = struct.pack('<BB', MOTOR_SET_MULTI, len(speeds))
    for motor_id, rps in speeds:
        if not 0 <= motor_id < NUM_MOTORS:
            raise ValueError(f'motor_id must be 0..3, got {motor_id}')
        data += struct.pack('<Bf', motor_id, rps)
    return build_frame(FUNC_MOTOR, data)


def motor_stop_frame(mask: int = 0x0F) -> bytes:
    return build_frame(FUNC_MOTOR, struct.pack('<BB', MOTOR_STOP_MULTI, mask & 0x0F))


def parse_imu(data: bytes) -> Optional[Tuple[float, ...]]:
    """IMU 보고 -> (ax, ay, az [m/s^2], gx, gy, gz [rad/s]). 펌웨어 단위는 g, deg/s."""
    if len(data) != 24:
        return None
    ax, ay, az, gx, gy, gz = struct.unpack('<6f', data)
    d2r = math.pi / 180.0
    return ax * G, ay * G, az * G, gx * d2r, gy * d2r, gz * d2r


def parse_encoder_report(data: bytes) -> Optional[Tuple[tuple, tuple]]:
    """
    POLI 패치 펌웨어의 엔코더 보고 (func 3, sub 0x10, 50 Hz).

    -> ((count0..3: int32 누적 tick), (rps0..3: 출력축 rev/s, 펌웨어 ticks_per_circle 기준)).
    """
    if len(data) != 33 or data[0] != MOTOR_REPORT_ENCODER:
        return None
    values = struct.unpack('<4i4f', data[1:])
    return values[:4], values[4:]


def parse_battery_mv(data: bytes) -> Optional[int]:
    """SYS 보고 sub 0x04 -> 배터리 전압 mV (약 1 Hz)."""
    if len(data) != 3 or data[0] != 0x04:
        return None
    return struct.unpack('<H', data[1:3])[0]


class FrameParser:
    """바이트 스트림 -> (func, data) 프레임. 펌웨어 packet_recv와 같은 상태기계."""

    def __init__(self):
        self.ok = 0
        self.bad = 0
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> List[Tuple[int, bytes]]:
        self._buf += chunk
        frames = []
        while True:
            i = self._buf.find(START)
            if i < 0:
                # 마지막 바이트가 AA면 다음 청크의 55를 기다린다
                del self._buf[:-1 if self._buf[-1:] == b'\xaa' else len(self._buf)]
                return frames
            del self._buf[:i]
            if len(self._buf) < 4:
                return frames
            func, length = self._buf[2], self._buf[3]
            if func >= FUNC_NONE:
                self.bad += 1
                del self._buf[:2]
                continue
            end = 4 + length + 1
            if len(self._buf) < end:
                return frames
            body = bytes(self._buf[2:4 + length])
            if crc8(body) == self._buf[end - 1]:
                self.ok += 1
                frames.append((func, body[2:]))
                del self._buf[:end]
            else:
                self.bad += 1
                del self._buf[:2]


def wheel_to_motor_rps(wheel_rad_s: float, sign: float, motor_ticks_per_rev: float,
                       limit_rps: float) -> float:
    """
    바퀴 각속도(rad/s, 전진 +) -> RRC 모터 명령 rps.

    펌웨어는 JGB37을 1980 ticks/rev(45:1)로 가정하고 rps를 계산한다. 실제 모터의
    ticks/rev가 다르면 비율만큼 보정해야 실제 바퀴가 원하는 속도로 돈다.
    펌웨어의 모터 명령 경로는 rps 제한을 적용하지 않으므로 여기서 제한한다.
    """
    rps = wheel_rad_s / (2.0 * math.pi) * sign * motor_ticks_per_rev / FW_JGB37_TICKS
    return max(-limit_rps, min(limit_rps, rps))


def motor_rps_to_wheel(rps: float, sign: float, motor_ticks_per_rev: float) -> float:
    """wheel_to_motor_rps의 역변환 (제한 없음)."""
    return rps * FW_JGB37_TICKS / motor_ticks_per_rev * sign * 2.0 * math.pi
