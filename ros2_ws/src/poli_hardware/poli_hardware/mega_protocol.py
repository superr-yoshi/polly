"""Mega <-> Pi 시리얼 프로토콜 v1.1 (docs/serial_protocol.md).

rclpy / pyserial에 의존하지 않는 순수 모듈이다. PC에서 pytest로 바로 검증한다.
packet 형식을 바꾸면 docs/serial_protocol.md와 펌웨어를 함께 고친다.
"""
from collections import deque
from dataclasses import dataclass
from typing import Optional, Union

# 타입 뒤 필드 수 (seq, millis 포함)
FIELD_COUNT = {'RNG': 6, 'GST': 5}

# RNG 거리값 순서. 토픽 이름과 1:1로 대응한다.
RANGE_ORDER = ('front_left', 'front_right', 'rear_left', 'rear_right')

GRIP_OPEN = 0
GRIP_CLOSE = 1
GRIP_STATE_NAMES = {0: 'open', 1: 'closed', 2: 'moving'}

MAX_LINE = 96


@dataclass(frozen=True)
class RngPacket:
    seq: int
    millis: int
    mm: tuple  # RANGE_ORDER 순서, 0 = 측정 실패 또는 범위 밖


@dataclass(frozen=True)
class GstPacket:
    seq: int
    millis: int
    last_id: int
    state: int  # 0 열림, 1 닫힘, 2 이동 중
    angle: int

    @property
    def state_name(self) -> str:
        return GRIP_STATE_NAMES.get(self.state, 'unknown')


Packet = Union[RngPacket, GstPacket]


def checksum(body: str) -> int:
    """$와 * 사이 모든 문자의 XOR."""
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return cs


def make_packet(body: str) -> bytes:
    return f'${body}*{checksum(body):02X}\r\n'.encode('ascii')


def make_grip_command(cmd_id: int, action: int) -> bytes:
    if action not in (GRIP_OPEN, GRIP_CLOSE):
        raise ValueError(f'action must be 0 or 1, got {action}')
    return make_packet(f'GRIP,{cmd_id},{action}')


def parse_line(line: Union[str, bytes]) -> Optional[Packet]:
    """정상 packet이면 RngPacket/GstPacket, 아니면 None. 예외를 올리지 않는다."""
    if isinstance(line, bytes):
        line = line.decode('ascii', errors='replace')
    line = line.strip()
    if len(line) > MAX_LINE or not line.startswith('$'):
        return None
    body, sep, cs = line[1:].rpartition('*')
    if not sep or len(cs) != 2:
        return None
    try:
        if int(cs, 16) != checksum(body):
            return None
        parts = body.split(',')
        kind = parts[0]
        if FIELD_COUNT.get(kind) != len(parts) - 1:
            return None
        values = [int(x) for x in parts[1:]]
    except ValueError:
        return None
    if any(v < 0 for v in values):
        return None

    if kind == 'RNG':
        return RngPacket(values[0], values[1], tuple(values[2:6]))
    return GstPacket(*values)


class MegaParser:
    """줄 단위 파서 + 진단 카운터 (ok / bad / gap)."""

    def __init__(self):
        self.ok = 0
        self.bad = 0
        self.gap = 0
        self._last_seq = {}

    def feed(self, line: Union[str, bytes]) -> Optional[Packet]:
        pkt = parse_line(line)
        if pkt is None:
            self.bad += 1
            return None
        self.ok += 1
        kind = type(pkt).__name__
        prev = self._last_seq.get(kind)
        # seq가 줄어들면 Mega 리셋으로 보고 누락으로 세지 않는다
        if prev is not None and pkt.seq > prev + 1:
            self.gap += pkt.seq - prev - 1
        self._last_seq[kind] = pkt.seq
        return pkt


def mm_to_range_m(mm: int, min_range: float, max_range: float) -> float:
    """Range.range 값(m). 0(실패)은 REP-117에 따라 +inf(감지 없음)로 변환한다.

    주의: 펌웨어는 20 mm 미만(너무 가까움)도 0으로 보내므로 +inf에 섞인다.
    근접 안전은 LiDAR/Collision Monitor와 함께 판단한다.
    """
    if mm <= 0:
        return float('inf')
    m = mm / 1000.0
    return min(max(m, min_range), max_range)


class RangeMedianFilter:
    """센서별 최근 window개 값의 중간값. 한 번 튀는 값(반사·가장자리)을 걸러낸다.

    0(측정 실패)은 '감지 없음(무한대)'으로 보고 중간값을 구한다. 결과가 무한대면 0을 돌려준다.
    window=1이면 필터 없음. 지연은 약 (window-1)/2 패킷 (8 Hz에서 window 3 = 약 0.125 s).
    """

    def __init__(self, window: int = 3, count: int = len(RANGE_ORDER)):
        if window < 1 or window % 2 == 0:
            raise ValueError('window must be an odd number >= 1')
        self._hist = [deque(maxlen=window) for _ in range(count)]

    def update(self, mm: tuple) -> tuple:
        out = []
        for hist, v in zip(self._hist, mm):
            hist.append(float('inf') if v <= 0 else v)
            s = sorted(hist)
            med = s[len(s) // 2]
            out.append(0 if med == float('inf') else int(med))
        return tuple(out)
