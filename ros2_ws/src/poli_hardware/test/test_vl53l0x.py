"""VL53L0X 드라이버 순수 계산·읽기와 잡힘 판단 시험 (센서 없이)."""
from poli_hardware import vl53l0x


class FakeBus:
    """레지스터 배열만 있는 가짜 I2C 버스."""

    def __init__(self):
        self.regs = [0] * 256
        self.writes = []

    def read_byte_data(self, addr, reg):
        return self.regs[reg]

    def write_byte_data(self, addr, reg, value):
        self.writes.append((reg, value))
        self.regs[reg] = value
        if reg == vl53l0x.SOFT_RESET_GO2_SOFT_RESET_N:
            # 리셋 중에는 리비전 ID가 0, 끝나면 다시 값이 보인다 (실제 칩은 0x10)
            self.regs[vl53l0x.IDENTIFICATION_REVISION_ID] = 0x10 if value else 0x00

    def read_i2c_block_data(self, addr, reg, n):
        return self.regs[reg:reg + n]

    def write_i2c_block_data(self, addr, reg, data):
        self.regs[reg:reg + len(data)] = data


def test_timeout_encode_decode_round_trip():
    for mclks in (1, 2, 100, 255, 256, 1000, 12345, 60000):
        decoded = vl53l0x.decode_timeout(vl53l0x.encode_timeout(mclks))
        # 인코딩은 아래 비트를 버리므로 같거나 조금 작다 (상대 오차 1% 이내)
        assert decoded <= mclks + 1 and decoded >= mclks * 0.99
    assert vl53l0x.encode_timeout(0) == 0


def test_timeout_us_mclks_round_trip():
    vcsel = vl53l0x.decode_vcsel_period(0x06)  # 14 PCLK
    for us in (500, 5000, 30000):
        mclks = vl53l0x.timeout_us_to_mclks(us, vcsel)
        # MCLK 하나(매크로 주기, 약 53 us) 단위로 반올림된다
        half_mclk_us = vl53l0x.macro_period_ns(vcsel) / 1000 / 2
        assert abs(vl53l0x.timeout_mclks_to_us(mclks, vcsel) - us) <= half_mclk_us + 1


def test_ref_spad_map_keeps_only_requested_count():
    full = [0xFF] * 6
    spads = vl53l0x.ref_spad_map(full, 5, aperture=False)
    assert sum(bin(b).count('1') for b in spads) == 5
    assert spads[0] == 0x1F
    # aperture면 12번 SPAD부터 센다
    spads = vl53l0x.ref_spad_map(full, 3, aperture=True)
    assert spads[0] == 0 and spads[1] == 0x70


def test_read_mm_only_when_new_result_ready():
    bus = FakeBus()
    sensor = vl53l0x.VL53L0X(bus)
    assert sensor.read_mm() is None
    bus.regs[vl53l0x.RESULT_INTERRUPT_STATUS] = 0x04
    bus.regs[vl53l0x.RESULT_RANGE_STATUS + 10:vl53l0x.RESULT_RANGE_STATUS + 12] = [0x01, 0x2C]
    assert sensor.read_mm() == 300
    assert (vl53l0x.SYSTEM_INTERRUPT_CLEAR, 0x01) in bus.writes


def test_init_rejects_wrong_model_id():
    bus = FakeBus()
    bus.regs[vl53l0x.IDENTIFICATION_MODEL_ID] = 0x00
    try:
        vl53l0x.VL53L0X(bus).init()
    except OSError as e:
        assert '모델 ID' in str(e)
    else:
        raise AssertionError('OSError expected')


def test_hold_detector_debounces_both_ways():
    det = vl53l0x.HoldDetector(hold_max_mm=60, on_samples=3, off_samples=5)
    assert det.add(200) is None           # 아직 모름 (멂이 5번 안 됨)
    for _ in range(4):
        det.add(200)
    assert det.holding is False
    det.add(40)
    det.add(40)
    assert det.holding is False           # 가까움 2번으로는 안 바뀜
    assert det.add(40) is True
    det.add(None)                         # 범위 밖이 잠깐 튀어도
    det.add(45)
    assert det.holding is True
    # 가까이 있다가 범위 밖 = 너무 가까움 (센서 최소 약 30 mm 아래) -> 계속 잡음
    for _ in range(10):
        det.add(None)
    assert det.holding is True
    # 큐브가 빠지면 닫힌 집게 날(108 mm)이 보인다 -> 안 잡음
    for _ in range(5):
        det.add(108)
    assert det.holding is False
    # 그 뒤 집게를 열어 범위 밖이 되어도 안 잡음 그대로
    for _ in range(5):
        det.add(None)
    assert det.holding is False


def test_init_soft_resets_first():
    """초기화가 도중에 끊겨 보정 상태에 걸린 센서도 다시 쓰도록 init은 먼저 소프트 리셋한다 (2026-11-01 실기)."""
    bus = FakeBus()
    bus.regs[vl53l0x.IDENTIFICATION_MODEL_ID] = 0x00   # 모델 확인에서 멈추게
    try:
        vl53l0x.VL53L0X(bus).init()
    except OSError:
        pass
    reset = vl53l0x.SOFT_RESET_GO2_SOFT_RESET_N
    assert bus.writes[:2] == [(reset, 0x00), (reset, 0x01)]
