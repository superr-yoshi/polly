"""
VL53L0X 거리(ToF) 센서 드라이버. rclpy에 의존하지 않는 순수 모듈 (2026-11-01).

Pololu VL53L0X Arduino 라이브러리(ST API를 줄인 것)의 초기화 순서를 그대로 옮겼다.
bus는 smbus2.SMBus처럼 read_byte_data / write_byte_data / read_i2c_block_data /
write_i2c_block_data를 가진 객체면 된다 (시험에서는 가짜 버스를 넣는다).

배선 (2026-11-01 팀): VCC 파이 1번(3.3V), GND 20번, SDA 11번(GPIO17), SCL 13번(GPIO27)
-> 소프트웨어 I2C 버스 4 (docs/pin_map.md). 주소 0x29.
측정 범위 약 30 mm ~ 1.2 m (실내). 너무 가깝거나(약 30 mm 아래) 물체가 없으면 8190 근처 값을 낸다.
"""
import time

ADDRESS = 0x29

SYSRANGE_START = 0x00
SYSTEM_SEQUENCE_CONFIG = 0x01
SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A
SYSTEM_INTERRUPT_CLEAR = 0x0B
RESULT_INTERRUPT_STATUS = 0x13
RESULT_RANGE_STATUS = 0x14
FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44
MSRC_CONFIG_TIMEOUT_MACROP = 0x46
PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50
PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51
MSRC_CONFIG_CONTROL = 0x60
FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70
FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71
GPIO_HV_MUX_ACTIVE_HIGH = 0x84
VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89
IDENTIFICATION_MODEL_ID = 0xC0
IDENTIFICATION_REVISION_ID = 0xC2
SOFT_RESET_GO2_SOFT_RESET_N = 0xBF
GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0
GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6
DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E
DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F

MODEL_ID = 0xEE
# 이 값 이상이면 '물체 없음 / 범위 밖'으로 본다 (센서가 8190·8191을 낸다)
OUT_OF_RANGE_MM = 8000

# Pololu DefaultTuningSettings (ST API 기본 조정값)
TUNING = (
    (0xFF, 0x01), (0x00, 0x00), (0xFF, 0x00), (0x09, 0x00), (0x10, 0x00), (0x11, 0x00),
    (0x24, 0x01), (0x25, 0xFF), (0x75, 0x00), (0xFF, 0x01), (0x4E, 0x2C), (0x48, 0x00),
    (0x30, 0x20), (0xFF, 0x00), (0x30, 0x09), (0x54, 0x00), (0x31, 0x04), (0x32, 0x03),
    (0x40, 0x83), (0x46, 0x25), (0x60, 0x00), (0x27, 0x00), (0x50, 0x06), (0x51, 0x00),
    (0x52, 0x96), (0x56, 0x08), (0x57, 0x30), (0x61, 0x00), (0x62, 0x00), (0x64, 0x00),
    (0x65, 0x00), (0x66, 0xA0), (0xFF, 0x01), (0x22, 0x32), (0x47, 0x14), (0x49, 0xFF),
    (0x4A, 0x00), (0xFF, 0x00), (0x7A, 0x0A), (0x7B, 0x00), (0x78, 0x21), (0xFF, 0x01),
    (0x23, 0x34), (0x42, 0x00), (0x44, 0xFF), (0x45, 0x26), (0x46, 0x05), (0x40, 0x40),
    (0x0E, 0x06), (0x20, 0x1A), (0x43, 0x40), (0xFF, 0x00), (0x34, 0x03), (0x35, 0x44),
    (0xFF, 0x01), (0x31, 0x04), (0x4B, 0x09), (0x4C, 0x05), (0x4D, 0x04), (0xFF, 0x00),
    (0x44, 0x00), (0x45, 0x20), (0x47, 0x08), (0x48, 0x28), (0x67, 0x00), (0x70, 0x04),
    (0x71, 0x01), (0x72, 0xFE), (0x76, 0x00), (0x77, 0x00), (0xFF, 0x01), (0x0D, 0x01),
    (0xFF, 0x00), (0x80, 0x01), (0x01, 0xF8), (0xFF, 0x01), (0x8E, 0x01), (0x00, 0x01),
    (0xFF, 0x00), (0x80, 0x00),
)

# 측정 시간 계산용 고정 시간 (us, Pololu)
START_OVERHEAD = 1910
END_OVERHEAD = 960
MSRC_OVERHEAD = 660
TCC_OVERHEAD = 590
DSS_OVERHEAD = 690
PRE_RANGE_OVERHEAD = 660
FINAL_RANGE_OVERHEAD = 550


def decode_vcsel_period(reg):
    return (reg + 1) << 1


def macro_period_ns(vcsel_period_pclks):
    return (2304 * vcsel_period_pclks * 1655 + 500) // 1000


def decode_timeout(reg):
    """레지스터 형식 (LSByte * 2^MSByte) + 1 -> MCLK 수."""
    return ((reg & 0x00FF) << ((reg & 0xFF00) >> 8)) + 1


def encode_timeout(mclks):
    if mclks <= 0:
        return 0
    ls_byte = mclks - 1
    ms_byte = 0
    while ls_byte & 0xFFFFFF00:
        ls_byte >>= 1
        ms_byte += 1
    return (ms_byte << 8) | (ls_byte & 0xFF)


def timeout_mclks_to_us(mclks, vcsel_period_pclks):
    return (mclks * macro_period_ns(vcsel_period_pclks) + 500) // 1000


def timeout_us_to_mclks(us, vcsel_period_pclks):
    macro = macro_period_ns(vcsel_period_pclks)
    return (us * 1000 + macro // 2) // macro


def ref_spad_map(spad_map, spad_count, aperture):
    """기준 SPAD 중 앞에서부터 spad_count개만 켠 6바이트 지도 (aperture면 12번부터)."""
    spad_map = list(spad_map)
    first = 12 if aperture else 0
    enabled = 0
    for i in range(48):
        if i < first or enabled == spad_count:
            spad_map[i // 8] &= ~(1 << (i % 8)) & 0xFF
        elif (spad_map[i // 8] >> (i % 8)) & 0x1:
            enabled += 1
    return spad_map


class VL53L0X:
    """연속 측정 모드로 쓰는 VL53L0X. init() 뒤 read_mm()을 주기적으로 부른다."""

    def __init__(self, bus, address=ADDRESS, timeout_s=0.5):
        self.bus = bus
        self.addr = address
        self.timeout_s = timeout_s
        self.stop_variable = 0

    # ---- 레지스터 ----
    def _w(self, reg, value):
        self.bus.write_byte_data(self.addr, reg, value & 0xFF)

    def _r(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def _w16(self, reg, value):
        self.bus.write_i2c_block_data(self.addr, reg, [(value >> 8) & 0xFF, value & 0xFF])

    def _r16(self, reg):
        hi, lo = self.bus.read_i2c_block_data(self.addr, reg, 2)
        return (hi << 8) | lo

    def _wait(self, condition, what):
        end = time.monotonic() + self.timeout_s
        while not condition():
            if time.monotonic() > end:
                raise OSError(f'VL53L0X 시간 초과: {what}')
            time.sleep(0.001)

    # ---- 초기화 ----
    def _soft_reset(self):
        """
        센서를 처음 켠 상태로 되돌린다 (ST API VL53L0X_ResetDevice).

        2026-11-01 실기: 초기화가 도중에 끊긴 센서는 기준 보정 상태(SYSRANGE_START 0x40)에 걸려 다음 초기화가
        전원을 끄기 전까지 계속 시간 초과였다. 리셋 중에는 I2C가 응답하지 않을 수 있어 그 실패는 무시한다.
        """
        def revision(expect_zero):
            try:
                return (self._r(IDENTIFICATION_REVISION_ID) == 0) == expect_zero
            except OSError:
                return False
        self._w(SOFT_RESET_GO2_SOFT_RESET_N, 0x00)
        self._wait(lambda: revision(True), '리셋 들어감')
        self._w(SOFT_RESET_GO2_SOFT_RESET_N, 0x01)
        self._wait(lambda: revision(False), '리셋 끝남')
        time.sleep(0.002)

    def init(self):
        """센서를 리셋·설정하고 연속 측정을 시작한다. 실패하면 OSError."""
        self._soft_reset()
        model = self._r(IDENTIFICATION_MODEL_ID)
        if model != MODEL_ID:
            raise OSError(f'VL53L0X가 아님 (모델 ID 0x{model:02x}, 기대 0xee)')
        # 2.8 V I/O 모드
        pad = self._r(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV)
        self._w(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, pad | 0x01)
        self._w(0x88, 0x00)  # I2C 표준 모드
        self._w(0x80, 0x01)
        self._w(0xFF, 0x01)
        self._w(0x00, 0x00)
        self.stop_variable = self._r(0x91)
        self._w(0x00, 0x01)
        self._w(0xFF, 0x00)
        self._w(0x80, 0x00)
        # SIGNAL_RATE_MSRC, SIGNAL_RATE_PRE_RANGE 한계 검사 끄기
        self._w(MSRC_CONFIG_CONTROL, self._r(MSRC_CONFIG_CONTROL) | 0x12)
        self._w16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, int(0.25 * (1 << 7)))
        self._w(SYSTEM_SEQUENCE_CONFIG, 0xFF)

        count, aperture = self._spad_info()
        spad_map = self.bus.read_i2c_block_data(self.addr, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6)
        self._w(0xFF, 0x01)
        self._w(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)
        self._w(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C)
        self._w(0xFF, 0x00)
        self._w(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)
        self.bus.write_i2c_block_data(self.addr, GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                                      ref_spad_map(spad_map, count, aperture))

        for reg, value in TUNING:
            self._w(reg, value)

        # 측정 완료 인터럽트 (GPIO 핀은 안 쓰지만 상태 레지스터로 본다)
        self._w(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
        self._w(GPIO_HV_MUX_ACTIVE_HIGH, self._r(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10)
        self._w(SYSTEM_INTERRUPT_CLEAR, 0x01)

        budget = self.timing_budget_us()
        # MSRC·TCC 단계를 끄고 남은 시간을 마지막 측정에 다시 나눈다
        self._w(SYSTEM_SEQUENCE_CONFIG, 0xE8)
        self.set_timing_budget_us(budget)
        self._w(SYSTEM_SEQUENCE_CONFIG, 0x01)
        self._single_ref_calibration(0x40)
        self._w(SYSTEM_SEQUENCE_CONFIG, 0x02)
        self._single_ref_calibration(0x00)
        self._w(SYSTEM_SEQUENCE_CONFIG, 0xE8)
        self.start_continuous()

    def _spad_info(self):
        self._w(0x80, 0x01)
        self._w(0xFF, 0x01)
        self._w(0x00, 0x00)
        self._w(0xFF, 0x06)
        self._w(0x83, self._r(0x83) | 0x04)
        self._w(0xFF, 0x07)
        self._w(0x81, 0x01)
        self._w(0x80, 0x01)
        self._w(0x94, 0x6B)
        self._w(0x83, 0x00)
        self._wait(lambda: self._r(0x83) != 0x00, 'SPAD 정보')
        self._w(0x83, 0x01)
        tmp = self._r(0x92)
        self._w(0x81, 0x00)
        self._w(0xFF, 0x06)
        self._w(0x83, self._r(0x83) & ~0x04)
        self._w(0xFF, 0x01)
        self._w(0x00, 0x01)
        self._w(0xFF, 0x00)
        self._w(0x80, 0x00)
        return tmp & 0x7F, bool((tmp >> 7) & 0x01)

    def _single_ref_calibration(self, vhv_init_byte):
        self._w(SYSRANGE_START, 0x01 | vhv_init_byte)
        self._wait(lambda: self._r(RESULT_INTERRUPT_STATUS) & 0x07, '기준 보정')
        self._w(SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._w(SYSRANGE_START, 0x00)

    # ---- 측정 시간 ----
    def _steps(self):
        seq = self._r(SYSTEM_SEQUENCE_CONFIG)
        return {'tcc': (seq >> 4) & 1, 'dss': (seq >> 3) & 1, 'msrc': (seq >> 2) & 1,
                'pre_range': (seq >> 6) & 1, 'final_range': (seq >> 7) & 1}

    def _timeouts(self, pre_range_enabled):
        pre_vcsel = decode_vcsel_period(self._r(PRE_RANGE_CONFIG_VCSEL_PERIOD))
        msrc_mclks = self._r(MSRC_CONFIG_TIMEOUT_MACROP) + 1
        pre_mclks = decode_timeout(self._r16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        final_vcsel = decode_vcsel_period(self._r(FINAL_RANGE_CONFIG_VCSEL_PERIOD))
        final_mclks = decode_timeout(self._r16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        if pre_range_enabled:
            final_mclks -= pre_mclks
        return {
            'msrc_us': timeout_mclks_to_us(msrc_mclks, pre_vcsel),
            'pre_mclks': pre_mclks,
            'pre_us': timeout_mclks_to_us(pre_mclks, pre_vcsel),
            'final_vcsel': final_vcsel,
            'final_us': timeout_mclks_to_us(final_mclks, final_vcsel),
        }

    def _used_us(self, steps, t):
        used = START_OVERHEAD + END_OVERHEAD
        if steps['tcc']:
            used += t['msrc_us'] + TCC_OVERHEAD
        if steps['dss']:
            used += 2 * (t['msrc_us'] + DSS_OVERHEAD)
        elif steps['msrc']:
            used += t['msrc_us'] + MSRC_OVERHEAD
        if steps['pre_range']:
            used += t['pre_us'] + PRE_RANGE_OVERHEAD
        return used

    def timing_budget_us(self):
        steps = self._steps()
        t = self._timeouts(steps['pre_range'])
        budget = self._used_us(steps, t)
        if steps['final_range']:
            budget += t['final_us'] + FINAL_RANGE_OVERHEAD
        return budget

    def set_timing_budget_us(self, budget_us):
        steps = self._steps()
        t = self._timeouts(steps['pre_range'])
        if not steps['final_range']:
            return
        used = self._used_us(steps, t) + FINAL_RANGE_OVERHEAD
        if used > budget_us:
            raise OSError(f'VL53L0X 측정 시간 {budget_us} us가 너무 짧음 (최소 {used} us)')
        mclks = timeout_us_to_mclks(budget_us - used, t['final_vcsel'])
        if steps['pre_range']:
            mclks += t['pre_mclks']
        self._w16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encode_timeout(mclks))

    # ---- 측정 ----
    def start_continuous(self):
        """쉬지 않고 연속 측정 (기본 측정 시간 약 33 ms)."""
        self._w(0x80, 0x01)
        self._w(0xFF, 0x01)
        self._w(0x00, 0x00)
        self._w(0x91, self.stop_variable)
        self._w(0x00, 0x01)
        self._w(0xFF, 0x00)
        self._w(0x80, 0x00)
        self._w(SYSRANGE_START, 0x02)

    def read_mm(self):
        """새 측정값이 있으면 mm, 아직 없으면 None (기다리지 않는다). 범위 밖이면 OUT_OF_RANGE_MM 이상."""
        if not self._r(RESULT_INTERRUPT_STATUS) & 0x07:
            return None
        mm = self._r16(RESULT_RANGE_STATUS + 10)
        self._w(SYSTEM_INTERRUPT_CLEAR, 0x01)
        return mm


class HoldDetector:
    """
    거리로 '집게 안에 물체가 있음'을 판단한다 (깜빡임 방지).

    가까움(<= hold_max_mm)이 on_samples번 이어지면 True, 멂이 off_samples번 이어지면 False.
    처음에는 None (모름).
    범위 밖(None)은 직전 측정값이 가까웠으면 '너무 가까움'(센서 최소 약 30 mm 아래)으로 보고 가까움으로 센다.
    큐브가 빠지면 닫힌 집게 날(2026-11-01 실측 108 mm)이나 멀어지는 값이 먼저 보이기 때문이다.
    """

    def __init__(self, hold_max_mm, on_samples, off_samples):
        self.hold_max_mm = hold_max_mm
        self.on_samples = on_samples
        self.off_samples = off_samples
        self.holding = None
        self._run = 0
        self._run_value = None
        self._last_mm = None

    def add(self, mm):
        if mm is None:
            near = self._last_mm is not None and self._last_mm <= self.hold_max_mm
        else:
            near = mm <= self.hold_max_mm
            self._last_mm = mm
        if near == self._run_value:
            self._run += 1
        else:
            self._run_value, self._run = near, 1
        if near and self._run >= self.on_samples:
            self.holding = True
        elif not near and self._run >= self.off_samples:
            self.holding = False
        return self.holding
