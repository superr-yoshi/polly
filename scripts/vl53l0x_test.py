#!/usr/bin/env python3
"""
VL53L0X 거리 시험 (Pi에서 실행, ROS 없이): 거리를 계속 찍는다. Ctrl+C = 끝.

  python3 ~/polly/scripts/vl53l0x_test.py [버스 번호, 기본 4] [주소, 기본 0x29]

먼저 scripts/pi_setup/vl53l0x_i2c.sh로 버스 4를 켜고 재부팅한다.
집게를 열고 닫으며, 큐브를 넣었다 뺐다 하면서 거리를 보고 hardware.yaml tof_node hold_max_mm를 정한다.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ros2_ws',
                                'src', 'poli_hardware'))
from poli_hardware import vl53l0x  # noqa: E402

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


def main():
    bus_id = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    addr = int(sys.argv[2], 0) if len(sys.argv) > 2 else vl53l0x.ADDRESS
    sensor = vl53l0x.VL53L0X(SMBus(bus_id), addr)
    sensor.init()
    print(f'VL53L0X 시작 (버스 {bus_id}, 0x{addr:02x}, 측정 시간 {sensor.timing_budget_us()} us)')
    try:
        while True:
            mm = sensor.read_mm()
            if mm is not None:
                text = '범위 밖 / 물체 없음' if mm >= vl53l0x.OUT_OF_RANGE_MM else f'{mm:4d} mm'
                print(f'\r{text:20s}', end='', flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    main()
