#!/usr/bin/env python3
"""
Mega 바퀴 막힘 처리 시험 (Pi에서 실행, ROS 없이): 두 바퀴를 천천히 돌리며 $ENC status 변화를 찍는다.

  python3 ~/polly/scripts/stall_test.py [초, 기본 20] [tick/s, 기본 600]
⚠ 바퀴를 바닥에서 띄우고 한다. 도는 중 한쪽 바퀴를 손(장갑·천)으로 꽉 잡아 멈추면
  0.3 s 뒤 stall_wait(5) -> 1 s 뒤 다시 돎. 잡았다 놓기를 3번 연속(그 사이 1 s 넘게 안 돌리고) 하면 fault_stall(4).
mega_bridge_node가 켜져 있으면 안 된다 (포트를 같이 쓰면 안 됨). Ctrl+C = 정지 명령 후 끝.
"""
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ros2_ws',
                                'src', 'poli_hardware'))
from poli_hardware.mega_protocol import (  # noqa: E402
    EncPacket, make_drive_command, MegaParser, MOTOR_STATUS_NAMES)


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    tps = int(sys.argv[2]) if len(sys.argv) > 2 else 600
    ser = serial.Serial('/dev/robot_mega', 115200, timeout=0.01)
    time.sleep(2.0)   # 포트를 열면 Mega가 리셋된다
    ser.reset_input_buffer()
    parser = MegaParser()
    print(f'{seconds:.0f}초 동안 두 바퀴 {tps} tick/s. 한쪽 바퀴를 잡아 멈춰 보세요. Ctrl+C = 정지', flush=True)
    start = time.monotonic()
    last_send = 0.0
    last_status = None
    last_enc = None      # (millis, left, right)
    last_print = 0.0
    buf = b''
    try:
        while time.monotonic() - start < seconds:
            now = time.monotonic()
            if now - last_send >= 0.04:   # 25 Hz
                ser.write(make_drive_command(tps, tps))
                last_send = now
            buf += ser.read(256)
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                pkt = parser.feed(line)
                if not isinstance(pkt, EncPacket):
                    continue
                if pkt.status != last_status:
                    name = MOTOR_STATUS_NAMES.get(pkt.status, '?')
                    print(f'\n{now - start:6.2f}s  status {pkt.status} {name}', flush=True)
                    last_status = pkt.status
                # 바퀴 속도 (tick/s): 막힘 기준은 100 tick/s 미만 (거의 완전히 멈춤)
                if last_enc is not None and pkt.millis > last_enc[0] and now - last_print > 0.2:
                    dt = (pkt.millis - last_enc[0]) / 1000.0
                    left = (pkt.left - last_enc[1]) / dt
                    right = (pkt.right - last_enc[2]) / dt
                    print(f'\r  왼쪽 {left:6.0f}  오른쪽 {right:6.0f} tick/s   ', end='', flush=True)
                    last_print = now
                last_enc = (pkt.millis, pkt.left, pkt.right)
    except KeyboardInterrupt:
        print('중단', flush=True)
    finally:
        for _ in range(10):
            ser.write(make_drive_command(0, 0))
            time.sleep(0.03)
        ser.close()
        print('정지 명령 보냄', flush=True)


if __name__ == '__main__':
    main()
