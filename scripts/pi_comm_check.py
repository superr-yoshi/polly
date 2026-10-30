#!/usr/bin/env python3
"""Pi <-> Mega 통신 확인 (듣기 전용).

보드에 아무것도 보내지 않는다. 모터·서보가 움직이지 않는다 (Mega는 첫 명령 전에는 모터를 돌리지 않음).
Mega가 스스로 보내는 엔코더 ENC(약 50 Hz), IMU(MPU-6050 연결 시 약 25 Hz), 초음파 RNG(약 8 Hz)를
받아서 개수·오류·값을 보여 준다. 2026-10-29부터 RRC Lite는 쓰지 않는다 (MDD10 + Mega).

실행 (Pi에서):  python3 ~/polly/scripts/pi_comm_check.py           # 장치 자동 찾기, 각 5초
               python3 ~/polly/scripts/pi_comm_check.py --seconds 10
               python3 ~/polly/scripts/pi_comm_check.py --mega /dev/ttyACM0
준비: pyserial (Ubuntu에 기본 설치됨), 사용자가 dialout 그룹.
"""
import argparse
import os
import sys
import time

import serial
from serial.tools import list_ports

# 패키지의 순수 프로토콜 모듈을 그대로 쓴다 (ROS 없이 import 가능)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2_ws', 'src', 'poli_hardware'))
from poli_hardware import mega_protocol  # noqa: E402

# docs/udev_template.md 확정값 (VID, PID)
MEGA_USB = ('2341', '0042')   # 정품 Mega 2560
MEGA_BAUD = 115200            # docs/serial_protocol.md


def find_port(alias, vid_pid):
    if os.path.exists(alias):
        return alias
    for p in list_ports.comports():
        if p.vid is not None and (f'{p.vid:04x}', f'{p.pid:04x}') == vid_pid:
            return p.device
    return None


def check_mega(port, seconds):
    print(f'\n=== Mega ({port}, {MEGA_BAUD} baud) ===')
    parser = mega_protocol.MegaParser()
    rng_n, last = 0, None
    enc_n, enc_first, enc_last = 0, None, None
    imu_n, imu_last = 0, None
    with serial.Serial(port, MEGA_BAUD, timeout=0.2) as ser:
        print('포트를 열면 Mega가 재시작한다. 2초 기다림...')
        time.sleep(2.0)
        ser.reset_input_buffer()
        end = time.time() + seconds
        while time.time() < end:
            line = ser.readline()
            if not line:
                continue
            pkt = parser.feed(line)
            if isinstance(pkt, mega_protocol.RngPacket):
                rng_n += 1
                last = pkt
            elif isinstance(pkt, mega_protocol.EncPacket):
                enc_n += 1
                enc_first = enc_first or pkt
                enc_last = pkt
            elif isinstance(pkt, mega_protocol.ImuPacket):
                imu_n += 1
                imu_last = pkt
    print(f'정상 {parser.ok}, 오류 {parser.bad}, seq 누락 {parser.gap}')
    print(f'RNG {rng_n}개 = {rng_n / seconds:.1f} Hz (기대 약 8 Hz)')
    if last:
        vals = ', '.join(f'{n}={mm} mm' for n, mm in zip(mega_protocol.RANGE_ORDER, last.mm))
        print(f'  마지막 거리: {vals} (0 = 측정 실패/범위 밖)')
    print(f'ENC {enc_n}개 = {enc_n / seconds:.1f} Hz (기대 약 50 Hz)')
    if enc_last:
        print(f'  엔코더 tick: 왼쪽 {enc_first.left} -> {enc_last.left}, '
              f'오른쪽 {enc_first.right} -> {enc_last.right} '
              '(검사 중 바퀴를 손으로 앞으로 돌리면 둘 다 커져야 정상)')
        print(f'  모터 상태: {enc_last.status_name} (명령을 안 보냈으니 idle이 정상)')
    print(f'IMU {imu_n}개 = {imu_n / seconds:.1f} Hz (MPU-6050 연결 시 약 25 Hz, 없으면 0)')
    if imu_last:
        _, gyro, accel = imu_last.to_si()
        print(f'  가속도 m/s^2 x={accel[0]:+.2f} y={accel[1]:+.2f} z={accel[2]:+.2f} '
              '(평평하게 두면 z 약 +9.8)')
        print(f'  자이로 rad/s x={gyro[0]:+.3f} y={gyro[1]:+.3f} z={gyro[2]:+.3f} (가만히 두면 약 0)')
    ok = (rng_n >= seconds * 4 and enc_n >= seconds * 30 and parser.bad <= 2
          and (enc_last is None or enc_last.status not in mega_protocol.MOTOR_FAULTS))
    print('결과:', 'PASS' if ok else 'FAIL')
    return ok


def main():
    ap = argparse.ArgumentParser(description='Pi <-> Mega 통신 확인 (듣기 전용)')
    ap.add_argument('--mega', help='Mega 포트 (기본: 자동)')
    ap.add_argument('--seconds', type=float, default=5.0)
    args = ap.parse_args()

    print('연결된 시리얼 장치:')
    for p in list_ports.comports():
        vid = f'{p.vid:04x}:{p.pid:04x}' if p.vid is not None else '-'
        print(f'  {p.device}  {vid}  {p.description}')

    results = {}
    for name, given, alias, usb, fn in (
            ('Mega', args.mega, '/dev/robot_mega', MEGA_USB, check_mega),):
        port = given or find_port(alias, usb)
        if port is None:
            print(f'\n=== {name}: 연결 안 됨 -> 건너뜀 ===')
            results[name] = None
            continue
        try:
            results[name] = fn(port, args.seconds)
        except serial.SerialException as e:
            print(f'\n=== {name} ({port}) 열기 실패: {e}')
            if 'ermission' in str(e):
                print('  → dialout 그룹 확인: groups 명령에 dialout이 있어야 함 (추가 후 재로그인)')
            results[name] = False

    print('\n==== 요약 ====')
    for name, ok in results.items():
        print(f'  {name}: {"건너뜀" if ok is None else "PASS" if ok else "FAIL"}')
    tested = [ok for ok in results.values() if ok is not None]
    return 0 if tested and all(tested) else 1


if __name__ == '__main__':
    sys.exit(main())
