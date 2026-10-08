#!/usr/bin/env python3
"""Pi <-> RRC Lite / Mega 통신 확인 (듣기 전용).

보드에 아무것도 보내지 않는다. 모터·서보가 움직이지 않는다.
RRC Lite는 IMU(약 50 Hz)와 전압(약 1 Hz), Mega는 초음파 RNG(약 8 Hz)를 스스로 보내므로
그것을 받아서 개수·오류·값을 보여 준다.

꽂혀 있는 보드만 검사하고, 안 꽂힌 보드는 건너뛴다.

실행 (Pi에서):  python3 ~/polly/scripts/pi_comm_check.py           # 장치 자동 찾기, 각 5초
               python3 ~/polly/scripts/pi_comm_check.py --seconds 10
               python3 ~/polly/scripts/pi_comm_check.py --rrc /dev/ttyACM0 --mega /dev/ttyACM1
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
from poli_hardware import mega_protocol, rrc_protocol  # noqa: E402

# docs/udev_template.md 확정값 (VID, PID)
RRC_USB = ('1a86', '55d4')    # CH9102
MEGA_USB = ('2341', '0042')   # 정품 Mega 2560
MEGA_BAUD = 115200            # docs/serial_protocol.md


def find_port(alias, vid_pid):
    if os.path.exists(alias):
        return alias
    for p in list_ports.comports():
        if p.vid is not None and (f'{p.vid:04x}', f'{p.pid:04x}') == vid_pid:
            return p.device
    return None


def check_rrc(port, seconds):
    print(f'\n=== RRC Lite ({port}, {rrc_protocol.BAUD} baud) ===')
    parser = rrc_protocol.FrameParser()
    imu_n, last_imu, batt_mv = 0, None, None
    with serial.Serial(port, rrc_protocol.BAUD, timeout=0.1) as ser:
        ser.reset_input_buffer()
        end = time.time() + seconds
        while time.time() < end:
            for func, data in parser.feed(ser.read(512)):
                if func == rrc_protocol.FUNC_IMU:
                    imu = rrc_protocol.parse_imu(data)
                    if imu:
                        imu_n += 1
                        last_imu = imu
                elif func == rrc_protocol.FUNC_SYS:
                    mv = rrc_protocol.parse_battery_mv(data)
                    if mv is not None:
                        batt_mv = mv
    print(f'정상 프레임 {parser.ok}, 오류 {parser.bad}')
    print(f'IMU {imu_n}개 = {imu_n / seconds:.1f} Hz (기대 약 50 Hz)')
    if last_imu:
        ax, ay, az, gx, gy, gz = last_imu
        print(f'  가속도 m/s^2 x={ax:+.2f} y={ay:+.2f} z={az:+.2f} '
              '(평평하게 두면 z 약 +9.8)')
        print(f'  자이로 rad/s x={gx:+.3f} y={gy:+.3f} z={gz:+.3f} (가만히 두면 약 0)')
    if batt_mv is not None:
        print(f'  입력 전압 {batt_mv / 1000:.2f} V (LiPo 잔량 아님. 12 V 스위치 꺼짐이면 낮게 나옴)')
    else:
        print('  전압 보고 없음 (1 Hz라 시간이 짧으면 못 받을 수 있음)')
    ok = imu_n >= seconds * 30 and parser.bad <= max(2, parser.ok // 50)
    print('결과:', 'PASS' if ok else 'FAIL')
    return ok


def check_mega(port, seconds):
    print(f'\n=== Mega ({port}, {MEGA_BAUD} baud) ===')
    parser = mega_protocol.MegaParser()
    rng_n, last = 0, None
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
    print(f'정상 {parser.ok}, 오류 {parser.bad}, seq 누락 {parser.gap}')
    print(f'RNG {rng_n}개 = {rng_n / seconds:.1f} Hz (기대 약 8 Hz)')
    if last:
        vals = ', '.join(f'{n}={mm} mm' for n, mm in zip(mega_protocol.RANGE_ORDER, last.mm))
        print(f'  마지막 거리: {vals} (0 = 측정 실패/범위 밖)')
    ok = rng_n >= seconds * 4 and parser.bad <= 2
    print('결과:', 'PASS' if ok else 'FAIL')
    return ok


def main():
    ap = argparse.ArgumentParser(description='Pi <-> RRC Lite / Mega 통신 확인 (듣기 전용)')
    ap.add_argument('--rrc', help='RRC 포트 (기본: 자동)')
    ap.add_argument('--mega', help='Mega 포트 (기본: 자동)')
    ap.add_argument('--seconds', type=float, default=5.0)
    args = ap.parse_args()

    print('연결된 시리얼 장치:')
    for p in list_ports.comports():
        vid = f'{p.vid:04x}:{p.pid:04x}' if p.vid is not None else '-'
        print(f'  {p.device}  {vid}  {p.description}')

    results = {}
    for name, given, alias, usb, fn in (
            ('RRC Lite', args.rrc, '/dev/robot_rrc', RRC_USB, check_rrc),
            ('Mega', args.mega, '/dev/robot_mega', MEGA_USB, check_mega)):
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
