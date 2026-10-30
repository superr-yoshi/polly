#!/usr/bin/env python3
"""
PC에서 RRC Lite에 바로 연결해 집게(DS3218)를 시험하는 도구 (파이 없이).

파이의 rrc_adapter_node와 같은 프로토콜 모듈(poli_hardware/rrc_protocol.py)과
같은 설정 파일(poli_hardware/config/hardware.yaml)의 열림·닫힘 값을 쓴다.
바퀴 명령은 보내지 않는다 (연결 직후와 종료 때 '모터 정지'만 보낸다).
첫 명령을 입력하기 전에는 서보를 움직이지 않는다.

준비:  pip install pyserial
       RRC Lite에 12 V 전원, 서보 포트 전원 점퍼 5V (12 V 쪽이면 DS3218 파손)
실행:  python scripts/pc_gripper_test.py              (RRC Lite 자동 찾기)
       python scripts/pc_gripper_test.py --port COM5
입력:  o = 열기 / c = 닫아 잡기 (hardware.yaml 값)
       숫자(500~2500) = 그 펄스로 이동 (닫힘 값 찾기용) / r = 현재 펄스 / v = 전압 / q = 종료
"""
import argparse
import os
import re
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit('pyserial이 없습니다: pip install pyserial')

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', 'poli_hardware'))
from poli_hardware import rrc_protocol  # noqa: E402

CONFIG = os.path.join(ROOT, 'ros2_ws', 'src', 'poli_hardware', 'config', 'hardware.yaml')
RRC_USB = (0x1A86, 0x55D4)   # CH9102 (docs/udev_template.md, scripts/pi_comm_check.py)
PULSE_MIN, PULSE_MAX = 500, 2500   # DS3218 PWM 범위 (AGENTS.md 4장 9)


def load_gripper_config():
    """hardware.yaml에서 집게 값만 읽는다 (PyYAML 없이)."""
    keys = {'gripper_servo_id': 1, 'gripper_open_us': 1500,
            'gripper_close_us': 1833, 'gripper_move_ms': 1000}
    with open(CONFIG, encoding='utf-8') as f:
        text = f.read()
    for k in keys:
        m = re.search(rf'^\s*{k}\s*:\s*(\d+)', text, re.M)
        if m:
            keys[k] = int(m.group(1))
        else:
            print(f'경고: {CONFIG}에 {k} 없음 → 기본값 {keys[k]} 사용')
    return keys


def find_port():
    found = [p for p in list_ports.comports() if (p.vid, p.pid) == RRC_USB]
    if len(found) == 1:
        return found[0].device
    print('RRC Lite(1a86:55d4)를 하나로 특정하지 못했습니다. 지금 보이는 포트:')
    for p in list_ports.comports():
        vid = f'{p.vid:04x}:{p.pid:04x}' if p.vid else '-'
        print(f'  {p.device}  {vid}  {p.description}')
    print('→ --port COM번호 로 지정하세요.')
    return None


class Rrc:
    def __init__(self, port):
        self.ser = serial.Serial(port, rrc_protocol.BAUD, timeout=0.05)
        self.lock = threading.Lock()
        self.parser = rrc_protocol.FrameParser()
        self.servo_us = {}
        self.supply_mv = None
        self.supply_t = 0.0
        self.imu_t = 0.0
        self.stop = False
        threading.Thread(target=self._read_loop, daemon=True).start()

    def write(self, frame):
        with self.lock:
            self.ser.write(frame)

    def _read_loop(self):
        while not self.stop:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as e:
                print(f'\n읽기 오류 (USB 빠짐?): {e}')
                self.stop = True
                return
            for func, data in self.parser.feed(chunk):
                if func == rrc_protocol.FUNC_PWM_SERVO:
                    rep = rrc_protocol.parse_pwm_servo_report(data)
                    if rep is not None:
                        self.servo_us[rep[0]] = rep[1]
                elif func == rrc_protocol.FUNC_SYS:
                    mv = rrc_protocol.parse_battery_mv(data)
                    if mv is not None:
                        self.supply_mv, self.supply_t = mv, time.monotonic()
                elif func == rrc_protocol.FUNC_IMU:
                    self.imu_t = time.monotonic()

    def supply_text(self):
        if self.supply_mv is None or time.monotonic() - self.supply_t > 3.0:
            return '전압 보고 없음'
        return f'RRC 입력 전압 {self.supply_mv / 1000:.2f} V'

    def close(self):
        self.stop = True
        try:
            self.write(rrc_protocol.motor_stop_frame())
            time.sleep(0.05)
        finally:
            self.ser.close()


def move(rrc, cfg, pulse, label):
    if not PULSE_MIN <= pulse <= PULSE_MAX:
        print(f'{PULSE_MIN}~{PULSE_MAX} us만 가능합니다.')
        return
    sid = cfg['gripper_servo_id']
    rrc.servo_us.pop(sid, None)
    rrc.write(rrc_protocol.pwm_servo_set_frame(sid, pulse, cfg['gripper_move_ms']))
    print(f'{label}: 포트 {sid} → {pulse} us ({cfg["gripper_move_ms"]} ms 동안)')
    # 이동 중 명령 위치를 받아 본다 (RRC가 보고하는 값은 서보 실제 위치가 아니라 명령 위치)
    end = time.monotonic() + cfg['gripper_move_ms'] / 1000 + 0.5
    while time.monotonic() < end:
        rrc.write(rrc_protocol.pwm_servo_read_frame(sid))
        time.sleep(0.2)
    got = rrc.servo_us.get(sid)
    if got is None:
        print('  RRC 펄스 보고 없음 (명령은 보냈음. 집게가 움직였는지 눈으로 확인)')
    else:
        print(f'  RRC 보고 펄스 {got} us' + ('' if got == pulse else ' (아직 이동 중?)'))


def main():
    ap = argparse.ArgumentParser(description='PC에서 RRC Lite 집게 시험 (파이 없이)')
    ap.add_argument('--port', help='예: COM5 (없으면 자동 찾기)')
    ap.add_argument('--servo-id', type=int, choices=rrc_protocol.PWM_SERVO_IDS,
                    help='펌웨어 서보 번호 1~4 (없으면 hardware.yaml gripper_servo_id). '
                         '보드 S3~S6 중 어느 것이 몇 번인지 찾을 때')
    args = ap.parse_args()
    cfg = load_gripper_config()
    if args.servo_id:
        cfg['gripper_servo_id'] = args.servo_id
    port = args.port or find_port()
    if port is None:
        return 1
    print(f"설정 (hardware.yaml): 포트 {cfg['gripper_servo_id']}, 열림 {cfg['gripper_open_us']} us, "
          f"닫힘 {cfg['gripper_close_us']} us, 이동 {cfg['gripper_move_ms']} ms")
    print('⚠ 서보 포트 전원 점퍼 5V 확인. 바퀴는 바닥에서 띄워 두기.')
    try:
        rrc = Rrc(port)
    except serial.SerialException as e:
        print(f'{port} 열기 실패: {e} (다른 프로그램이 쓰는 중이거나 드라이버 없음)')
        return 1
    try:
        rrc.write(rrc_protocol.motor_stop_frame())
        time.sleep(1.5)
        imu_ok = time.monotonic() - rrc.imu_t < 1.0
        print(f"연결: {port} | {rrc.supply_text()} | IMU 보고 {'있음' if imu_ok else '없음'}")
        if not imu_ok:
            print('  RRC가 아무것도 보내지 않습니다. 포트·전원·속도(1,000,000 baud)를 확인하세요.')
        print('입력: o 열기 / c 닫기 / 숫자 펄스 / r 현재 펄스 / v 전압 / q 종료')
        while not rrc.stop:
            cmd = input('> ').strip().lower()
            if cmd == 'q':
                break
            if cmd == 'o':
                move(rrc, cfg, cfg['gripper_open_us'], '열기')
            elif cmd == 'c':
                move(rrc, cfg, cfg['gripper_close_us'], '닫아 잡기')
            elif cmd.isdigit():
                move(rrc, cfg, int(cmd), '펄스 지정')
            elif cmd == 'r':
                sid = cfg['gripper_servo_id']
                rrc.write(rrc_protocol.pwm_servo_read_frame(sid))
                time.sleep(0.2)
                print(f'  포트 {sid} 명령 펄스: {rrc.servo_us.get(sid, "보고 없음")}')
            elif cmd == 'v':
                print(f'  {rrc.supply_text()}')
            elif cmd:
                print('o / c / 숫자 / r / v / q 중 하나')
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        rrc.close()
        print('종료 (모터 정지 명령 보냄. 집게는 마지막 위치에 그대로 있음)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
