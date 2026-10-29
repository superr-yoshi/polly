#!/usr/bin/env python3
"""PC에서 Mega와 시리얼 통신을 확인하는 도구 (Pi 대용).

준비:  pip install pyserial
실행:  python pc_serial_tester.py COM3            (Windows, 포트는 Arduino IDE에서 확인)
       python pc_serial_tester.py /dev/ttyACM0    (Linux / Pi)

키보드:  o + Enter = 집게 열기 / c + Enter = 집게 닫기 / q + Enter = 종료
         f + Enter = 두 바퀴 앞으로 1초 (PWM 80, 엔코더 안 봄) / b + Enter = 뒤로 1초
         ⚠️ f/b는 바퀴를 바닥에서 띄우고 한다. 배선·방향 확인용 (docs/pin_map.md)
화면:    RNG·ENC·IMU는 1초에 한 번만 표시, GST는 전부 표시. 체크섬 오류(bad)와 seq 누락(gap)도 센다.
"""
import sys
import threading
import time
from functools import reduce

import serial  # pip install pyserial

FIELDS = {"RNG": 6, "GST": 5, "ENC": 5, "IMU": 12}   # 타입 뒤 필드 수 (seq, millis 포함)
MOTOR = {0: "idle", 1: "drive", 2: "pwm", 3: "FAULT 방향 반대", 4: "FAULT 신호 없음/막힘"}
TEST_DUTY = 80                   # f/b 시험 세기 (-255~255)


def checksum(body: str) -> int:
    return reduce(lambda acc, ch: acc ^ ord(ch), body, 0)


def make_packet(body: str) -> bytes:
    return f"${body}*{checksum(body):02X}\r\n".encode("ascii")


def parse_line(line: str):
    """정상이면 (kind, [int...]), 아니면 None. 예외를 올리지 않는다."""
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None
    body, _, cs = line[1:].rpartition("*")
    if len(cs) != 2:
        return None
    try:
        if int(cs, 16) != checksum(body):
            return None
        parts = body.split(",")
        if FIELDS.get(parts[0]) != len(parts) - 1:
            return None
        return parts[0], [int(x) for x in parts[1:]]
    except ValueError:
        return None


STATE = {0: "OPEN", 1: "CLOSED", 2: "MOVING"}
EVERY = {"RNG": 8, "ENC": 50, "IMU": 25}   # 이 개수마다 한 번 표시 (약 1초)


def reader(ser, stats):
    last_seq = {}
    count = {}
    while not stats["stop"]:
        raw = ser.readline()
        if not raw:
            continue
        pkt = parse_line(raw.decode("ascii", errors="ignore"))
        if pkt is None:
            stats["bad"] += 1
            continue
        kind, v = pkt
        stats["ok"] += 1
        prev = last_seq.get(kind)
        if prev is not None and v[0] != prev + 1:
            stats["gap"] += 1
        last_seq[kind] = v[0]
        count[kind] = count.get(kind, 0) + 1
        show = count[kind] % EVERY.get(kind, 1) == 1 or kind == "GST"
        if not show:
            continue
        if kind == "RNG":
            print(f"[RNG] 전방={v[2]} 좌측={v[3]} 우측={v[4]} 후방={v[5]} (mm, 0=실패)")
        elif kind == "ENC":
            print(f"[ENC] 왼쪽={v[2]} 오른쪽={v[3]} tick  모터={MOTOR.get(v[4], v[4])}")
        elif kind == "IMU":
            print(f"[IMU] 자이로 z={v[8] / 1000:+.3f} rad/s  가속도 z={v[11] / 1000:+.2f} m/s^2")
        elif kind == "GST":
            print(f"[GST] last_cmd_id={v[2]} state={STATE.get(v[3], v[3])} angle={v[4]}")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    ser = serial.Serial(sys.argv[1], 115200, timeout=0.5)
    time.sleep(2.0)                      # 포트를 열면 Mega가 자동 리셋된다
    ser.reset_input_buffer()
    stats = {"ok": 0, "bad": 0, "gap": 0, "stop": False}
    threading.Thread(target=reader, args=(ser, stats), daemon=True).start()
    cmd_id = 0
    print("o=열기  c=닫기  f=앞으로 1초  b=뒤로 1초  q=종료")
    try:
        while True:
            key = input().strip().lower()
            if key == "q":
                break
            if key in ("o", "c"):
                cmd_id += 1
                ser.write(make_packet(f"GRIP,{cmd_id},{1 if key == 'c' else 0}"))
                print(f"-> 명령 전송 id={cmd_id} ({'닫기' if key == 'c' else '열기'})")
            if key in ("f", "b"):
                d = TEST_DUTY if key == "f" else -TEST_DUTY
                print(f"-> PWM {d} 1초 (두 바퀴가 {'앞' if d > 0 else '뒤'}으로, ENC 두 값이 "
                      f"{'커져야' if d > 0 else '작아져야'} 정상)")
                end = time.monotonic() + 1.0
                while time.monotonic() < end:     # Mega는 0.3초 명령이 없으면 멈추므로 계속 보낸다
                    ser.write(make_packet(f"PWM,{d},{d}"))
                    time.sleep(0.1)
                ser.write(make_packet("PWM,0,0"))
    finally:
        stats["stop"] = True
        print(f"통계: ok={stats['ok']} bad={stats['bad']} gap={stats['gap']}")
        ser.close()


if __name__ == "__main__":
    main()
