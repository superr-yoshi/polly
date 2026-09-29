#!/usr/bin/env python3
"""PC에서 Mega와 시리얼 통신을 확인하는 도구 (Pi 대용).

준비:  pip install pyserial
실행:  python pc_serial_tester.py COM3            (Windows, 포트는 Arduino IDE에서 확인)
       python pc_serial_tester.py /dev/ttyACM0    (Linux / Pi)

키보드:  o + Enter = 집게 열기 / c + Enter = 집게 닫기 / q + Enter = 종료
화면:    RNG는 1초에 한 번만 표시, GST는 전부 표시. 체크섬 오류(bad)와 seq 누락(gap)도 센다.
"""
import sys
import threading
import time
from functools import reduce

import serial  # pip install pyserial

FIELDS = {"RNG": 6, "GST": 5}   # 타입 뒤 필드 수 (seq, millis 포함)


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


def reader(ser, stats):
    last_seq = {}
    rng_count = 0
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
        if kind == "RNG":
            rng_count += 1
            if rng_count % 8 == 1:
                print(f"[RNG] fl={v[2]} fr={v[3]} rl={v[4]} rr={v[5]} (mm, 0=실패)")
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
    print("o=열기  c=닫기  q=종료")
    try:
        while True:
            key = input().strip().lower()
            if key == "q":
                break
            if key in ("o", "c"):
                cmd_id += 1
                ser.write(make_packet(f"GRIP,{cmd_id},{1 if key == 'c' else 0}"))
                print(f"-> 명령 전송 id={cmd_id} ({'닫기' if key == 'c' else '열기'})")
    finally:
        stats["stop"] = True
        print(f"통계: ok={stats['ok']} bad={stats['bad']} gap={stats['gap']}")
        ser.close()


if __name__ == "__main__":
    main()
