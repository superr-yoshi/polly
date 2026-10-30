#!/usr/bin/env python3
"""
시험용: 카메라로 주황(또는 빨강) 대상을 찾아 다가가서 집게로 잡는다 (Pi에서 실행, ROS 없이).

Mega에 firmware/motor_test가 올라가 있어야 한다 (바퀴 "왼쪽% 오른쪽% ms", 집게 o/g, 정지 s/x).
전진 = 왼쪽 -, 오른쪽 + (왼쪽 모터가 거꾸로 달려 있다). 제자리 왼쪽 회전 = 둘 다 +.

  python3 grab_demo.py --look   모터 없이 탐지만. 결과를 출력하고 /tmp/grab_look.jpg를 1초마다 저장
  python3 grab_demo.py          찾기 -> 다가가기 -> 마지막 전진 -> 잡기 -> 뒤로 끌기 -> 멈춤
Ctrl+C = 즉시 정지 + 집게 힘 빼기. 전체 시간 제한 90초.
보통은 노트북에서 bash scripts/laptop/grab_demo.sh 로 연다.
임무용 코드가 아니다 (임무는 ROS 노드로 만든다). 숫자는 전부 시험하며 맞출 임시값 (TODO_MEASURE).
"""
import argparse
import math
import signal
import time

import cv2
import numpy as np
from picamera2 import Picamera2
import serial

# OpenCV HSV (H 0~179) 범위. 조명에 따라 --look으로 보고 맞춘다 (TODO_MEASURE)
COLORS = {
    'orange': [((2, 170, 70), (16, 255, 255))],    # 집게의 황동 부품(H 19~21, S 145)은 빠지게
    'red': [((0, 120, 70), (6, 255, 255)), ((170, 120, 70), (179, 255, 255))],
}
W, H = 320, 240
MIN_AREA = 0.0008         # 화면의 0.08%(약 60픽셀)보다 작은 덩어리는 무시 (잡음)
FWD = 30                  # 다가갈 때 전진 힘 (%)
FWD_SLOW = 25             # 대상이 커지면 (가까우면) 천천히
SLOW_AREA = 0.03
STEER_K = 15              # 전진하며 방향 보정 (화면 반폭만큼 옆이면 15%)
ALIGN_ERR = 0.3           # 대상이 이보다 옆에 있으면 멈춰서 제자리 회전
TURN_MIN = 30             # 제자리 회전 최소 힘 (바퀴가 미끄러지며 돌아서 커야 한다)
TURN_K = 10
SEARCH_CMD = '35 35 250'  # 못 찾으면 제자리 왼쪽으로 약 20도씩 돌며 본다
CLOSE_BOTTOM = 0.95       # 대상 아래끝이 화면 맨 아래에 닿거나
CLOSE_AREA = 0.12         # 화면의 12%를 넘으면 마지막 전진
LOST_LOW = 0.8            # 아래끝이 이보다 아래였다가 사라지면 = 가까워서 화면 밑으로 빠진 것


class Mega:
    """motor_test 스케치에 한 줄 명령을 보낸다. 출력은 읽어서 버린다 (안 읽으면 Mega가 멈춘다)."""

    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0)
        time.sleep(2.0)               # 포트를 열면 Mega가 리셋된다
        self.ser.reset_input_buffer()

    def send(self, line):
        self.ser.write((line + '\n').encode())
        self.ser.read(4096)

    def drive(self, fwd, turn, ms=300):
        """fwd·turn은 %. turn이 +면 오른쪽(시계 방향)으로 돈다. ms 안에 다음 명령이 없으면 선다."""
        self.send(f'{round(-(fwd + turn))} {round(fwd - turn)} {ms}')

    def wait(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            self.ser.read(4096)
            time.sleep(0.02)


class Camera:
    def __init__(self, rot180):
        self.rot180 = rot180
        self.cam = Picamera2()
        self.cam.configure(self.cam.create_preview_configuration(
            main={'size': (640, 480), 'format': 'RGB888'}))   # RGB888 = 메모리상 BGR 순서
        self.cam.start()

    def frame(self):
        f = cv2.resize(self.cam.capture_array(), (W, H))
        return cv2.rotate(f, cv2.ROTATE_180) if self.rot180 else f

    def stop(self):
        self.cam.stop()


def detect(frame, ranges):
    """가장 큰 색 덩어리. cx: -1(왼쪽)~+1(오른쪽), area: 화면 비율, bottom: 아래끝 0(위)~1(아래)."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(frame.shape[:2], np.uint8)
    for lo, hi in ranges:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not cnts:
        return None, mask, hsv
    c = max(cnts, key=cv2.contourArea)
    x, y, bw, bh = cv2.boundingRect(c)
    blob = {'cx': (x + bw / 2) / W * 2 - 1, 'area': cv2.contourArea(c) / (W * H),
            'bottom': (y + bh) / H, 'box': (x, y, bw, bh), 'contour': c}
    return blob, mask, hsv


def look(cam, ranges):
    print('탐지만 합니다 (모터 안 움직임). Ctrl+C = 끝. 이미지: /tmp/grab_look.jpg (1초마다)')
    t_save = 0.0
    while True:
        f = cam.frame()
        blob, mask, hsv = detect(f, ranges)
        if blob:
            m = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(m, [blob['contour']], -1, 255, -1)
            h, s, v, _ = cv2.mean(hsv, mask=m)
            ok = blob['area'] >= MIN_AREA
            print(f"{'찾음' if ok else '작음'}: 가로 {blob['cx']:+.2f} (-왼쪽 +오른쪽)  "
                  f"크기 {blob['area'] * 100:.1f}%  아래끝 {blob['bottom']:.2f}  "
                  f"HSV ({h:.0f}, {s:.0f}, {v:.0f})  색 픽셀 전체 {mask.mean() / 2.55:.1f}%")
            x, y, bw, bh = blob['box']
            cv2.rectangle(f, (x, y), (x + bw, y + bh), (0, 255, 0) if ok else (0, 255, 255), 2)
        else:
            print('못 찾음')
        if time.monotonic() - t_save >= 1.0:
            t_save = time.monotonic()
            cv2.imwrite('/tmp/grab_look.jpg', np.hstack([f, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)]))
        time.sleep(0.5)


def run(cam, mega, ranges, a):
    t0 = time.monotonic()
    mega.send('o')
    print('집게 열기')
    mega.wait(1.0)
    state, lost, last = 'search', 0, None
    while time.monotonic() - t0 < a.timeout:
        blob, _, _ = detect(cam.frame(), ranges)
        if blob and blob['area'] < MIN_AREA:
            blob = None
        if state == 'search':
            if blob:
                print(f"찾음: 가로 {blob['cx']:+.2f}, 크기 {blob['area'] * 100:.1f}%")
                state, lost, last = 'approach', 0, blob
                continue
            mega.send(SEARCH_CMD)
            mega.wait(0.6)
            cam.frame()               # 돌면서 흔들린 영상은 버린다
            cam.frame()
            continue
        if blob is None:
            lost += 1
            if last['bottom'] > LOST_LOW:
                print('대상이 화면 아래로 빠짐 -> 마지막 전진')
                break
            if lost >= 5:
                mega.send('s')
                print('놓침 -> 다시 찾기')
                state = 'search'
            mega.wait(0.1)
            continue
        lost, last = 0, blob
        if blob['bottom'] >= CLOSE_BOTTOM or blob['area'] >= CLOSE_AREA:
            print(f"가까움 (아래끝 {blob['bottom']:.2f}, 크기 {blob['area'] * 100:.1f}%) -> 마지막 전진")
            break
        err = blob['cx']
        if abs(err) > ALIGN_ERR:
            mega.drive(0, math.copysign(TURN_MIN + TURN_K * abs(err), err))
        else:
            mega.drive(FWD_SLOW if blob['area'] > SLOW_AREA else FWD, STEER_K * err)
        mega.wait(0.1)
    else:
        mega.send('s')
        print(f'{a.timeout:.0f}초 안에 못 잡음 -> 멈춤')
        return
    mega.send(f'{-a.final_pct} {a.final_pct} {a.final_ms}')
    mega.wait((a.final_ms + a.final_pct * 10) / 1000 + 0.3)   # 램프로 천천히 서는 시간까지
    mega.send('g')
    print('잡기')
    mega.wait(1.5)
    mega.send(f'{a.drag_pct} {-a.drag_pct} {a.drag_ms}')
    print('뒤로 끌기')
    mega.wait((a.drag_ms + a.drag_pct * 10) / 1000 + 0.3)
    print('끝. 대상이 같이 끌려왔는지 보세요. Enter = 집게 열기, Ctrl+C = 힘 빼기')
    input()
    mega.send('o')
    mega.wait(1.2)


def on_hup(*_):
    raise KeyboardInterrupt


def main():
    p = argparse.ArgumentParser(description='카메라로 대상을 찾아 잡는 시험')
    p.add_argument('--look', action='store_true', help='모터 없이 탐지만')
    p.add_argument('--color', choices=sorted(COLORS), default='orange')
    p.add_argument('--rot180', type=int, default=0,
                   help='카메라 케이블이 위로 달렸으면 1 (2026-10-30 케이블 아래로 다시 달아 0)')
    p.add_argument('--final-ms', type=int, default=200, help='마지막 전진 시간 (ms). 집게 사이가 화면에 보여 짧게')
    p.add_argument('--final-pct', type=int, default=30)
    p.add_argument('--drag-ms', type=int, default=800, help='잡고 뒤로 끄는 시간 (ms)')
    p.add_argument('--drag-pct', type=int, default=30)
    p.add_argument('--timeout', type=float, default=90.0)
    p.add_argument('--port', default='/dev/robot_mega')
    a = p.parse_args()
    signal.signal(signal.SIGHUP, on_hup)   # ssh가 끊겨도 멈추고 끝낸다
    cam = Camera(bool(a.rot180))
    mega = None
    try:
        if a.look:
            look(cam, COLORS[a.color])
        else:
            mega = Mega(a.port)
            run(cam, mega, COLORS[a.color], a)
    except KeyboardInterrupt:
        print('\n멈춤')
    finally:
        if mega:
            mega.send('x')            # 모터 정지 + 집게 힘 빼기
            time.sleep(0.2)
        cam.stop()


if __name__ == '__main__':
    main()
