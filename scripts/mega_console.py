#!/usr/bin/env python3
"""Mega 시험 스케치(servo_test, motor_test, imu_find)용 줄 단위 시리얼 콘솔 (Pi에서 실행).

Enter를 누르면 친 줄을 한 번에 보낸다 (글자마다 보내는 miniterm·picocom과 다름).
포트를 열면 Mega가 리셋되므로 시험 스케치는 정지 상태로 다시 시작한다.
Ctrl+C나 ssh 끊김으로 끝나면 Mega에 x를 보내고 닫는다 (servo_test: 힘 빼기, motor_test: 모터 정지·집게 힘 빼기).

실행 (Pi에서):  python3 mega_console.py [/dev/robot_mega]
보통은 노트북에서 bash scripts/laptop/mega_test.sh <스케치이름> 으로 연다.
"""
import codecs
import signal
import sys
import threading
import time

import serial


def reader(ser):
    dec = codecs.getincrementaldecoder('utf-8')('replace')  # 한글이 읽기 단위에서 잘려도 깨지지 않게
    while True:
        try:
            data = ser.read(256)
        except Exception:  # 포트를 닫는 중
            return
        if data:
            sys.stdout.write(dec.decode(data))
            sys.stdout.flush()


def on_hup(*_):
    raise KeyboardInterrupt


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/robot_mega'
    ser = serial.Serial(port, 115200, timeout=0.1)
    signal.signal(signal.SIGHUP, on_hup)
    threading.Thread(target=reader, args=(ser,), daemon=True).start()
    print('Mega 리셋 중... 시작 메시지가 나오면 한 줄씩 입력 후 Enter. 끝내기: Ctrl+C')
    time.sleep(2)  # 리셋 직후 부트로더가 도는 동안은 보내지 않는다 (그동안 친 줄은 뒤에 보낸다)
    try:
        while True:
            ser.write((input().strip() + '\n').encode())
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        ser.write(b'x\n')
        time.sleep(0.5)
        ser.close()
        print('\nMega에 x(정지)를 보내고 닫았다.')


if __name__ == '__main__':
    main()
