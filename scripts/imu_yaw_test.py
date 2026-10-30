#!/usr/bin/env python3
"""MPU6050 회전 시험: 자이로를 적분해 로봇이 몇 도 돌았는지 보여 준다 (Pi에서 실행, ROS 없이).

배선: VCC 핀 17 (3.3 V), GND 핀 14, SDA 핀 15 (GPIO22), SCL 핀 16 (GPIO23)
설정: config.txt에 dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=22,i2c_gpio_scl=23 -> /dev/i2c-3
시작 후 2초 동안 로봇을 가만히 둔다 (자이로 영점). 그다음 돌리면 각도가 바뀐다.
Enter = 각도를 0으로, Ctrl+C = 끝.

실행 (Pi에서):  python3 imu_yaw_test.py [버스 번호, 기본 3] [주소, 기본 0x68]
임무용 코드가 아니다 (/imu/data 노드는 따로 만든다).
"""
import fcntl
import os
import select
import sys
import time

I2C_SLAVE = 0x0703
PWR_MGMT_1 = 0x6B
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
GYRO_XOUT_H = 0x43
WHO_AM_I = 0x75
GYRO_LSB_PER_DPS = 65.5   # +-500 dps 범위


def write_reg(fd, reg, val):
    os.write(fd, bytes([reg, val]))


def read_regs(fd, reg, n):
    os.write(fd, bytes([reg]))
    return os.read(fd, n)


def read_gyro(fd):
    d = read_regs(fd, GYRO_XOUT_H, 6)
    out = []
    for i in range(0, 6, 2):
        v = (d[i] << 8) | d[i + 1]
        out.append((v - 65536 if v >= 32768 else v) / GYRO_LSB_PER_DPS)
    return out   # x, y, z (도/초)


def main():
    bus = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    addr = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0x68
    fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)
    fcntl.ioctl(fd, I2C_SLAVE, addr)
    who = read_regs(fd, WHO_AM_I, 1)[0]
    print(f'/dev/i2c-{bus} 주소 0x{addr:02x}: WHO_AM_I = 0x{who:02x} (MPU6050이면 0x68)')
    write_reg(fd, PWR_MGMT_1, 0x01)    # 잠 깨우기, 자이로 클럭 사용
    time.sleep(0.1)
    write_reg(fd, CONFIG, 0x03)        # 저역 필터 약 44 Hz
    write_reg(fd, GYRO_CONFIG, 0x08)   # +-500 dps
    time.sleep(0.1)

    print('영점 잡는 중... 2초 동안 로봇을 움직이지 마세요')
    n, s = 0, [0.0, 0.0, 0.0]
    t_end = time.monotonic() + 2.0
    while time.monotonic() < t_end:
        s = [a + b for a, b in zip(s, read_gyro(fd))]
        n += 1
        time.sleep(0.005)
    bias = [a / n for a in s]
    print(f'영점: x {bias[0]:+.2f}, y {bias[1]:+.2f}, z {bias[2]:+.2f} 도/초 ({n}번 읽음)')
    print('이제 돌려 보세요. z가 회전 각도 (칩이 위를 보면 위에서 보아 반시계가 +). '
          'Enter = 0으로, Ctrl+C = 끝')

    ang = [0.0, 0.0, 0.0]
    t_prev = time.monotonic()
    t_print = 0.0
    try:
        while True:
            g = read_gyro(fd)
            t = time.monotonic()
            dt, t_prev = t - t_prev, t
            for i in range(3):
                ang[i] += (g[i] - bias[i]) * dt
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                ang = [0.0, 0.0, 0.0]
                print('0으로 맞춤')
            if t - t_print >= 0.1:
                t_print = t
                print(f'\r회전 z {ang[2]:+8.1f}도   (x {ang[0]:+6.1f}, y {ang[1]:+6.1f})   ',
                      end='', flush=True)
            time.sleep(0.005)
    except KeyboardInterrupt:
        print()
    finally:
        os.close(fd)


if __name__ == '__main__':
    main()
