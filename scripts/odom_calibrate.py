#!/usr/bin/env python3
"""
odom 보정 (2026-11-01): 엔코더 /odom_raw와 IMU /imu/data를 실제 거리·각도와 비교해 보정값을 계산한다.

  python3 odom_calibrate.py line [거리 m, 기본 1.0]   # Pi: odom 기준으로 그 거리만큼 0.25 m/s 직진
  python3 odom_calibrate.py spin [바퀴 수, 기본 1]     # Pi: odom 기준으로 제자리 360도 x 바퀴 수
  python3 odom_calibrate.py calc --line ODOM_M REAL_M --spin ODOM_DEG REAL_DEG   # 계산만 (ROS 없이)

line·spin은 mega_bridge_node(hardware.launch.py use_fake_hardware:=false)가 켜져 있어야 한다.
끝나면 odom 값과 IMU 자이로 적분값을 보여 주고, 줄자·각도기로 잰 실제 값을 물어 보정값을 계산한다.
⚠️ 바닥에서 한다. line은 앞쪽을 거리 + 0.5 m 이상 비운다 (장애물 감시 없음, 비상정지에 손을 둔다).
Ctrl+C·SSH 끊김이면 정지 명령을 보내고, 이 프로그램이 죽어도 mega_bridge_node가 0.3초 뒤 바퀴를 세운다.
보정값은 poli_hardware/config/hardware.yaml(wheel_radius, wheel_separation)에 넣는다.
경기장 공개 전(제출 전)에 끝낸다 (규정 1.5).
"""
import math
import os
import re
import signal
import sys
import time

LINE_SPEED = 0.25      # m/s
SPIN_SPEED = 1.5       # rad/s
RATE_HZ = 20.0
TIMEOUT_MARGIN = 3.0   # 명령대로라면 걸릴 시간의 이 배를 넘으면 멈춘다 (바퀴 헛돎·엔코더 없음)
HARDWARE_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ros2_ws', 'src',
                             'poli_hardware', 'config', 'hardware.yaml')


def current_value(name):
    """hardware.yaml에서 지금 값을 읽는다 (없으면 None)."""
    try:
        with open(HARDWARE_YAML, encoding='utf-8') as f:
            match = re.search(rf'^\s*{name}:\s*([0-9.]+)', f.read(), re.MULTILINE)
    except OSError:
        return None
    return float(match.group(1)) if match else None


def corrected_radius(radius, odom_m, real_m):
    """odom이 odom_m이라고 할 때 실제로 real_m 갔으면 바퀴 반지름은 그 비율만큼 크다."""
    return radius * real_m / odom_m


def corrected_separation(separation, odom_deg, real_deg):
    """odom 회전각은 바퀴 간격에 반비례한다. 실제로 덜 돌았으면(odom이 크게 셈) 간격을 늘린다."""
    return separation * odom_deg / real_deg


def calc(args):
    radius = old_radius = current_value('wheel_radius')
    separation = current_value('wheel_separation')
    if '--line' in args:
        i = args.index('--line')
        odom_m, real_m = float(args[i + 1]), float(args[i + 2])
        new = corrected_radius(radius, odom_m, real_m)
        print(f'직진: odom {odom_m:.3f} m, 실제 {real_m:.3f} m ({(real_m / odom_m - 1) * 100:+.1f}%)')
        print(f'  wheel_radius: {radius} -> {new:.4f}')
        radius = new
    if '--spin' in args:
        i = args.index('--spin')
        odom_deg, real_deg = float(args[i + 1]), float(args[i + 2])
        print(f'회전: odom {odom_deg:.1f}도, 실제 {real_deg:.1f}도')
        # 반지름을 고치면 같은 엔코더 값의 odom 회전각도 같은 비율로 바뀐다 (각도 = 바퀴 거리 차 / 간격)
        odom_deg *= radius / old_radius
        new = corrected_separation(separation, odom_deg, real_deg)
        print(f'  wheel_separation: {separation} -> {new:.4f}')


def run_robot(mode, amount):
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Imu

    def raise_interrupt(signum, frame):
        raise KeyboardInterrupt

    # rclpy 기본 Ctrl+C 처리는 통신을 먼저 닫아 정지 명령을 못 보낸다 (wheel_test.py와 같음)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGHUP, raise_interrupt)
    signal.signal(signal.SIGTERM, raise_interrupt)
    node = rclpy.create_node('odom_calibrate')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    state = {'odom': None, 'imu_yaw': 0.0, 'imu_t': None, 'imu_n': 0}

    def on_odom(msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        state['odom'] = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_imu(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if state['imu_t'] is not None and 0.0 < t - state['imu_t'] < 0.5:
            state['imu_yaw'] += msg.angular_velocity.z * (t - state['imu_t'])
        state['imu_t'] = t
        state['imu_n'] += 1

    node.create_subscription(Odometry, '/odom_raw', on_odom, 20)
    node.create_subscription(Imu, '/imu/data', on_imu, 50)
    end = time.monotonic() + 3.0
    while state['odom'] is None and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if state['odom'] is None:
        print('❌ /odom_raw가 안 와요. mega_bridge_node를 먼저 켜세요.')
        return
    x0, y0, yaw0 = state['odom']
    state['imu_yaw'] = 0.0
    turned, last_yaw = 0.0, yaw0
    if mode == 'line':
        target, cmd = amount, (LINE_SPEED, 0.0)
        limit = amount / LINE_SPEED * TIMEOUT_MARGIN
        print(f'  odom 기준 {amount:.2f} m 직진 ({LINE_SPEED} m/s)')
    else:
        target, cmd = 2.0 * math.pi * amount, (0.0, SPIN_SPEED)
        limit = target / SPIN_SPEED * TIMEOUT_MARGIN
        print(f'  odom 기준 {360 * amount:.0f}도 제자리 왼쪽 회전 ({SPIN_SPEED} rad/s)')
    msg = Twist()
    started = time.monotonic()
    try:
        while time.monotonic() - started < limit:
            rclpy.spin_once(node, timeout_sec=0.0)
            x, y, yaw = state['odom']
            turned += math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
            last_yaw = yaw
            done = math.hypot(x - x0, y - y0) if mode == 'line' else abs(turned)
            if done >= target:
                break
            msg.linear.x, msg.angular.z = cmd
            pub.publish(msg)
            time.sleep(1.0 / RATE_HZ)
        else:
            print('  ⚠️ 시간 초과로 멈춤 (바퀴 헛돎·엔코더 확인)')
    except KeyboardInterrupt:
        print('  중단')
    finally:
        for sig in (signal.SIGINT, signal.SIGHUP, signal.SIGTERM):
            signal.signal(sig, signal.SIG_IGN)
        stop = Twist()
        for _ in range(10):
            pub.publish(stop)
            time.sleep(0.05)
    # 멈춘 뒤 밀린 만큼까지 읽는다
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        x, y, yaw = state['odom']
        turned += math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
        last_yaw = yaw
    node.destroy_node()
    rclpy.shutdown()

    imu = f'{math.degrees(state["imu_yaw"]):+.1f}도' if state['imu_n'] else '없음 (/imu/data 안 옴)'
    if mode == 'line':
        odom_m = math.hypot(x - x0, y - y0)
        print(f'  odom 거리 {odom_m:.3f} m, odom 방향 변화 {math.degrees(turned):+.1f}도, IMU {imu}')
        real = input('  줄자로 잰 실제 거리 (m, 엔터 = 건너뜀): ').strip()
        if real:
            calc(['--line', str(odom_m), real])
    else:
        odom_deg = math.degrees(abs(turned))
        print(f'  odom 회전 {odom_deg:.1f}도, IMU 자이로 적분 {imu} '
              '(임무 1은 imu_yaw_sign으로 부호를 맞춘다)')
        real = input('  실제로 돈 각도 (도, 엔터 = 건너뜀): ').strip()
        if real:
            calc(['--spin', str(odom_deg), real])


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('line', 'spin', 'calc'):
        print(__doc__)
        sys.exit(1)
    if sys.argv[1] == 'calc':
        calc(sys.argv[2:])
        return
    amount = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    run_robot(sys.argv[1], amount)


if __name__ == '__main__':
    main()
