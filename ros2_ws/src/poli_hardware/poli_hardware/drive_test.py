"""
보정용 주행 시험 (조원 A). docs/rrc_adapter_plan.md의 실측 순서를 명령 하나로 실행한다.

  ros2 run poli_hardware drive_test straight --distance 1.0 [--speed 0.15]
  ros2 run poli_hardware drive_test rotate --angle 360 [--speed 0.6]
  ros2 run poli_hardware drive_test wheel --revs 10 [--rps 1.0]     # 바퀴를 띄우고!

끝나면 odom(명령 기반)과 IMU 적분값을 보여주고, 실제로 잰 값을 입력하면
config/hardware.yaml에 넣을 새 값을 계산한다. (--measured로 미리 줘도 된다)
Ctrl+C로 언제든 정지 (0 속도 발행, RRC 쪽 watchdog도 0.3초 뒤 정지).
"""
import argparse
import math
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from poli_hardware.calibration import (
    corrected_ticks_per_rev, corrected_wheel_radius, corrected_wheel_separation)
from poli_hardware.diff_drive import DiffDriveParams, wheels_to_twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu
import yaml

RATE = 20.0  # /cmd_vel 발행 Hz (watchdog 0.3 s보다 충분히 빠르게)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DriveTest(Node):

    def __init__(self):
        super().__init__('drive_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom_raw', self._on_odom, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 50)
        self.odom = None
        self.odom_yaw_unwrapped = 0.0
        self._last_yaw = None
        self.imu_yaw = 0.0
        self._imu_t = None

    def _on_odom(self, msg):
        self.odom = msg
        yaw = yaw_of(msg.pose.pose.orientation)
        if self._last_yaw is not None:
            d = math.atan2(math.sin(yaw - self._last_yaw), math.cos(yaw - self._last_yaw))
            self.odom_yaw_unwrapped += d
        self._last_yaw = yaw

    def _on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._imu_t is not None and 0.0 < t - self._imu_t < 0.5:
            self.imu_yaw += msg.angular_velocity.z * (t - self._imu_t)
        self._imu_t = t

    def wait_for_odom(self, timeout=5.0):
        end = time.monotonic() + timeout
        while self.odom is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom is not None

    def settle(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive(self, linear, angular, duration):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = linear, angular
        end = time.monotonic() + duration
        next_pub = 0.0
        try:
            while time.monotonic() < end:
                if time.monotonic() >= next_pub:
                    self.pub.publish(cmd)
                    next_pub = time.monotonic() + 1.0 / RATE
                rclpy.spin_once(self, timeout_sec=0.01)
        finally:
            self.stop()

    def stop(self):
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.02)

    def snapshot(self):
        p = self.odom.pose.pose.position
        return p.x, p.y, self.odom_yaw_unwrapped, self.imu_yaw


def load_current_params():
    """설치된 config/hardware.yaml에서 현재 값을 읽는다 (없으면 기본값)."""
    cur = {'wheel_radius': 0.0325, 'wheel_separation': 0.18, 'motor_ticks_per_rev': 1320.0}
    try:
        share = get_package_share_directory('poli_hardware')
        path = os.path.join(share, 'config', 'hardware.yaml')
        with open(path, encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        for section in ('/**', 'rrc_adapter_node'):
            params = cfg.get(section, {}).get('ros__parameters', {})
            cur.update({k: float(v) for k, v in params.items() if k in cur})
    except Exception as e:  # noqa: BLE001 - 설정 파일이 없어도 시험은 가능
        print(f'  (hardware.yaml 읽기 실패, 기본값 사용: {e})')
    return cur


def ask(prompt, given):
    if given is not None:
        return given
    while True:
        try:
            text = input(prompt).strip()
        except EOFError:
            return None
        if not text:
            return None
        try:
            return float(text)
        except ValueError:
            print('  숫자로 입력하세요 (예: 0.98)')


def countdown(seconds=3):
    for i in range(seconds, 0, -1):
        print(f'  {i}...', flush=True)
        time.sleep(1.0)


def parse_args(argv):
    ap = argparse.ArgumentParser(prog='drive_test', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='mode', required=True)
    s = sub.add_parser('straight', help='직진 -> wheel_radius 보정')
    s.add_argument('--distance', type=float, default=1.0, help='m')
    s.add_argument('--speed', type=float, default=0.15, help='m/s')
    r = sub.add_parser('rotate', help='제자리 회전 -> wheel_separation 보정')
    r.add_argument('--angle', type=float, default=360.0, help='도, +가 반시계(좌회전)')
    r.add_argument('--speed', type=float, default=0.6, help='rad/s')
    w = sub.add_parser('wheel', help='바퀴 띄우고 N바퀴 -> motor_ticks_per_rev 보정')
    w.add_argument('--revs', type=float, default=10.0)
    w.add_argument('--rps', type=float, default=1.0, help='바퀴 초당 회전수')
    for p in (s, r, w):
        p.add_argument('--measured', type=float, default=None,
                       help='실측값 (straight: m, rotate: 도, wheel: 센 바퀴 수). 없으면 끝나고 물어봄')
        p.add_argument('--wheel-radius', type=float, default=None, help='현재 값 (기본: hardware.yaml)')
        p.add_argument('--wheel-separation', type=float, default=None,
                       help='현재 값 (기본: hardware.yaml)')
        p.add_argument('--ticks', type=float, default=None,
                       help='현재 motor_ticks_per_rev (기본: hardware.yaml)')
        p.add_argument('--yes', action='store_true', help='시작 확인 생략')
    a = ap.parse_args(argv)
    cur = load_current_params()
    a.wheel_radius = a.wheel_radius or cur['wheel_radius']
    a.wheel_separation = a.wheel_separation or cur['wheel_separation']
    a.ticks = a.ticks or cur['motor_ticks_per_rev']
    return a


def run(node, a):
    if a.mode == 'straight':
        speed = min(abs(a.speed), 0.25)
        duration = abs(a.distance) / speed
        lin, ang = math.copysign(speed, a.distance), 0.0
        what = f'{a.distance:.2f} m 직진 ({speed} m/s, {duration:.1f} s)'
    elif a.mode == 'rotate':
        rate = min(abs(a.speed), 1.0)
        duration = math.radians(abs(a.angle)) / rate
        lin, ang = 0.0, math.copysign(rate, a.angle)
        what = f'제자리 {a.angle:.0f}도 회전 ({rate} rad/s, {duration:.1f} s)'
    else:
        wheel_rad_s = a.rps * 2.0 * math.pi
        p = DiffDriveParams(a.wheel_radius, a.wheel_separation, 10.0, 10.0, 1e9)
        lin, ang = wheels_to_twist(p, wheel_rad_s, wheel_rad_s)
        duration = a.revs / a.rps
        what = f'바퀴 {a.revs:.0f}바퀴 ({a.rps} rps, {duration:.1f} s) — 바퀴를 띄웠는지 확인!'
        if lin > 0.25:
            print(f'  주의: {lin:.2f} m/s는 max_linear(0.25)에 잘려서 덜 돈다. --rps를 낮추세요.')

    print(f'\n[drive_test] {what}')
    if not a.yes:
        try:
            input('  주변을 비우고 Enter (취소: Ctrl+C) ')
        except EOFError:
            pass
    countdown()

    if not node.wait_for_odom():
        print('  /odom_raw가 안 들어옵니다. hardware.launch.py를 먼저 실행하세요.')
        return 1
    node.settle(0.3)
    x0, y0, yaw0, imu0 = node.snapshot()
    node.drive(lin, ang, duration)
    node.settle(1.0)
    x1, y1, yaw1, imu1 = node.snapshot()

    odom_dist = math.hypot(x1 - x0, y1 - y0)
    odom_deg = math.degrees(yaw1 - yaw0)
    imu_deg = math.degrees(imu1 - imu0)
    print('\n[결과]')
    print(f'  odom 이동거리 : {odom_dist:.3f} m   (명령 기반 추정)')
    print(f'  odom 회전     : {odom_deg:+.1f} 도')
    print(f'  IMU 적분 회전 : {imu_deg:+.1f} 도   (실제 회전에 가까움, fake 모드에선 odom과 같음)')

    if a.mode == 'straight':
        m = ask('  줄자로 잰 실제 이동거리(m), 모르면 Enter: ', a.measured)
        if m:
            new = corrected_wheel_radius(a.wheel_radius, odom_dist, m)
            print(f'\n  -> wheel_radius: {a.wheel_radius} -> {new:.5f}')
    elif a.mode == 'rotate':
        m = ask(f'  실제 회전각(도), Enter면 IMU값 {abs(imu_deg):.1f} 사용: ', a.measured)
        m = abs(m) if m else abs(imu_deg)
        new = corrected_wheel_separation(a.wheel_separation, odom_deg, m)
        print(f'\n  -> wheel_separation: {a.wheel_separation} -> {new:.4f}'
              '  (wheel_radius를 먼저 보정한 뒤의 값이어야 정확)')
    else:
        m = ask('  실제로 센 바퀴 수, 모르면 Enter: ', a.measured)
        if m:
            new = corrected_ticks_per_rev(a.ticks, a.revs, m)
            print(f'\n  -> motor_ticks_per_rev: {a.ticks} -> {new:.1f}'
                  '  (11PPR x 4 x 감속비: 30:1=1320, PPR 12면 1440)')
    print('  config/hardware.yaml에 반영하고 docs/calibration.md에 날짜와 기록하세요.')
    return 0


def main(args=None):
    a = parse_args(remove_ros_args(sys.argv)[1:])
    # rclpy 신호 처리를 끈다: Ctrl+C 때 ROS가 먼저 꺼지면 마지막 정지 명령을 못 보낸다
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DriveTest()
    code = 1
    try:
        code = run(node, a)
    except KeyboardInterrupt:
        print('\n  중단 -> 정지')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
