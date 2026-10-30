"""
앞으로 가기 시험 (Pi에서 실행): 0.25 m/s로 약 0.5 m 전진하고, 실제로 간 거리를 LiDAR로 잰다.

mega_bridge_node, LiDAR(/scan), robot_state_publisher(TF base_link -> laser)가 켜져 있어야 한다.
노트북에서는 scripts/laptop/forward_test.sh로 RViz(LiDAR·카메라)를 보면서 실행한다.
⚠️ 바닥에서 한다. 앞쪽 1 m 이상을 비우고, 1~3 m 앞에 벽이나 상자를 둔다 (그 거리 변화를 잰다).
달리는 중 앞쪽 0.35 m 안에 뭐가 보이거나 LiDAR가 끊기면 바로 멈춘다.
Ctrl+C나 SSH 끊김으로 이 프로그램이 멈추면 /cmd_vel이 끊기고,
mega_bridge_node가 0.3초 뒤 바퀴를 세운다 (cmd_timeout).
"""
import math
import signal
import statistics
import sys
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

SPEED = 0.25            # m/s. 조원 A가 정한 주행 속도 (hardware.yaml max_linear)
DRIVE_S = 2.0           # 초. 0.25 m/s x 2초 = 약 0.5 m
RATE_HZ = 20.0          # /cmd_vel 보내는 횟수 (mega_bridge_node cmd_timeout 0.3초보다 자주)
MIN_FRONT_CLEAR = 1.0   # m. 출발 전 앞쪽이 이만큼 비어 있어야 출발한다
STOP_DIST = 0.35        # m. 달리는 중 앞쪽에 이보다 가까운 게 보이면 멈춘다
SAFETY_HALF_DEG = 30.0  # 출발·멈춤 판단에 쓰는 앞쪽 범위 (정면 ±30도)
MEASURE_HALF_DEG = 10.0  # 거리 변화는 앞·뒤·왼쪽·오른쪽 ±10도 거리의 중간값으로 잰다
# LiDAR에서 이보다 가까운 값은 로봇 몸체(카메라·집게 등)에 맞은 것으로 보고 무시한다.
# TODO_MEASURE: 조립 후 LiDAR가 보는 가장 먼 몸체 부품까지 거리 + 여유
SELF_HIT_M = 0.15
SCAN_STALE_S = 0.5      # LiDAR가 이 시간 넘게 안 오면 멈춘다
MEASURE_S = 1.0         # 출발 전·멈춘 뒤 이 시간 동안 LiDAR를 모아 거리를 잰다
SETTLE_S = 1.0          # 멈추고 흔들림이 가라앉을 때까지 기다리는 시간
WAIT_S = 5.0            # LiDAR·TF가 처음 들어올 때까지 기다리는 시간
TURN_WARN_DEG = 5.0     # 달리는 동안 방향이 이보다 많이 틀어지면 알린다

BASE_FRAME = 'base_link'
# 로봇 기준 방향 (도, ROS 기준: 0 = 앞, +90 = 왼쪽)
DIRECTIONS = {'앞': 0.0, '왼쪽': 90.0, '뒤': 180.0, '오른쪽': -90.0}


def wrap(angle):
    """각도(rad)를 -pi ~ pi로 맞춘다."""
    return math.atan2(math.sin(angle), math.cos(angle))


def ranges_toward(scan, direction, half_width):
    """Scan frame 기준 direction(rad) ± half_width 안의 쓸 만한 거리들."""
    values = []
    for i, distance in enumerate(scan.ranges):
        if not (math.isfinite(distance) and scan.range_min <= distance <= scan.range_max):
            continue
        if distance < SELF_HIT_M:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        if abs(wrap(angle - direction)) <= half_width:
            values.append(distance)
    return values


def direction_distances(scans, mount_yaw):
    """여러 scan을 모아 로봇 기준 앞·왼쪽·뒤·오른쪽 거리(중간값)를 구한다. 안 보이면 None."""
    result = {}
    for name, degrees in DIRECTIONS.items():
        # 로봇 기준 방향 -> LiDAR 기준 방향 (LiDAR가 mount_yaw만큼 돌아 붙어 있다)
        direction = math.radians(degrees) - mount_yaw
        values = []
        for scan in scans:
            values += ranges_toward(scan, direction, math.radians(MEASURE_HALF_DEG))
        result[name] = statistics.median(values) if values else None
    return result


def front_min(scan, mount_yaw):
    """로봇 정면 ±SAFETY_HALF_DEG 안에서 가장 가까운 거리. 아무것도 안 보이면 None."""
    values = ranges_toward(scan, -mount_yaw, math.radians(SAFETY_HALF_DEG))
    return min(values) if values else None


def moved_by_lidar(before, after):
    """앞 벽이 가까워진 만큼, 뒤 벽이 멀어진 만큼 = 앞으로 간 거리 (m). 못 재면 None."""
    estimates = []
    if before['앞'] is not None and after['앞'] is not None:
        estimates.append(before['앞'] - after['앞'])
    if before['뒤'] is not None and after['뒤'] is not None:
        estimates.append(after['뒤'] - before['뒤'])
    return sum(estimates) / len(estimates) if estimates else None


def side_shift(before, after):
    """왼쪽·오른쪽 거리 변화 중 큰 쪽 (m). LiDAR 장착 방향이 90도 틀렸는지 볼 때 쓴다."""
    shifts = [abs(after[name] - before[name]) for name in ('왼쪽', '오른쪽')
              if before[name] is not None and after[name] is not None]
    return max(shifts) if shifts else 0.0


def verdict(moved, expected, side):
    """(앞으로 잘 갔는지, 알려 줄 문장들)을 돌려준다."""
    if moved is None:
        return False, ['❓ 앞뒤에 벽이 안 보여 거리 변화를 못 쟀어요. '
                       '1~3 m 앞에 벽이나 상자를 두고 다시 하세요.']
    ratio = moved / expected if expected > 0 else 0.0
    if ratio >= 0.5:
        lines = [f'✅ 앞으로 갔어요: 약 {moved:.2f} m '
                 f'(명령대로라면 {expected:.2f} m, {ratio * 100:.0f}%)']
        if not 0.8 <= ratio <= 1.2:
            lines.append('   명령과 20% 넘게 달라요. 바퀴 미끄러짐이나 바퀴 지름 설정'
                         '(hardware.yaml wheel_radius, 조원 A)을 확인하세요.')
        return True, lines
    if ratio <= -0.5:
        return False, [
            f'❌ LiDAR로 보면 뒤로 갔어요 (약 {-moved:.2f} m).',
            '   - 눈으로 봐도 뒤로 갔다면: 바퀴 방향 설정이 반대예요 '
            '(hardware.yaml left_sign·right_sign, 조원 A).',
            '   - 눈으로는 앞으로 갔다면: LiDAR 장착 방향이 URDF와 180도 달라요 '
            '(poli_description lidar_yaw).',
        ]
    if side >= 0.5 * expected:
        return False, [
            '❌ LiDAR로 보면 옆으로 움직인 것처럼 보여요.',
            '   눈으로는 앞으로 갔다면: LiDAR 장착 방향이 URDF와 90도 달라요 '
            '(poli_description lidar_yaw).',
        ]
    if abs(ratio) < 0.2:
        return False, [
            f'❌ 거의 안 움직였어요 ({moved:+.2f} m).',
            '   배터리·비상정지 스위치, MDD10 전원(12 V), 모터 선을 확인하세요. '
            '바퀴 노드 로그: /tmp/forward_drive.log',
        ]
    return False, [
        f'⚠️ 조금만 움직였어요: 약 {moved:.2f} m (명령대로라면 {expected:.2f} m, '
        f'{ratio * 100:.0f}%).',
        '   바퀴 미끄러짐, 배터리 전압, 바퀴 지름 설정(hardware.yaml wheel_radius)을 확인하세요.',
    ]


def describe_turn(turned):
    """달리는 동안 IMU로 잰 방향 변화(rad)를 문장으로 만든다."""
    if turned is None:
        return 'IMU(/imu/data)가 안 와서 방향이 틀어졌는지는 못 쟀어요'
    degrees = math.degrees(turned)
    if abs(degrees) < TURN_WARN_DEG:
        return f'방향 변화 {degrees:+.0f}도 (곧게 갔어요)'
    side = '왼쪽' if degrees > 0 else '오른쪽'
    return f'⚠️ {side}으로 {abs(degrees):.0f}도 휘었어요 (두 바퀴 속도 차이·바퀴 방향 설정 확인)'


def format_distances(distances):
    return ' · '.join(
        f'{name} {"---" if d is None else f"{d:.2f} m"}' for name, d in distances.items())


def lookup_yaw(tf_buffer, frame):
    """TF base_link -> frame의 방향(yaw, rad). 아직 없으면 None."""
    if frame == BASE_FRAME:
        return 0.0
    try:
        transform = tf_buffer.lookup_transform(BASE_FRAME, frame, Time())
    except TransformException:
        return None
    q = transform.transform.rotation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Recorder:
    """/scan과 /imu/data를 받아 둔다."""

    def __init__(self, node):
        self.scan = None
        self.scan_at = None
        self.scans = None       # list이면 measure()가 모으는 중
        self.turned = 0.0       # IMU 각속도를 적분한 방향 변화 (rad, + = 왼쪽)
        self.imu_at = None
        self.imu_count = 0
        self.integrating = False
        node.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        node.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)

    def on_scan(self, msg):
        self.scan = msg
        self.scan_at = time.monotonic()
        if self.scans is not None:
            self.scans.append(msg)

    def on_imu(self, msg):
        now = time.monotonic()
        if self.integrating:
            self.imu_count += 1
            if self.imu_at is not None:
                self.turned += msg.angular_velocity.z * (now - self.imu_at)
        self.imu_at = now

    def start_turn(self):
        self.turned = 0.0
        self.imu_count = 0
        self.integrating = True

    def stop_turn(self):
        self.integrating = False
        return self.turned if self.imu_count > 1 else None


def spin_for(node, seconds, pub=None, twist=None):
    """Seconds 동안 콜백을 처리한다. twist를 주면 그동안 RATE_HZ로 계속 보낸다."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if twist is not None:
            pub.publish(twist)
        tick_end = min(end, time.monotonic() + 1.0 / RATE_HZ)
        while time.monotonic() < tick_end:
            rclpy.spin_once(node, timeout_sec=max(0.0, tick_end - time.monotonic()))


def wait_for_lidar_and_tf(node, rec, tf_buffer):
    """첫 LiDAR scan과 TF를 기다려 LiDAR 장착 방향(yaw, rad)을 돌려준다. 실패하면 None."""
    end = time.monotonic() + WAIT_S
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if rec.scan is not None:
            yaw = lookup_yaw(tf_buffer, rec.scan.header.frame_id)
            if yaw is not None:
                return yaw
    if rec.scan is None:
        print('❌ LiDAR(/scan)가 안 와요. LiDAR USB와 LiDAR 노드를 확인하세요.', flush=True)
    else:
        print(f'❌ TF {BASE_FRAME} -> {rec.scan.header.frame_id}가 없어요. '
              'poli_description(robot_state_publisher)을 켜세요.', flush=True)
    return None


def measure(node, pub, rec, mount_yaw):
    """MEASURE_S 동안 정지 명령을 보내며 LiDAR를 모아 앞·왼쪽·뒤·오른쪽 거리를 잰다."""
    rec.scans = []
    spin_for(node, MEASURE_S, pub, Twist())
    scans, rec.scans = rec.scans, None
    return direction_distances(scans, mount_yaw)


def drive(node, pub, rec, mount_yaw):
    """앞으로 달린다. (달린 시간, 일찍 멈춘 이유 또는 None)을 돌려준다."""
    forward = Twist()
    forward.linear.x = SPEED
    start = time.monotonic()
    while True:
        elapsed = time.monotonic() - start
        if elapsed >= DRIVE_S:
            return elapsed, None
        if time.monotonic() - rec.scan_at > SCAN_STALE_S:
            return elapsed, 'LiDAR가 끊겨서 멈췄어요'
        near = front_min(rec.scan, mount_yaw)
        if near is not None and near < STOP_DIST:
            return elapsed, f'앞쪽 {near:.2f} m에 뭐가 보여서 멈췄어요'
        pub.publish(forward)
        spin_for(node, 1.0 / RATE_HZ)


def run(node, pub, rec, tf_buffer):
    mount_yaw = wait_for_lidar_and_tf(node, rec, tf_buffer)
    if mount_yaw is None:
        return 1
    print(f'  LiDAR 장착 방향 (URDF): {math.degrees(mount_yaw):.0f}도', flush=True)

    before = measure(node, pub, rec, mount_yaw)
    print(f'  출발 전 거리: {format_distances(before)}', flush=True)
    near = front_min(rec.scan, mount_yaw)
    if near is not None and near < MIN_FRONT_CLEAR:
        print(f'❌ 앞쪽 {near:.2f} m에 뭐가 있어요. '
              f'앞을 {MIN_FRONT_CLEAR:.1f} m 이상 비우고 다시 하세요.', flush=True)
        return 1
    if before['앞'] is None and before['뒤'] is None:
        print('❌ 앞뒤에 벽이 안 보여 거리 변화를 잴 수 없어요. '
              '1~3 m 앞에 벽이나 상자를 두세요.', flush=True)
        return 1

    print(f'  앞으로 {SPEED:.2f} m/s, {DRIVE_S:.1f}초 '
          '(멈추려면 Ctrl+C, 위험하면 비상정지)', flush=True)
    rec.start_turn()
    driven, reason = drive(node, pub, rec, mount_yaw)
    spin_for(node, SETTLE_S, pub, Twist())
    turned = rec.stop_turn()
    after = measure(node, pub, rec, mount_yaw)

    expected = SPEED * driven
    print(f'  멈춘 뒤 거리: {format_distances(after)}', flush=True)
    if reason:
        print(f'  ⚠️ {reason}', flush=True)
    print(f'  달린 시간 {driven:.1f}초 -> 명령대로라면 약 {expected:.2f} m', flush=True)
    print(f'  {describe_turn(turned)}', flush=True)
    ok, lines = verdict(moved_by_lidar(before, after), expected, side_shift(before, after))
    for line in lines:
        print(line, flush=True)
    return 0 if ok else 2


def raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    # rclpy 기본 Ctrl+C 처리는 통신을 먼저 닫아 버려 정지 명령을 못 보낸다. 그래서 끄고,
    # Ctrl+C·SSH 끊김(SIGHUP)·SIGTERM을 모두 KeyboardInterrupt로 받아 정지 명령부터 보낸다.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGHUP, raise_interrupt)
    signal.signal(signal.SIGTERM, raise_interrupt)
    node = rclpy.create_node('forward_test')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    rec = Recorder(node)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)  # noqa: F841 (지우면 TF를 못 받는다)
    code = 1
    try:
        code = run(node, pub, rec, tf_buffer)
    except KeyboardInterrupt:
        print('  중단', flush=True)
    finally:
        # 정지 명령을 보내는 동안 또 Ctrl+C가 와도 끊기지 않게 한다.
        for sig in (signal.SIGINT, signal.SIGHUP, signal.SIGTERM):
            signal.signal(sig, signal.SIG_IGN)
        for _ in range(10):
            pub.publish(Twist())
            time.sleep(0.05)
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
