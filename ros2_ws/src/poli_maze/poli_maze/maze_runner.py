"""미로를 달려 목표에서 물건을 집고 시작점으로 돌아오는 노드.

흐름 (플로우 차트와 같음)
  INIT     odom, scan 이 들어오면 잠깐 맵핑한 뒤 첫 방향을 정한다
  TURN     옆 칸으로 갈 때 제자리 90도 회전
  DRIVE    칸 중앙 사이를 달린다. 다음 칸 중앙 '결정 지점' 에서 방향을 정해
           직진이면 멈추지 않고 이어 달리고, 아니면 칸 중앙에 멈춘다.
           뒤쪽 칸은 돌지 않고 후진한다.
  GRAB     목표 칸에서 물건을 집는다
  RETURN   지나온 칸에 시작점 기준 숫자표를 만들고 구간별로 달려 돌아온다
  DONE     정지

구독: /odom_raw (nav_msgs/Odometry), /scan (sensor_msgs/LaserScan)
발행: /cmd_vel (geometry_msgs/Twist), /gripper/command (std_msgs/String),
      /maze/status, /maze/map (std_msgs/String, 디버그용)

실행 예: ros2 run poli_maze maze_runner
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from poli_maze import maze_logic as ml
from poli_maze.maze_mapper import GridFrame, WallMapper, wrap_angle

# ---------------------------------------------------------------------------
# 토픽
# ---------------------------------------------------------------------------
# TODO_HW: 조원 A 하드웨어 노드가 실제로 내보내는 토픽 이름과 메시지 형식 확인.
#          /odom_raw 가 nav_msgs/Odometry 가 아니면 odom_callback 을 고친다.
ODOM_TOPIC = '/odom_raw'
SCAN_TOPIC = '/scan'
CMD_TOPIC = '/cmd_vel'
GRIPPER_TOPIC = '/gripper/command'

CONTROL_HZ = 20.0

# ---------------------------------------------------------------------------
# 직진 / 후진
# ---------------------------------------------------------------------------
MAX_SPEED = 0.30          # TODO_MEASURE: 미끄러지지 않고 칸 중앙에 설 수 있는 최고 속도 (m/s)
MAX_REVERSE_SPEED = 0.20  # TODO_MEASURE: 후진 최고 속도 (m/s)
ACCEL = 0.6               # TODO_MEASURE: 가속/감속 (m/s^2). 바퀴가 헛돌지 않는 값
MIN_SPEED = 0.05          # TODO_MEASURE: 이보다 느리면 모터가 안 도는 최소 속도 (m/s)
DECISION_MARGIN = 0.05    # 감속 거리에 더하는 여유 (m). 결정 지점 = 감속 거리 + 이것
POS_TOL = 0.02            # TODO_MEASURE: 칸 중앙에 '도착' 으로 볼 오차 (m)
HEADING_KP = 2.5          # TODO_MEASURE: 방향 유지 게인
CROSS_KP = 2.0            # TODO_MEASURE: 칸 가운데 줄로 돌아오는 게인
MAX_CROSS_ANGLE = math.radians(20)
MAX_DRIVE_W = 1.0         # 직진 중 최대 회전 속도 (rad/s)

# ---------------------------------------------------------------------------
# 제자리 회전
# ---------------------------------------------------------------------------
# 로봇 24 x 20 cm, 칸 40 cm: 회전 중심이 로봇 한가운데면 회전 원 지름 약 31 cm.
# TODO_HW: 바퀴 축 가운데 ~ 가장 먼 끝(집게, 들고 있는 물건 포함) 거리가
#          18 cm 이하인지 확인. 넘으면 칸 안에서 제자리 회전 불가.
TURN_KP = 3.0              # TODO_MEASURE
MAX_TURN_SPEED = 1.5       # TODO_MEASURE: 최대 회전 속도 (rad/s)
MIN_TURN_SPEED = 0.3       # TODO_MEASURE: 이보다 느리면 회전이 멈추는 최소값 (rad/s)
YAW_TOL = math.radians(2)  # TODO_MEASURE: 회전 완료로 볼 각도 오차
TURN_SETTLE_CYCLES = 3     # 오차 안에 이만큼 연속으로 있으면 완료

# ---------------------------------------------------------------------------
# 그 밖
# ---------------------------------------------------------------------------
START_SETTLE_S = 1.0     # 출발 전 맵핑 시간 (s)
GRASP_WAIT_S = 1.5       # 조원 A 요청: 집게가 다 닫히는 데 약 1.2초
SAFETY_STOP_DIST = 0.10  # TODO_MEASURE: 진행 방향 이 거리 안에 물체가 있으면 정지 (m)
SAFETY_HALF_ANGLE = math.radians(15)


def yaw_from_quat(q):
    """쿼터니언 -> yaw."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def clamp(v, lo, hi):
    """범위 제한."""
    return max(lo, min(hi, v))


class MazeRunner(Node):
    """미로 탐색 + 집기 + 복귀."""

    def __init__(self):
        """토픽, 상태를 준비한다."""
        super().__init__('maze_runner')

        self.frame = GridFrame()
        self.mapper = WallMapper(self.frame)
        self.maze = ml.MazeState()

        self.origin = None   # 출발할 때 odom 위치 (x, y, yaw)
        self.pose = None     # 출발 좌표계 위치 (x, y, yaw)
        self.scan_ok = False
        self.front_min = {True: math.inf, False: math.inf}  # 앞/뒤 최소 거리

        self.state = 'INIT'
        self.state_since = None
        self.phys_facing = ml.START_DIR  # 실제로 로봇이 보는 경기장 방향
        self.v_cmd = 0.0

        # DRIVE 용
        self.move_dir = None   # 달리는 경기장 방향
        self.queue = []        # [{'cell', 'dir', 'arrived'}]
        self.after_stop = None # 멈춘 뒤 할 일: ('next', d) / ('grab',) / ('segment',) / ('done',) / ('redecide',)
        self.mode = 'explore'  # 'explore' 또는 'return'

        # TURN 용
        self.turn_target = None
        self.turn_ok_count = 0
        self.after_turn = None

        # RETURN 용
        self.segments = []
        self.cell_cursor = None

        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.gripper_pub = self.create_publisher(String, GRIPPER_TOPIC, 10)
        self.status_pub = self.create_publisher(String, '/maze/status', 10)
        self.map_pub = self.create_publisher(String, '/maze/map', 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, 20)
        self.create_subscription(LaserScan, SCAN_TOPIC, self.scan_callback, 10)
        self.create_timer(1.0 / CONTROL_HZ, self.control)
        self.create_timer(1.0, self.publish_map)

        # TODO_HW: 대회 시작 신호(버튼 등)가 정해지면 INIT 에서 그 신호를 기다리게 한다.
        #          지금은 odom 과 scan 이 들어오면 START_SETTLE_S 뒤 바로 출발한다.
        self.get_logger().info('maze_runner 시작: odom, scan 기다리는 중')

    # ------------------------------------------------------------------
    # 센서
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        """odom 을 출발 좌표계로 바꿔 저장한다."""
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        # TODO_HW: 바퀴 odom 의 yaw 가 많이 틀어지면 /imu/data 의 yaw 를 대신 쓴다.
        if self.origin is None:
            self.origin = (p.x, p.y, yaw)
        ox, oy, oyaw = self.origin
        dx, dy = p.x - ox, p.y - oy
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        self.pose = (dx * c - dy * s, dx * s + dy * c, wrap_angle(yaw - oyaw))

    def scan_callback(self, msg):
        """벽 맵핑 + 진행 방향 안전 거리 확인."""
        if self.pose is None:
            return
        self.scan_ok = True
        free = self.maze.visited_cells() | {self.maze.cell}
        new_walls = self.mapper.add_scan(
            self.pose, msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max, free_cells=free)
        for cell in new_walls:
            if self.maze.add_wall(cell):
                self.say('벽 발견 %s' % (cell,))
                self.check_queue_blocked(cell)

        # 진행 방향(앞/뒤) 가까운 물체
        # TODO_HW: 라이다 0도가 로봇 정면이 아니면 LASER_YAW 를 고친 뒤 여기도 확인.
        for forward in (True, False):
            center = 0.0 if forward else math.pi
            best = math.inf
            for i, r in enumerate(msg.ranges):
                if not math.isfinite(r) or r < msg.range_min:
                    continue
                a = wrap_angle(msg.angle_min + i * msg.angle_increment - center)
                if abs(a) <= SAFETY_HALF_ANGLE:
                    best = min(best, r)
            self.front_min[forward] = best

    # ------------------------------------------------------------------
    # 제어 루프
    # ------------------------------------------------------------------
    def control(self):
        """상태별로 한 번씩 실행된다."""
        if self.pose is None or not self.scan_ok:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.state_since is None:
            self.state_since = now

        if self.state == 'INIT':
            self.publish_cmd(0.0, 0.0)
            if now - self.state_since >= START_SETTLE_S:
                self.say('출발')
                self.decide_and_go()
        elif self.state == 'TURN':
            self.do_turn()
        elif self.state == 'DRIVE':
            self.do_drive()
        elif self.state == 'GRAB':
            self.publish_cmd(0.0, 0.0)
            if now - self.state_since >= GRASP_WAIT_S:
                # TODO_HW: /gripper/holding 센서가 생기면 잡았는지 확인하고 못 잡았으면 다시 시도.
                self.start_return()
        else:  # DONE
            self.publish_cmd(0.0, 0.0)

    def set_state(self, state):
        """상태를 바꾼다."""
        self.state = state
        self.state_since = self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------
    # 갈 때: 판단
    # ------------------------------------------------------------------
    def decide_and_go(self):
        """현재 칸에서 다음 방향을 정해 출발한다 (멈춰 있을 때)."""
        d = self.maze.choose_next()
        if d is None:
            self.say('갈 곳이 없어 정지')
            self.set_state('DONE')
            return
        self.begin_move(d, [(ml.step(self.maze.cell, d), d, False)])

    def on_arrive(self, item):
        """결정 지점에 왔을 때: 그 칸에 도착한 것으로 치고 다음 방향을 정한다."""
        self.maze.move(item['dir'])  # 도착 칸 방문 +10
        item['arrived'] = True
        if self.maze.at_goal():
            self.after_stop = ('grab',)
            self.say('목표 칸 도착 예정, 멈춤')
            return
        d = self.maze.choose_next()
        if d is None:
            self.after_stop = ('done',)
            self.say('갈 곳이 없음')
        elif d == self.move_dir:
            # 같은 방향(직진 또는 후진 이어가기): 멈추지 않고 다음 칸까지 연장
            self.queue.append({'cell': ml.step(item['cell'], d), 'dir': d, 'arrived': False})
        else:
            self.after_stop = ('next', d)

    def check_queue_blocked(self, wall):
        """이미 가기로 한 칸이 벽으로 밝혀지면 그 앞에서 멈추고 다시 정한다."""
        if self.state != 'DRIVE':
            return
        for i, item in enumerate(self.queue):
            if item['cell'] == wall:
                self.say('가려던 칸 %s 가 벽: 앞 칸에서 멈추고 다시 판단' % (wall,))
                del self.queue[i:]
                if not self.queue:
                    # 출발하자마자 바로 다음 칸이 벽으로 밝혀진 경우: 서서 다시 정한다.
                    # (로봇이 현재 칸 중앙에서 조금 나와 있을 수 있음)
                    self.publish_cmd(0.0, 0.0)
                    self.v_cmd = 0.0
                    self.decide_and_go()
                    return
                self.after_stop = ('redecide',)
                return

    # ------------------------------------------------------------------
    # 움직임 시작
    # ------------------------------------------------------------------
    def begin_move(self, d, cells):
        """d 방향으로 cells 를 달린다. 옆이면 먼저 90도 돈다.

        cells: [(칸, 방향, 이미 판단했는지)]
        """
        how = ml.how_to_move(self.phys_facing, d)
        self.move_dir = d
        self.queue = [{'cell': c, 'dir': dd, 'arrived': a} for c, dd, a in cells]
        self.after_stop = None
        if how in (ml.TURN_LEFT, ml.TURN_RIGHT):
            # TODO_HW: 회전 전에 라이다로 벽까지 거리를 재서 칸 중앙에 맞추기 (여유 약 4 cm).
            self.turn_target = d
            self.turn_ok_count = 0
            self.set_state('TURN')
            self.say('%s 방향으로 90도 회전' % ml.DIR_NAME[d])
        else:
            self.v_cmd = 0.0
            self.set_state('DRIVE')
            self.say('%s 방향으로 %s' % (ml.DIR_NAME[d], '후진' if how == ml.REVERSE else '전진'))

    # ------------------------------------------------------------------
    # 회전
    # ------------------------------------------------------------------
    def do_turn(self):
        """제자리에서 turn_target 방향을 볼 때까지 돈다."""
        target = self.frame.dir_yaw(self.turn_target)
        err = wrap_angle(target - self.pose[2])
        if abs(err) < YAW_TOL:
            self.turn_ok_count += 1
            self.publish_cmd(0.0, 0.0)
            if self.turn_ok_count >= TURN_SETTLE_CYCLES:
                self.phys_facing = self.turn_target
                self.v_cmd = 0.0
                self.set_state('DRIVE')
            return
        self.turn_ok_count = 0
        w = clamp(TURN_KP * err, -MAX_TURN_SPEED, MAX_TURN_SPEED)
        if abs(w) < MIN_TURN_SPEED:
            w = math.copysign(MIN_TURN_SPEED, w)
        self.publish_cmd(0.0, w)

    # ------------------------------------------------------------------
    # 주행
    # ------------------------------------------------------------------
    def do_drive(self):
        """queue 의 칸들을 따라 달린다."""
        x, y, yaw = self.pose
        ux, uy = self.frame.dir_vec(self.move_dir)
        reverse = self.move_dir == ml.opposite(self.phys_facing)

        def ahead(cell):
            cx, cy = self.frame.cell_center(cell)
            return (cx - x) * ux + (cy - y) * uy

        # 지나간 칸 중앙은 버린다 (마지막 칸은 남김)
        while len(self.queue) > 1 and ahead(self.queue[0]['cell']) < 0:
            self.queue.pop(0)

        # 결정 지점: 아직 판단 안 한 칸에 감속 거리 + 여유만큼 가까워지면 판단
        if self.mode == 'explore':
            for item in self.queue:
                if not item['arrived']:
                    brake = self.v_cmd * self.v_cmd / (2 * ACCEL)
                    if ahead(item['cell']) <= brake + DECISION_MARGIN:
                        self.on_arrive(item)
                    break

        last = self.queue[-1]
        dist = ahead(last['cell'])
        stopping = last['arrived'] and dist <= POS_TOL
        if stopping:
            self.publish_cmd(0.0, 0.0)
            self.v_cmd = 0.0
            self.on_stopped()
            return

        # 속도: 마지막 칸 중앙에 서도록 감속, 가속은 ACCEL 로 제한
        vmax = MAX_REVERSE_SPEED if reverse else MAX_SPEED
        v_des = min(vmax, math.sqrt(2 * ACCEL * max(dist, 0.0)))
        v = min(v_des, self.v_cmd + ACCEL / CONTROL_HZ)
        if dist > POS_TOL:
            v = max(v, MIN_SPEED)
        self.v_cmd = v

        # 진행 방향에 너무 가까운 물체가 있으면 정지
        if self.front_min[not reverse] < SAFETY_STOP_DIST:
            self.publish_cmd(0.0, 0.0)
            self.v_cmd = 0.0
            self.say('진행 방향 %.2f m 에 물체: 정지' % self.front_min[not reverse], throttle=True)
            return

        # 방향: 칸 가운데 줄에서 벗어난 만큼 돌아오도록 목표 각도를 살짝 꺾는다
        cx, cy = self.frame.cell_center(last['cell'])
        nx, ny = -uy, ux  # 진행 방향 왼쪽
        cross = (x - cx) * nx + (y - cy) * ny
        offset = clamp(-math.atan(CROSS_KP * cross), -MAX_CROSS_ANGLE, MAX_CROSS_ANGLE)
        move_yaw = self.frame.dir_yaw(self.move_dir) + offset
        face_yaw = move_yaw + (math.pi if reverse else 0.0)
        w = clamp(HEADING_KP * wrap_angle(face_yaw - yaw), -MAX_DRIVE_W, MAX_DRIVE_W)
        self.publish_cmd(-v if reverse else v, w)

    def on_stopped(self):
        """칸 중앙에 멈춘 뒤 할 일."""
        action = self.after_stop or ('done',)
        self.after_stop = None
        if action[0] == 'next':
            d = action[1]
            self.begin_move(d, [(ml.step(self.maze.cell, d), d, False)])
        elif action[0] == 'redecide':
            self.decide_and_go()
        elif action[0] == 'grab':
            self.say('목표 도착: 물건 집기')
            # TODO_HW: 물건이 칸 안 어디에 있는지에 따라 비전(/vision/target)으로 정렬 후 집기.
            self.gripper_pub.publish(String(data='grab'))
            self.set_state('GRAB')
        elif action[0] == 'segment':
            self.next_segment()
        else:
            self.say('정지')
            self.set_state('DONE')

    # ------------------------------------------------------------------
    # 복귀
    # ------------------------------------------------------------------
    def start_return(self):
        """지나온 칸으로 복귀 숫자표를 만들고 구간별로 달린다."""
        self.mode = 'return'
        rmap = ml.build_return_map(self.maze.visited_cells(), self.maze.walls)
        dirs = ml.plan_return(self.maze.cell, self.phys_facing, rmap)
        self.segments = ml.group_segments(dirs)
        self.say('복귀: %d칸, 구간 %s' % (len(dirs), [(ml.DIR_NAME[d], n) for d, n in self.segments]))
        self.cell_cursor = self.maze.cell
        self.next_segment()

    def next_segment(self):
        """복귀 구간 하나를 시작한다. 다 끝났으면 DONE."""
        if not self.segments:
            self.say('시작점 복귀 완료')
            # TODO_HW: 규정상 시작점에서 물건을 내려놓아야 하면 여기서 'open' 발행.
            self.set_state('DONE')
            return
        d, n = self.segments.pop(0)
        cells = []
        c = self.cell_cursor
        for _ in range(n):
            c = ml.step(c, d)
            cells.append((c, d, True))  # 아는 길이라 판단 없이 달림
        self.cell_cursor = c
        self.begin_move(d, cells)
        self.after_stop = ('segment',)

    # ------------------------------------------------------------------
    # 출력
    # ------------------------------------------------------------------
    def publish_cmd(self, v, w):
        """속도 명령."""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def say(self, text, throttle=False):
        """로그 + /maze/status."""
        if throttle:
            self.get_logger().info(text, throttle_duration_sec=1.0)
        else:
            self.get_logger().info(text)
        self.status_pub.publish(String(data='[%s] %s' % (self.state, text)))

    def publish_map(self):
        """현재 숫자표 (1초에 한 번)."""
        self.map_pub.publish(String(data=self.maze.text_map()))

    def destroy_node(self):
        """종료 전에 로봇을 세운다."""
        self.publish_cmd(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    """노드를 실행한다."""
    rclpy.init(args=args)
    node = MazeRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
