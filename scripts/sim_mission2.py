#!/usr/bin/env python3
"""
임무 2 2D 가상 시뮬레이션 (SIM_ONLY, ROS 없이 실행).

우리 로봇은 실제 임무 2 판단 코드(mission2_logic.Mission2Logic)를 그대로 쓰고 사양을 바꾸지 않는다.
상대 로봇만 옵션으로 바꾼다. 결과는 HTML 재생 화면으로 저장해 브라우저로 연다.

사용 (저장소 폴더에서):
  python3 scripts/sim_mission2.py                          # 상대 1대, 0.4 m/s, 1분
  python3 scripts/sim_mission2.py --opp-speed 0.6 --opp-start 2,2
  python3 scripts/sim_mission2.py --opp-count 3 --time 300 --opp-behavior wander

가정 (실제와 다를 수 있다):
- 우리 위치는 참값을 쓴다 (실제는 엔코더 + IMU + 벽 보정).
- 모터 지연 0.15 s, 카메라 지연 0.1 s, 하드웨어 최저 바퀴 속도 (test_mission2_logic.LaggedSimRobot과 같음).
- 로봇은 원, 부딪히면 둘이 반씩 밀린다. 큐브(50 mm)는 밀리고, 열린 집게 입구로는 들어온다.
- 상대 로봇은 단순한 규칙으로 움직인다 (--opp-behavior).
"""

import argparse
import collections
from datetime import datetime
import json
import math
import os
import random
import shutil
import subprocess
import sys
from types import SimpleNamespace

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', 'poli_navigation'))
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', 'poli_navigation', 'test'))

from hw_model import actual_twist  # noqa: E402
from poli_navigation import mission2_logic as m2  # noqa: E402
from poli_navigation.opponent_detector import find_opponents  # noqa: E402


DT = 0.05                 # 판단 주기 (mission2_node.CONTROL_PERIOD)
SCAN_PERIOD = 0.1         # LiDAR 주기
OPPONENT_TIMEOUT = 0.5    # mission2_node.OPPONENT_TIMEOUT
MOTOR_LAG_S = 0.15        # SIM_ONLY: 2026-11-01 실기 로그 기준
CAMERA_LAG_S = 0.10       # SIM_ONLY

# SIM_ONLY: 우리 로봇 모양 (hardware_reference.md 근삿값)
OUR_RADIUS_M = 0.16       # 폭 30.5 cm의 절반 + 여유
HELD_CUBE_AHEAD_M = 0.13  # 바퀴 축 ~ 잡은 큐브 중심 (2026-10-31 실측)
MOUTH_BACK_M = 0.12       # 열린 집게 안으로 큐브 중심이 들어올 수 있는 가장 안쪽
MOUTH_HALF_M = 0.04       # 큐브 중심이 이 폭 안이면 집게 입구로 들어온다
GRAB_MIN_M = 0.09         # 닫힐 때 큐브 중심이 이 범위 안이면 잡힌다
GRAB_MAX_M = 0.20
TOF_FACE_M = 0.10         # 적외선 거리 = 큐브 중심 앞뒤 거리 - 이 값 (잡은 큐브 약 30 mm)
TOF_MAX_M = 0.5
GRIPPER_MOVE_S = 0.18     # 집게 닫힘·열림 시간 (2026-11-01)
CAMERA_MIN_AREA = 300.0   # 카메라 노드 최소 면적 (약 1.45 m)
CAMERA_BLIND_M = 0.06     # 카메라에서 이보다 가까우면 화면 아래로 사라진다
AREA_SCALE = m2.TARGET_AREA_AT_50CM * 0.5 ** 2

CUBE_RADIUS_M = 0.03      # 50 mm 큐브
NUM_RAYS = 360
RANGE_MAX = 12.0

START_CELLS = {'8,2': 0, '8,8': 1, '2,8': 2, '2,2': 3}  # 우리 출발 (8,2)에서 90도씩


def rotate_about_center(pose, quarter_turns):
    """우리 출발 위치를 경기장 중앙 기준 90도 x quarter_turns 돌린 위치."""
    angle = quarter_turns * math.pi / 2.0
    dx, dy = pose[0] - m2.CENTER_M[0], pose[1] - m2.CENTER_M[1]
    return (m2.CENTER_M[0] + dx * math.cos(angle) - dy * math.sin(angle),
            m2.CENTER_M[1] + dx * math.sin(angle) + dy * math.cos(angle),
            m2.normalize_angle(pose[2] + angle))


def to_local(px, py, x, y, yaw):
    """경기장 점 (px, py)를 로봇 (x, y, yaw) 기준 (앞, 왼쪽) 좌표로."""
    dx, dy = px - x, py - y
    return dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)


def segment_hits_circle(a, b, center, radius):
    """선분 a-b가 원과 만나는지 (카메라 시야를 상대 로봇이 가리는지)."""
    ax, ay = a
    bx, by = b
    cx, cy = center
    abx, aby = bx - ax, by - ay
    length2 = abx * abx + aby * aby
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((cx - ax) * abx + (cy - ay) * aby) / length2))
    return math.hypot(ax + t * abx - cx, ay + t * aby - cy) < radius


def ray_circle(x, y, dx, dy, center, radius):
    """(x, y)에서 (dx, dy) 방향 광선이 원에 닿는 거리. 안 닿으면 inf."""
    fx, fy = x - center[0], y - center[1]
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - c
    if disc < 0.0:
        return math.inf
    t = -b - math.sqrt(disc)
    return t if t > 0.0 else math.inf


class Opponent:
    """단순 규칙으로 움직이는 상대 로봇 (SIM_ONLY)."""

    def __init__(self, name, pose, args, rng):
        self.name = name
        self.x, self.y, self.yaw = pose
        self.v = 0.0
        self.w = 0.0
        self.alive = True
        self.args = args
        self.rng = rng
        self.contact = 0.0       # 우리가 잡은 큐브에 닿아 있던 시간 (뺏기)
        self.waypoint = None

    def front(self):
        r = self.args.opp_radius
        return self.x + r * math.cos(self.yaw), self.y + r * math.sin(self.yaw)

    def goal(self, world):
        behavior = self.args.opp_behavior
        if behavior == 'still' or world.cube_holder == self.name:
            return None
        if behavior == 'wander':
            if self.waypoint is None or math.dist((self.x, self.y), self.waypoint) < 0.15:
                low, high = m2.SAFE_ZONE_MIN_M + 0.3, m2.SAFE_ZONE_MAX_M - 0.3
                self.waypoint = (self.rng.uniform(low, high), self.rng.uniform(low, high))
            return self.waypoint
        # chase: 큐브(누가 잡고 있든)로 간다. 스스로 탈락선으로 들어가지는 않는다.
        low, high = m2.SAFE_ZONE_MIN_M + 0.25, m2.SAFE_ZONE_MAX_M - 0.25
        return (min(max(world.cube[0], low), high), min(max(world.cube[1], low), high))

    def step(self, world, now):
        if not self.alive or now < self.args.opp_delay:
            return
        goal = self.goal(world)
        target_v, target_w = 0.0, 0.0
        if goal is not None and math.dist((self.x, self.y), goal) > 0.02:
            error = m2.normalize_angle(math.atan2(goal[1] - self.y, goal[0] - self.x) - self.yaw)
            target_w = max(-self.args.opp_turn, min(self.args.opp_turn, 3.0 * error))
            target_v = self.args.opp_speed * max(0.0, math.cos(error))
        step_v = self.args.opp_accel * DT
        self.v += max(-step_v, min(step_v, target_v - self.v))
        self.w = target_w
        self.yaw = m2.normalize_angle(self.yaw + self.w * DT)
        self.x += self.v * math.cos(self.yaw) * DT
        self.y += self.v * math.sin(self.yaw) * DT


class World:

    def __init__(self, args):
        self.args = args
        self.rng = random.Random(args.seed)
        self.logic = m2.Mission2Logic()
        self.x, self.y, self.yaw = m2.START_POSE
        self.cube = m2.CENTER_M
        self.cube_holder = None   # None / 'us' / 상대 이름
        self.gripper = 'open'     # open / moving / closed
        self.gripper_target = 'open'
        self.gripper_done_at = 0.0
        self.commands = collections.deque([m2.Command()] * round(MOTOR_LAG_S / DT))
        self.frames_cam = collections.deque()
        self.push_speed = 0.0
        self.actual_linear = 0.0
        self.opponents_seen = []
        self.opponents_seen_at = None
        self.next_scan = 0.0
        self.our_eliminated = False
        self.hold_since = None    # 우리가 지금까지 연속으로 잡은 시작 시각
        self.events = []
        self.frames = []
        self.last_state = None

        opp_cells = [c for c in ('2,8', '2,2', '8,8') if c != args.opp_start]
        cells = [args.opp_start] + opp_cells
        self.opps = []
        for i in range(args.opp_count):
            pose = rotate_about_center(m2.START_POSE, START_CELLS[cells[i]])
            self.opps.append(Opponent(f'상대{i + 1}', pose, args, self.rng))

    def event(self, now, text, kind='info'):
        self.events.append({'t': round(now, 2), 'text': text, 'kind': kind})

    # ---------- 센서 ----------

    def cube_local(self):
        return to_local(self.cube[0], self.cube[1], self.x, self.y, self.yaw)

    def camera(self):
        """지금 카메라에 보이는 (detected, x_offset, area)."""
        if self.cube_holder == 'us':
            return False, 0.0, 0.0
        cam = (self.x + m2.CAMERA_AHEAD_M * math.cos(self.yaw),
               self.y + m2.CAMERA_AHEAD_M * math.sin(self.yaw))
        distance = math.dist(cam, self.cube)
        bearing = m2.normalize_angle(
            math.atan2(self.cube[1] - cam[1], self.cube[0] - cam[0]) - self.yaw)
        area = AREA_SCALE / max(distance, 0.01) ** 2
        if (abs(bearing) >= m2.CAMERA_HALF_FOV_RAD or area < CAMERA_MIN_AREA
                or distance < CAMERA_BLIND_M):
            return False, 0.0, 0.0
        for opp in self.opps:
            if opp.alive and segment_hits_circle(cam, self.cube, (opp.x, opp.y),
                                                 self.args.opp_radius):
                return False, 0.0, 0.0
        return True, -bearing / m2.CAMERA_HALF_FOV_RAD, area

    def gripper_range(self):
        lx, ly = self.cube_local()
        if abs(ly) > MOUTH_HALF_M or not 0.0 < lx - TOF_FACE_M < TOF_MAX_M:
            return math.inf
        return lx - TOF_FACE_M

    def scan(self):
        """벽과 상대 로봇까지 거리로 만든 가짜 LaserScan (LiDAR는 바퀴 축 중앙)."""
        inc = 2.0 * math.pi / NUM_RAYS
        angle_min = -math.pi + inc / 2.0
        size = m2.ARENA_SIZE_M
        ranges = []
        for i in range(NUM_RAYS):
            angle = self.yaw + angle_min + i * inc
            dx, dy = math.cos(angle), math.sin(angle)
            hits = []
            if dx > 0.0:
                hits.append((size - self.x) / dx)
            elif dx < 0.0:
                hits.append(-self.x / dx)
            if dy > 0.0:
                hits.append((size - self.y) / dy)
            elif dy < 0.0:
                hits.append(-self.y / dy)
            for opp in self.opps:
                if opp.alive:
                    hits.append(ray_circle(self.x, self.y, dx, dy, (opp.x, opp.y),
                                           self.args.opp_radius))
            ranges.append(max(0.0, min(hits)))
        return SimpleNamespace(angle_min=angle_min, angle_increment=inc, range_min=0.05,
                               range_max=RANGE_MAX, ranges=ranges)

    def observe(self, now):
        # 카메라는 CAMERA_LAG_S 늦게 온다
        self.frames_cam.append(self.camera())
        if len(self.frames_cam) > round(CAMERA_LAG_S / DT) + 1:
            self.frames_cam.popleft()
        detected, x_offset, area = self.frames_cam[0]

        if now >= self.next_scan:
            self.next_scan = now + SCAN_PERIOD
            if self.logic.wants_opponents():
                self.opponents_seen = find_opponents(self.scan(), (self.x, self.y, self.yaw))
                self.opponents_seen_at = now

        measured = self.actual_linear
        if abs(measured) < 1e-3:
            measured += self.push_speed  # 서 있다가 밀리면 바퀴가 돈다
        obs = m2.Observation(
            x=self.x, y=self.y, yaw=self.yaw, now=now,
            detected=detected, x_offset=x_offset, area=area, vision_stamp=now,
            holding=self.cube_holder == 'us',
            gripper_state=self.gripper,
            gripper_range=self.gripper_range(),
            measured_linear=measured,
        )
        if self.opponents_seen_at is not None and now - self.opponents_seen_at <= OPPONENT_TIMEOUT:
            obs.opponents = tuple(self.opponents_seen)
            obs.opponent = self.opponents_seen[0] if self.opponents_seen else None
        return obs

    # ---------- 물리 ----------

    def apply_gripper(self, command, now):
        if command is None or command == self.gripper_target:
            return
        self.gripper_target = command
        self.gripper = 'moving'
        self.gripper_done_at = now + GRIPPER_MOVE_S
        self.event(now, f'집게 명령: {command}')
        if command == m2.GRIPPER_OPEN and self.cube_holder == 'us':
            self.release_cube(now, '집게를 열어 큐브를 놓음')

    def update_gripper(self, now):
        if self.gripper != 'moving' or now < self.gripper_done_at:
            return
        if self.gripper_target == m2.GRIPPER_OPEN:
            self.gripper = 'open'
            return
        self.gripper = m2.GRIPPER_CLOSED
        lx, ly = self.cube_local()
        if GRAB_MIN_M <= lx <= GRAB_MAX_M and abs(ly) <= MOUTH_HALF_M:
            stolen = self.cube_holder not in (None, 'us')
            self.cube_holder = 'us'
            self.hold_since = now
            self.event(now, '잡기 성공 (상대에게서 탈환)' if stolen else '잡기 성공', 'good')
        else:
            self.event(now, f'잡기 실패 (큐브 앞 {lx:.2f} m, 옆 {ly:+.2f} m)', 'bad')

    def release_cube(self, now, why):
        if self.cube_holder == 'us':
            self.hold_since = None
        self.cube_holder = None
        self.event(now, why, 'bad')

    def move_us(self, command):
        self.commands.append(command)
        delayed = self.commands.popleft()
        linear, angular = actual_twist(delayed.linear, delayed.angular)
        self.actual_linear = linear
        self.yaw = m2.normalize_angle(self.yaw + angular * DT)
        self.x += linear * math.cos(self.yaw) * DT
        self.y += linear * math.sin(self.yaw) * DT

    def collide(self):
        """로봇끼리, 로봇과 큐브, 벽. 우리가 밀린 앞뒤 속도를 push_speed에 남긴다."""
        before = (self.x, self.y)
        radius = self.args.opp_radius
        bodies = [o for o in self.opps if o.alive]
        for opp in bodies:
            self._separate(self, OUR_RADIUS_M, opp, radius)
        for i, a in enumerate(bodies):
            for b in bodies[i + 1:]:
                self._separate(a, radius, b, radius)
        size = m2.ARENA_SIZE_M
        self.x = min(max(self.x, OUR_RADIUS_M), size - OUR_RADIUS_M)
        self.y = min(max(self.y, OUR_RADIUS_M), size - OUR_RADIUS_M)
        for opp in bodies:
            opp.x = min(max(opp.x, radius), size - radius)
            opp.y = min(max(opp.y, radius), size - radius)
        dx, dy = self.x - before[0], self.y - before[1]
        self.push_speed = (dx * math.cos(self.yaw) + dy * math.sin(self.yaw)) / DT

        if self.cube_holder is None:
            self._push_cube_by_us()
            for opp in bodies:
                self._push_cube_out(opp.x, opp.y, radius + CUBE_RADIUS_M)
            self.cube = (min(max(self.cube[0], CUBE_RADIUS_M), size - CUBE_RADIUS_M),
                         min(max(self.cube[1], CUBE_RADIUS_M), size - CUBE_RADIUS_M))

    @staticmethod
    def _separate(a, ra, b, rb):
        d = math.hypot(b.x - a.x, b.y - a.y)
        overlap = ra + rb - d
        if overlap <= 0.0:
            return
        nx, ny = ((b.x - a.x) / d, (b.y - a.y) / d) if d > 1e-6 else (1.0, 0.0)
        a.x -= nx * overlap / 2.0
        a.y -= ny * overlap / 2.0
        b.x += nx * overlap / 2.0
        b.y += ny * overlap / 2.0

    def _push_cube_by_us(self):
        lx, ly = self.cube_local()
        if self.gripper != m2.GRIPPER_CLOSED and abs(ly) <= MOUTH_HALF_M and 0.0 < lx < GRAB_MAX_M:
            # 열린 집게 입구 안: 안쪽 끝까지는 들어오고, 더 들어가면 앞으로 민다
            if lx < MOUTH_BACK_M:
                lx = MOUTH_BACK_M
                self.cube = (self.x + lx * math.cos(self.yaw) - ly * math.sin(self.yaw),
                             self.y + lx * math.sin(self.yaw) + ly * math.cos(self.yaw))
            return
        self._push_cube_out(self.x, self.y, OUR_RADIUS_M + CUBE_RADIUS_M)

    def _push_cube_out(self, x, y, reach):
        d = math.dist((x, y), self.cube)
        if d >= reach:
            return
        nx, ny = ((self.cube[0] - x) / d, (self.cube[1] - y) / d) if d > 1e-6 else (1.0, 0.0)
        self.cube = (x + nx * reach, y + ny * reach)

    def update_cube(self, now):
        radius = self.args.opp_radius
        if self.cube_holder == 'us':
            self.cube = (self.x + HELD_CUBE_AHEAD_M * math.cos(self.yaw),
                         self.y + HELD_CUBE_AHEAD_M * math.sin(self.yaw))
            for opp in self.opps:
                touching = opp.alive and math.dist(opp.front(), self.cube) < CUBE_RADIUS_M + 0.04
                opp.contact = opp.contact + DT if touching else 0.0
                if self.args.steal_s > 0 and opp.contact >= self.args.steal_s:
                    self.release_cube(now, f'{opp.name}에게 큐브를 뺏김')
                    self.cube_holder = opp.name
                    opp.contact = 0.0
                    break
        elif self.cube_holder is None:
            for opp in self.opps:
                if (opp.alive and self.args.opp_behavior == 'chase'
                        and math.dist(opp.front(), self.cube) < CUBE_RADIUS_M + 0.04):
                    self.cube_holder = opp.name
                    self.event(now, f'{opp.name}이(가) 큐브를 잡음', 'bad')
                    break
        for opp in self.opps:
            if self.cube_holder == opp.name:
                self.cube = (opp.x + (radius + CUBE_RADIUS_M) * math.cos(opp.yaw),
                             opp.y + (radius + CUBE_RADIUS_M) * math.sin(opp.yaw))

    def check_elimination(self, now):
        """로봇 중심이 노란 선 밖(바깥 테두리 격자)으로 나가면 탈락 (규정 3.4.6)."""
        def outside(x, y):
            return not (m2.SAFE_ZONE_MIN_M <= x <= m2.SAFE_ZONE_MAX_M
                        and m2.SAFE_ZONE_MIN_M <= y <= m2.SAFE_ZONE_MAX_M)
        for opp in self.opps:
            if opp.alive and outside(opp.x, opp.y):
                opp.alive = False
                self.event(now, f'{opp.name} 탈락 (노란 선 밖)', 'good')
                if self.cube_holder == opp.name:
                    self.cube_holder = None
                    self.cube = m2.CENTER_M
                    self.event(now, '큐브가 중앙으로 재배치됨')
        if outside(self.x, self.y):
            self.our_eliminated = True
            self.event(now, '우리 로봇 탈락 (노란 선 밖)', 'bad')
            if self.cube_holder == 'us':
                self.hold_since = None
                self.cube_holder = None
                self.cube = m2.CENTER_M

    # ---------- 실행 ----------

    def record(self, now, obs):
        target = self.logic.target
        self.frames.append([
            round(now, 2),
            round(self.x, 3), round(self.y, 3), round(self.yaw, 3),
            self.logic.state, self.gripper,
            round(self.cube[0], 3), round(self.cube[1], 3), self.cube_holder,
            [[round(o.x, 3), round(o.y, 3), round(o.yaw, 3), int(o.alive)] for o in self.opps],
            [[round(p[0], 3), round(p[1], 3)] for p in obs.opponents],
            int(obs.detected),
            None if target is None else [round(target[0], 3), round(target[1], 3)],
        ])

    def run(self):
        steps = round(self.args.time / DT)
        for i in range(steps + 1):
            now = i * DT
            obs = self.observe(now)
            if self.our_eliminated:
                command = m2.Command()
            else:
                command = self.logic.step(obs)
                self.apply_gripper(command.gripper, now)
            if self.logic.state != self.last_state:
                self.event(now, f'상태: {self.logic.state}', 'state')
                self.last_state = self.logic.state
            self.record(now, obs)
            if i == steps:
                break
            if not self.our_eliminated:
                self.move_us(command)
            for opp in self.opps:
                opp.step(self, now)
            self.collide()
            self.update_gripper(now + DT)
            self.update_cube(now + DT)
            self.check_elimination(now + DT)
        return self.result()

    def result(self):
        end = self.args.time
        if self.our_eliminated:
            verdict = '우리 탈락'
        elif self.cube_holder == 'us':
            verdict = f'우리가 확보 (연속 {end - self.hold_since:.1f} s)'
        elif self.cube_holder is None:
            verdict = '아무도 확보 못 함'
        else:
            verdict = f'{self.cube_holder} 확보'
        return verdict


HTML_TEMPLATE = r"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>임무 2 시뮬레이션</title>
<style>
:root {
  --bg: #f4f5f7; --panel: #ffffff; --text: #1d2330; --muted: #667085; --line: #d0d5dd;
  --floor: #fbfbf8; --grid: #e4e4dc; --out: #ece6cf; --yellow: #e0b100;
  --us: #2563eb; --opp: #6b7280; --cube: #e11d48; --good: #15803d; --bad: #b42318;
  --state: #475467; --accent: #2563eb;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    --bg: #0f1218; --panel: #171b23; --text: #e6e8ec; --muted: #98a2b3; --line: #2b3240;
    --floor: #1b1f27; --grid: #272c36; --out: #2e2a1d; --yellow: #d4a300;
    --us: #60a5fa; --opp: #9ca3af; --cube: #f43f5e; --good: #4ade80; --bad: #f87171;
    --state: #a5b0c2; --accent: #60a5fa;
  }
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--text);
  font: 14px/1.45 system-ui, -apple-system, "Segoe UI", "Malgun Gothic", sans-serif; }
.wrap { display: flex; gap: 16px; padding: 16px; flex-wrap: wrap; justify-content: center; }
.stage { background: var(--panel); border: 1px solid var(--line); border-radius: 10px; padding: 12px; }
canvas { display: block; max-width: 100%; }
.side { width: 340px; max-width: 100%; display: flex; flex-direction: column; gap: 12px; }
.card { background: var(--panel); border: 1px solid var(--line); border-radius: 10px; padding: 12px; }
h1 { font-size: 16px; margin: 0 0 4px; }
.muted { color: var(--muted); font-size: 12px; }
.kv { display: grid; grid-template-columns: auto 1fr; gap: 4px 12px; }
.kv b { font-weight: 600; font-variant-numeric: tabular-nums; }
.controls { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; margin-top: 10px; }
button, select { font: inherit; padding: 4px 10px; border-radius: 6px; border: 1px solid var(--line);
  background: var(--panel); color: var(--text); cursor: pointer; }
input[type=range] { width: 100%; accent-color: var(--accent); }
.log { max-height: 340px; overflow: auto; font-size: 12px; }
.log div { padding: 2px 4px; border-radius: 4px; cursor: pointer; display: flex; gap: 8px; }
.log div:hover { background: var(--bg); }
.log .t { color: var(--muted); font-variant-numeric: tabular-nums; min-width: 42px; }
.log .good { color: var(--good); } .log .bad { color: var(--bad); } .log .state { color: var(--state); }
.log .now { background: var(--bg); }
.verdict { font-size: 15px; font-weight: 600; }
.legend { display: flex; gap: 12px; flex-wrap: wrap; font-size: 12px; color: var(--muted); }
.dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 4px;
  vertical-align: -1px; }
</style>
</head>
<body>
<div class="wrap">
  <div class="stage">
    <canvas id="c" width="720" height="720"></canvas>
    <div class="controls">
      <button id="play">▶ 재생</button>
      <select id="speed">
        <option value="0.5">0.5배</option><option value="1" selected>1배</option>
        <option value="2">2배</option><option value="4">4배</option>
      </select>
      <span id="clock" class="muted"></span>
    </div>
    <input id="seek" type="range" min="0" value="0">
    <div class="legend">
      <span><span class="dot" style="background:var(--us)"></span>우리 로봇</span>
      <span><span class="dot" style="background:var(--opp)"></span>상대 로봇</span>
      <span><span class="dot" style="background:var(--cube)"></span>큐브</span>
      <span>× 우리 LiDAR가 찾은 상대</span><span>○ 카메라로 추정한 큐브 위치</span>
    </div>
  </div>
  <div class="side">
    <div class="card">
      <h1>임무 2 시뮬레이션</h1>
      <div class="muted" id="setup"></div>
      <div class="verdict" id="verdict" style="margin-top:8px"></div>
    </div>
    <div class="card kv">
      <span>시간</span><b id="k-time"></b>
      <span>우리 상태</span><b id="k-state"></b>
      <span>집게</span><b id="k-grip"></b>
      <span>큐브</span><b id="k-cube"></b>
      <span>연속 확보</span><b id="k-hold"></b>
      <span>카메라</span><b id="k-cam"></b>
    </div>
    <div class="card">
      <div class="muted" style="margin-bottom:6px">사건 (누르면 그 시각으로)</div>
      <div class="log" id="log"></div>
    </div>
  </div>
</div>
<script>
const DATA = __DATA__;
const F = DATA.frames, A = DATA.arena;
const canvas = document.getElementById('c'), ctx = canvas.getContext('2d');
const seek = document.getElementById('seek');
seek.max = F.length - 1;
let idx = 0, playing = false, lastTs = null, acc = 0;

function css(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }
function resize() {
  const size = Math.min(720, window.innerWidth - 56, window.innerHeight - 120);
  const dpr = window.devicePixelRatio || 1;
  canvas.style.width = canvas.style.height = Math.max(280, size) + 'px';
  canvas.width = canvas.height = Math.max(280, size) * dpr;
  draw();
}
function S() { return canvas.width / A.size; }
function P(x, y) { const s = S(); return [x * s, canvas.height - y * s]; }

function drawRobot(x, y, yaw, r, color, gripper, held) {
  const s = S(), [cx, cy] = P(x, y);
  ctx.save(); ctx.translate(cx, cy); ctx.rotate(-yaw);
  ctx.fillStyle = color; ctx.globalAlpha = 0.85;
  ctx.beginPath(); ctx.arc(0, 0, r * s, 0, Math.PI * 2); ctx.fill();
  ctx.globalAlpha = 1; ctx.strokeStyle = css('--panel'); ctx.lineWidth = Math.max(2, s * 0.012);
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(r * s, 0); ctx.stroke();
  if (gripper) {
    const open = gripper === 'open' ? 0.06 : (gripper === 'moving' ? 0.045 : 0.03);
    ctx.strokeStyle = color; ctx.lineWidth = Math.max(2, s * 0.014);
    for (const side of [-1, 1]) {
      ctx.beginPath(); ctx.moveTo(0.08 * s, side * 0.05 * s);
      ctx.lineTo(0.17 * s, side * open * s); ctx.stroke();
    }
  }
  ctx.restore();
}

function draw() {
  if (!F.length) return;
  const f = F[idx], s = S(), W = canvas.width;
  const [t, x, y, yaw, state, grip, cx, cy, holder, opps, seen, cam, target] = f;
  ctx.fillStyle = css('--out'); ctx.fillRect(0, 0, W, W);
  const [sx, sy] = P(A.safe_min, A.safe_max);
  ctx.fillStyle = css('--floor'); ctx.fillRect(sx, sy, (A.safe_max - A.safe_min) * s, (A.safe_max - A.safe_min) * s);
  ctx.strokeStyle = css('--grid'); ctx.lineWidth = 1;
  for (let i = 1; i < 9; i++) {
    const p = i * A.cell * s;
    ctx.beginPath(); ctx.moveTo(p, 0); ctx.lineTo(p, W); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, p); ctx.lineTo(W, p); ctx.stroke();
  }
  ctx.strokeStyle = css('--yellow'); ctx.lineWidth = Math.max(2, s * 0.012);
  ctx.strokeRect(sx, sy, (A.safe_max - A.safe_min) * s, (A.safe_max - A.safe_min) * s);
  ctx.fillStyle = css('--muted'); ctx.font = `${Math.max(10, s * 0.07)}px system-ui`;
  for (const [label, px, py] of A.starts) { const [lx, ly] = P(px, py); ctx.fillText(label, lx - s * 0.17, ly - s * 0.13); }
  // 지키는 원
  const [ccx, ccy] = P(A.center[0], A.center[1]);
  ctx.setLineDash([6, 6]); ctx.strokeStyle = css('--muted'); ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(ccx, ccy, A.guard_r * s, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);
  // 궤적
  const from = Math.max(0, idx - Math.round(3 / A.dt));
  ctx.lineWidth = Math.max(1.5, s * 0.008);
  ctx.strokeStyle = css('--us'); ctx.globalAlpha = 0.35; ctx.beginPath();
  for (let i = from; i <= idx; i++) { const [px, py] = P(F[i][1], F[i][2]); i === from ? ctx.moveTo(px, py) : ctx.lineTo(px, py); }
  ctx.stroke();
  ctx.strokeStyle = css('--opp');
  opps.forEach((_, k) => { ctx.beginPath();
    for (let i = from; i <= idx; i++) { const o = F[i][9][k]; const [px, py] = P(o[0], o[1]); i === from ? ctx.moveTo(px, py) : ctx.lineTo(px, py); }
    ctx.stroke(); });
  ctx.globalAlpha = 1;
  // 카메라 시야
  const camX = x + A.cam_ahead * Math.cos(yaw), camY = y + A.cam_ahead * Math.sin(yaw);
  const [px0, py0] = P(camX, camY);
  ctx.fillStyle = css('--us'); ctx.globalAlpha = cam ? 0.16 : 0.06;
  ctx.beginPath(); ctx.moveTo(px0, py0);
  ctx.arc(px0, py0, A.cam_range * s, -yaw - A.cam_fov, -yaw + A.cam_fov); ctx.closePath(); ctx.fill();
  ctx.globalAlpha = 1;
  // 상대
  opps.forEach((o, k) => { if (o[3]) drawRobot(o[0], o[1], o[2], A.opp_r, css('--opp'), null);
    else { const [ox, oy] = P(o[0], o[1]); ctx.fillStyle = css('--muted'); ctx.fillText('탈락', ox - s * 0.08, oy); } });
  // 우리
  drawRobot(x, y, yaw, A.our_r, css('--us'), grip);
  // 큐브
  const [qx, qy] = P(cx, cy), q = Math.max(6, 0.05 * s);
  ctx.fillStyle = css('--cube'); ctx.fillRect(qx - q / 2, qy - q / 2, q, q);
  // 우리가 본 상대, 추정 큐브
  ctx.strokeStyle = css('--text'); ctx.lineWidth = 2;
  for (const [ox, oy] of seen) { const [a, b] = P(ox, oy), m = Math.max(5, s * 0.04);
    ctx.beginPath(); ctx.moveTo(a - m, b - m); ctx.lineTo(a + m, b + m); ctx.moveTo(a + m, b - m); ctx.lineTo(a - m, b + m); ctx.stroke(); }
  if (target) { const [a, b] = P(target[0], target[1]); ctx.strokeStyle = css('--cube');
    ctx.beginPath(); ctx.arc(a, b, Math.max(6, s * 0.05), 0, Math.PI * 2); ctx.stroke(); }
  updatePanel();
}

function holdTime() {
  const h = F[idx][8]; if (h !== 'us') return null;
  let i = idx; while (i > 0 && F[i - 1][8] === 'us') i--;
  return F[idx][0] - F[i][0];
}
function updatePanel() {
  const f = F[idx];
  document.getElementById('k-time').textContent = `${f[0].toFixed(1)} / ${DATA.time} s`;
  document.getElementById('k-state').textContent = f[4];
  document.getElementById('k-grip').textContent = {open: '열림', moving: '움직이는 중', closed: '닫힘'}[f[5]] || f[5];
  document.getElementById('k-cube').textContent = f[8] === 'us' ? '우리가 잡음' : (f[8] ? `${f[8]}이(가) 잡음` : '아무도 안 잡음');
  const h = holdTime();
  document.getElementById('k-hold').textContent = h === null ? '-' : `${h.toFixed(1)} s`;
  document.getElementById('k-cam').textContent = f[11] ? '큐브 보임' : '안 보임';
  document.getElementById('clock').textContent = `${f[0].toFixed(1)} s`;
  seek.value = idx;
  let cur = -1;
  DATA.events.forEach((e, i) => { if (e.t <= f[0]) cur = i; });
  document.querySelectorAll('.log div').forEach((el, i) => el.classList.toggle('now', i === cur));
}

const log = document.getElementById('log');
DATA.events.forEach(e => {
  const d = document.createElement('div');
  d.innerHTML = `<span class="t">${e.t.toFixed(1)}s</span><span class="${e.kind}"></span>`;
  d.lastChild.textContent = e.text;
  d.onclick = () => { idx = Math.min(F.length - 1, Math.round(e.t / A.dt)); draw(); };
  log.appendChild(d);
});
document.getElementById('setup').textContent = DATA.setup;
document.getElementById('verdict').textContent = '결과: ' + DATA.verdict;

function tick(ts) {
  if (!playing) { lastTs = null; return; }
  if (lastTs !== null) acc += (ts - lastTs) / 1000 * parseFloat(document.getElementById('speed').value);
  lastTs = ts;
  while (acc >= A.dt) { acc -= A.dt; if (idx < F.length - 1) idx++; else { setPlaying(false); break; } }
  draw();
  requestAnimationFrame(tick);
}
function setPlaying(on) {
  playing = on; document.getElementById('play').textContent = on ? '⏸ 일시정지' : '▶ 재생';
  if (on) { if (idx >= F.length - 1) idx = 0; requestAnimationFrame(tick); }
}
document.getElementById('play').onclick = () => setPlaying(!playing);
seek.oninput = () => { idx = parseInt(seek.value, 10); draw(); };
document.addEventListener('keydown', e => {
  if (e.code === 'Space') { e.preventDefault(); setPlaying(!playing); }
  if (e.code === 'ArrowRight') { idx = Math.min(F.length - 1, idx + Math.round(1 / A.dt)); draw(); }
  if (e.code === 'ArrowLeft') { idx = Math.max(0, idx - Math.round(1 / A.dt)); draw(); }
});
window.addEventListener('resize', resize);
resize();
setPlaying(true);
</script>
</body>
</html>
"""


def write_html(world, args, verdict, path):
    cell = m2.CELL_SIZE_M
    starts = [('우리 (8,2)', *m2.START_POSE[:2])]
    for opp, cell_name in zip(world.opps, [args.opp_start] + [c for c in ('2,8', '2,2', '8,8')
                                                               if c != args.opp_start]):
        pose = rotate_about_center(m2.START_POSE, START_CELLS[cell_name])
        starts.append((f'{opp.name} ({cell_name})', pose[0], pose[1]))
    data = {
        'time': args.time,
        'verdict': verdict,
        'setup': (f'상대 {args.opp_count}대 · {args.opp_behavior} · 속도 {args.opp_speed} m/s · '
                  f'회전 {args.opp_turn} rad/s · 반지름 {args.opp_radius} m · '
                  f'뺏기 {args.steal_s or "없음"}{" s" if args.steal_s else ""} · '
                  f'출발 지연 {args.opp_delay} s (우리 로봇: 실제 임무 2 코드)'),
        'arena': {
            'size': m2.ARENA_SIZE_M, 'cell': cell, 'dt': DT,
            'safe_min': m2.SAFE_ZONE_MIN_M, 'safe_max': m2.SAFE_ZONE_MAX_M,
            'center': m2.CENTER_M, 'guard_r': m2.GUARD_RADIUS_M,
            'our_r': OUR_RADIUS_M, 'opp_r': args.opp_radius,
            'cam_ahead': m2.CAMERA_AHEAD_M, 'cam_fov': m2.CAMERA_HALF_FOV_RAD,
            'cam_range': math.sqrt(AREA_SCALE / CAMERA_MIN_AREA),
            'starts': starts,
        },
        'frames': world.frames,
        'events': world.events,
    }
    with open(path, 'w', encoding='utf-8') as out:
        out.write(HTML_TEMPLATE.replace('__DATA__', json.dumps(data, ensure_ascii=False)))


def open_in_browser(path):
    """WSL이면 Windows 브라우저로, 아니면 기본 브라우저로 연다."""
    if shutil.which('wslpath') and shutil.which('explorer.exe'):
        win = subprocess.run(['wslpath', '-w', path], capture_output=True, text=True).stdout.strip()
        subprocess.run(['explorer.exe', win], check=False)
        return
    import webbrowser
    webbrowser.open('file://' + os.path.abspath(path))


def parse_args():
    parser = argparse.ArgumentParser(description='임무 2 2D 가상 시뮬레이션 (상대 로봇만 바꾼다)')
    parser.add_argument('--time', type=float, default=60.0, help='경기 시간 s (규정 300)')
    parser.add_argument('--opp-count', type=int, default=1, choices=(1, 2, 3), help='상대 수')
    parser.add_argument('--opp-start', default='2,8', choices=('2,8', '2,2', '8,8'),
                        help='첫 상대 출발 격자 (우리는 8,2)')
    parser.add_argument('--opp-speed', type=float, default=0.4, help='상대 최고 속도 m/s (우리 0.8)')
    parser.add_argument('--opp-turn', type=float, default=2.0, help='상대 최고 회전 rad/s')
    parser.add_argument('--opp-accel', type=float, default=1.0, help='상대 가속 m/s^2')
    parser.add_argument('--opp-radius', type=float, default=0.15, help='상대 반지름 m (규정 최대 0.2)')
    parser.add_argument('--opp-behavior', default='chase', choices=('chase', 'still', 'wander'),
                        help='chase = 큐브(누가 잡든)로 감, still = 가만히, wander = 아무 데나')
    parser.add_argument('--opp-delay', type=float, default=0.0, help='상대 출발 지연 s')
    parser.add_argument('--steal-s', type=float, default=2.0,
                        help='상대가 우리가 잡은 큐브에 이만큼 붙어 있으면 뺏김 (0 = 안 뺏김)')
    parser.add_argument('--seed', type=int, default=1, help='wander 무작위 씨앗')
    parser.add_argument('--out', default=None, help='HTML 저장 경로')
    parser.add_argument('--no-open', action='store_true', help='브라우저로 열지 않음')
    return parser.parse_args()


def main():
    args = parse_args()
    world = World(args)
    verdict = world.run()
    out = args.out or os.path.join(
        os.path.expanduser('~'), 'poli_sim', f'mission2_{datetime.now():%Y%m%d_%H%M%S}.html')
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    write_html(world, args, verdict, out)

    print(f'결과: {verdict}')
    for e in world.events:
        if e['kind'] != 'state':
            print(f"  {e['t']:5.1f} s  {e['text']}")
    print(f'HTML: {out}')
    if not args.no_open:
        open_in_browser(out)


if __name__ == '__main__':
    main()
