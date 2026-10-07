"""
집게(DS3218 1개, RRC Lite PWM 서보 포트) 상태 계산. rclpy에 의존하지 않는 순수 모듈.

/gripper/command: "open" = 열기, "grab" = 닫아 잡기 (들어 올리기 없음, 조원 A 결정)
/gripper/state:   "open" / "closed" / "moving"

규정 3.5.6 (시작 선언 전 기동 금지): 첫 명령을 받기 전에는 서보 명령을 보내지 않는다.
RRC 공장 펌웨어는 전원을 켤 때 서보를 1500 us로 보내므로, 열림 위치를 1500 us로 두면
전원을 켤 때도 집게가 움직이지 않는다 (hardware.yaml gripper_open_us).
"""
from typing import Optional

COMMANDS = ('open', 'grab')


class GripperController:

    def __init__(self, open_us: int, close_us: int, move_ms: int, done_margin_s: float = 0.5):
        self.pulse = {'open': int(open_us), 'grab': int(close_us)}
        self.move_s = move_ms / 1000.0
        self.move_ms = int(move_ms)
        self.done_margin_s = done_margin_s
        self.target = None          # 'open' / 'grab'
        self.started_at = None
        self.reported_us = None     # RRC가 보고한 현재 명령 펄스 (없으면 None)

    def command(self, word: str, now: float) -> Optional[int]:
        """명령 -> 보낼 펄스(us). 모르는 명령이거나 이미 같은 목표면 None."""
        word = word.strip().lower()
        if word not in COMMANDS:
            raise ValueError(word)
        if word == self.target:
            return None
        self.target = word
        self.started_at = now
        self.reported_us = None
        return self.pulse[word]

    def report(self, pulse_us: int):
        self.reported_us = pulse_us

    def state(self, now: float) -> str:
        if self.target is None:
            return 'open'           # 첫 명령 전: 전원 켤 때 위치(열림)로 가정
        done_name = 'closed' if self.target == 'grab' else 'open'
        if self.reported_us is not None and self.reported_us == self.pulse[self.target]:
            return done_name
        if now - self.started_at >= self.move_s + self.done_margin_s:
            return done_name        # 보고가 없어도 이동 시간이 지나면 끝난 것으로 본다
        return 'moving'
