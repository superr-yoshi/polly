"""
Arduino Mega <-> ROS 2 브리지 (프로토콜 v2.0, docs/serial_protocol.md).

2026-10-29부터 Mega가 주행까지 맡는다 (RRC Lite 사용 중단, 모터 드라이버 MDD10).
  Pi -> Mega: $DRV 목표 바퀴 속도 25 Hz, $GRIP 집게
  Mega -> Pi: $ENC 엔코더 50 Hz, $IMU MPU-6050 25 Hz, $RNG 초음파 8 Hz, $GST 집게 2 Hz
Topic 계약은 drive_node.DriveNode와 같다 (fake_mega_node와 바꿔 끼울 수 있음).

시리얼 읽기는 별도 스레드, packet 해석은 mega_protocol(순수 모듈)이 담당한다.
"""
import threading
import time

from poli_hardware import drive_node
from poli_hardware.mega_protocol import (
    EncPacket, gripper_action, GstPacket, ImuPacket, make_drive_command, make_grip_command,
    MegaParser, MOTOR_FAULTS, RngPacket, wheel_to_ticks, WheelSpeedEstimator)
import rclpy

MAX_GRIP_TRIES = 3


class MegaTransport:

    def __init__(self, node):
        self._node = node
        self._log = node.get_logger()
        p = node.declare_parameter
        p('port', '/dev/robot_mega')
        p('baud', 115200)
        p('open_delay', 2.0)            # 포트를 열면 Mega가 자동 리셋된다
        p('stale_timeout', 1.0)         # s, RNG 끊김 경고
        p('motor_ticks_per_rev', 1320.0)  # 바퀴 1회전 엔코더 tick (4체배, TODO_MEASURE)
        p('encoder_timeout', 0.2)       # s, ENC(50 Hz)가 이만큼 끊기면 바퀴 속도 0으로 봄
        p('imu_timeout', 0.5)
        p('ack_timeout', 0.5)           # s, $GRIP 응답 기다리는 시간
        g = node.get_parameter
        self.port = g('port').value
        self.baud = g('baud').value
        self._open_delay = g('open_delay').value
        self._stale_timeout = g('stale_timeout').value
        self._ticks = g('motor_ticks_per_rev').value
        self._enc_timeout = g('encoder_timeout').value
        self._imu_timeout = g('imu_timeout').value
        self._ack_timeout = g('ack_timeout').value

        self.parser = MegaParser()
        self._speeds = WheelSpeedEstimator(self._ticks)
        self._enc_t = None
        self._enc_warned = False
        self._motor_status = None
        self._imu = None
        self._imu_t = 0.0
        self._imu_warned = False
        self._last_rng = None
        self._stale_warned = False
        self._grip_state = 'open'       # 첫 GST 전: 부팅 위치(열림)로 가정
        self._grip_word = None
        self._cmd_id = 0
        self._last_ack = 0              # 읽기 스레드가 GST의 last_id로 갱신
        self._pending = None            # [action, cmd_id, 보낸 시각, 보낸 횟수]

        self._ser = None
        self._write_lock = threading.Lock()
        self._stop = False
        node.create_timer(0.1, self._on_gripper_retry)
        node.create_timer(1.0, self._on_health)
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    # ---- serial ---------------------------------------------------------
    def _open(self):
        import serial  # pyserial (python3-serial)
        ser = serial.Serial(self.port, self.baud, timeout=0.2)
        time.sleep(self._open_delay)
        ser.reset_input_buffer()
        self._log.info(f'Mega connected: {self.port} @ {self.baud}')
        return ser

    def _read_loop(self):
        while not self._stop:
            if self._ser is None:
                try:
                    self._ser = self._open()
                except Exception as e:  # noqa: BLE001 - 포트 없음/권한 등 모두 재시도
                    self._log.error(f'Mega open failed ({self.port}): {e}; 2초 후 재시도',
                                    throttle_duration_sec=10.0)
                    time.sleep(2.0)
                    continue
            try:
                raw = self._ser.readline()
            except Exception as e:  # noqa: BLE001 - USB 분리 등
                self._log.error(f'Mega read error: {e}; 재연결')
                self._close()
                continue
            if raw:
                try:
                    self._handle(self.parser.feed(raw))
                except Exception:  # noqa: BLE001
                    if not rclpy.ok():  # Ctrl+C로 ROS가 먼저 종료됨 -> 조용히 끝낸다
                        break
                    raise

    def _close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
        self._ser = None

    def _write(self, data):
        ser = self._ser
        if ser is None:
            return False
        try:
            with self._write_lock:
                ser.write(data)
            return True
        except Exception as e:  # noqa: BLE001
            self._log.error(f'Mega write error: {e}', throttle_duration_sec=2.0)
            return False

    def _handle(self, pkt):
        if isinstance(pkt, EncPacket):
            self._speeds.update(pkt)
            self._enc_t = time.monotonic()
            if pkt.status != self._motor_status:
                self._on_motor_status(pkt)
                self._motor_status = pkt.status
        elif isinstance(pkt, ImuPacket):
            self._imu, self._imu_t = pkt.to_si(), time.monotonic()
        elif isinstance(pkt, RngPacket):
            self._last_rng = time.monotonic()
            self._stale_warned = False
            self._node.publish_ranges(pkt.mm)
        elif isinstance(pkt, GstPacket):
            self._grip_state = pkt.state_name
            self._last_ack = pkt.last_id

    def _on_motor_status(self, pkt):
        if pkt.status in MOTOR_FAULTS:
            self._log.error(
                f'Mega 모터 정지: {pkt.status_name}. 엔코더(C1·C2)·모터선 방향 또는 바퀴 막힘 확인. '
                '이 노드를 다시 켜야 (Mega 리셋) 다시 움직인다 (docs/pin_map.md)')
        else:
            self._log.info(f'Mega 모터 상태: {pkt.status_name}')

    # ---- 주행 -----------------------------------------------------------
    def set_wheel_speeds(self, left, right):
        self._write(make_drive_command(wheel_to_ticks(left, self._ticks),
                                       wheel_to_ticks(right, self._ticks)))

    def read_wheel_speeds(self):
        if self._enc_t is None or time.monotonic() - self._enc_t > self._enc_timeout:
            if not self._enc_warned and self._ser is not None:
                self._log.warn(f'Mega ENC 수신 없음 ({self._enc_timeout}s) -> 바퀴 속도 0으로 봄')
                self._enc_warned = True
            return 0.0, 0.0
        self._enc_warned = False
        return self._speeds.read()

    def read_imu(self):
        if self._imu is None or time.monotonic() - self._imu_t > self._imu_timeout:
            if not self._imu_warned:
                self._log.warn(f'Mega IMU 수신 없음 ({self._imu_timeout}s, MPU-6050 미연결?) '
                               '-> /imu/data는 휠 각속도로 대신함')
                self._imu_warned = True
            return None
        self._imu_warned = False
        return self._imu

    # ---- 집게 -----------------------------------------------------------
    def set_gripper(self, word):
        if word != self._grip_word:
            self._log.info(f'gripper command: {word}')   # 1초마다 같은 명령이 와서 바뀔 때만
            self._grip_word = word
        # 같은 명령도 Mega에 다시 보낸다 (Mega가 리셋됐을 때 복구). 펌웨어는 같은 목표면 그대로 둔다.
        self._send_grip(gripper_action(word), tries=1)

    def _send_grip(self, action, tries):
        self._cmd_id += 1
        if not self._write(make_grip_command(self._cmd_id, action)):
            self._log.warn('집게 명령 실패: Mega 연결 안 됨')
            self._pending = None
            return
        self._pending = [action, self._cmd_id, time.monotonic(), tries]

    def _on_gripper_retry(self):
        """Mega가 GST로 받았다고 답할 때까지 최대 MAX_GRIP_TRIES번 보낸다."""
        if self._pending is None:
            return
        action, cmd_id, sent_at, tries = self._pending
        if self._last_ack == cmd_id:
            self._pending = None
        elif time.monotonic() - sent_at > self._ack_timeout:
            if tries < MAX_GRIP_TRIES:
                self._send_grip(action, tries + 1)
            else:
                self._log.error('집게 명령: Mega 응답 없음')
                self._pending = None

    def gripper_state(self):
        return self._grip_state

    # ---- 진단 -----------------------------------------------------------
    def _on_health(self):
        if self._ser is None or self._last_rng is None:
            return
        if time.monotonic() - self._last_rng > self._stale_timeout and not self._stale_warned:
            self._log.warn(f'RNG 수신 없음 {self._stale_timeout}s 이상')
            self._stale_warned = True
        if self._motor_status in MOTOR_FAULTS:
            self._log.error('Mega 모터 고장 정지 상태 (노드 재시작 필요)',
                            throttle_duration_sec=5.0)
        p = self.parser
        self._log.debug(f'packets ok={p.ok} bad={p.bad} gap={p.gap}')

    def close(self):
        self._write(make_drive_command(0, 0))
        self._stop = True
        self._reader.join(timeout=1.0)
        self._close()
        p = self.parser
        print(f'[mega_bridge_node] packets ok={p.ok} bad={p.bad} gap={p.gap}')


def main(args=None):
    drive_node.run('mega_bridge_node', MegaTransport, args)


if __name__ == '__main__':
    main()
