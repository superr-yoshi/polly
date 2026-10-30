#!/usr/bin/env python3
"""
라즈베리파이 상태 진단 - 아두이노 USB 연결 중 관찰용 (읽기 전용, 낮은 부하).

하는 일: 온도, get_throttled 비트, (Pi 5) PMIC 외부 5 V 참고값, USB 장치 목록 변화,
커널 로그의 저전압·USB 과전류·연결 해제·reset·인식 오류 기록을 정해진 시간 동안 관찰하고
결과를 화면·CSV·JSON으로 남긴다. 시작할 때 AI Camera(IMX500)가 인식되는지도 한 번 본다
(목록만 읽고 영상은 켜지 않는다. --camera-capture를 붙이면 사진 한 장을 찍는다).

하지 않는 일: 시리얼 포트 열기(DTR/RTS 변경 포함 - 여는 것만으로 아두이노가 리셋될 수 있음),
GPIO 출력, USB 전원 제어, 재부팅, 부하 시험, 패키지 설치, 시스템 설정 변경, 펌웨어 변경.

실행 (Pi의 Linux에서):
    python3 scripts/pi_health_check.py                     # 120초, 2초 간격
    python3 scripts/pi_health_check.py --duration 60 --interval 5
    python3 scripts/pi_health_check.py --baseline          # 아두이노 분리 상태 기준 기록
    python3 scripts/pi_health_check.py --camera absent     # 카메라를 꽂기 전 기준 기록
    python3 scripts/pi_health_check.py --duration 10 --camera-capture   # 카메라 사진 시험
    sudo python3 scripts/pi_health_check.py                # 커널 로그 읽기 권한이 없을 때 (선택)
외부 패키지 필요 없음 (표준 라이브러리만). 사용법·결과 해석: docs/pi_health_check.md
"""
import argparse
import csv
import datetime
import glob
import json
import os
import platform
import re
import shutil
import signal
import subprocess
import sys
import time

OK = '관찰된 이상 없음'
BAD = '이상 관찰'
UNK = '확인 불가'
UNKNOWN_INPUT = '미확인'

# 판정 기준 (손상 기준이 아니라 "눈여겨볼 값" 기준)
TEMP_LIMIT_C = 80.0      # Pi 5 펌웨어가 클록을 낮추기 시작하는 무렵의 온도
EXT5V_LOW_V = 4.75       # 참고 기준. USB 역류 판단에는 쓰지 않는다
RESET_REPEAT = 3         # 같은 USB 장치 reset이 이 횟수 이상이면 '반복 reset'

THROTTLE_BITS = {
    0: '저전압(현재)',
    1: 'ARM 주파수 제한(현재)',
    2: 'throttling(현재)',
    3: '온도 제한(현재)',
    16: '저전압 발생 이력',
    17: 'ARM 주파수 제한 발생 이력',
    18: 'throttling 발생 이력',
    19: '온도 제한 발생 이력',
}
CURRENT_MASK = 0x0000F
HISTORY_MASK = 0xF0000

# VID는 USB 칩/등록 회사 정보일 뿐이다. 보드 모델·정품 여부·전원 회로를 알려 주지 않는다.
USB_VID_HINTS = {
    '2341': 'Arduino SA 등록 VID (호환보드가 같은 VID를 쓸 수도 있음)',
    '2a03': 'Arduino.org 등록 VID',
    '1a86': 'QinHeng CH340/CH341/CH9102 계열 USB-시리얼 칩',
    '0403': 'FTDI USB-시리얼 칩',
    '10c4': 'Silicon Labs CP210x USB-시리얼 칩',
    '067b': 'Prolific PL2303 USB-시리얼 칩',
}
SERIAL_DRIVERS = ('cdc_acm', 'ch341', 'ftdi_sio', 'cp210x', 'pl2303')

# (종류, 한국어 이름, 패턴). 한 줄은 처음 맞는 규칙 하나로만 센다.
LOG_RULES = [
    ('camera_error', '카메라 오류', re.compile(
        r'(?:imx500|rp1-cfe|rp1_cfe).*(?:fail|error|timed? ?out)', re.I)),
    ('under_voltage', '저전압', re.compile(r'under-?voltage', re.I)),
    ('voltage_normal', '전압 정상 복귀 알림(직전 저전압)',
     re.compile(r'voltage normali[sz]ed', re.I)),
    ('over_current', 'USB 과전류', re.compile(r'over-?current', re.I)),
    ('enum_error', '장치 인식 오류', re.compile(
        r'device descriptor read|not accepting address|unable to enumerate'
        r"|can't set config|cannot enable|cannot reset|error -(?:71|110|32)\b", re.I)),
    ('usb_reset', 'USB 장치 reset',
     re.compile(r'reset (?:low|full|high|super|super\+)[- ]speed', re.I)),
    ('usb_disconnect', 'USB 연결 해제', re.compile(r'USB disconnect', re.I)),
    ('usb_new', '새 USB 장치 인식',
     re.compile(r'new (?:low|full|high|super|super\+)[- ]speed USB device', re.I)),
]
LOG_LABEL = {kind: label for kind, label, _ in LOG_RULES}
HISTORY_SERIOUS = ('camera_error', 'under_voltage', 'voltage_normal', 'over_current',
                   'enum_error')

# 카메라: Raspberry Pi AI Camera (Sony IMX500). AGENTS.md 1장, hardware_reference.md 1장
CAMERA_MODEL = 'imx500'
CAMERA_FIRMWARE = ('/lib/firmware/imx500_loader.fpk', '/lib/firmware/imx500_main.fpk')
CAMERA_CONFIG_FILES = ('/boot/firmware/config.txt', '/boot/config.txt')
CAMERA_LIST_TIMEOUT = 15.0   # 카메라 목록 읽기는 libcamera 초기화 때문에 몇 초 걸릴 수 있다
CAMERA_CAPTURE_TIMEOUT = 40.0
DARK_MEAN = 5.0              # 테스트 사진 평균 밝기(0~255)가 이보다 낮으면 '거의 검정'

# Picamera2로 카메라 목록만 읽는다 (영상은 켜지 않음). 별도 프로세스로 실행해 timeout을 건다.
PICAM_LIST_CODE = '''
import json
from picamera2 import Picamera2
print(json.dumps([{k: str(v) for k, v in c.items()} for c in Picamera2.global_camera_info()]))
'''
# --camera-capture 일 때만: camera_vision 노드와 같은 640x480으로 사진 한 장을 찍어 저장한다.
PICAM_CAPTURE_CODE = '''
import json, sys, time
from picamera2 import Picamera2
cam = Picamera2()
cam.configure(cam.create_still_configuration(main={"size": (640, 480), "format": "RGB888"}))
cam.start()
time.sleep(2)  # 자동 노출·화이트밸런스가 맞춰질 시간
arr = cam.capture_array()
cam.capture_file(sys.argv[1])
cam.stop()
cam.close()
print(json.dumps({"shape": list(arr.shape), "mean": float(arr.mean())}))
'''
USB_DEV_RE = re.compile(r'\busb (\d+-[\d.]+)')
LOG_LINE_RE = re.compile(r'^\[\s*(\d+\.\d+)\]\s?(.*)$')
PERM_RE = re.compile(
    r'permission|not permitted|denied|insufficient|/dev/vcio|VCHI|failed to open', re.I)

STATUS_TEXT = {
    'missing': '명령 없음',
    'permission': '권한 부족',
    'timeout': '시간 초과',
    'error': '오류',
    'unsupported': '미지원',
    'parse': '출력 해석 실패',
    'absent': '센서/파일 없음',
    'not_applicable': '해당 없음',
}


def now_text():
    return datetime.datetime.now().strftime('%H:%M:%S')


def read_file(path):
    """sysfs/procfs 파일을 읽는다. (값, 상태) 반환."""
    try:
        with open(path, 'rb') as f:
            return f.read().decode('utf-8', 'replace').replace('\x00', '').strip(), 'ok'
    except FileNotFoundError:
        return None, 'absent'
    except PermissionError:
        return None, 'permission'
    except OSError:
        return None, 'error'


class Runner:
    """외부 명령을 timeout과 함께 실행한다. 표준입력은 닫는다."""

    def __init__(self, timeout):
        self.timeout = timeout
        self.missing = set()

    def run(self, args, timeout=None):
        name = args[0]
        timeout = timeout or self.timeout
        if name in self.missing or shutil.which(name) is None:
            self.missing.add(name)
            return {'status': 'missing', 'detail': f'{name} 명령 없음'}
        try:
            proc = subprocess.run(args, stdin=subprocess.DEVNULL, capture_output=True,
                                  text=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            return {'status': 'timeout', 'detail': f'{" ".join(args[:2])}: {timeout:g}초 초과'}
        except OSError as exc:
            return {'status': 'error', 'detail': str(exc)}
        out, err = proc.stdout.strip(), proc.stderr.strip()
        if proc.returncode != 0:
            status = 'permission' if PERM_RE.search(f'{out} {err}') else 'error'
            detail = (err or out or f'종료 코드 {proc.returncode}')[:300]
            return {'status': status, 'detail': detail, 'stdout': out, 'stderr': err}
        return {'status': 'ok', 'stdout': out, 'stderr': err}

    def vc(self, *args):
        r = self.run(['vcgencmd', *args])
        if r['status'] == 'ok' and re.search(r'error=|error_msg', r['stdout']):
            return {'status': 'unsupported', 'detail': r['stdout'][:200]}
        return r


def fail(r):
    """측정 실패 결과를 통일된 dict로."""
    return {'value': None, 'status': r['status'],
            'detail': f"{STATUS_TEXT.get(r['status'], r['status'])}: {r.get('detail', '')}"}


# ---------------------------------------------------------------- 측정

def read_static(runner):
    info = {}
    model, _ = read_file('/proc/device-tree/model')
    if not model:
        model, _ = read_file('/sys/firmware/devicetree/base/model')
    cpuinfo, _ = read_file('/proc/cpuinfo')
    if not model and cpuinfo:
        m = re.search(r'^Model\s*:\s*(.+)$', cpuinfo, re.M)
        model = m.group(1).strip() if m else None
    info['pi_model'] = model or UNK
    m = re.search(r'^Revision\s*:\s*(\S+)', cpuinfo or '', re.M)
    info['pi_revision'] = m.group(1) if m else UNK
    osr, _ = read_file('/etc/os-release')
    m = re.search(r'^PRETTY_NAME="?([^"\n]+)"?', osr or '', re.M)
    info['os'] = m.group(1) if m else UNK
    info['kernel'] = f'{platform.release()} ({platform.machine()})'
    up, _ = read_file('/proc/uptime')
    info['uptime_s'] = float(up.split()[0]) if up else None
    info['is_pi5'] = 'Raspberry Pi 5' in (model or '')
    info['euid_root'] = hasattr(os, 'geteuid') and os.geteuid() == 0
    r = runner.run(['lsusb'])
    info['lsusb'] = r['stdout'].splitlines() if r['status'] == 'ok' else fail(r)['detail']
    return info


def read_temp(runner):
    r = runner.vc('measure_temp')
    if r['status'] == 'ok':
        m = re.search(r'temp=([-\d.]+)', r['stdout'])
        if m:
            return {'value': float(m.group(1)), 'status': 'ok', 'source': 'vcgencmd'}
        r = {'status': 'parse', 'detail': r['stdout'][:100]}
    # vcgencmd를 못 쓰면 커널 온도 센서를 대신 읽는다 (출처를 남긴다)
    raw, st = read_file('/sys/class/thermal/thermal_zone0/temp')
    if raw and raw.lstrip('-').isdigit():
        return {'value': int(raw) / 1000.0, 'status': 'ok',
                'source': f"sysfs thermal_zone0 (vcgencmd {STATUS_TEXT.get(r['status'])})"}
    return fail(r)


def read_throttled(runner):
    r = runner.vc('get_throttled')
    if r['status'] != 'ok':
        return fail(r)
    m = re.search(r'throttled=(0x[0-9a-fA-F]+)', r['stdout'])
    if not m:
        return fail({'status': 'parse', 'detail': r['stdout'][:100]})
    return {'value': int(m.group(1), 16), 'raw': m.group(1), 'status': 'ok'}


def read_core_volts(runner):
    r = runner.vc('measure_volts', 'core')
    if r['status'] != 'ok':
        return fail(r)
    m = re.search(r'volt=([\d.]+)V', r['stdout'])
    if not m:
        return fail({'status': 'parse', 'detail': r['stdout'][:100]})
    return {'value': float(m.group(1)), 'status': 'ok'}


def read_ext5v(runner, is_pi5):
    if not is_pi5:
        return {'value': None, 'status': 'not_applicable', 'detail': 'Pi 5가 아님'}
    r = runner.vc('pmic_read_adc', 'EXT5V_V')
    if r['status'] != 'ok':
        return fail(r)
    m = re.search(r'EXT5V_V\s+volt\(\d+\)=([\d.]+)V', r['stdout'])
    if not m:
        return fail({'status': 'parse', 'detail': r['stdout'][:100]})
    return {'value': float(m.group(1)), 'status': 'ok'}


def find_rpi_volt_alarm():
    """Pi 5 커널의 rpi_volt hwmon 저전압 알람 파일 경로 (없으면 None)."""
    for d in sorted(glob.glob('/sys/class/hwmon/hwmon*')):
        name, _ = read_file(os.path.join(d, 'name'))
        path = os.path.join(d, 'in0_lcrit_alarm')
        if name == 'rpi_volt' and os.path.exists(path):
            return path
    return None


def read_rpi_volt_alarm(path):
    if path is None:
        return {'value': None, 'status': 'absent', 'detail': 'rpi_volt hwmon 없음'}
    raw, st = read_file(path)
    if raw is None or not raw.isdigit():
        return fail({'status': st if raw is None else 'parse', 'detail': path})
    return {'value': int(raw), 'status': 'ok'}


def scan_usb():
    """sysfs에서 USB 장치 목록을 읽는다. 장치 파일(/dev/tty*)은 열지 않는다."""
    base = '/sys/bus/usb/devices'
    try:
        names = os.listdir(base)
    except OSError as exc:
        return None, f'sysfs 읽기 실패: {exc}'
    devs = {}
    for name in sorted(names):
        if ':' in name:          # 인터페이스 항목은 건너뛴다
            continue
        path = os.path.join(base, name)
        vid, _ = read_file(os.path.join(path, 'idVendor'))
        if not vid:
            continue

        def attr(a, p=path):
            return read_file(os.path.join(p, a))[0] or ''

        info = {'path': name, 'vid': vid, 'pid': attr('idProduct'),
                'manufacturer': attr('manufacturer'), 'product': attr('product'),
                'serial': attr('serial'), 'speed_mbps': attr('speed'),
                'devnum': attr('devnum'), 'hub': attr('bDeviceClass') == '09',
                'drivers': [], 'ttys': []}
        for iface in sorted(glob.glob(os.path.join(path, name + ':*'))):
            drv = os.path.join(iface, 'driver')
            if os.path.islink(drv):
                d = os.path.basename(os.readlink(drv))
                if d not in info['drivers']:
                    info['drivers'].append(d)
            ttys = glob.glob(os.path.join(iface, 'tty', '*')) + \
                glob.glob(os.path.join(iface, 'ttyUSB*'))
            info['ttys'] += ['/dev/' + os.path.basename(t) for t in sorted(ttys)]
        devs[name] = info
    return devs, 'ok'


def usb_key(d):
    return f"{d['vid']}:{d['pid']}:{d['serial']}:{d['devnum']}"


def usb_label(d):
    text = f"{d['path']} {d['vid']}:{d['pid']} {d['manufacturer']} {d['product']}".strip()
    if d['ttys']:
        text += f" tty={','.join(d['ttys'])}"
    if d['drivers']:
        text += f" driver={','.join(d['drivers'])}"
    return text


def is_serial_dev(d):
    return bool(d['ttys']) or any(x in SERIAL_DRIVERS for x in d['drivers'])


def diff_usb(prev, cur, t):
    events = []
    for p in sorted(set(prev) | set(cur)):
        if p not in cur:
            events.append({'time': t, 'event': '제거', 'device': usb_label(prev[p])})
        elif p not in prev:
            events.append({'time': t, 'event': '추가', 'device': usb_label(cur[p])})
        elif usb_key(prev[p]) != usb_key(cur[p]):
            events.append({'time': t, 'event': '재연결(장치 번호 변경)',
                           'device': usb_label(cur[p])})
    return events


class KernelLog:
    """journalctl -k -b (안 되면 dmesg)를 읽어 검사 전 이력과 검사 중 새 기록을 나눈다."""

    def __init__(self, runner, start_uptime, start_epoch):
        self.runner = runner
        self.start_uptime = start_uptime
        self.start_epoch = start_epoch
        self.boot_epoch = start_epoch - start_uptime
        self.source = None
        self.detail = ''
        self.total_lines = 0
        self.poll_failures = 0
        self.seen = set()
        self.history = []
        self.new = []

    def _journal(self, since):
        args = ['journalctl', '-k', '-b', '--no-pager', '-o', 'short-monotonic']
        if since is not None:
            args += ['--since', f'@{int(since)}']
        r = self.runner.run(args)
        if r['status'] != 'ok':
            return None, fail(r)['detail']
        if re.search(r'insufficient permissions|not seeing messages',
                     f"{r['stdout']}\n{r['stderr']}", re.I):
            return None, '권한 부족: journalctl (adm 또는 systemd-journal 그룹 필요)'
        return [ln for ln in r['stdout'].splitlines() if not ln.startswith('-- ')], ''

    def _dmesg(self):
        r = self.runner.run(['dmesg'])
        if r['status'] != 'ok':
            return None, fail(r)['detail']
        return r['stdout'].splitlines(), ''

    def poll(self, first=False):
        """새 줄을 읽어 분류한다. 이번에 새로 찾은 검사 중 이상 기록을 반환."""
        if first:
            lines, why_j = self._journal(None)
            if lines:
                self.source = 'journalctl -k -b'
            else:
                lines, why_d = self._dmesg()
                if lines:
                    self.source = 'dmesg'
                else:
                    self.detail = (f"journalctl: {why_j or '0줄'} / "
                                   f"dmesg: {why_d or '0줄'} → 커널 로그를 읽지 못함")
                    return []
        elif self.source is None:
            return []
        elif self.source == 'dmesg':
            lines, why = self._dmesg()
        else:
            lines, why = self._journal(self.start_epoch - 2)
        if lines is None:
            self.poll_failures += 1
            self.detail = why
            return []
        found = []
        for line in lines:
            m = LOG_LINE_RE.match(line)
            if not m:
                continue
            ts = float(m.group(1))
            msg = re.sub(r'^\S+ kernel: ', '', m.group(2))
            if (ts, msg) in self.seen:
                continue
            self.seen.add((ts, msg))
            self.total_lines += 1
            kind = next((k for k, _, rx in LOG_RULES if rx.search(msg)), None)
            if kind is None:
                continue
            dev = USB_DEV_RE.search(msg)
            ev = {'mono_s': ts,
                  'time': datetime.datetime.fromtimestamp(self.boot_epoch + ts)
                  .strftime('%Y-%m-%d %H:%M:%S'),
                  'kind': kind, 'label': LOG_LABEL[kind],
                  'device': dev.group(1) if dev else '', 'message': msg[:300]}
            if ts < self.start_uptime:
                self.history.append(ev)
            else:
                self.new.append(ev)
                found.append(ev)
        return found


def last_json_line(text):
    for line in reversed((text or '').splitlines()):
        try:
            return json.loads(line)
        except ValueError:
            continue
    return None


def err_tail(r):
    """실패한 명령의 오류 문구 마지막 줄 (파이썬 traceback이면 예외 줄)."""
    lines = [ln for ln in (r.get('stderr') or r.get('detail') or '').splitlines() if ln.strip()]
    return lines[-1].strip()[:300] if lines else ''


def check_camera(runner, klog, capture_path=None):
    """카메라 상태를 한 번 확인한다. capture_path가 없으면 카메라 영상을 켜지 않는다."""
    cam = {}
    # 1) 커널이 센서를 찾았는지 (I2C 장치 + 드라이버 연결)
    try:
        sensors = []
        for d in sorted(glob.glob('/sys/bus/i2c/devices/*')):
            name, _ = read_file(os.path.join(d, 'name'))
            if name and re.search(r'imx|ov\d|camera', name, re.I):
                drv = os.path.join(d, 'driver')
                sensors.append({'i2c': os.path.basename(d), 'name': name,
                                'driver': os.path.basename(os.readlink(drv))
                                if os.path.islink(drv) else ''})
        cam['sensors'] = sensors
        cam['sensors_status'] = 'ok' if os.path.isdir('/sys/bus/i2c/devices') else 'absent'
    except OSError as exc:
        cam['sensors'], cam['sensors_status'] = [], f'error: {exc}'
    cam['video_nodes'] = []
    for d in sorted(glob.glob('/sys/class/video4linux/*')):
        name, _ = read_file(os.path.join(d, 'name'))
        cam['video_nodes'].append(f'/dev/{os.path.basename(d)} {name or ""}'.strip())
    # 2) 센서 펌웨어 파일, 부팅 설정 (읽기만)
    cam['firmware'] = {p: os.path.exists(p) for p in CAMERA_FIRMWARE}
    cam['config'] = {}
    for path in CAMERA_CONFIG_FILES:
        text, _ = read_file(path)
        if text is not None:
            cam['config'][path] = [ln.strip() for ln in text.splitlines()
                                   if re.search(r'camera_auto_detect|dtoverlay=\s*imx', ln)]
    # 3) 커널 로그의 카메라 관련 줄 (KernelLog가 이미 읽은 줄에서 찾는다)
    cam['kernel_lines'] = [f'[{ts:.3f}] {msg}' for ts, msg in sorted(klog.seen)
                           if re.search(r'imx500|rp1-cfe|rp1_cfe', msg, re.I)][-15:]
    # 4) libcamera 앱으로 카메라 목록 (영상 안 켬)
    r = runner.run(['rpicam-hello', '--list-cameras'], CAMERA_LIST_TIMEOUT)
    if r['status'] == 'missing':
        r = runner.run(['libcamera-hello', '--list-cameras'], CAMERA_LIST_TIMEOUT)
    text = f"{r.get('stdout', '')}\n{r.get('stderr', '')}"
    cam['libcamera_list'] = {'status': r['status'], 'detail': r.get('detail', ''),
                             'output': [ln for ln in text.splitlines() if ln.strip()][:20]}
    # 5) Picamera2(camera_vision 노드가 쓰는 라이브러리)로 카메라 목록 (영상 안 켬)
    r = runner.run(['python3', '-c', PICAM_LIST_CODE], CAMERA_LIST_TIMEOUT)
    info = last_json_line(r.get('stdout')) if r['status'] == 'ok' else None
    cam['picamera2'] = {'status': r['status'] if info is not None or r['status'] != 'ok'
                        else 'parse', 'cameras': info or [], 'detail': err_tail(r)}
    # 6) (선택) 사진 한 장
    cam['capture'] = None
    if capture_path:
        r = runner.run(['python3', '-c', PICAM_CAPTURE_CODE, capture_path],
                       CAMERA_CAPTURE_TIMEOUT)
        res = last_json_line(r.get('stdout')) if r['status'] == 'ok' else None
        cam['capture'] = {'status': 'ok' if res else (r['status'] if r['status'] != 'ok'
                                                      else 'parse'),
                          'path': capture_path if res else '', 'result': res,
                          'detail': err_tail(r)}
    return cam


def camera_models(cam):
    """libcamera·Picamera2 목록에서 보이는 센서 이름들 (소문자)."""
    models = set()
    for c in cam['picamera2']['cameras']:
        if c.get('Model'):
            models.add(c['Model'].lower())
    for ln in cam['libcamera_list']['output']:
        m = re.match(r'\s*\d+\s*:\s*(\w+)', ln)
        if m:
            models.add(m.group(1).lower())
    return models


def summarize_camera(cam, mode):
    """카메라 항목 판정. mode: expect(연결됨) / absent(아직 연결 안 함)."""
    out = []
    expect = mode == 'expect'
    name = '카메라 센서 인식 (커널 I2C)'
    imx = [s for s in cam['sensors'] if CAMERA_MODEL in s['name'].lower()]
    if cam['sensors_status'] != 'ok':
        out.append((name, UNK, f"I2C sysfs 읽기 불가 ({cam['sensors_status']})"))
    elif imx and all(s['driver'] for s in imx):
        out.append((name, OK if expect else BAD,
                    ', '.join(f"{s['i2c']} {s['name']} (드라이버 {s['driver']})" for s in imx)
                    + ('' if expect else ' — "미연결" 기준인데 센서가 보임')))
    elif imx:
        out.append((name, BAD, '센서 장치는 있지만 드라이버가 연결되지 않음: '
                    + ', '.join(f"{s['i2c']} {s['name']}" for s in imx)))
    elif expect:
        other = ', '.join(s['name'] for s in cam['sensors'])
        out.append((name, BAD, '커널에 IMX500 센서가 보이지 않음'
                    + (f' (다른 센서: {other})' if other else '')
                    + '. 리본 케이블 방향·끝까지 꽂혔는지·포트(CAM0/CAM1)·'
                      'config.txt 설정·전원 끈 뒤 연결했는지 확인'))
    else:
        out.append((name, OK, '미연결 기준 측정 - 카메라 센서 없음'))
    if mode == 'absent':
        return out

    name = f'카메라 모델 ({CAMERA_MODEL} 기대)'
    models = camera_models(cam)
    tools_ok = cam['picamera2']['status'] == 'ok' or cam['libcamera_list']['status'] == 'ok'
    if CAMERA_MODEL in models:
        out.append((name, OK, f"카메라 목록: {', '.join(sorted(models))}"))
    elif models:
        out.append((name, BAD, f"기대 {CAMERA_MODEL}, 실제 {', '.join(sorted(models))}"))
    elif tools_ok:
        out.append((name, BAD, '카메라 목록 도구는 실행됐지만 카메라가 0개 '
                    f"(출력: {' / '.join(cam['libcamera_list']['output'][:3]) or '-'})"))
    else:
        out.append((name, UNK, f"rpicam-hello: {cam['libcamera_list']['status']} "
                    f"{cam['libcamera_list']['detail'][:100]} / Picamera2: "
                    f"{cam['picamera2']['status']} {cam['picamera2']['detail'][-150:]}"))

    name = 'Picamera2 사용 가능 (camera_vision 노드가 쓰는 라이브러리)'
    pc = cam['picamera2']
    if pc['status'] == 'ok' and pc['cameras']:
        out.append((name, OK, f"카메라 {len(pc['cameras'])}개 열거됨"))
    elif pc['status'] == 'ok':
        out.append((name, BAD, 'Picamera2는 import 되지만 카메라 0개'))
    elif 'No module named' in pc['detail']:
        out.append((name, UNK, 'picamera2 또는 libcamera 파이썬 모듈 없음 (설치는 팀이 판단해서 직접)'))
    else:
        out.append((name, UNK, f"{STATUS_TEXT.get(pc['status'], pc['status'])}: "
                    f"{pc['detail'][-200:]}"))

    name = 'IMX500 펌웨어 파일'
    missing = [p for p, ok in cam['firmware'].items() if not ok]
    if missing:
        out.append((name, BAD, f"없음: {', '.join(missing)} (센서가 동작하려면 필요. "
                    '커널 로그에 firmware 오류가 있는지 함께 본다)'))
    else:
        out.append((name, OK, ', '.join(cam['firmware'])))

    name = '카메라 커널 로그'
    errs = [ln for ln in cam['kernel_lines'] if LOG_RULES[0][2].search(ln)]
    if errs:
        out.append((name, BAD, ' | '.join(errs[-3:])))
    elif cam['kernel_lines']:
        out.append((name, OK, f"카메라 관련 {len(cam['kernel_lines'])}줄, 오류 없음 "
                    f"(마지막: {cam['kernel_lines'][-1][:120]})"))
    else:
        out.append((name, UNK, '커널 로그에 카메라 관련 줄이 없음 (로그를 못 읽었거나 센서 미인식)'))

    cap = cam['capture']
    name = '테스트 사진 (--camera-capture)'
    if cap is None:
        out.append((name, UNK, '찍지 않음. 실제 영상이 나오는지는 --camera-capture로 확인'))
    elif cap['status'] == 'ok':
        res = cap['result']
        verdict = BAD if res['mean'] < DARK_MEAN else OK
        out.append((name, verdict, f"{cap['path']} 저장, 크기 {res['shape']}, 평균 밝기 "
                    f"{res['mean']:.1f}/255" + (' — 거의 검정 (렌즈 덮개·조명 확인)'
                                               if verdict == BAD else '. 사진을 열어 직접 본다')))
    elif 'No module named' in cap['detail'] or cap['status'] == 'missing':
        out.append((name, UNK, f"촬영 도구 없음: {cap['detail']} (카메라 고장 근거 아님)"))
    else:
        busy = ' 다른 프로그램(camera_vision 노드 등)이 카메라를 쓰는 중일 수 있음' \
            if re.search(r'busy|in use', cap['detail'], re.I) else ''
        out.append((name, BAD, f"촬영 실패 ({STATUS_TEXT.get(cap['status'], cap['status'])}): "
                    f"{cap['detail'][-200:]}{busy}"))
    return out


def print_camera(cam):
    print('-- 카메라 --')
    for s in cam['sensors']:
        print(f"  I2C 센서: {s['i2c']} {s['name']} 드라이버={s['driver'] or '없음'}")
    if not cam['sensors']:
        print('  I2C 센서: 없음')
    print(f"  video 장치: {len(cam['video_nodes'])}개")
    for path, lines in cam['config'].items():
        print(f"  {path}: {'; '.join(lines) or '카메라 관련 설정 줄 없음'}")
    lc = cam['libcamera_list']
    print(f"  카메라 목록(rpicam-hello): {' / '.join(lc['output'][:4]) or lc['detail'] or UNK}")
    pc = cam['picamera2']
    print(f"  Picamera2: {pc['cameras'] if pc['status'] == 'ok' else pc['detail'][-120:]}")


# ---------------------------------------------------------------- 판정

def bit_names(value, mask):
    return [name for bit, name in THROTTLE_BITS.items() if value & mask & (1 << bit)]


def count_kinds(events):
    counts = {}
    for e in events:
        counts[e['kind']] = counts.get(e['kind'], 0) + 1
    return counts


def repeated_resets(events):
    per_dev = {}
    for e in events:
        if e['kind'] == 'usb_reset':
            per_dev[e['device']] = per_dev.get(e['device'], 0) + 1
    return {d: n for d, n in per_dev.items() if n >= RESET_REPEAT}


def kinds_text(counts):
    return ', '.join(f'{LOG_LABEL[k]} {n}건' for k, n in counts.items()) or '없음'


def examples(events, n=3):
    return ' / 예: ' + ' | '.join(f"[{e['time']}] {e['message']}" for e in events[:n])


def judge_series(name, values, fails, last_fail, bad_cond, bad_text, ok_text):
    """값 목록 판정. 실패가 섞이면 '이상 없음'으로 처리하지 않는다."""
    if not values:
        return (name, UNK, f'측정 {fails}회 모두 실패 ({last_fail})')
    if bad_cond(values):
        return (name, BAD, bad_text(values))
    if fails:
        return (name, UNK, f'성공 {len(values)}회는 기준 안이지만 {fails}회 측정 실패 '
                f'({last_fail}) → 누락이 있어 이상 없음으로 판정하지 않음')
    return (name, OK, ok_text(values))


def summarize(ctx):
    s = ctx['samples']
    out = []

    def series(key):
        vals = [x[key]['value'] for x in s if x[key]['value'] is not None]
        bad = [x[key] for x in s if x[key]['value'] is None]
        return vals, len(bad), (bad[-1].get('detail', '') if bad else '')

    if not s:
        return [('전체', UNK, '표본이 하나도 없음 (시작 직후 중단)')]

    vals, nf, lf = series('temp')
    out.append(judge_series(
        'CPU 온도', vals, nf, lf, lambda v: max(v) >= TEMP_LIMIT_C,
        lambda v: f'최고 {max(v):.1f}°C (기준 {TEMP_LIMIT_C:g}°C 이상)',
        lambda v: f'{len(v)}회 측정, {min(v):.1f}~{max(v):.1f}°C '
                  f'(기준 {TEMP_LIMIT_C:g}°C 미만, 출처 {s[0]["temp"].get("source", "-")})'))

    vals, nf, lf = series('throttled')
    out.append(judge_series(
        'throttled 현재 상태 (비트 0~3, 검사 중)', vals, nf, lf,
        lambda v: any(x & CURRENT_MASK for x in v),
        lambda v: f'{sum(1 for x in v if x & CURRENT_MASK)}/{len(v)}회 표본에서 켜짐: '
                  + ', '.join(sorted({n for x in v for n in bit_names(x, CURRENT_MASK)})),
        lambda v: f'{len(v)}회 표본 모두 0. 표본 사이 짧은 순간은 놓칠 수 있어 이력 비트를 함께 본다'))

    if vals:
        first = vals[0]
        hb = first & HISTORY_MASK
        if hb:
            out.append(('throttled 이력 (비트 16~19, 검사 시작 전부터)', BAD,
                        f'시작 시점 값 {hex(first)}: ' + ', '.join(bit_names(first, HISTORY_MASK))
                        + '. 부팅 후 언제·무엇 때문에 생겼는지 이 값으로는 알 수 없고, '
                          '이번 아두이노 연결 탓으로 단정하지 않는다'))
        else:
            out.append(('throttled 이력 (비트 16~19, 검사 시작 전부터)', OK,
                        f'시작 시점 값 {hex(first)}: 부팅 후 펌웨어가 기록한 저전압·제한 없음. '
                        '이것만으로 손상 없음·전기적 안전을 뜻하지 않는다'))
        union = 0
        for x in vals:
            union |= x
        new_bits = union & HISTORY_MASK & ~first
        name = 'throttled 이력 (검사 중 새로 기록)'
        if len(vals) < 2:
            out.append((name, UNK, '성공한 표본이 1개뿐이라 비교 불가'))
        elif new_bits:
            out.append((name, BAD, '검사 중 새로 켜짐: ' + ', '.join(bit_names(new_bits, ~0))))
        else:
            out.append((name, OK, f'시작 {hex(first)} → 끝 {hex(vals[-1])}, 새로 켜진 이력 비트 없음 '
                        '(이력 비트는 재부팅 전까지 유지되므로 실패한 표본 사이도 포함)'))
    else:
        out.append(('throttled 이력', UNK, f'get_throttled 읽기 실패 ({lf})'))

    vals, nf, lf = series('rpi_volt_alarm')
    out.append(judge_series(
        'Pi 5 rpi_volt 저전압 알람 (hwmon)', vals, nf, lf, lambda v: any(v),
        lambda v: f'{sum(1 for x in v if x)}/{len(v)}회 표본에서 알람 1',
        lambda v: f'{len(v)}회 표본 모두 0'))

    vals, nf, lf = series('ext5v')
    note = ('참고값: Pi 5 PMIC가 읽은 Pi 외부 5 V 입력. 아두이노 쪽 USB 5 V·VIN 전압이 아니며 '
            'USB 역류 여부도 알 수 없다')
    out.append(judge_series(
        'EXT5V_V (Pi 5 PMIC, 참고)', vals, nf, lf, lambda v: min(v) < EXT5V_LOW_V,
        lambda v: f'최저 {min(v):.3f} V (참고 기준 {EXT5V_LOW_V} V 미만). {note}',
        lambda v: f'{min(v):.3f}~{max(v):.3f} V. {note}'))

    if ctx['usb_initial'] is None:
        out.append(('USB 장치 연결 변화 (검사 중)', UNK, ctx['usb_error']))
    elif ctx['usb_events']:
        ev = ctx['usb_events']
        out.append(('USB 장치 연결 변화 (검사 중)', BAD, f'{len(ev)}건: ' + ' | '.join(
            f"[{e['time']}] {e['event']} {e['device']}" for e in ev[:8])
            + ' (검사 중 손으로 꽂거나 뺐다면 그 때문일 수 있음)'))
    elif ctx['usb_failures']:
        out.append(('USB 장치 연결 변화 (검사 중)', UNK,
                    f"USB 목록 읽기 {ctx['usb_failures']}회 실패 ({ctx['usb_error']})"))
    else:
        out.append(('USB 장치 연결 변화 (검사 중)', OK,
                    f'{len(s)}회 스캔 동안 추가·제거·재연결 없음'))

    final = ctx['usb_final'] or ctx['usb_initial']
    name = 'USB 시리얼 장치 인식'
    if final is None:
        out.append((name, UNK, ctx['usb_error']))
    else:
        ser = [d for d in final.values() if is_serial_dev(d)]
        caveat = ('VID·칩 이름만으로 아두이노 모델·정품 여부·전원 회로를 확정할 수 없다. '
                  'RRC Lite·RPLIDAR도 USB 시리얼 장치다')
        listed = ' | '.join(usb_label(d) + (f" [{USB_VID_HINTS[d['vid']]}]"
                                            if d['vid'] in USB_VID_HINTS else '') for d in ser)
        if ser:
            extra = ' (기준 측정인데 USB 시리얼 장치가 있음: 무엇이 꽂혀 있는지 확인)' \
                if ctx['baseline'] else ''
            out.append((name, OK, f'{len(ser)}개: {listed}. {caveat}{extra}'))
        elif ctx['baseline']:
            out.append((name, OK, '기준 측정(아두이노 분리) - USB 시리얼 장치 없음'))
        else:
            out.append((name, BAD, 'USB 시리얼 장치가 하나도 인식되지 않음. 아두이노가 꽂혀 있다면 '
                        '케이블·포트·보드 USB 칩 쪽 문제일 수 있음 (원인은 이것만으로 모름)'))

    klog = ctx['klog']
    if klog.source is None:
        out.append(('커널 로그 이력 (부팅 후 검사 시작 전)', UNK, klog.detail))
        out.append(('커널 로그 새 기록 (검사 중)', UNK, klog.detail))
        return out
    h = klog.history
    counts = count_kinds(h)
    rr = repeated_resets(h)
    serious = [e for e in h if e['kind'] in HISTORY_SERIOUS]
    src = f'출처 {klog.source}'
    if klog.source == 'dmesg':
        src += ' (버퍼가 넘치면 오래된 기록은 빠질 수 있음)'
    name = '커널 로그 이력 (부팅 후 검사 시작 전)'
    if serious or rr:
        rr_text = f', 반복 reset 장치 {rr}' if rr else ''
        out.append((name, BAD, f'{kinds_text(counts)}{rr_text}{examples(serious or h)}. {src}. '
                    '과거 기록이며, 언제 무엇 때문에 생겼는지는 따로 확인해야 한다. '
                    '이번 아두이노 연결 탓으로 단정하지 않는다'))
    else:
        out.append((name, OK, f'저전압·과전류·인식 오류·반복 reset 없음 '
                    f'(참고: {kinds_text(counts)}). {src}'))

    name = '커널 로그 새 기록 (검사 중)'
    if klog.new:
        rr = repeated_resets(klog.new)
        rr_text = f', 반복 reset 장치 {rr}' if rr else ''
        out.append((name, BAD, f'{kinds_text(count_kinds(klog.new))}{rr_text}'
                    f'{examples(klog.new)}'))
    elif klog.poll_failures:
        out.append((name, UNK, f'로그 읽기 {klog.poll_failures}회 실패 ({klog.detail})'))
    else:
        out.append((name, OK, f'검사 중 저전압·과전류·연결 해제·reset·인식 오류 기록 없음. {src}'))
    return out


# ---------------------------------------------------------------- 출력

def fmt(m, unit, digits=1):
    if m['value'] is None:
        return UNK
    return f"{m['value']:.{digits}f}{unit}"


def print_sample(x):
    th = x['throttled']
    if th['value'] is None:
        th_text = f'throttled {UNK}'
    else:
        cur = bit_names(th['value'], CURRENT_MASK)
        th_text = f"throttled {th['raw']} (" + (','.join(cur) if cur else '현재 비트 없음') + ')'
    parts = [f"[{x['clock']}] 온도 {fmt(x['temp'], '°C')}", th_text,
             f"코어 {fmt(x['core_v'], 'V', 4)}"]
    if x['ext5v']['status'] != 'not_applicable':
        parts.append(f"EXT5V {fmt(x['ext5v'], 'V', 3)}")
    if x['rpi_volt_alarm']['status'] != 'absent':
        parts.append(f"저전압알람 {x['rpi_volt_alarm']['value']}")
    parts.append(f"USB {x['usb_count'] if x['usb_count'] is not None else UNK}개")
    print(' | '.join(parts), flush=True)
    for e in x['usb_events']:
        print(f"    ! USB {e['event']}: {e['device']}", flush=True)
    for e in x['log_events']:
        print(f"    ! 커널 로그 [{e['label']}] {e['message']}", flush=True)


def save(ctx, args, summary, static, user_info):
    os.makedirs(args.output_dir, exist_ok=True)
    stamp = ctx['start_dt'].strftime('%Y%m%d_%H%M%S')
    stem = os.path.join(args.output_dir,
                        f"pi_health_{stamp}{'_baseline' if args.baseline else ''}")
    saved = []
    if args.format in ('csv', 'both'):
        path = stem + '.csv'
        cols = ['time', 'elapsed_s', 'temp_c', 'temp_status', 'throttled_hex',
                'throttled_current', 'throttled_history', 'core_v', 'ext5v_v',
                'ext5v_status', 'rpi_volt_alarm', 'usb_count', 'usb_events', 'new_log_events']
        with open(path, 'w', newline='', encoding='utf-8-sig') as f:
            w = csv.writer(f)
            w.writerow(cols)
            for x in ctx['samples']:
                th = x['throttled']['value']
                w.writerow([
                    x['time'], x['elapsed_s'], x['temp']['value'], x['temp']['status'],
                    x['throttled'].get('raw', ''),
                    ';'.join(bit_names(th, CURRENT_MASK)) if th is not None else UNK,
                    ';'.join(bit_names(th, HISTORY_MASK)) if th is not None else UNK,
                    x['core_v']['value'], x['ext5v']['value'], x['ext5v']['status'],
                    x['rpi_volt_alarm']['value'], x['usb_count'],
                    ' | '.join(f"{e['event']} {e['device']}" for e in x['usb_events']),
                    ' | '.join(e['message'] for e in x['log_events'])])
        saved.append(path)
    if args.format in ('json', 'both'):
        path = stem + '.json'
        klog = ctx['klog']
        data = {
            'meta': {'start': ctx['start_dt'].isoformat(timespec='seconds'),
                     'end': datetime.datetime.now().isoformat(timespec='seconds'),
                     'interrupted': ctx['interrupted'], 'baseline': args.baseline,
                     'duration_s': args.duration, 'interval_s': args.interval,
                     'note': '읽기 전용 소프트웨어 관찰 결과. 하드웨어 손상·전기적 안전 판정이 아님'},
            'system': static, 'user_info': user_info,
            'usb_initial': ctx['usb_initial'], 'usb_final': ctx['usb_final'],
            'usb_events': ctx['usb_events'], 'camera': ctx.get('camera'),
            'kernel_log': {'source': klog.source, 'detail': klog.detail,
                           'lines_read': klog.total_lines,
                           'history_events': klog.history, 'new_events': klog.new},
            'samples': [{k: v for k, v in x.items() if k != 'clock'} for x in ctx['samples']],
            'summary': [{'item': a, 'verdict': b, 'evidence': c} for a, b, c in summary],
        }
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        saved.append(path)
    return saved


# ---------------------------------------------------------------- main

def parse_args():
    p = argparse.ArgumentParser(
        description='라즈베리파이 상태 진단 (읽기 전용). 시리얼 포트를 열지 않는다.')
    p.add_argument('--duration', type=float, default=120.0, help='관찰 시간 [초] (기본 120)')
    p.add_argument('--interval', type=float, default=2.0, help='표본 간격 [초] (기본 2)')
    p.add_argument('--log-interval', type=float, default=10.0,
                   help='커널 로그 다시 읽는 간격 [초] (기본 10, 부하를 낮추려고 따로 둠)')
    p.add_argument('--cmd-timeout', type=float, default=5.0, help='외부 명령 timeout [초]')
    p.add_argument('--output-dir', default='.', help='결과 저장 폴더 (기본 현재 폴더)')
    p.add_argument('--format', choices=('csv', 'json', 'both'), default='both')
    p.add_argument('--baseline', action='store_true',
                   help='아두이노를 분리한 기준 측정이라고 표시')
    p.add_argument('--camera', choices=('expect', 'absent', 'skip'), default='expect',
                   help='AI Camera 확인: expect=꽂혀 있어야 함(기본), absent=아직 안 꽂음, '
                        'skip=안 봄')
    p.add_argument('--camera-capture', action='store_true',
                   help='사진 한 장을 찍어 저장 (카메라 영상을 잠깐 켠다. 기본은 안 찍음)')
    g = p.add_argument_group('사용자 확인 정보 (모르면 비워 두면 "미확인"으로 기록)')
    g.add_argument('--arduino-model', default=UNKNOWN_INPUT, help='예: "Mega 2560 R3"')
    g.add_argument('--arduino-origin', default=UNKNOWN_INPUT,
                   help='정품 / 호환 (보드 인쇄·구매처로 확인, USB ID로 정하지 말 것)')
    g.add_argument('--pi-power', default=UNKNOWN_INPUT,
                   help='예: "USB-C 5V 5A 어댑터", "XL4015 5V → GPIO 5V 핀"')
    g.add_argument('--ground-12v', default=UNKNOWN_INPUT,
                   help='12 V 전원 마이너스가 연결된 곳. 예: "아두이노 GND 핀만"')
    g.add_argument('--extra-wiring', default=UNKNOWN_INPUT,
                   help='USB 외 추가 연결. 예: "없음", "Pi GND-아두이노 GND"')
    a = p.parse_args()
    if a.duration <= 0 or a.interval < 0.5 or a.log_interval < a.interval or a.cmd_timeout <= 0:
        p.error('duration > 0, interval >= 0.5, log-interval >= interval, cmd-timeout > 0')
    return a


def on_sigterm(signum, frame):
    raise KeyboardInterrupt


def main():
    args = parse_args()
    if not sys.platform.startswith('linux'):
        print('이 프로그램은 라즈베리파이의 Linux에서 실행한다. '
              'Windows VS Code라면 Remote SSH로 Pi에 접속해 Pi 터미널에서 실행 '
              '(docs/pi_health_check.md 2장).')
        return 2
    signal.signal(signal.SIGTERM, on_sigterm)
    runner = Runner(args.cmd_timeout)

    start_dt = datetime.datetime.now()
    start_epoch = time.time()
    static = read_static(runner)
    start_uptime = static['uptime_s'] if static['uptime_s'] is not None else 0.0
    user_info = {
        '아두이노 모델': args.arduino_model, '아두이노 정품/호환': args.arduino_origin,
        '라즈베리파이 전원 공급': args.pi_power, '12 V 마이너스 연결 위치': args.ground_12v,
        'USB 외 추가 연결(GPIO·5V·GND 등)': args.extra_wiring,
    }

    print('=== 라즈베리파이 상태 진단 (읽기 전용: 시리얼 포트·GPIO·USB 전원을 건드리지 않음) ===')
    print(f"시작 {start_dt:%Y-%m-%d %H:%M:%S} | {args.duration:g}초 동안 {args.interval:g}초 간격"
          + (' | 기준 측정(아두이노 분리)' if args.baseline else ''))
    print(f"Pi 모델: {static['pi_model']} (revision {static['pi_revision']}, 시스템에서 자동 읽음)")
    print(f"OS: {static['os']} | 커널: {static['kernel']}")
    up = static['uptime_s']
    print('부팅 후 경과: ' + (f'{up / 60:.1f}분' if up is not None else UNK))
    print('-- 사용자 확인 정보 --')
    for k, v in user_info.items():
        print(f"  {k}: {v if v != UNKNOWN_INPUT else '미확인 (직접 확인 필요)'}")

    usb_initial, usb_status = scan_usb()
    usb_error = '' if usb_initial is not None else usb_status
    print('-- 현재 USB 장치 (sysfs) --')
    if usb_initial is None:
        print(f'  {UNK}: {usb_status}')
    else:
        for d in usb_initial.values():
            if d['hub']:
                continue
            hint = USB_VID_HINTS.get(d['vid'])
            print(f'  {usb_label(d)}' + (f'  [{hint}]' if hint else ''))
        print('  (VID·칩 정보는 USB 칩 정보일 뿐, 보드 회로·정품 여부를 확정하지 않는다)')

    klog = KernelLog(runner, start_uptime, start_epoch)
    klog.poll(first=True)
    if klog.source:
        print(f'-- 커널 로그 ({klog.source}, {klog.total_lines}줄) 검사 전 이력: '
              f'{kinds_text(count_kinds(klog.history))}')
    else:
        print(f'-- 커널 로그: {UNK} ({klog.detail})')

    cam = None
    if args.camera != 'skip':
        capture_path = None
        if args.camera_capture and args.camera == 'expect':
            os.makedirs(args.output_dir, exist_ok=True)
            capture_path = os.path.join(os.path.abspath(args.output_dir),
                                        f'camera_{start_dt:%Y%m%d_%H%M%S}.jpg')
            print('-- 카메라 테스트 사진 찍는 중 (몇 초 걸림) --')
        cam = check_camera(runner, klog, capture_path)
        print_camera(cam)
    print('-- 관찰 시작 (Ctrl+C로 중단하면 그때까지 결과를 저장) --')

    rpi_volt_path = find_rpi_volt_alarm()
    ctx = {'samples': [], 'usb_events': [], 'usb_initial': usb_initial, 'usb_final': None,
           'usb_failures': 0, 'usb_error': usb_error, 'klog': klog,
           'baseline': args.baseline, 'start_dt': start_dt, 'interrupted': False}
    prev_usb = usb_initial
    t0 = time.monotonic()
    next_log = t0 + args.log_interval
    k = 0
    try:
        while True:
            t = time.monotonic()
            if t - t0 > args.duration:
                break
            clock = now_text()
            x = {'time': datetime.datetime.now().isoformat(timespec='seconds'), 'clock': clock,
                 'elapsed_s': round(t - t0, 1),
                 'temp': read_temp(runner), 'throttled': read_throttled(runner),
                 'core_v': read_core_volts(runner),
                 'ext5v': read_ext5v(runner, static['is_pi5']),
                 'rpi_volt_alarm': read_rpi_volt_alarm(rpi_volt_path),
                 'usb_events': [], 'log_events': []}
            cur, st = scan_usb()
            if cur is None:
                ctx['usb_failures'] += 1
                ctx['usb_error'] = st
                x['usb_count'] = None
            else:
                x['usb_count'] = sum(1 for d in cur.values() if not d['hub'])
                if prev_usb is not None:
                    x['usb_events'] = diff_usb(prev_usb, cur, clock)
                    ctx['usb_events'] += x['usb_events']
                prev_usb = cur
                ctx['usb_final'] = cur
            if time.monotonic() >= next_log:
                x['log_events'] = klog.poll()
                next_log += args.log_interval
            ctx['samples'].append(x)
            print_sample(x)
            k += 1
            time.sleep(max(0.0, t0 + k * args.interval - time.monotonic()))
    except KeyboardInterrupt:
        ctx['interrupted'] = True
        print('\n중단됨 (Ctrl+C). 지금까지 결과로 정리한다.')

    late = klog.poll()          # 마지막 표본 이후 기록까지 반영
    if late and ctx['samples']:
        ctx['samples'][-1]['log_events'] += late
        for e in late:
            print(f"    ! 커널 로그 [{e['label']}] {e['message']}")

    summary = summarize(ctx)
    if cam is not None:
        summary += summarize_camera(cam, args.camera)
    ctx['camera'] = cam
    print('\n=== 결과 요약 ===')
    for item, verdict, evidence in summary:
        print(f'[{verdict}] {item}\n    근거: {evidence}')
    unk_info = [k for k, v in user_info.items() if v == UNKNOWN_INPUT]
    if unk_info:
        print(f"[{UNK}] 사용자 확인 정보: {', '.join(unk_info)} → 직접 확인 필요")
    n_bad = sum(1 for _, v, _ in summary if v == BAD)
    n_unk = sum(1 for _, v, _ in summary if v == UNK)
    print(f'\n종합: 이상 관찰 {n_bad}개 항목, 확인 불가 {n_unk}개 항목'
          + (' (중단되어 관찰 시간이 짧음)' if ctx['interrupted'] else ''))
    print('주의: 소프트웨어로 볼 수 있는 범위만 관찰한 결과다. "이상 없음"은 하드웨어 손상 없음이나 '
          '전기적 안전을 뜻하지 않는다. 전원 회로·역류·접지는 멀티미터 등 실측으로 확인한다.')
    try:
        for path in save(ctx, args, summary, static, user_info):
            print(f'저장: {path}')
    except OSError as exc:
        print(f'결과 저장 실패: {exc}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
