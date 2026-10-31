#!/bin/bash
# 임무 실행 명령 (Pi에서 실행). 대회에서 노트북이 보내는 "실행 명령"은 이 한 줄이다.
#
#   노트북:  ssh poli@<파이IP> '~/polly/scripts/start_mission.sh 1'        # 임무 1
#            ssh poli@<파이IP> '~/polly/scripts/start_mission.sh 2'        # 임무 2
#   연습:    ssh poli@<파이IP> '~/polly/scripts/start_mission.sh 1 --test' # 무선을 끄지 않음
#            ... start_mission.sh 2 --test-noview   # 무선은 켜되 Foxglove·카메라 영상은 끔 (대회와 같은 부하)
#   멈추기:  ~/polly/scripts/stop_mission.sh
#
#   빠른 시작 (2026-11-01, 엔터 -> 출발 약 4 s -> 0.1 s):
#     start_mission.sh 2 prepare [--test|--test-noview]   # 시작 선언 전: 모든 노드를 켜고 대기
#     start_mission.sh go                                  # 시작 신호만 (prepare 때 고른 모드를 따른다)
#   prepare 상태의 임무 노드는 시작 신호 전에 /cmd_vel·/gripper/command를 내지 않는다 (규정 3.5.6).
#   무선 차단(대회 모드)은 go 2초 뒤에 한다.
#
# 하는 일
# 1. 실제 하드웨어로 mission<N>.launch.py를 SSH와 분리해서 띄운다 (SSH가 끊겨도 계속 실행).
#    로그: ~/poli_logs/mission<N>_<시각>.log
# 2. --test가 없으면 2초 뒤 Wi-Fi·Bluetooth를 끈다 (규정 1.3, 3.5.3, 문의 1번).
#    → SSH가 끊긴다. 되돌리기: 로봇에서 scripts/comm_off.sh --restore (모니터·키보드)
#
# 근거: 규정 1.6 (임무별 독립 프로그램, 실행 명령만 무선 전달), 3.5.4 (대회 중 코드·매개변수 수정 금지)
#       → 임무 전환은 코드를 고치지 않고 이 명령의 숫자만 바꾼다.
# 준비: comm_off.sh 맨 위의 rfkill 비밀번호 없이 쓰기 설정 (Pi에서 1회).
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
START_FILE=/tmp/poli_mission_start
PREPARED_MODE_FILE=/tmp/poli_mission_prepared

# 시작 신호: 가장 빨리 끝나야 하므로 ROS 환경을 불러오지 않는다.
if [ "${1:-}" = "go" ]; then
  if [ ! -f "$PREPARED_MODE_FILE" ] || ! pgrep -f "[p]oli_navigation mission[12].launch.py" > /dev/null; then
    echo "준비된 임무가 없다. 먼저 $0 <1|2> prepare" >&2
    exit 1
  fi
  touch "$START_FILE"
  echo "시작 신호 보냄"
  if [ -z "$(cat "$PREPARED_MODE_FILE")" ]; then
    echo "2초 뒤 Wi-Fi·Bluetooth를 끈다. SSH가 끊기는 것은 정상이다."
    LOG="$(ls -t "$HOME"/poli_logs/mission*_*.log | head -1)"
    setsid nohup "$HERE/comm_off.sh" 2 >> "$LOG" 2>&1 < /dev/null &
  fi
  exit 0
fi

MISSION="${1:-}"
PREPARE=false
if [ "${2:-}" = "prepare" ]; then
  PREPARE=true
  shift
fi
MODE="${2:-}"
if [ "$MISSION" != "1" ] && [ "$MISSION" != "2" ]; then
  echo "사용법: $0 <1|2> [prepare] [--test|--test-noview]  /  $0 go" >&2
  exit 2
fi
if [ -n "$MODE" ] && [ "$MODE" != "--test" ] && [ "$MODE" != "--test-noview" ]; then
  echo "알 수 없는 옵션: $MODE (연습은 --test 또는 --test-noview)" >&2
  exit 2
fi

REPO="$(dirname "$HERE")"

if pgrep -f "[p]oli_navigation mission[12].launch.py" > /dev/null; then
  echo "이미 임무가 실행 중이다. 먼저 $HERE/stop_mission.sh" >&2
  exit 1
fi

# ROS 환경 (setup.bash는 set -u에서 정의 안 된 변수를 읽으므로 잠시 끈다)
set +u
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/lidar_ws/install/setup.bash" ] && source "$HOME/lidar_ws/install/setup.bash"
source "$REPO/ros2_ws/install/setup.bash"
set -u

if [ -z "$MODE" ] && ! sudo -n /usr/sbin/rfkill list > /dev/null 2>&1; then
  echo "rfkill을 비밀번호 없이 쓸 수 없다. comm_off.sh 맨 위 '준비'를 먼저 한다." >&2
  echo "(무선 차단 없이 연습하려면 --test)" >&2
  exit 1
fi

LOG_DIR="$HOME/poli_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/mission${MISSION}_$(date +%Y%m%d_%H%M%S).log"

# 백그라운드(&)로 띄우면 bash가 SIGINT를 무시 상태로 넘겨서 stop_mission.sh(Ctrl+C와 같은 SIGINT)로
# 멈추지 않는다 (2026-10-07 Pi에서 확인). env --default-signal로 SIGINT를 되살린다 (coreutils 8.31+).
# --test(연습)에서는 노트북 Foxglove 확인용 브리지·카메라 영상도 켠다 (대회에서는 끔, 규정 1.3).
# 연습 경기장은 장애물이 점대칭이 아니라서 임무 1의 점대칭 지도도 끈다 (대회 실행은 켬).
DEBUG_VIEW=false
EXTRA=""
if [ "$MODE" = "--test" ]; then
  DEBUG_VIEW=true
  [ "$MISSION" = "1" ] && EXTRA="point_symmetry:=false rolling_decision:=true"
fi
# --test-noview: 연습이지만 Foxglove·카메라 영상을 켜지 않는다 (대회와 같은 부하로 시간 재기).
# 2026-11-01: 시작 2~3초 뒤 Pi 전체가 0.3 s 멈춤 -> Foxglove 재연결 때문인지 확인용
if [ "$MODE" = "--test-noview" ]; then
  [ "$MISSION" = "1" ] && EXTRA="point_symmetry:=false rolling_decision:=true"
fi
rm -f "$START_FILE" "$PREPARED_MODE_FILE"
setsid nohup env --default-signal=INT ros2 launch poli_navigation "mission${MISSION}.launch.py" \
  use_fake_hardware:=false debug_view:=$DEBUG_VIEW wait_for_start:=$PREPARE $EXTRA \
  > "$LOG" 2>&1 < /dev/null &
if [ "$PREPARE" = true ]; then
  echo "$MODE" > "$PREPARED_MODE_FILE"
  echo "임무 ${MISSION} 준비 (PID $!), 로그: $LOG"
  echo "로그에 'Ready'가 나오면 시작 가능: $0 go"
  exit 0
fi
echo "임무 ${MISSION} 시작 (PID $!), 로그: $LOG"
[ "$DEBUG_VIEW" = true ] && echo "Foxglove: ws://<파이IP>:8765 (/scan, /vision/debug_image/compressed)"

if [ -z "$MODE" ]; then
  echo "2초 뒤 Wi-Fi·Bluetooth를 끈다. SSH가 끊기는 것은 정상이다."
  setsid nohup "$HERE/comm_off.sh" 2 >> "$LOG" 2>&1 < /dev/null &
else
  echo "--test: 무선을 끄지 않는다 (대회에서는 쓰지 않는다)."
fi
