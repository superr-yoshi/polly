#!/bin/bash
# 임무 실행 명령 (Pi에서 실행). 대회에서 노트북이 보내는 "실행 명령"은 이 한 줄이다.
#
#   노트북:  ssh poli@<파이IP> '~/polly/scripts/start_mission.sh 1'        # 임무 1
#            ssh poli@<파이IP> '~/polly/scripts/start_mission.sh 2'        # 임무 2
#   연습:    ssh poli@<파이IP> '~/polly/scripts/start_mission.sh 1 --test' # 무선을 끄지 않음
#   멈추기:  ~/polly/scripts/stop_mission.sh
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

MISSION="${1:-}"
MODE="${2:-}"
if [ "$MISSION" != "1" ] && [ "$MISSION" != "2" ]; then
  echo "사용법: $0 <1|2> [--test]" >&2
  exit 2
fi
if [ -n "$MODE" ] && [ "$MODE" != "--test" ]; then
  echo "알 수 없는 옵션: $MODE (연습은 --test)" >&2
  exit 2
fi

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(dirname "$HERE")"

if pgrep -f "poli_navigation mission[12].launch.py" > /dev/null; then
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

setsid nohup ros2 launch poli_navigation "mission${MISSION}.launch.py" \
  use_fake_hardware:=false > "$LOG" 2>&1 < /dev/null &
echo "임무 ${MISSION} 시작 (PID $!), 로그: $LOG"

if [ -z "$MODE" ]; then
  echo "2초 뒤 Wi-Fi·Bluetooth를 끈다. SSH가 끊기는 것은 정상이다."
  setsid nohup "$HERE/comm_off.sh" 2 >> "$LOG" 2>&1 < /dev/null &
else
  echo "--test: 무선을 끄지 않는다 (대회에서는 쓰지 않는다)."
fi
