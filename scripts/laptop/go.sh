#!/bin/bash
# 노트북에서 임무 바로 시작 + 진행 보기. Ctrl+C = 즉시 정지 (Pi의 stop_mission.sh).
#
#   bash scripts/laptop/go.sh            # 임무 2, 연습 (무선 유지, Foxglove 끔 = 대회와 같은 부하)
#   bash scripts/laptop/go.sh 1          # 임무 1
#   bash scripts/laptop/go.sh 2 --test   # Foxglove·카메라 영상까지 켬
#   PI=10.175.8.69 bash scripts/laptop/go.sh   # Pi IP를 직접 줄 때
#
# 시작 전에 이전 임무가 남아 있으면 먼저 끈다. 집게 열기는 scripts/laptop/grip_open.sh.
# 진짜 급할 때는 물리 비상정지가 가장 빠르다.
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/_common.sh"
MISSION=${1:-2}
MODE=${2:---test-noview}
find_pi "${PI:-}"

stop_now() {
  trap - INT TERM
  echo
  echo "정지 중..."
  pi '~/polly/scripts/stop_mission.sh | tail -1; pkill -f "^tail -n \\+1 -F /home/poli/poli_logs/"'
  exit 0
}

pi '~/polly/scripts/stop_mission.sh > /dev/null 2>&1'
pi "~/polly/scripts/start_mission.sh $MISSION $MODE" | grep -E "시작|오류|이미" || exit 1
trap stop_now INT TERM
echo "진행 보기 (Ctrl+C = 정지)"
# 로그 파일이 생길 때까지 잠깐 기다린 뒤 상태 줄만 보여 준다
pi 'sleep 0.5; f=$(ls -t ~/poli_logs/mission*_*.log | head -1); tail -n +1 -F "$f"' \
  | grep --line-buffered -E "State:|Phase:|Gripper:|fault|ERROR|지연|timeout|Mega connected" \
  | sed -u -E 's/^\[[a-z_0-9-]+\] \[[A-Z]+\] \[[0-9.]+\] //'
# Ctrl+C로 위 보기가 끝나면(또는 연결이 끊기면) 임무를 멈춘다
stop_now
