#!/bin/bash
# 노트북에서 임무 준비 -> 엔터 = 바로 시작 -> 진행 보기. Ctrl+C = 즉시 정지 (Pi의 stop_mission.sh).
#
#   bash scripts/laptop/go.sh            # 임무 2, 연습 (무선 유지, Foxglove 끔 = 대회와 같은 부하)
#   bash scripts/laptop/go.sh 1          # 임무 1
#   bash scripts/laptop/go.sh 2 --test   # Foxglove·카메라 영상까지 켬
#   PI=10.175.8.69 bash scripts/laptop/go.sh   # Pi IP를 직접 줄 때
#
# 1) 이전 임무를 끄고 모든 노드를 미리 켠다 (start_mission.sh prepare, 시작 신호 전에는 바퀴·집게 명령 없음)
# 2) 'Ready'가 나오면 엔터를 기다린다. 엔터 -> 시작 신호(start_mission.sh go)까지 약 0.1 s
#    (SSH 연결을 미리 열어 두고, 노드는 이미 켜져 있다. 2026-11-01: 예전에는 약 4 s)
# 집게 열기는 scripts/laptop/grip_open.sh. 진짜 급할 때는 물리 비상정지가 가장 빠르다.
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
trap stop_now INT TERM

pi '~/polly/scripts/stop_mission.sh > /dev/null 2>&1'
pi "~/polly/scripts/start_mission.sh $MISSION prepare $MODE" | grep -E "준비|오류|이미|rfkill" || stop_now
echo "준비 중 (노드 켜는 중)..."
if ! pi 'f=$(ls -t ~/poli_logs/mission*_*.log | head -1)
         timeout 30 tail -n +1 -F "$f" | grep -m1 -q "Ready"'; then
  echo "30초 안에 준비가 안 됐다 (카메라·Mega·LiDAR 확인). 로그: Pi ~/poli_logs/"
  stop_now
fi
read -r -p "준비 완료. 엔터 = 시작 (Ctrl+C = 취소) "
pi '~/polly/scripts/start_mission.sh go' | grep -v "^$"
echo "진행 보기 (Ctrl+C = 정지)"
pi 'f=$(ls -t ~/poli_logs/mission*_*.log | head -1); tail -n +1 -F "$f"' \
  | grep --line-buffered -E "Start signal|State:|Phase:|Gripper:|fault|ERROR|지연|timeout" \
  | sed -u -E 's/^\[[a-z_0-9-]+\] \[[A-Z]+\] \[[0-9.]+\] //'
# Ctrl+C로 위 보기가 끝나면(또는 연결이 끊기면) 임무를 멈춘다
stop_now
