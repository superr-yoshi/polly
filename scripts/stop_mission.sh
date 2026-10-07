#!/bin/bash
# start_mission.sh로 띄운 임무를 멈춘다 (Pi에서 실행).
# Ctrl+C와 같은 SIGINT를 보내므로 node_runner가 모터 정지 명령을 보내고 끝난다.
set -u

PIDS="$(pgrep -f "poli_navigation mission[12].launch.py")"
if [ -z "$PIDS" ]; then
  echo "실행 중인 임무가 없다."
  exit 0
fi

kill -INT $PIDS
for _ in $(seq 20); do
  pgrep -f "poli_navigation mission[12].launch.py" > /dev/null || { echo "멈춤."; exit 0; }
  sleep 0.5
done
echo "10초 안에 안 끝났다. 바퀴를 확인하고 필요하면 물리 비상정지를 누른다." >&2
exit 1
