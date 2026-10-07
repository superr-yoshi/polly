#!/bin/bash
# start_mission.sh로 띄운 임무를 멈춘다 (Pi에서 실행).
# 먼저 Ctrl+C와 같은 SIGINT를 보낸다 (launch가 노드에 SIGINT -> node_runner가 모터 정지 명령 후 종료).
# 10초 안에 안 끝나면 SIGTERM을 보낸다 (node_runner는 SIGTERM에도 모터를 멈춘다).
# [p]처럼 쓴 것은 pgrep이 이 명령을 실행한 셸 자신을 찾지 않게 하려는 것이다.
set -u

PATTERN="[p]oli_navigation mission[12].launch.py"

wait_exit() {
  for _ in $(seq 20); do
    pgrep -f "$PATTERN" > /dev/null || return 0
    sleep 0.5
  done
  return 1
}

PIDS="$(pgrep -f "$PATTERN")"
if [ -z "$PIDS" ]; then
  echo "실행 중인 임무가 없다."
  exit 0
fi

kill -INT $PIDS
if wait_exit; then
  echo "멈춤."
  exit 0
fi

echo "SIGINT로 안 멈춰서 SIGTERM을 보낸다." >&2
kill -TERM $(pgrep -f "$PATTERN") 2> /dev/null
if wait_exit; then
  echo "멈춤 (SIGTERM)."
  exit 0
fi
echo "아직 안 멈췄다. 바퀴를 확인하고 필요하면 물리 비상정지를 누른다." >&2
exit 1
