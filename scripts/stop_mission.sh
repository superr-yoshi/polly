#!/bin/bash
# start_mission.sh로 띄운 임무를 멈춘다 (Pi에서 실행). 노트북에서는 scripts/laptop/stop.sh.
# 먼저 임무 노드를 바로 끝내 바퀴를 세우고 (아래 1), 나머지 노드를 정리한다 (아래 2).
# 2)에서는 Ctrl+C와 같은 SIGINT를 보낸다 (launch가 노드에 SIGINT -> node_runner가 모터 정지 명령 후 종료).
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

# 1) 먼저 임무 노드만 바로 끝낸다 (kill -9, 즉시 정지). 바퀴 명령이 끊기면 하드웨어 노드의
#    /cmd_vel 0.3초 timeout과 Mega 펌웨어 0.3초 timeout이 바퀴를 세운다 (하드웨어 노드는 살려 둔다).
#    2026-10-31: 임무 노드가 SIGINT에 끝나지 않아 정지 명령 뒤에도 로봇이 계속 돌았다.
# 실제 노드 프로세스(/usr/bin/python3 .../lib/poli_navigation/missionN)만 잡는다.
# (이 문자열을 담은 다른 셸 명령까지 잡지 않게 맨 앞을 고정한다. 2026-10-31)
NODE_PIDS="$(pgrep -f "^/usr/bin/python3 [^ ]*/lib/poli_navigation/mission[12]( |$)")"
if [ -n "$NODE_PIDS" ]; then
  kill -KILL $NODE_PIDS
  echo "임무 노드 즉시 정지 (바퀴는 0.3초 안에 선다)."
fi

# 2) 나머지(하드웨어·LiDAR·카메라 노드)는 SIGINT로 정리한다.
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
