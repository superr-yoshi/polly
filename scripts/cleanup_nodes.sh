#!/bin/bash
# 남아 있는 로봇 ROS 노드를 정리한다 (Pi에서 실행). start_mission.sh·stop_mission.sh가 부른다.
#
# 2026-11-03 20:20 실기: 이전 실행의 하드웨어 노드가 launch 없이 남아 Mega 포트를 같이 열었다
# ('multiple access on port' 오류, 연결이 계속 끊김). /odom_raw도 둘이 내보내 위치가 1~2 m씩 튀었고,
# 경기장 밖으로 계산된 위치로 RETREAT가 엉뚱한 쪽으로 후진해 선 밖으로 나갔다.
# start_mission.sh는 launch 프로세스만 확인해서 launch가 죽고 노드만 남은 것을 몰랐다.
#
# 우리 노드: ros2_ws·lidar_ws install 안의 실행 파일, robot_state_publisher, foxglove_bridge (--ros-args로 뜬 것),
# 그리고 Mega 포트를 여는 시험 스크립트. [r]처럼 쓴 것은 pgrep이 자기 셸을 찾지 않게 하려는 것이다.
set -u
PATTERN='(/ros2_ws/install/|/lidar_ws/install/|[r]obot_state_publisher|[f]oxglove_bridge).*--ros-args|[p]c_serial_tester'

PIDS="$(pgrep -f "$PATTERN")"
[ -z "$PIDS" ] && exit 0
echo "남아 있는 노드 정리: $(ps -o comm= -p $(echo $PIDS) | sort | uniq -c | awk '{printf "%s(%s) ", $2, $1}')"
kill -INT $PIDS 2> /dev/null
for _ in $(seq 10); do
  pgrep -f "$PATTERN" > /dev/null || exit 0
  sleep 0.3
done
kill -KILL $(pgrep -f "$PATTERN") 2> /dev/null
sleep 0.3
pgrep -f "$PATTERN" > /dev/null && { echo "노드가 아직 남아 있다. Pi를 재부팅한다." >&2; exit 1; }
exit 0
