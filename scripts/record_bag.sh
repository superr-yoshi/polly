#!/bin/bash
# 실기 시험 rosbag 기록 (Pi에서 실행). 노트북에서는 go.sh를 BAG=1로 실행하면 같이 켜고 끈다.
#   ~/polly/scripts/record_bag.sh start   # 백그라운드로 기록 시작 (~/poli_bags/)
#   ~/polly/scripts/record_bag.sh stop    # 기록 끝 (SIGINT로 bag 파일을 닫는다)
# 연습 분석용이다. bag 파일은 저장소에 커밋하지 않는다 (AGENTS.md 6장).
# 카메라 영상은 무거워서 빼고 탐지 결과(/vision/target)만 남긴다.
set -u

BAG_DIR=~/poli_bags
PID_FILE=/tmp/poli_bag.pid
TOPICS="/scan /odom_raw /imu/data /vision/target /cmd_vel /gripper/command /gripper/state \
/mission1/state /rosout"

case "${1:-}" in
  start)
    if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2> /dev/null; then
      echo "이미 기록 중 (PID $(cat "$PID_FILE"))"
      exit 0
    fi
    source /opt/ros/jazzy/setup.bash
    source ~/polly/ros2_ws/install/setup.bash
    mkdir -p "$BAG_DIR"
    OUT="$BAG_DIR/poli_$(date +%Y%m%d_%H%M%S)"
    # shellcheck disable=SC2086
    setsid nohup env --default-signal=INT ros2 bag record -o "$OUT" $TOPICS \
      > /tmp/poli_bag.log 2>&1 < /dev/null &
    echo $! > "$PID_FILE"
    echo "rosbag 기록 시작: $OUT"
    ;;
  stop)
    if [ ! -f "$PID_FILE" ]; then
      echo "기록 중이 아님"
      exit 0
    fi
    PID=$(cat "$PID_FILE")
    kill -INT "$PID" 2> /dev/null
    for _ in $(seq 10); do
      kill -0 "$PID" 2> /dev/null || break
      sleep 0.5
    done
    kill -0 "$PID" 2> /dev/null && kill -TERM "$PID"
    rm -f "$PID_FILE"
    echo "rosbag 기록 끝: $(ls -td "$BAG_DIR"/poli_* 2> /dev/null | head -1)"
    ;;
  *)
    echo "사용: $0 start|stop"
    exit 1
    ;;
esac
