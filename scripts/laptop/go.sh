#!/bin/bash
# 노트북에서 임무 준비 -> 엔터 = 바로 시작 -> 진행 보기. Ctrl+C = 즉시 정지 (Pi의 stop_mission.sh).
#
#   bash scripts/laptop/go.sh            # 임무 2, 연습 (무선 유지, Foxglove 끔 = 대회와 같은 부하)
#   bash scripts/laptop/go.sh 1          # 임무 1
#   bash scripts/laptop/go.sh 2 --test   # Foxglove·카메라 영상까지 켬
#   PI=10.175.8.69 bash scripts/laptop/go.sh   # Pi IP를 직접 줄 때
#   BAG=1 bash scripts/laptop/go.sh 1    # rosbag도 기록 (Pi ~/poli_bags/, scripts/record_bag.sh)
#   NO_UPDATE=1 bash scripts/laptop/go.sh 2   # Pi 코드 업데이트를 묻지 않음 (옛 커밋으로 비교 시험할 때)
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
  [ -n "${WARM:-}" ] && kill "$WARM" 2> /dev/null
  echo
  echo "정지 중..."
  pi '~/polly/scripts/stop_mission.sh | tail -1; pkill -f "^tail -n \\+1 -F /home/poli/poli_logs/"'
  [ "${BAG:-0}" = 1 ] && pi '~/polly/scripts/record_bag.sh stop'
  exit 0
}
trap stop_now INT TERM

pi '~/polly/scripts/stop_mission.sh > /dev/null 2>&1'
# Pi의 install은 복사본이라 git pull만으로는 실행 코드가 안 바뀐다 (2026-11-01: 옛 코드로 두 번 시험).
# 노트북에 push된 최신 커밋과 Pi 빌드가 다르면 알려 주고, y면 받아서 빌드한다.
# NO_UPDATE=1이면 건너뛴다 (일부러 옛 커밋으로 시험할 때, 2026-11-03)
[ "${NO_UPDATE:-0}" = 1 ] && echo "NO_UPDATE=1: Pi 코드 업데이트 확인을 건너뛴다 ($(pi 'cd ~/polly && git log --oneline -1'))"
[ "${NO_UPDATE:-0}" = 1 ] || STALE=$(pi 'cd ~/polly && git fetch -q origin 2>/dev/null; H=$(git rev-parse --short HEAD); \
  for f in $(cd ros2_ws/src && ls */*/*.py | grep -E "^poli_(navigation|hardware)/poli_"); do \
    cmp -s ros2_ws/src/$f ros2_ws/install/${f%%/*}/lib/python3.12/site-packages/${f#*/} || { echo "빌드 안 됨: $f"; break; }; done; \
  [ "$H" = "$(git rev-parse --short origin/integrate/mdd10-control)" ] || echo "Pi 저장소 $H, 원격 최신 $(git rev-parse --short origin/integrate/mdd10-control)"')
if [ -n "$STALE" ]; then
  echo "⚠️  Pi 코드가 최신이 아니다: $STALE"
  read -r -p "지금 받아서 빌드하려면 y (약 10초), 그냥 옛 코드로 하려면 엔터: " ANSWER
  if [[ "$ANSWER" == [yYㅛ] ]]; then
    pi 'cd ~/polly && git pull -q --ff-only origin integrate/mdd10-control && git log --oneline -1 && \
        cd ros2_ws && source /opt/ros/jazzy/setup.bash && \
        colcon build --packages-select poli_navigation poli_hardware 2>&1 | tail -1' || stop_now
  fi
fi
# 따로 켜 둔 LiDAR·Foxglove(view_lidar.sh 등)가 있으면 임무 LiDAR가 장치를 못 잡는다 (2026-11-01: health 80008002)
pi 'pkill -INT -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 (launch sllidar|run foxglove_bridge)" && sleep 2
    pgrep -x sllidar_node | xargs -r kill -9; pgrep -x foxglove_bridge | xargs -r kill -9; true'
[ "${BAG:-0}" = 1 ] && pi '~/polly/scripts/record_bag.sh start'
pi "~/polly/scripts/start_mission.sh $MISSION prepare $MODE" | grep -E "준비|오류|이미|rfkill" || stop_now
echo "준비 중 (노드 켜는 중)..."
if ! pi 'f=$(ls -t ~/poli_logs/mission*_*.log | head -1)
         timeout 30 tail -n +1 -F "$f" | grep -m1 -q "Ready"'; then
  echo "30초 안에 준비가 안 됐다 (카메라·Mega·LiDAR 확인). 로그: Pi ~/poli_logs/"
  stop_now
fi
# LiDAR가 죽은 채로도 임무는 Ready가 된다 (2026-11-01: USB가 빠져 18:58·19:30 실행이 LiDAR 없이 돎)
if ! pi 'f=$(ls -t ~/poli_logs/mission*_*.log | head -1)
         grep -q "SLLidar health status : OK" "$f" && ! grep -q "sllidar_node-[0-9]*\]: process has died" "$f"'; then
  echo "⚠️  LiDAR가 안 켜졌다 (USB·/dev/robot_lidar 확인). 이대로면 위치 보정·상대 감지·탈락선 판단이 안 된다."
  read -r -p "그래도 시작하려면 y, 멈추려면 엔터: " ANSWER
  [[ "$ANSWER" == [yYㅛ] ]] || stop_now
fi
# 엔터를 기다리는 동안 1초마다 연결을 확인해, 끊겼으면 미리 다시 열어 둔다 (엔터 뒤에 다시 연결하면 수 초 늦음)
( while true; do PI_TIMEOUT=3 pi true > /dev/null 2>&1 || echo "⚠️  Pi 연결 끊김 - 다시 여는 중 (핫스팟을 로봇 가까이)"; sleep 1; done ) &
WARM=$!
read -r -p "준비 완료. 엔터 = 시작 (Ctrl+C = 취소) "
kill "$WARM" 2> /dev/null
T0=$(date +%s%N)
pi '~/polly/scripts/start_mission.sh go' | grep -v "^$"
GO_MS=$(( ($(date +%s%N) - T0) / 1000000 ))
echo "시작 신호까지 ${GO_MS} ms"
[ "$GO_MS" -gt 500 ] && echo "⚠️  시작 신호가 ${GO_MS} ms 걸렸다 (Wi-Fi가 약하거나 끊겼다 다시 연결됨)"
echo "진행 보기 (Ctrl+C = 정지)"
pi 'f=$(ls -t ~/poli_logs/mission*_*.log | head -1); tail -n +1 -F "$f"' \
  | grep --line-buffered -E "Start signal|State:|Phase:|Gripper:|fault|ERROR|지연|timeout|출발 자세|막힘 표시|막다른|감시|칸 한 번에" \
  | sed -u -E 's/^\[[a-z_0-9-]+\] \[[A-Z]+\] \[[0-9.]+\] //'
# Ctrl+C로 위 보기가 끝나면(또는 연결이 끊기면) 임무를 멈춘다
stop_now
