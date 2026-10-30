#!/bin/bash
# 노트북에서 "앞으로 가기" 시험: RViz로 LiDAR·카메라를 보면서 로봇을 바닥에서 약 0.5 m 전진시키고,
# 실제로 간 거리를 LiDAR로 재서 알려 준다 (Pi에서 scripts/forward_test.py 실행).
# 사용: bash scripts/laptop/forward_test.sh [PiIP]
# 필요: Pi에 LiDAR, 카메라(CAM/DISP 1), Mega(USB, MDD10 주행) + 배터리. 로봇은 바닥에 둔다.
# ⚠️ 앞쪽 1 m 이상을 비우고, 1~3 m 앞에 벽이나 상자를 둔다.
#    비상정지 스위치와 Mega USB 케이블을 바로 누르거나 뽑을 수 있게 한다.
#    (조원 A 2026-10-08: 폭주 시 배터리 스위치를 꺼도 USB가 꽂혀 있으면 모터가 돈다)
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/_common.sh"
find_pi "$1"

pi 'test -e /dev/robot_mega' || {
  echo "Pi에 Mega가 안 보인다 (/dev/robot_mega 없음). Mega USB를 Pi에 꽂는다."; exit 1; }
pi 'test -e /dev/robot_lidar' || {
  echo "Pi에 LiDAR가 안 보인다 (/dev/robot_lidar 없음). LiDAR USB를 Pi에 꽂는다."; exit 1; }

# 프로세스 찾기 패턴은 ^로 시작을 고정한다. 안 그러면 pi '...' 원격 셸의 명령 줄에도
# 같은 글자가 들어 있어 셸 자신이 걸린다 (pkill이 셸을 끄면 뒤 명령이 안 돈다).
stop_all() {
  echo "Pi의 바퀴·LiDAR·카메라·TF 노드를 끈다."
  pi 'pkill -INT -f "^/usr/bin/python3 [^ ]*/lib/poli_hardware/mega_bridge_node";
      pkill -INT -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch sllidar";
      pkill -INT -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch poli_description";
      pkill -INT -f "^python3 /home/poli/polly/scripts/camera_stream_test.py"; sleep 3;
      pgrep -x sllidar_node | xargs -r kill;
      pkill -f "^/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher"; echo "  모두 꺼짐"'
}

echo "Pi 코드 업데이트·빌드 (처음엔 1~2분)..."
pi 'cd ~/polly && { git pull -q || echo "  git pull 실패 (인터넷 확인) -> 지금 있는 코드로 한다"; }
    test -f scripts/forward_test.py || {
      echo "  Pi에 scripts/forward_test.py가 없다. 노트북에서 커밋·push 했는지 확인"; exit 1; }
    source /opt/ros/jazzy/setup.bash && cd ros2_ws &&
    colcon build --symlink-install --packages-select poli_description poli_hardware \
      > /tmp/forward_build.log 2>&1 &&
    echo "  빌드 OK" || { echo "  빌드 실패"; tail -5 /tmp/forward_build.log; exit 1; }' || exit 1

echo "Pi에서 켜는 중..."
pi 'source /opt/ros/jazzy/setup.bash; source ~/lidar_ws/install/setup.bash
    source ~/polly/ros2_ws/install/setup.bash
    setsid nohup ros2 launch poli_description description.launch.py \
      > /tmp/forward_tf.log 2>&1 < /dev/null &
    setsid nohup ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/robot_lidar \
      > /tmp/forward_lidar.log 2>&1 < /dev/null &
    setsid nohup python3 ~/polly/scripts/camera_stream_test.py > /tmp/forward_camera.log 2>&1 < /dev/null &
    P=$(ros2 pkg prefix poli_hardware)/share/poli_hardware/config/hardware.yaml
    setsid nohup ros2 run poli_hardware mega_bridge_node --ros-args --params-file $P \
      > /tmp/forward_drive.log 2>&1 < /dev/null &
    sleep 7
    ok=yes
    grep -q "health status : OK" /tmp/forward_lidar.log && echo "  LiDAR: OK" \
      || { echo "  LiDAR: 응답 없음 -> LiDAR USB를 뺐다 다시 꽂고 다시 실행"; ok=no; }
    if grep -q "timed out" /tmp/forward_camera.log; then
      echo "  카메라: 영상이 안 들어온다 -> 전원을 끄고 리본 케이블을 다시 꽂기 (주행 시험은 계속)"
    elif pgrep -f "^python3 /home/poli/polly/scripts/camera_stream_test.py" > /dev/null; then
      echo "  카메라: OK"
    else
      echo "  카메라: 실패 (주행 시험은 계속)"; tail -3 /tmp/forward_camera.log
    fi
    pgrep -f "^/usr/bin/python3 [^ ]*/lib/poli_hardware/mega_bridge_node" > /dev/null \
      && echo "  바퀴 노드: OK" || { echo "  바퀴 노드: 실패"; tail -3 /tmp/forward_drive.log; ok=no; }
    pgrep -f "^/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher" > /dev/null \
      && echo "  TF(URDF): OK" || { echo "  TF(URDF): 실패"; tail -3 /tmp/forward_tf.log; ok=no; }
    [ $ok = yes ]' || { echo "필요한 노드가 안 켜졌다. 위 메시지를 보고 고친 뒤 다시 실행한다."; stop_all; exit 1; }

echo "RViz를 연다 (가운데 빨간 점 = LiDAR, 왼쪽 아래 = 카메라)."
echo "앞으로 가는 동안 앞쪽 벽(빨간 점)이 로봇 쪽으로 다가와야 정상이다."
rviz2 -d "$HERE/poli_view.rviz" > /dev/null 2>&1 &
RVIZ=$!
trap 'stop_all; kill $RVIZ 2>/dev/null; exit 1' INT

while true; do
  echo
  echo "⚠️  앞으로 가기 전 확인:"
  echo "   1. 로봇이 바닥에 있고, 앞쪽 1 m 이상이 비어 있다"
  echo "   2. 1~3 m 앞에 벽이나 상자가 있다 (LiDAR로 거리 변화를 잰다)"
  echo "   3. 비상정지 스위치와 Mega USB 케이블을 바로 누르거나 뽑을 수 있다"
  echo "   4. 로봇 앞과 바퀴 근처에 손·발·케이블이 없다"
  read -r -p "y를 누르면 약 0.5 m 앞으로 간다 (끝내려면 Enter): " ok
  [ "$ok" = "y" ] || break
  pi -t 'source /opt/ros/jazzy/setup.bash; python3 ~/polly/scripts/forward_test.py'
done

echo
echo "다 보면 RViz 창을 닫는다 (저장하지 않고 닫기)."
wait $RVIZ
stop_all
