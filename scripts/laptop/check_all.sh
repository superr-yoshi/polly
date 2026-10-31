#!/bin/bash
# 노트북에서 한 번에 확인: LiDAR + 카메라(RViz 창 하나) + 바퀴(정해진 짧은 동작).
# 사용: bash scripts/laptop/check_all.sh [PiIP]
# 필요: Pi에 LiDAR, 카메라(CAM/DISP 1), RRC Lite(USB) + 배터리. RRC가 없으면 바퀴는 건너뛴다.
# ⚠️ 바퀴를 바닥에서 띄우고, RRC Lite USB 케이블을 바로 뽑을 수 있게 한다.
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/_common.sh"
find_pi "$1"

stop_all() {
  echo "Pi의 LiDAR·카메라·바퀴 노드를 끈다."
  pi 'pkill -INT -f "lib/poli_hardware/rrc_adapter_node";
      pkill -INT -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch sllidar";
      pkill -INT -f "^python3 /home/poli/polly/scripts/camera_stream_test.py"; sleep 3;
      pgrep -x sllidar_node | xargs -r kill; echo "  모두 꺼짐"'
}

WHEELS=yes
pi 'test -e /dev/robot_rrc' || { WHEELS=no; echo "RRC Lite가 안 보인다 -> 바퀴 시험은 건너뛴다 (LiDAR·카메라만)"; }

echo "Pi 코드 업데이트·빌드 (처음엔 1~2분)..."
pi 'cd ~/polly && git pull -q && source /opt/ros/jazzy/setup.bash && cd ros2_ws &&
    colcon build --symlink-install --packages-select poli_description poli_hardware > /tmp/check_build.log 2>&1 &&
    echo "  빌드 OK" || { echo "  빌드 실패"; tail -5 /tmp/check_build.log; exit 1; }' || exit 1

echo "Pi에서 켜는 중..."
pi "source /opt/ros/jazzy/setup.bash; source ~/lidar_ws/install/setup.bash; source ~/polly/ros2_ws/install/setup.bash
    setsid nohup ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/robot_lidar \
      > /tmp/check_lidar.log 2>&1 < /dev/null &
    setsid nohup python3 ~/polly/scripts/camera_stream_test.py > /tmp/check_camera.log 2>&1 < /dev/null &
    if [ $WHEELS = yes ]; then
      P=\$(ros2 pkg prefix poli_hardware)/share/poli_hardware/config/hardware.yaml
      setsid nohup ros2 run poli_hardware rrc_adapter_node --ros-args --params-file \$P \
        > /tmp/check_wheel.log 2>&1 < /dev/null &
    fi
    sleep 6
    grep -q 'health status : OK' /tmp/check_lidar.log && echo '  LiDAR: OK' \
      || echo '  LiDAR: 응답 없음 -> LiDAR USB를 뺐다 다시 꽂기'
    pgrep -f camera_stream_test.py > /dev/null && echo '  카메라: OK' \
      || { echo '  카메라: 실패'; tail -3 /tmp/check_camera.log; }
    if [ $WHEELS = yes ]; then
      pgrep -f rrc_adapter_node > /dev/null && echo '  바퀴 노드: OK' \
        || { echo '  바퀴 노드: 실패'; tail -3 /tmp/check_wheel.log; }
    fi"

echo "RViz를 연다 (가운데 빨간 점 = LiDAR, 왼쪽 아래 = 카메라)."
rviz2 -d "$HERE/poli_view.rviz" > /dev/null 2>&1 &
RVIZ=$!
trap 'stop_all; kill $RVIZ 2>/dev/null; exit 1' INT

if [ $WHEELS = yes ]; then
  echo
  echo "⚠️  바퀴 시험 전 확인:"
  echo "   1. 로봇을 받침 위에 올려 바퀴가 바닥에 닿지 않는다"
  echo "   2. RRC Lite USB 케이블을 손으로 바로 뽑을 수 있다 (비상정지)"
  echo "   3. 바퀴 근처에 손·케이블이 없다"
  read -r -p "확인했으면 y (바퀴 건너뛰기는 Enter): " ok
  if [ "$ok" = "y" ]; then
    echo "바퀴 동작 (앞 -> 뒤 -> 왼쪽 -> 오른쪽, 멈추려면 Ctrl+C):"
    pi -t 'source /opt/ros/jazzy/setup.bash; python3 ~/polly/scripts/wheel_test.py'
    echo "확인: 앞으로 때 두 바퀴가 같은 방향? 왼쪽 회전 때 오른쪽 바퀴가 앞으로? 정지 때 바로 섰나?"
  fi
fi

echo
echo "다 보면 RViz 창을 닫는다 (저장하지 않고 닫기)."
wait $RVIZ
stop_all
