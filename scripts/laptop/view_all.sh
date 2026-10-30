#!/bin/bash
# 노트북에서 LiDAR + 카메라를 한 번에 확인: Pi에서 둘 다 켜고 RViz 창 하나에 같이 띄운다.
# (왼쪽 아래 Camera 칸 = 카메라 영상, 가운데 빨간 점 = LiDAR, 격자 한 칸 = 0.4 m, 위쪽 = 로봇 정면)
# RViz 창을 닫으면 Pi 쪽도 끈다.   사용: bash scripts/laptop/view_all.sh [PiIP]
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/_common.sh"
find_pi "$1"

pi 'test -f ~/polly/scripts/camera_stream_test.py' || {
  echo "Pi 코드가 옛 버전이라 업데이트한다 (git pull)"; pi 'cd ~/polly && git pull -q' || exit 1; }

echo "Pi에서 LiDAR와 카메라를 켠다..."
pi 'source /opt/ros/jazzy/setup.bash; source ~/lidar_ws/install/setup.bash;
    setsid nohup ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/robot_lidar \
      > /tmp/view_lidar.log 2>&1 < /dev/null &
    setsid nohup python3 ~/polly/scripts/camera_stream_test.py > /tmp/view_camera.log 2>&1 < /dev/null &
    sleep 6
    grep -q "health status : OK" /tmp/view_lidar.log && echo "  LiDAR: OK" \
      || echo "  LiDAR: 응답 없음 -> LiDAR USB를 뺐다 다시 꽂고 다시 실행"
    pgrep -f "^python3 /home/poli/polly/scripts/camera_stream_test.py" > /dev/null && echo "  카메라: OK" \
      || { echo "  카메라: 실패"; tail -3 /tmp/view_camera.log; }'

echo "RViz를 연다. 다 보면 RViz 창을 닫는다 (저장하지 않고 닫기)."
rviz2 -d "$HERE/poli_view.rviz" > /dev/null 2>&1

echo "Pi의 LiDAR와 카메라를 끈다."
pi 'pkill -INT -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch sllidar";
    pkill -INT -f "^python3 /home/poli/polly/scripts/camera_stream_test.py"; sleep 3;
    pgrep -x sllidar_node | xargs -r kill; echo "  꺼짐"'
