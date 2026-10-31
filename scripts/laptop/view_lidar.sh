#!/bin/bash
# 노트북에서 LiDAR 확인: Pi에서 LiDAR를 켜고 노트북 RViz에 /scan을 띄운다.
# RViz 창을 닫으면 Pi의 LiDAR도 끈다.   사용: bash scripts/laptop/view_lidar.sh [PiIP]
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/_common.sh"
find_pi "$1"

pi 'source /opt/ros/jazzy/setup.bash; source ~/lidar_ws/install/setup.bash;
    setsid nohup ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/robot_lidar \
      > /tmp/view_lidar.log 2>&1 < /dev/null & sleep 5;
    grep -E "health status : OK|ERROR" /tmp/view_lidar.log | tail -1' || exit 1

CFG=/tmp/sllidar_ros2.rviz
scp -q -o BatchMode=yes poli@"$PI":lidar_ws/install/sllidar_ros2/share/sllidar_ros2/rviz/sllidar_ros2.rviz $CFG
echo "RViz를 연다. 다 보면 RViz 창을 닫는다 (저장하지 않고 닫기)."
rviz2 -d $CFG -f laser > /dev/null 2>&1

echo "Pi의 LiDAR를 끈다."
pi 'pkill -INT -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch sllidar"; sleep 3;
    pgrep -x sllidar_node | xargs -r kill'
