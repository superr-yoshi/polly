#!/bin/bash
# 노트북에서 집게 열기 (임무가 켜져 있으면 먼저 끈다). 약 6초 걸린다.
#   bash scripts/laptop/grip_open.sh [PiIP]
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/_common.sh"
find_pi "$1"
pi '~/polly/scripts/stop_mission.sh > /dev/null 2>&1
source /opt/ros/jazzy/setup.bash && source ~/polly/ros2_ws/install/setup.bash && cd ~
setsid nohup env --default-signal=INT ros2 launch poli_hardware hardware.launch.py \
  use_fake_hardware:=false > /tmp/hw_grip.log 2>&1 < /dev/null &
sleep 4
for i in 1 2 3; do
  ros2 topic pub --once /gripper/command std_msgs/msg/String "{data: open}" > /dev/null 2>&1
  sleep 0.3
done
echo "집게: $(timeout 5 ros2 topic echo --once /gripper/state --field data 2>/dev/null | head -1)"
pkill -INT -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch poli_hardware"
sleep 2'
