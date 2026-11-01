#!/bin/bash
# 노트북에서 집게 + VL53L0X 확인: 키(o 열기, g 닫기, q 끝)로 집게를 움직이며 거리·잡힘 판단을 본다.
#   bash scripts/laptop/gripper_tof.sh [PiIP]
# Pi에서 임무가 켜져 있으면 먼저 끄고, 하드웨어 노드(mega_bridge_node, tof_node 등)를 켰다가 끝나면 끈다.
# 바퀴 명령은 보내지 않는다. 따로 켠 vl53l0x_test.py가 있으면 먼저 Ctrl+C로 끈다 (센서를 같이 쓰면 값이 섞임).
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/_common.sh"
find_pi "$1"
pi '~/polly/scripts/stop_mission.sh > /dev/null 2>&1
pkill -INT -f "^python3 /home/poli/polly/scripts/vl53l0x_test.py"
source /opt/ros/jazzy/setup.bash && source ~/polly/ros2_ws/install/setup.bash && cd ~
setsid nohup env --default-signal=INT ros2 launch poli_hardware hardware.launch.py \
  use_fake_hardware:=false > /tmp/hw_tof.log 2>&1 < /dev/null &
sleep 4
grep -E "VL53L0X|tof_node" /tmp/hw_tof.log | tail -2' || exit 1
ssh -t -o ConnectTimeout=8 -o ControlMaster=auto -o ControlPath=/tmp/poli-ssh-%C \
  -o ControlPersist=600 poli@"$PI" \
  'source /opt/ros/jazzy/setup.bash && source ~/polly/ros2_ws/install/setup.bash &&
   python3 ~/polly/scripts/gripper_tof_console.py'
echo "하드웨어 노드를 끈다."
pi 'pkill -INT -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch poli_hardware"; sleep 2'
