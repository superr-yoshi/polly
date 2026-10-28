#!/bin/bash
# 노트북에서 바퀴만 시험: Pi에서 rrc_adapter_node를 켜고 정해진 짧은 동작(앞·뒤·좌·우)을 보낸 뒤 끈다.
# 사용: bash scripts/laptop/wheel_test.sh [PiIP]
# ⚠️ 바퀴를 바닥에서 띄우고, RRC Lite USB 케이블을 바로 뽑을 수 있게 한다.
#    (조원 A 2026-10-08: 폭주 시 배터리 스위치를 꺼도 USB가 꽂혀 있으면 모터가 돈다)
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/_common.sh"
find_pi "$1"

if ! pi 'test -e /dev/robot_rrc'; then
  echo "Pi에 RRC Lite가 안 보인다 (/dev/robot_rrc 없음). RRC Lite USB를 Pi에 꽂는다."
  exit 1
fi

echo
echo "⚠️  확인:"
echo "   1. 로봇을 받침 위에 올려 바퀴가 바닥에 닿지 않는다"
echo "   2. RRC Lite USB 케이블을 손으로 바로 뽑을 수 있다 (비상정지)"
echo "   3. 바퀴 근처에 손·케이블이 없다"
read -r -p "모두 확인했으면 y 입력: " ok
[ "$ok" = "y" ] || { echo "취소"; exit 1; }

echo "Pi 코드 업데이트·빌드 (처음엔 1~2분)..."
pi 'cd ~/polly && git pull -q && source /opt/ros/jazzy/setup.bash && cd ros2_ws &&
    colcon build --symlink-install --packages-select poli_description poli_hardware > /tmp/wheel_build.log 2>&1 &&
    echo "  빌드 OK" || { echo "  빌드 실패"; tail -5 /tmp/wheel_build.log; exit 1; }' || exit 1

echo "바퀴 노드(rrc_adapter_node)를 켠다..."
pi 'source /opt/ros/jazzy/setup.bash; source ~/polly/ros2_ws/install/setup.bash;
    P=$(ros2 pkg prefix poli_hardware)/share/poli_hardware/config/hardware.yaml;
    setsid nohup ros2 run poli_hardware rrc_adapter_node --ros-args --params-file $P \
      > /tmp/wheel_node.log 2>&1 < /dev/null &
    sleep 4; pgrep -f rrc_adapter_node > /dev/null && echo "  노드 OK" \
      || { echo "  노드 실패"; tail -5 /tmp/wheel_node.log; exit 1; }' || exit 1

echo "동작 시작 (멈추려면 Ctrl+C, 위험하면 RRC USB 뽑기):"
pi -t 'source /opt/ros/jazzy/setup.bash; python3 ~/polly/scripts/wheel_test.py'

echo "바퀴 노드를 끈다."
pi 'pkill -INT -f "lib/poli_hardware/rrc_adapter_node"; sleep 2; echo "  꺼짐"; tail -3 /tmp/wheel_node.log'

echo
echo "확인할 것: 앞으로 때 두 바퀴가 같은 방향(앞)으로 돌았나? 왼쪽 회전 때 오른쪽 바퀴가 앞으로 돌았나? 정지 때 바로 섰나?"
