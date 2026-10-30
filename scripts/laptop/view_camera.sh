#!/bin/bash
# 노트북에서 카메라 확인: Pi에서 시험용 영상 발행을 켜고 노트북 rqt_image_view에 띄운다.
# 창을 닫으면 Pi의 영상 발행도 끈다.   사용: bash scripts/laptop/view_camera.sh [PiIP]
# 카메라는 한 프로그램만 쓸 수 있다 -> 임무나 red_object_detector가 켜져 있으면 먼저 끈다.
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/_common.sh"
find_pi "$1"

pi 'test -f ~/polly/scripts/camera_stream_test.py' || {
  echo "Pi의 저장소가 옛 버전이다. 먼저: ssh poli@$PI 'cd ~/polly && git pull'"; exit 1; }

pi 'source /opt/ros/jazzy/setup.bash;
    setsid nohup python3 ~/polly/scripts/camera_stream_test.py > /tmp/view_camera.log 2>&1 < /dev/null &
    sleep 5; pgrep -f "^python3 /home/poli/polly/scripts/camera_stream_test.py" > /dev/null && echo "영상 발행 시작" || tail -5 /tmp/view_camera.log' || exit 1

echo "영상 창을 연다. 다 보면 창을 닫는다."
ros2 run rqt_image_view rqt_image_view /camera/image_raw > /dev/null 2>&1

echo "Pi의 영상 발행을 끈다."
pi 'pkill -INT -f "^python3 /home/poli/polly/scripts/camera_stream_test.py"'
