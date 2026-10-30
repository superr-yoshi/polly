#!/bin/bash
# 노트북에서 집게 시험: 카메라로 주황 대상을 찾아 다가가서 잡는다 (Pi에서 scripts/grab_demo.py 실행).
# 사용: bash scripts/laptop/grab_demo.sh --look     모터 없이 탐지만 (먼저 이걸로 색·방향 확인)
#       bash scripts/laptop/grab_demo.sh            실제로 움직여 잡기 (motor_test를 Mega에 먼저 올린다)
#       UPLOAD=0 bash scripts/laptop/grab_demo.sh   업로드 없이 다시 실행 (값만 바꿔 볼 때)
#       뒤에 붙인 옵션은 그대로 grab_demo.py로 간다 (예: --final-ms 500 --rot180 0)
# ⚠️ 로봇이 바닥에서 스스로 움직인다. 비상정지에 손을 두고, Ctrl+C로 바로 멈춘다.
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
source "$HERE/_common.sh"
find_pi "${PI_IP:-}"

pi 'pgrep -f "^python3 /home/poli/polly/scripts/camera_stream_test.py" > /dev/null' \
  && { echo "카메라를 다른 프로그램(view_camera 등)이 쓰고 있다 -> 그 창을 먼저 닫는다"; exit 1; }
if [[ " $* " != *" --look "* ]]; then
  pi 'test -e /dev/robot_mega' || { echo "Pi에서 Mega(/dev/robot_mega)가 안 보인다"; exit 1; }
  pi 'fuser -s "$(readlink -f /dev/robot_mega)"' 2>/dev/null \
    && { echo "다른 프로그램이 Mega 포트를 쓰고 있다 (motor_test 콘솔 등) -> Ctrl+C로 먼저 끈다"; exit 1; }
  if [ "${UPLOAD:-1}" = 1 ]; then
    echo "motor_test를 Mega에 올린다 (30초쯤)..."
    scp -q -o BatchMode=yes -r "$ROOT/firmware/motor_test" poli@"$PI":/tmp/ || exit 1
    pi "~/.local/bin/arduino-cli compile --fqbn arduino:avr:mega --upload --port /dev/robot_mega \
          /tmp/motor_test > /tmp/mega_upload.log 2>&1" \
      && echo "  업로드 OK" \
      || { echo "  실패:"; pi 'tail -15 /tmp/mega_upload.log'; exit 1; }
  fi
fi
scp -q -o BatchMode=yes "$ROOT/scripts/grab_demo.py" poli@"$PI":/tmp/ || exit 1
ssh -t -o ConnectTimeout=8 poli@"$PI" "python3 /tmp/grab_demo.py $*"
