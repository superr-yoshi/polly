#!/bin/bash
# 노트북에서 Mega 시험 스케치 실행: firmware/<이름>을 Pi에서 컴파일·업로드하고 줄 단위 콘솔을 연다.
# Mega는 로봇의 Pi에 USB로 꽂힌 그대로 둔다 (노트북으로 옮기지 않는다).
# 사용: bash scripts/laptop/mega_test.sh servo_test|motor_test|imu_find [PiIP]
# 한 줄 입력 후 Enter. 끝내기는 Ctrl+C (Mega에 x를 보내 멈춘 뒤 닫는다).
# 시험 스케치는 초음파를 보내지 않는다. 그래도 옛 본 펌웨어(mega_sensor_controller)는 MDD10A용으로 고치기 전까지
# 다시 올리지 않는다 (MDD10A 모터 핀을 안 잡아 바퀴가 돌 수 있다). 시험 스케치를 올려 둔 채로 둔다.
# ⚠️ 업로드·콘솔 열기 때 Mega가 리셋된다. 처음엔 바퀴를 바닥에서 띄운다.
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
SK=${1%/}
if [ -z "$SK" ] || [ ! -f "$ROOT/firmware/$SK/$SK.ino" ]; then
  echo "사용: bash $0 servo_test|motor_test|imu_find [PiIP]"; exit 1
fi
source "$HERE/_common.sh"
find_pi "$2"

pi 'test -e /dev/robot_mega' || { echo "Pi에서 Mega(/dev/robot_mega)가 안 보인다 -> Mega USB 케이블 확인"; exit 1; }
pi 'fuser -s "$(readlink -f /dev/robot_mega)"' 2>/dev/null \
  && { echo "다른 프로그램이 Mega 포트를 쓰고 있다 (임무, mega_bridge 등) -> 먼저 끈다"; exit 1; }

echo "$SK 를 Pi에서 컴파일·업로드 (30초쯤)..."
scp -q -o BatchMode=yes -r "$ROOT/firmware/$SK" "$ROOT/scripts/mega_console.py" poli@"$PI":/tmp/ || exit 1
pi "~/.local/bin/arduino-cli compile --fqbn arduino:avr:mega --upload --port /dev/robot_mega /tmp/$SK \
      > /tmp/mega_upload.log 2>&1" \
  && echo "  업로드 OK" \
  || { echo "  실패:"; pi 'tail -15 /tmp/mega_upload.log'; exit 1; }

echo "콘솔을 연다."
ssh -t -o ConnectTimeout=8 poli@"$PI" "python3 /tmp/mega_console.py /dev/robot_mega"
