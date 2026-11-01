#!/bin/bash
# 노트북 터미널에서 Pi에 바로 접속한다 (IP를 몰라도 됨).
#
#   bash scripts/laptop/pi.sh                       # Pi 셸 (poli@poli-desktop)
#   bash scripts/laptop/pi.sh 'cd ~/polly && git pull'   # 명령 하나만 실행
#   PI=10.175.8.69 bash scripts/laptop/pi.sh        # Pi IP를 직접 줄 때
#
# 지난번 IP를 먼저 써 보고(약 0.2 s), 안 되면 poli-desktop.local로 찾는다.
# go.sh 등과 같은 연결(ControlMaster)을 다시 써서 두 번째부터는 거의 바로 붙는다.
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/_common.sh"
find_pi "${PI:-}" > /dev/null
if [ $# -eq 0 ]; then
  echo "Pi: $PI (나가기: exit 또는 Ctrl+D)"
  exec ssh -t -o ConnectTimeout=8 -o ControlMaster=auto \
      -o ControlPath=/tmp/poli-ssh-%C -o ControlPersist=600 poli@"$PI"
fi
pi "$@"
