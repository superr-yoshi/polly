#!/bin/bash
# 임무 시작 후 Wi-Fi·Bluetooth를 소프트웨어로 차단한다 (Pi에서 실행).
#
# 근거: 대회 규정 1.3 / 3.5.3 (임무 시작 후 외부 통신 금지),
#       문의 1번 답변 "통신 시스템을 소프트웨어적으로 비활성화 하는 것으로 인정합니다."
#       docs/competition/rules_compliance.md
#
# 사용:  scripts/comm_off.sh [대기초]
#        예) 실행 명령을 받은 SSH 세션이 끊길 시간을 주려면 scripts/comm_off.sh 2
#
# 준비 (Pi에서 1회): rfkill 설치 후, 비밀번호 없이 쓸 수 있게 한다.
#   sudo apt install -y rfkill
#   echo "$USER ALL=(root) NOPASSWD: /usr/sbin/rfkill" | sudo tee /etc/sudoers.d/poli-rfkill
#   sudo chmod 440 /etc/sudoers.d/poli-rfkill
#
# 주의
# - 실행하면 SSH가 끊긴다. 다시 켜려면 로봇에서 직접 `sudo rfkill unblock all`
#   (모니터·키보드 연결) 하거나 아래 "복구"를 쓴다.
# - Ubuntu는 rfkill 상태를 재부팅 후에도 기억할 수 있다 (systemd-rfkill).
#   경기가 끝나면 반드시 `scripts/comm_off.sh --restore`로 되돌린다.
# - 어느 시점에 실행할지(임무 launch에 연결)는 시작 절차 담당이 정한다.
set -u

if [ "${1:-}" = "--restore" ]; then
  sudo -n /usr/sbin/rfkill unblock wifi
  sudo -n /usr/sbin/rfkill unblock bluetooth
  /usr/sbin/rfkill list
  exit 0
fi

sleep "${1:-0}"
sudo -n /usr/sbin/rfkill block wifi
sudo -n /usr/sbin/rfkill block bluetooth
/usr/sbin/rfkill list
