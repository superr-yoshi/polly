# 노트북(WSL) 확인 스크립트 공통 부분. 직접 실행하지 않는다.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Pi IP: 인자로 주거나, Windows에서 poli-desktop.local로 찾는다 (WSL은 .local을 못 푼다)
find_pi() {
  if [ -n "$1" ]; then PI=$1; return; fi
  PI=$(/mnt/c/Windows/System32/PING.EXE -4 -n 1 poli-desktop.local 2>/dev/null \
       | grep -oE '\[[0-9.]+\]' | tr -d '[]' | head -1)
  if [ -z "$PI" ]; then
    echo "Pi를 못 찾았다. Pi와 노트북이 같은 핫스팟에 있는지 확인하거나 IP를 직접 준다: $0 <PiIP>"
    exit 1
  fi
  echo "Pi: $PI"
}

# 연결을 한 번 열어 두고 다시 쓴다 (ControlMaster). 두 번째 명령부터 약 0.4 s -> 0.05 s (2026-11-01)
pi() {
  ssh -o BatchMode=yes -o ConnectTimeout=8 -o ControlMaster=auto \
      -o ControlPath=/tmp/poli-ssh-%C -o ControlPersist=600 poli@"$PI" "$@"
}
