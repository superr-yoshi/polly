# 노트북(WSL) 확인 스크립트 공통 부분. 직접 실행하지 않는다.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Pi IP: 인자로 주거나, 지난번 IP(~/.cache/poli_pi_ip)를 먼저 써 보고,
# 안 되면 Windows에서 poli-desktop.local로 찾는다 (WSL은 .local을 못 푼다)
PI_CACHE=~/.cache/poli_pi_ip
find_pi() {
  if [ -n "$1" ]; then PI=$1; save_pi; return; fi
  PI=$(cat "$PI_CACHE" 2>/dev/null)
  # 지난번 IP로 연결을 열어 본다 (열리면 뒤 명령들이 그 연결을 다시 쓴다)
  if [ -n "$PI" ] && PI_TIMEOUT=2 pi true 2>/dev/null; then echo "Pi: $PI"; return; fi
  PI=$(/mnt/c/Windows/System32/PING.EXE -4 -n 1 poli-desktop.local 2>/dev/null \
       | grep -oE '\[[0-9.]+\]' | tr -d '[]' | head -1)
  if [ -z "$PI" ]; then
    echo "Pi를 못 찾았다. Pi와 노트북이 같은 핫스팟에 있는지 확인하거나 IP를 직접 준다: $0 <PiIP>" >&2
    exit 1
  fi
  save_pi
  echo "Pi: $PI"
}
save_pi() { mkdir -p "$(dirname "$PI_CACHE")" && echo "$PI" > "$PI_CACHE"; }

# 연결을 한 번 열어 두고 다시 쓴다 (ControlMaster). 두 번째 명령부터 약 0.4 s -> 0.05 s (2026-11-01)
# ServerAlive: Wi-Fi가 잠깐 끊겨 죽은 연결을 약 3초 안에 알아채고 버린다. 없으면 죽은 연결로 보낸 명령이
# TCP 재전송을 기다리며 오래 멈춘다 (2026-11-03: 엔터를 쳐도 한 번씩 출발이 늦음).
pi() {
  ssh -o BatchMode=yes -o ConnectTimeout="${PI_TIMEOUT:-8}" -o ControlMaster=auto \
      -o ControlPath=/tmp/poli-ssh-%C -o ControlPersist=600 \
      -o ServerAliveInterval=1 -o ServerAliveCountMax=3 poli@"$PI" "$@"
}
