#!/bin/bash
# 1단계: IMX500 커널 드라이버·센서 펌웨어·오버레이 설치 (Pi에서 sudo로 실행, 끝나면 재부팅)
#   sudo bash scripts/pi_setup/imx500/install_driver.sh
# 되돌리기: sudo cp /boot/firmware/config.txt.bak-imx500 /boot/firmware/config.txt
#           sudo rm -r /lib/modules/$(uname -r)/updates/imx500 && sudo depmod -a
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
K=$(uname -r)
FW_DEB=imx500-firmware_0.FF23+1_all.deb

apt-get update
apt-get install -y linux-headers-$K build-essential device-tree-compiler
# 커널이 업데이트되면 직접 넣은 드라이버가 맞지 않게 된다 -> 커널 고정
apt-mark hold linux-image-raspi linux-raspi linux-headers-raspi linux-image-$K 2>/dev/null || true

# 드라이버 + 오버레이 빌드 (작업 폴더에서)
B=$(mktemp -d)
cp "$HERE"/{Makefile,imx500.c,imx500_cpio.inc,spi-rp2040-gpio-bridge.c,imx500-pi5-overlay.dts,imx500.dtsi,rpi-rp2040-gpio-bridge.dtsi} "$B"/
make -C "$B"

mkdir -p /lib/modules/$K/updates/imx500
cp "$B"/imx500.ko "$B"/spi-rp2040-gpio-bridge.ko /lib/modules/$K/updates/imx500/
depmod -a $K
cp "$B"/imx500-pi5.dtbo /boot/firmware/overlays/

# 센서 펌웨어 (Raspberry Pi OS 저장소의 imx500-firmware 패키지에서 두 파일만)
curl -sf -o "$B/$FW_DEB" http://archive.raspberrypi.com/debian/pool/main/i/imx500-firmware/$FW_DEB
dpkg-deb -x "$B/$FW_DEB" "$B/fw"
cp "$B"/fw/usr/lib/firmware/imx500_*.fpk /lib/firmware/

# config.txt: 케이블은 CAM/DISP 1(J4)에 꽂는다 (오버레이 기본값). CAM/DISP 0이면 ",cam0" 추가
[ -e /boot/firmware/config.txt.bak-imx500 ] || cp /boot/firmware/config.txt /boot/firmware/config.txt.bak-imx500
if ! grep -q "^dtoverlay=imx500-pi5" /boot/firmware/config.txt; then
  printf "\n[pi5]\n# POLI: AI 카메라(IMX500), CAM/DISP 1 포트(J4). scripts/pi_setup/imx500\ndtoverlay=imx500-pi5\n[all]\n" >> /boot/firmware/config.txt
fi
rm -rf "$B"
echo "완료 - 재부팅: sudo reboot"
