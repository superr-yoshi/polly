#!/bin/bash
# 4단계: Picamera2 설치 (Pi에서 일반 사용자로). camera_vision(조원 B)이 사용한다.
#   bash scripts/pi_setup/imx500/install_picamera2.sh
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
PICAMERA2_TAG=v0.3.31   # libcamera v0.5.2+rpt20250903과 같은 시기
cd ~/camsrc
[ -d picamera2 ] || git clone --depth 1 -b $PICAMERA2_TAG https://github.com/raspberrypi/picamera2.git
pip3 install --user --break-system-packages PiDNG simplejpeg videodev2
pip3 install --user --break-system-packages --no-deps ~/camsrc/picamera2
# Ubuntu에는 pykms(kmsxx)가 없다. Picamera2 import에만 필요하므로 자리표시를 둔다.
cp "$HERE/pykms.py" ~/.local/lib/python3.12/site-packages/pykms.py
python3 -c "from picamera2 import Picamera2; print('picamera2 OK')"
