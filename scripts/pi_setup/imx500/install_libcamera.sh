#!/bin/bash
# 빌드 준비물 설치(빌드 전, --deps) 또는 빌드한 libcamera 설치(빌드 후) (Pi에서 sudo로 실행)
#   sudo bash scripts/pi_setup/imx500/install_libcamera.sh --deps   # build_libcamera.sh 전에
#   sudo bash scripts/pi_setup/imx500/install_libcamera.sh          # build_libcamera.sh 후에
set -e
if [ "$1" = "--deps" ]; then
  apt-get install -y meson ninja-build pkg-config python3-yaml python3-ply python3-jinja2 \
    libgnutls28-dev openssl libssl-dev libyaml-dev libudev-dev libdw-dev libunwind-dev \
    libboost-program-options-dev libdrm-dev libexif-dev libjpeg-dev libtiff-dev libpng-dev \
    libepoxy-dev libevent-dev nlohmann-json3-dev libboost-log-dev libboost-thread-dev \
    pybind11-dev libcap-dev python3-dev python3-pip \
    python3-av python3-pil python3-numpy python3-jsonschema python3-tqdm python3-piexif \
    python3-prctl python3-libarchive-c
  echo "완료"
  exit 0
fi
ninja -C /home/poli/camsrc/libcamera/build install
ldconfig
# meson이 Python 모듈을 /usr/local/lib/python3/dist-packages에 넣는다 -> Ubuntu python 경로에 등록
P=$(find /usr/local/lib -path "*python3*" -type d -name libcamera -prune | head -1)
[ -n "$P" ] && echo "$(dirname "$P")" > /usr/local/lib/python3.12/dist-packages/libcamera-local.pth
echo "완료 - 확인: cam -l (imx500이 보여야 함)"
