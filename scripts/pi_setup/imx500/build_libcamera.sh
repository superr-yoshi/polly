#!/bin/bash
# 2단계: Raspberry Pi 판 libcamera(+libpisp)를 빌드한다 (Pi에서 일반 사용자로, 30~40분)
#   bash scripts/pi_setup/imx500/build_libcamera.sh
# 끝나면 3단계: sudo bash scripts/pi_setup/imx500/install_libcamera.sh
# 버전은 Pi 커널(6.8)과 가까운 시기의 조합 (Raspberry Pi OS bookworm 2025-09 기준)
set -e
LIBCAMERA_TAG=v0.5.2+rpt20250903
LIBPISP_TAG=v1.2.1
SRC=~/camsrc
mkdir -p $SRC && cd $SRC
[ -d libpisp ] || git clone --depth 1 -b $LIBPISP_TAG https://github.com/raspberrypi/libpisp.git
[ -d libcamera ] || git clone --depth 1 -b $LIBCAMERA_TAG https://github.com/raspberrypi/libcamera.git
cd libcamera
[ -e subprojects/libpisp ] || ln -s ../../libpisp subprojects/libpisp
meson setup build --buildtype=release --prefix=/usr/local \
  -Dpipelines=rpi/pisp -Dipas=rpi/pisp -Dv4l2=enabled -Dgstreamer=disabled \
  -Dtest=false -Dlc-compliance=disabled -Dcam=enabled -Dqcam=disabled \
  -Ddocumentation=disabled -Dpycamera=enabled
ninja -C build -j3
echo BUILD_OK
