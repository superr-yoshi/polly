#!/bin/bash
# 노트북에서 빨간 물체 인식 결과(/vision/target) 보기. Ctrl+C로 끝낸다.
# 임무나 red_object_detector가 Pi에서 켜져 있어야 값이 나온다.
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/_common.sh"
ros2 topic echo /vision/target
