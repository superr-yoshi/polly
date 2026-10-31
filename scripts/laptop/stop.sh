#!/bin/bash
# 노트북에서 임무 즉시 정지: Pi의 stop_mission.sh를 실행한다 (임무 노드를 바로 끝내 0.3초 안에 바퀴가 선다).
# 사용: bash scripts/laptop/stop.sh [PiIP]   진짜 급할 때는 물리 비상정지가 가장 빠르다.
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/_common.sh"
find_pi "$1"
pi '~/polly/scripts/stop_mission.sh'
