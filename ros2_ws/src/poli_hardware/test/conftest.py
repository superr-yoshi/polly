# colcon 빌드 없이 `pytest ros2_ws/src/poli_hardware/test` 로도 실행되도록 패키지 경로 추가
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
