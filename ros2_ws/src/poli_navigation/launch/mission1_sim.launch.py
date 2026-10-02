"""
임무 1 시뮬레이션: 가짜 로봇 + 임무 1 노드를 한 번에 실행한다.

SIM_ONLY: 실제 로봇에서는 사용하지 않는다.
장애물은 규정 그림 1 예시 배치를 사용한다. (grid_map.RULE_EXAMPLE_BLOCKED)
실행: ros2 launch poli_navigation mission1_sim.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # SIM_ONLY: /cmd_vel, /gripper/command를 받아
        # /odom_raw, /scan, /gripper/holding, /vision/target을 보낸다.
        Node(
            package='poli_navigation',
            executable='fake_robot',
            output='screen',
            parameters=[{'mission': 1}],
        ),
        Node(
            package='poli_navigation',
            executable='mission1',
            output='screen',
        ),
    ])
