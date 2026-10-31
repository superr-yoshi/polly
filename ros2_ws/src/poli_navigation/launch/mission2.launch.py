"""
임무 2 통합 실행: 조원 A 하드웨어 + LiDAR + 카메라(조원 B) + 임무 2 노드.

  ros2 launch poli_navigation mission2.launch.py                           # fake 하드웨어 (기본)
  ros2 launch poli_navigation mission2.launch.py use_fake_hardware:=false  # 실제 로봇

- 하드웨어: poli_hardware/launch/hardware.launch.py (TF base_link -> laser 포함)
- 실제 로봇일 때만 LiDAR(sllidar_ros2, RPLIDAR C1)와 카메라(camera_vision)도 켠다.
  fake 모드에는 /scan, /vision/target이 없으므로 임무 2는 대상을 찾지 못하고 탐색만 한다.
  연결(토픽, TF) 확인용이다. 판단 로직 시험은 mission2_sim.launch.py를 쓴다.
- mission2_sim.launch.py(fake_robot)와 동시에 실행하면 안 된다. (/odom_raw가 겹친다)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    fake = LaunchConfiguration('use_fake_hardware')
    # 연습 확인용: 카메라 확인 영상 + Foxglove 브리지(노트북에서 /scan·TF·영상 보기).
    # 대회에서는 끈다 (임무 시작 후 외부 통신 금지, 규정 1.3). start_mission.sh --test에서만 켠다.
    debug_view = LaunchConfiguration('debug_view')
    hardware = os.path.join(
        get_package_share_directory('poli_hardware'),
        'launch', 'hardware.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hardware', default_value='true'),
        DeclareLaunchArgument('debug_view', default_value='false'),
        # true: 임무 노드를 미리 켜 두고 시작 신호(start_mission.sh go)를 기다린다
        DeclareLaunchArgument('wait_for_start', default_value='false'),
        # TODO_MEASURE: udev 규칙(docs/udev_template.md) 적용 후 이름 확인
        DeclareLaunchArgument('lidar_port', default_value='/dev/robot_lidar'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware),
            launch_arguments={'use_fake_hardware': fake}.items(),
        ),

        # RPLIDAR C1: 460800 baud. frame_id는 URDF와 같은 'laser'.
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'angle_compensate': True,
            }],
            output='screen',
            condition=UnlessCondition(fake),
        ),

        Node(
            package='camera_vision',
            executable='red_object_detector',
            # 2026-11-01 실기: 시작 약 2초 뒤 Pi 전체가 0.3 s 멈춰 Mega가 바퀴를 세웠다.
            # 무거운 카메라·Foxglove는 우선순위를 낮춰 임무·Mega 노드가 먼저 돌게 한다.
            prefix='nice -n 10',
            parameters=[{'debug_image_hz': PythonExpression(
                ["5.0 if '", debug_view, "' == 'true' else 0.0"])}],
            output='screen',
            condition=UnlessCondition(fake),
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            # 2026-11-01 실기: 시작 약 2초 뒤 Pi 전체가 0.3 s 멈춰 Mega가 바퀴를 세웠다.
            # 무거운 카메라·Foxglove는 우선순위를 낮춰 임무·Mega 노드가 먼저 돌게 한다.
            prefix='nice -n 10',
            parameters=[{'port': 8765}],
            output='log',
            condition=IfCondition(debug_view),
        ),

        Node(
            package='poli_navigation',
            executable='mission2',
            parameters=[{'wait_for_start': LaunchConfiguration('wait_for_start')}],
            output='screen',
        ),
    ])
