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
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fake = LaunchConfiguration('use_fake_hardware')
    hardware = os.path.join(
        get_package_share_directory('poli_hardware'),
        'launch', 'hardware.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hardware', default_value='true'),
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
            output='screen',
            condition=UnlessCondition(fake),
        ),

        Node(
            package='poli_navigation',
            executable='mission2',
            output='screen',
        ),
    ])
