"""
임무 1 통합 실행: 조원 A 하드웨어 + LiDAR + 카메라(조원 B) + 임무 1 노드.

  ros2 launch poli_navigation mission1.launch.py                           # fake 하드웨어 (기본)
  ros2 launch poli_navigation mission1.launch.py use_fake_hardware:=false  # 실제 로봇

- 대회에서는 scripts/start_mission.sh 1 로 실행한다 (실제 하드웨어 + 무선 차단).
- 하드웨어 구성은 mission2.launch.py와 같다. 다른 것은 임무 노드뿐이다.
- 시작 격자는 grid_map.MISSION1_START (9, 1) 고정. (1, 9)에서 시작하는 경우는 아직 처리하지 않는다.
- mission1_sim.launch.py(fake_robot)와 동시에 실행하면 안 된다. (/odom_raw가 겹친다)
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
        # 대회 경기장은 점대칭(규정) -> true. 대칭이 아닌 연습 경기장에서만 false (start_mission.sh --test).
        DeclareLaunchArgument('point_symmetry', default_value='true'),
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
            parameters=[{'debug_image_hz': PythonExpression(
                ["5.0 if '", debug_view, "' == 'true' else 0.0"])}],
            output='screen',
            condition=UnlessCondition(fake),
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            parameters=[{'port': 8765}],
            output='log',
            condition=IfCondition(debug_view),
        ),

        Node(
            package='poli_navigation',
            executable='mission1',
            parameters=[{'require_laser_tf': True,
                         'point_symmetry': LaunchConfiguration('point_symmetry')}],
            output='screen',
        ),
    ])
