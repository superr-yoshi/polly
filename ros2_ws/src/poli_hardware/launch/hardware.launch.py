"""
하드웨어 계층 launch (조원 A).

  ros2 launch poli_hardware hardware.launch.py                         # fake (기본)
  ros2 launch poli_hardware hardware.launch.py use_fake_hardware:=false # 실제 RRC + Mega

통합 bringup에서는 IncludeLaunchDescription으로 이 파일을 포함한다.
fake/real 어느 쪽이든 Topic 계약은 같다 (docs/interfaces.md): /cmd_vel, /gripper/command 구독,
/odom_raw, /imu/data, /battery_state, /range/*, /gripper/state 발행.
odom -> base_link TF는 발행하지 않는다 (EKF 소유).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('poli_hardware'), 'config', 'hardware.yaml')
    fake = LaunchConfiguration('use_fake_hardware')
    params = LaunchConfiguration('params_file')

    def node(executable, condition):
        return Node(package='poli_hardware', executable=executable, name=executable,
                    parameters=[params], output='screen', condition=condition)

    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hardware', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        node('fake_rrc_node', IfCondition(fake)),
        node('fake_mega_node', IfCondition(fake)),
        node('rrc_adapter_node', UnlessCondition(fake)),
        node('mega_bridge_node', UnlessCondition(fake)),
    ])
