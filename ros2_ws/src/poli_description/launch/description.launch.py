"""
POLI 위치 모델 발행 (robot_state_publisher).

  ros2 launch poli_description description.launch.py

base_link -> laser, imu_link, camera_link, ultrasonic_*_link, gripper_link TF를 발행한다.
odom -> base_link는 발행하지 않는다 (robot_localization EKF 소유).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('poli_description'), 'urdf', 'poli.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
    ])
