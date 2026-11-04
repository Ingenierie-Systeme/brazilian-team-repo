"""Starts the depth controller with its parameters (config/depth_control.yaml).

    ros2 launch gnc_control depth_control.launch.py
    ros2 launch gnc_control depth_control.launch.py params:=/path/to/other.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('gnc_control'), 'config', 'depth_control.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=default_params, description='parameter file'),
        Node(package='gnc_control', executable='depth_control', name='depth_control',
             parameters=[LaunchConfiguration('params')], output='screen'),
    ])
