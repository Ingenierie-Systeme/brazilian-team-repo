"""Starts the heading controller with its parameters (config/heading_control.yaml).

    ros2 launch gnc_control heading_control.launch.py
    ros2 launch gnc_control heading_control.launch.py params:=/path/to/other.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('gnc_control'), 'config', 'heading_control.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=default_params, description='parameter file'),
        Node(package='gnc_control', executable='heading_control', name='heading_control',
             parameters=[LaunchConfiguration('params')], output='screen'),
    ])
