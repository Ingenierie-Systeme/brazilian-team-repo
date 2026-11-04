"""Starts the navigation (nodes fusion and navigation) with its parameters (config/navigation.yaml).

    ros2 launch gnc_control navigation.launch.py
    ros2 launch gnc_control navigation.launch.py params:=/path/to/other.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('gnc_control'), 'config', 'navigation.yaml')
    params = LaunchConfiguration('params')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=default_params, description='parameter file'),
        Node(package='gnc_control', executable='fusion', name='fusion', parameters=[params], output='screen'),
        Node(package='gnc_control', executable='navigation', name='navigation', parameters=[params], output='screen'),
    ])
