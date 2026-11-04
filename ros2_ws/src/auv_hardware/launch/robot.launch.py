"""Starts the nodes of the real robot with their parameters (config/robot.yaml).

    ros2 launch auv_hardware robot.launch.py                  # sensors, navigation, servos: the tail does not move
    ros2 launch auv_hardware robot.launch.py swim:=true       # also the heading control: THE TAIL STARTS FLAPPING
    ros2 launch auv_hardware robot.launch.py barometer:=false # without the pressure sensor

Start it with the pressure sensor at the surface: this pressure is the zero of the depth.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('auv_hardware'), 'config', 'robot.yaml')
    params = LaunchConfiguration('params')

    def node(package, executable, **kwargs):
        return Node(package=package, executable=executable, name=executable, parameters=[params], output='screen',
                    **kwargs)

    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=default_params, description='parameter file'),
        DeclareLaunchArgument('swim', default_value='false', description='start the heading control (the tail flaps)'),
        DeclareLaunchArgument('barometer', default_value='true', description='start the pressure sensor'),
        node('auv_hardware', 'navigator'),
        node('auv_hardware', 'barometer', condition=IfCondition(LaunchConfiguration('barometer'))),
        node('gnc_control', 'fusion'),
        node('gnc_control', 'navigation'),
        node('gnc_control', 'tail_oscillator'),
        node('gnc_control', 'heading_control', condition=IfCondition(LaunchConfiguration('swim'))),
    ])
