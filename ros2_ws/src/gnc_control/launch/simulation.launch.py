"""Starts all the GNC nodes for the simulation: navigation, heading and depth control, and optionally a mission.

    ros2 launch gnc_control simulation.launch.py
    ros2 launch gnc_control simulation.launch.py mission:=waypoints.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('gnc_control')
    config = os.path.join(share, 'config')
    mission = LaunchConfiguration('mission')

    def node(executable, params):
        return Node(package='gnc_control', executable=executable, name=executable,
                    parameters=[os.path.join(config, params)], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('mission', default_value='', description='file in config/missions; empty = no mission'),
        node('fusion', 'navigation.yaml'),
        node('navigation', 'navigation.yaml'),
        node('heading_control', 'heading_control.yaml'),
        node('depth_control', 'depth_control.yaml'),
        Node(package='gnc_control', executable='mission', name='mission', output='screen',
             parameters=[{'mission_file': PathJoinSubstitution([config, 'missions', mission])}],
             condition=IfCondition(PythonExpression(["'", mission, "' != ''"]))),
    ])
