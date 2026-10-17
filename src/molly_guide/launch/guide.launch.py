#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the guide (tours)."""
    default_params = os.path.join(
        get_package_share_directory('molly_guide'), 'config', 'guide_params.yaml'
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value=default_params),
            Node(
                package='molly_guide',
                executable='guide_node',
                name='guide_node',
                output='screen',
                parameters=[LaunchConfiguration('params_file')],
            ),
        ]
    )
