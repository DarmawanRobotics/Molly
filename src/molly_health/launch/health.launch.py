#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the health monitor."""
    share = get_package_share_directory('molly_health')
    params = os.path.join(share, 'config', 'health_params.yaml')
    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value=params),
            Node(
                package='molly_health',
                executable='health_monitor_node',
                name='health_monitor_node',
                output='screen',
                parameters=[LaunchConfiguration('params_file')],
            ),
        ]
    )
