#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the localization node (frame bridge + AprilTag map->odom)."""
    default_params = os.path.join(
        get_package_share_directory('molly_localization'), 'config', 'localization_params.yaml'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value=default_params),
            DeclareLaunchArgument(
                'tags_config_file',
                default_value='',
                description='JSON file with tag poses in the map frame',
            ),
            Node(
                package='molly_localization',
                executable='localization_node',
                name='localization_node',
                output='screen',
                parameters=[
                    LaunchConfiguration('params_file'),
                    {'tags_config_file': LaunchConfiguration('tags_config_file')},
                ],
            ),
        ]
    )
