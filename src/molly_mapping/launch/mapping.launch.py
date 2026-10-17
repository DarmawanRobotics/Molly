#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the grid mapper and the site manager."""
    default_params = os.path.join(
        get_package_share_directory('molly_mapping'), 'config', 'mapping_params.yaml'
    )
    params = LaunchConfiguration('params_file')
    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value=default_params),
            DeclareLaunchArgument(
                'sites_dir', default_value='~/dev/Molly/map', description='sites directory'
            ),
            DeclareLaunchArgument(
                'site', default_value='', description='site to load; empty: the last one'
            ),
            Node(
                package='molly_mapping',
                executable='grid_mapper_node',
                name='grid_mapper_node',
                output='screen',
                parameters=[params],
            ),
            Node(
                package='molly_mapping',
                executable='site_manager_node',
                name='site_manager_node',
                output='screen',
                parameters=[
                    params,
                    {
                        'sites_dir': LaunchConfiguration('sites_dir'),
                        'site': LaunchConfiguration('site'),
                    },
                ],
            ),
        ]
    )
