#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('molly_navigation'), 'config', 'navigation_params.yaml'
    )
    default_waypoints = '/home/robot/dev/Molly/map/example/example_waypoint.csv'

    params_file = LaunchConfiguration('params_file')
    waypoints_file = LaunchConfiguration('waypoints_file')

    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value=default_params),
            DeclareLaunchArgument('waypoints_file', default_value=default_waypoints),
            DeclareLaunchArgument(
                'cmd_vel_topic',
                default_value='cmd_vel',
                description="Robot velocity topic, e.g. '/robot/cmd_vel'",
            ),
            DeclareLaunchArgument(
                'battery_topic',
                default_value='battery',
                description="sensor_msgs/BatteryState topic, e.g. '/robot/battery'",
            ),
            Node(
                package='molly_navigation',
                executable='path_loader_node',
                name='path_loader_node',
                output='screen',
                parameters=[params_file, {'waypoints_file': waypoints_file}],
            ),
            Node(
                package='molly_navigation',
                executable='obstacle_avoidance_node',
                name='obstacle_avoidance_node',
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='molly_navigation',
                executable='path_follower_node',
                name='path_follower_node',
                output='screen',
                parameters=[params_file],
                remappings=[
                    ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
                    ('battery', LaunchConfiguration('battery_topic')),
                ],
            ),
        ]
    )
