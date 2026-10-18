#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the control mux and the UDP teleop gateway."""
    share = get_package_share_directory('molly_bridge')
    params = os.path.join(share, 'config', 'bridge_params.yaml')
    robot_ns = LaunchConfiguration('robot_ns')
    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value=params),
            DeclareLaunchArgument('robot_ns', default_value='robot'),
            Node(
                package='dharma_control',
                executable='control_mux_node',
                name='control_mux_node',
                output='screen',
                parameters=[LaunchConfiguration('params_file')],
                remappings=[('cmd_vel', ['/', robot_ns, '/cmd_vel'])],
            ),
            Node(
                package='molly_bridge',
                executable='teleop_udp_node',
                name='teleop_udp_node',
                output='screen',
                parameters=[
                    LaunchConfiguration('params_file'),
                    {'robot_command_prefix': ['/', robot_ns, '/']},
                ],
                remappings=[
                    ('cmd_vel', ['/', robot_ns, '/cmd_vel']),
                    ('battery', ['/', robot_ns, '/battery']),
                    ('diagnostics', ['/', robot_ns, '/diagnostics']),
                ],
            ),
        ]
    )
