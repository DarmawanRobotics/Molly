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
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value=default_params),
            Node(
                package='molly_navigation',
                executable='obstacle_avoidance_node',
                name='obstacle_avoidance_node',
                output='screen',
                parameters=[params_file],
            ),
        ]
    )
