#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch speech, gestures with the greeter and, optionally, the conversation model."""
    default_params = os.path.join(
        get_package_share_directory('molly_interaction'), 'config', 'interaction_params.yaml'
    )
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value=default_params),
            DeclareLaunchArgument('robot_ns', default_value='robot',
                                  description='namespace of the robot driver (gestures)'),
            DeclareLaunchArgument('speech', default_value='true', description='text to speech'),
            DeclareLaunchArgument('greeter', default_value='true',
                                  description='greet visitors who stop in front of the robot'),
            DeclareLaunchArgument('conversation', default_value='false',
                                  description='answer questions with Ollama'),
            Node(
                package='molly_interaction',
                executable='speech_node',
                name='speech_node',
                output='screen',
                parameters=[params_file],
                condition=IfCondition(LaunchConfiguration('speech')),
            ),
            Node(
                package='molly_interaction',
                executable='interaction_node',
                name='interaction_node',
                output='screen',
                parameters=[
                    params_file,
                    {
                        'robot_ns': LaunchConfiguration('robot_ns'),
                        'greeter_available': ParameterValue(
                            LaunchConfiguration('greeter'), value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package='molly_interaction',
                executable='conversation_node',
                name='conversation_node',
                output='screen',
                parameters=[params_file],
                condition=IfCondition(LaunchConfiguration('conversation')),
            ),
        ]
    )
