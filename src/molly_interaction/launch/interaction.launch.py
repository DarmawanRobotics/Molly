"""Launch speech, gestures with the greeter, the visitor tracker and optionally the model."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

import dharma_utils.ros.launch as dl

PACKAGE = 'molly_interaction'
# node -> the launch argument that starts it (None: always)
NODES = {
    'interaction_node': None,
    'speech_node': 'speech',
    'visitor_tracker_node': 'visitors',
    'conversation_node': 'conversation',
}


def truthy(value: str) -> bool:
    """Interpret a launch argument as a bool."""
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def parameter_files(context) -> list:
    """Return [package defaults, robot_params?, params_file?] (a set file must exist)."""
    share = get_package_share_directory(PACKAGE)
    files = [os.path.join(share, 'config', 'interaction_params.yaml')]
    for name in ('robot_params', 'params_file'):
        path = LaunchConfiguration(name).perform(context).strip()
        if not path:
            continue
        if not os.path.isfile(path):
            raise FileNotFoundError(f'{name}:={path} does not exist')
        files.append(path)
    return files


def overrides(context, node: str) -> dict:
    """Return the non-empty launch argument overrides of a node."""
    arg = lambda name: LaunchConfiguration(name).perform(context).strip()  # noqa: E731
    values = {}
    if arg('use_sim_time'):
        values['use_sim_time'] = truthy(arg('use_sim_time'))
    if node == 'interaction_node':
        if arg('robot_ns'):
            values['robot_ns'] = arg('robot_ns').strip('/')
        if arg('greeter'):
            values['greeter_available'] = truthy(arg('greeter'))
    return values


def launch_setup(context):
    files = parameter_files(context)
    log = ['--log-level', LaunchConfiguration('log_level').perform(context)]
    actions = []
    for name, switch in NODES.items():
        if switch and not truthy(LaunchConfiguration(switch).perform(context)):
            continue
        actions.append(
            dl.node(
                package=PACKAGE,
                executable=name,
                name=name,
                output='screen',
                parameters=files + dl.node_overrides(name, overrides(context, name)),
                ros_arguments=log,
                respawn=True,
                respawn_delay=3.0,
            )
        )
    return actions


def generate_launch_description():
    arguments = [
        ('params_file', '', 'overrides'),
        ('robot_params', '', 'robot profile (robot.*)'),
        ('use_sim_time', '', 'true | false'),
        ('log_level', 'info', 'debug, info, warn, error'),
        ('robot_ns', '', 'driver namespace for the gesture services; empty: from the files'),
        ('speech', 'true', 'text to speech (speech_node)'),
        ('greeter', '', 'true | false: greet visitors; empty: from the files'),
        ('visitors', 'true', 'visitors from the LiDAR (visitor_tracker_node)'),
        ('conversation', 'false', 'answer questions with Ollama (conversation_node)'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in arguments]
        + [OpaqueFunction(function=launch_setup)]
    )
