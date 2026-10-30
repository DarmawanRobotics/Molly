"""Launch the guide (tours) and the POIs and tours of the active site."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

import dharma_utils.ros.launch as dl

PACKAGE = 'molly_guide'
NODES = ('guide_node', 'site_content_node')


def parameter_files(context) -> list:
    """Return [package defaults, robot_params?, params_file?] (a set file must exist)."""
    files = [os.path.join(get_package_share_directory(PACKAGE), 'config', 'guide_params.yaml')]
    for name in ('robot_params', 'params_file'):
        path = LaunchConfiguration(name).perform(context).strip()
        if not path:
            continue
        if not os.path.isfile(path):
            raise FileNotFoundError(f'{name}:={path} does not exist')
        files.append(path)
    return files


def overrides(context) -> dict:
    """Return the non-empty launch argument overrides (an empty one overrides nothing)."""
    values = {}
    sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower()
    if sim_time:
        values['use_sim_time'] = sim_time in ('true', '1', 'yes')
    return values


def launch_setup(context):
    files, values = parameter_files(context), overrides(context)
    sites_dir = LaunchConfiguration('sites_dir').perform(context).strip()
    log = ['--log-level', LaunchConfiguration('log_level').perform(context)]
    actions = []
    for name in NODES:
        extra = {'sites_dir': sites_dir} if sites_dir and name == 'site_content_node' else {}
        actions.append(
            dl.node(
                package=PACKAGE,
                executable=name,
                name=name,
                output='screen',
                parameters=files + dl.node_overrides(name, {**values, **extra}),
                ros_arguments=log,
                respawn=True,
                respawn_delay=3.0,
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file', default_value='', description='overrides'),
            DeclareLaunchArgument(
                'robot_params', default_value='', description='robot profile (robot.*)'
            ),
            DeclareLaunchArgument('use_sim_time', default_value='', description='true | false'),
            DeclareLaunchArgument('log_level', default_value='info'),
            DeclareLaunchArgument(
                'sites_dir', default_value='', description='sites directory; empty: from files'
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
