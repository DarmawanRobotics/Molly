#!/usr/bin/env python3
import os
import re
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

METHODS = ('fast_lio', 'leg_kilo', 'external')  # external: simulator or another estimator


def truthy(value: str) -> bool:
    """Interpret a launch argument as a bool."""
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def leg_kilo_config(path: str, backend: bool) -> str:
    """Copy the Leg-KILO yaml with backend_enable set (it has no parameter overrides)."""
    with open(path) as f:
        text = f.read()
    line = f'backend_enable: {"true" if backend else "false"}'
    text, n = re.subn(r'(?m)^backend_enable:.*$', line, text)
    if n == 0:
        text += f'\n{line}\n'
    fd, out = tempfile.mkstemp(prefix='molly_leg_kilo_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        f.write(text)
    return out


def launch_setup(context):
    """Build the odometry nodes for the selected method."""
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    method = arg('method')
    if method not in METHODS:
        raise RuntimeError(f'method:={method!r}, choose one of {METHODS}')
    save_map = truthy(arg('save_map'))
    respawn = truthy(arg('respawn'))
    params = arg('params_file')
    site = [arg('site_config')] if os.path.isfile(arg('site_config')) else []
    lib = os.path.join(get_package_prefix('molly_odometry'), 'lib', 'molly_odometry')
    gate = os.path.join(lib, 'wait_still')
    # FAST-LIO and Leg-KILO both estimate gravity/biases at start: hold them until still.
    gate_params = ''.join(f'--params-file {p} ' for p in (params, *site))
    prefix = f'{gate} {gate_params}--' if truthy(arg('wait_still')) else None
    common = {'output': 'screen', 'respawn': respawn, 'respawn_delay': 3.0}
    sites_dir = [{'sites_dir': arg('sites_dir')}] if arg('sites_dir').strip() else []

    estimators = []  # external: /Odometry and camera_init -> body come from elsewhere (molly_sim)
    if method == 'fast_lio':
        estimators.append(
            Node(
                package='fast_lio',
                executable='fastlio_mapping',
                name='fastlio_mapping_node',
                parameters=[
                    arg('fast_lio_params_file'),
                    *site,
                    {'pcd_save.pcd_save_en': save_map},
                ],
                prefix=prefix,
                **common,
            )
        )
    elif method == 'leg_kilo':
        config = leg_kilo_config(arg('leg_kilo_config_file'), backend=save_map)
        viewer = 'true' if truthy(arg('leg_kilo_viewer')) else 'false'
        estimators.append(
            Node(
                package='legkilo',
                executable='legkilo_node',
                arguments=['--config_file', config, f'--viewer={viewer}'],
                prefix=prefix,
                **common,
            )
        )

    actions = [
        *estimators,
        Node(
            package='molly_odometry',
            executable='odometry_monitor_node',
            name='odometry_monitor_node',
            parameters=[params, *site, {'leg_odom_topic': arg('leg_odom_topic')}],
            **common,
        ),
    ]
    if truthy(arg('record')):
        actions.append(
            Node(
                package='molly_odometry',
                executable='route_recorder_node',
                name='route_recorder_node',
                parameters=[params, *site, *sites_dir],
                **common,
            )
        )
    if truthy(arg('localization')):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('molly_localization'),
                        'launch',
                        'localization.launch.py',
                    )
                ),
                launch_arguments={'tags_config_file': arg('tags_config_file')}.items(),
            )
        )
    return actions


def generate_launch_description():
    """Launch LiDAR-inertial odometry, its monitor and the route recorder."""
    share = get_package_share_directory('molly_odometry')
    config = lambda name: os.path.join(share, 'config', name)  # noqa: E731
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'method',
                default_value=os.environ.get('ODOMETRY', 'fast_lio'),
                description='fast_lio | leg_kilo | external (default: $ODOMETRY or fast_lio)',
            ),
            DeclareLaunchArgument(
                'save_map',
                default_value='false',
                description='Teach: keep the map (FAST-LIO PCD / Leg-KILO backend). RAM grows.',
            ),
            DeclareLaunchArgument(
                'wait_still',
                default_value='true',
                description='Hold the estimator (also on respawn) until the IMU is still.',
            ),
            DeclareLaunchArgument('respawn', default_value='false'),
            DeclareLaunchArgument('record', default_value='true', description='route recorder'),
            DeclareLaunchArgument(
                'sites_dir', default_value='', description='sites root; empty: the params file'
            ),
            DeclareLaunchArgument(
                'localization',
                default_value='false',
                description='Also start localization_node (map->odom, frame bridge).',
            ),
            DeclareLaunchArgument('tags_config_file', default_value=''),
            DeclareLaunchArgument(
                'site_config',
                default_value='',
                description='Extra parameter file applied on top of the defaults (if it exists).',
            ),
            DeclareLaunchArgument(
                'leg_odom_topic', default_value='/robot/odom', description='"" disables the check'
            ),
            DeclareLaunchArgument('params_file', default_value=config('odometry_params.yaml')),
            DeclareLaunchArgument('fast_lio_params_file', default_value=config('fast_lio.yaml')),
            DeclareLaunchArgument('leg_kilo_config_file', default_value=config('leg_kilo.yaml')),
            DeclareLaunchArgument(
                'leg_kilo_viewer',
                default_value='false',
                description='Open the Leg-KILO Iridescence viewer (needs a display).',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
