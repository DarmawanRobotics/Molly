"""
Launch the handheld's UDP gateway and, with MediaMTX installed, its WebRTC video.

MediaMTX (script/setup.sh install webrtc) serves WHEP on TCP 8890 and media on UDP 8190 and runs
video_stream_node while the handheld watches; without it the handheld gets JPEG frames.
"""

from __future__ import annotations

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from molly_bridge.core import mediamtx as mtx

import dharma_utils.ros.launch as dl

PACKAGE = 'molly_bridge'
NODE = 'teleop_udp_node'
MEDIAMTX_CONFIG = '~/.molly/mediamtx.yml'


def truthy(value: str) -> bool:
    """Interpret a launch argument as a bool."""
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def parameter_files(arg) -> list:
    """Return [package defaults, robot_params?, params_file?] (a set file must exist)."""
    files = [os.path.join(get_package_share_directory(PACKAGE), 'config', 'bridge_params.yaml')]
    for name in ('robot_params', 'params_file'):
        path = arg(name)
        if path and not os.path.isfile(path):
            raise FileNotFoundError(f'{name}:={path} does not exist')
        files += [path] if path else []
    return files


def video_server(arg, files: list):
    """Return (MediaMTX process or None, log line) for the handheld's WebRTC video."""
    if not truthy(arg('webrtc')):
        return None, 'WebRTC video off (webrtc:=false): the handheld gets JPEG frames'
    found, version, problem = mtx.usable(arg('mediamtx'))
    if found is None:
        return None, f'WebRTC video off: {problem}'
    try:
        executable = os.path.join(get_package_prefix(PACKAGE), 'lib', PACKAGE, 'video_stream_node')
    except PackageNotFoundError:
        executable = ''
    if not os.path.isfile(executable):
        return None, 'WebRTC video off: video_stream_node is not built'
    token = str(mtx.node_parameter(files, NODE, 'token', '') or '')
    command = mtx.node_command(executable, files, mtx.rtsp_url())
    try:
        path = mtx.write_config(mtx.mediamtx_config(command, token, version), MEDIAMTX_CONFIG)
    except OSError as error:
        return None, f'WebRTC video off: cannot write {MEDIAMTX_CONFIG}: {error}'
    process = ExecuteProcess(
        cmd=[found, path], name='mediamtx', output='screen', respawn=True, respawn_delay=3.0
    )
    return process, f'WebRTC video: MediaMTX {mtx.version_text(version)} on port {mtx.WEBRTC_PORT}'


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context).strip()  # noqa: E731
    files = parameter_files(arg)
    ns = arg('robot_ns').strip('/')
    overrides = {}
    if arg('use_sim_time'):
        overrides['use_sim_time'] = truthy(arg('use_sim_time'))
    if ns:
        overrides['robot_command_prefix'] = f'/{ns}/'
    if arg('sites_dir'):
        overrides['sites_dir'] = arg('sites_dir')
    process, note = video_server(arg, files)
    if process is not None:
        overrides['video_port'] = mtx.WEBRTC_PORT  # the handheld asks for WebRTC video here
    driver = f'/{ns}' if ns else '/robot'
    actions = [LogInfo(msg=note)] + ([process] if process is not None else [])
    actions.append(
        dl.node(
            package=PACKAGE,
            executable=NODE,
            name=NODE,
            output='screen',
            parameters=files + dl.node_overrides(NODE, overrides),
            remappings=[
                ('cmd_vel', f'{driver}/cmd_vel'),  # read only: what the control mux sends
                ('battery', f'{driver}/battery'),
                ('diagnostics', f'{driver}/diagnostics'),
            ],
            ros_arguments=['--log-level', arg('log_level') or 'info'],
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
        ('robot_ns', '', 'driver namespace (/<ns>/battery, services); empty: robot'),
        ('sites_dir', '', 'sites directory (routes); empty: from the files'),
        ('webrtc', 'true', 'handheld video over WebRTC when MediaMTX is installed'),
        ('mediamtx', '', 'MediaMTX binary; empty: /opt/molly/mediamtx, then PATH'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in arguments]
        + [OpaqueFunction(function=launch_setup)]
    )
