"""
Launch the dashboard's gateways: rosbridge (whitelisted names) and web_video_server.

The dashboard drives through control/remote/dashboard/cmd_vel and the control mux, never the
driver's cmd_vel. Packages that are not installed are skipped with a note.
"""

from __future__ import annotations

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from molly_bridge.core.web import web_globs

import dharma_utils.ros.launch as dl


def installed(package: str) -> bool:
    """Return True when a package is installed."""
    try:
        get_package_share_directory(package)
    except PackageNotFoundError:
        return False
    return True


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context).strip()  # noqa: E731
    globs = web_globs(arg('robot_ns').strip('/') or 'robot')
    sim = {'use_sim_time': arg('use_sim_time').lower() == 'true'} if arg('use_sim_time') else {}
    common = {'output': 'screen', 'respawn': True, 'respawn_delay': 3.0}
    actions = []
    if installed('rosbridge_server') and installed('rosapi'):
        bridge = {'port': int(arg('web_port')), 'address': '', **globs, **sim}
        bridge.update(call_services_in_new_thread=True, send_action_goals_in_new_thread=True)
        actions += [
            dl.node(package='rosbridge_server', executable='rosbridge_websocket',
                    name='rosbridge_websocket', parameters=[bridge], **common),
            dl.node(package='rosapi', executable='rosapi_node', name='rosapi_node',
                    parameters=[{**globs, **sim}], **common),
            LogInfo(msg=f'web: rosbridge on ws port {arg("web_port")} (whitelisted names)'),
        ]  # fmt: skip
    else:
        actions.append(LogInfo(msg='web: rosbridge_server not installed (setup.sh install web)'))
    if installed('web_video_server'):
        video = {'port': int(arg('web_video_port')), 'default_stream_type': 'mjpeg', **sim}
        actions += [
            dl.node(package='web_video_server', executable='web_video_server',
                    name='web_video_server', parameters=[video], **common),
            LogInfo(msg=f'web: camera MJPEG on http port {arg("web_video_port")}'),
        ]  # fmt: skip
    else:
        actions.append(LogInfo(msg='web: web_video_server not installed (setup.sh install web)'))
    return actions


def generate_launch_description():
    arguments = [
        ('robot_ns', '', 'driver namespace of the whitelisted robot names; empty: robot'),
        ('web_port', '9090', 'rosbridge WebSocket port'),
        ('web_video_port', '8080', 'web_video_server port'),
        ('use_sim_time', '', 'true | false'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in arguments]
        + [OpaqueFunction(function=launch_setup)]
    )
