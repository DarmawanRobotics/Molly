"""What the dashboard may reach through rosbridge: names, never the driver's cmd_vel (no ROS)."""

from __future__ import annotations

WEB_TOPICS = (
    '/map', '/localization/status', '/localization/pose', '/site/status', '/site/pois',
    '/site/tours', '/mapping/status', '/planner/status', '/planner/path', '/navigation/status',
    '/navigation/path', '/docking/status', '/tour/state', '/tour/cmd',
    '/interaction/speech_status', '/interaction/say_text', '/interaction/greeter',
    '/interaction/conversation', '/perception/visitors', '/control/status',
    '/control/remote/dashboard/cmd_vel', '/control/estop_request', '/health/status',
    '/odometry/status', '/rosout',
)  # fmt: skip
WEB_ROBOT_TOPICS = ('battery', 'imu', 'state', 'diagnostics', 'odom', 'joint_states')
WEB_SERVICES = (
    '/site/*', '/mapping/*', '/localization/start', '/localization/record_tags', '/planner/*',
    '/navigation/start', '/navigation/stop', '/navigation/pause', '/navigation/resume',
    '/navigation/load_path', '/route/*', '/tour/*', '/guide/*', '/interaction/*',
    '/control/set_mode', '/control/estop/engage', '/control/estop/release', '/docking/*',
    '/path_follower_node/get_parameters', '/path_follower_node/set_parameters', '/rosapi/*',
)  # fmt: skip
WEB_ROBOT_SERVICES = (
    'stand_up', 'sit_down', 'emergency_stop', 'gait_*', 'hello', 'nod', 'bow', 'look_left',
    'look_right', 'stop_action',
)  # fmt: skip


def glob_list(patterns) -> str:
    """Return patterns in rosbridge's glob parameter format: "['a', 'b']"."""
    return '[' + ', '.join(f"'{p}'" for p in patterns) + ']'


def web_globs(ns: str) -> dict:
    """Return the rosbridge / rosapi whitelist for the robot namespace ns."""
    topics = [*WEB_TOPICS, *(f'/{ns}/{name}' for name in WEB_ROBOT_TOPICS)]
    services = [*WEB_SERVICES, *(f'/{ns}/{name}' for name in WEB_ROBOT_SERVICES)]
    return {
        'topics_glob': glob_list(topics),
        'services_glob': glob_list(services),
        'params_glob': glob_list(['/path_follower_node/*']),
    }
