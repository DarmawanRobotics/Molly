"""
Pure helpers for the Molly bringup: argument validation and site configuration.

No launch or ROS imports, so everything here is unit-tested without a robot.
"""

import os
import re
import xml.etree.ElementTree as ElementTree

import yaml

MODES = {'patrol': 'patrol', 'teach': 'teach', 'mapping': 'teach'}  # mapping: old name of teach
ODOMETRY_METHODS = ('fast_lio', 'leg_kilo', 'external')  # external: simulator / own estimator
NAMESPACE_RE = re.compile(r'^[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*$')
# robot_bringup: '<package>' or '<package>/<file>.launch.(py|xml|yaml)'
ROBOT_BRINGUP_RE = re.compile(
    r'^([A-Za-z][A-Za-z0-9_]*)(?:/([A-Za-z0-9_.-]+\.launch\.(?:py|xml|yaml)))?$'
)
DEFAULT_ROBOT_LAUNCH = 'bringup.launch.py'

# Every node robot.launch.py starts, by node name (the keys of a site configuration file).
NODES = (
    'fastlio_mapping_node',
    'odometry_gate',
    'odometry_monitor_node',
    'route_recorder_node',
    'localization_node',
    'lidar_sector_node',
    'apriltag_throttle',
    'apriltag',
    'person_detector_node',
    'path_loader_node',
    'obstacle_avoidance_node',
    'path_follower_node',
    'control_mux_node',
    'teleop_udp_node',
    'video_stream_node',  # started by MediaMTX while the handheld watches the WebRTC video
    'health_monitor_node',
)


class BringupError(ValueError):
    """A launch argument or the site configuration is invalid."""


def truthy(value) -> bool:
    """Interpret a launch argument as a bool."""
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def resolve_mode(value: str) -> str:
    """Return 'patrol' or 'teach' ('mapping' is accepted as the old name of teach)."""
    mode = str(value).strip().lower()
    if mode not in MODES:
        raise BringupError(f"mode:={value!r}, use 'patrol' or 'teach'")
    return MODES[mode]


def resolve_odometry(value: str) -> str:
    """Return a valid odometry method."""
    method = str(value).strip().lower()
    if method not in ODOMETRY_METHODS:
        raise BringupError(f'odometry:={value!r}, use one of {", ".join(ODOMETRY_METHODS)}')
    return method


def resolve_namespace(value: str) -> str:
    """Return the robot driver namespace without slashes around it, e.g. 'robot'."""
    ns = str(value).strip().strip('/')
    if not NAMESPACE_RE.match(ns):
        raise BringupError(f'robot_ns:={value!r} is not a valid ROS namespace')
    return ns


def resolve_robot_bringup(value: str):
    """
    Parse robot_bringup: '<package>' or '<package>/<file>.launch.py' -> (package, launch file).

    The robot package (driver, URDF, sensors) is the only robot-specific part of the stack, see
    src/xtras/README.md. None when empty: the robot is started elsewhere (simulator, vendor
    service).
    """
    text = str(value).strip()
    if not text:
        return None
    match = ROBOT_BRINGUP_RE.match(text)
    if not match:
        raise BringupError(
            f"robot_bringup:={value!r}, use '<package>' or '<package>/<file>.launch.py'"
        )
    return match.group(1), match.group(2) or DEFAULT_ROBOT_LAUNCH


def load_site_config(path: str):
    """
    Read a site configuration file; return (path list for node parameters, warnings).

    The file is a normal ROS 2 parameter file keyed by node name, applied on top of the package
    defaults of every node. A missing file is fine (no overrides); a broken one stops the launch
    so a typo cannot silently fall back to defaults.
    """
    if not path or not os.path.isfile(path):
        return [], []
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as error:
        raise BringupError(f'site config {path}: {error}') from error
    if not isinstance(data, dict):
        raise BringupError(f'site config {path}: expected a mapping of node names')
    warnings = []
    for name, section in data.items():
        if not isinstance(section, dict) or not isinstance(section.get('ros__parameters'), dict):
            raise BringupError(f'site config {path}: "{name}" needs a ros__parameters mapping')
        if name not in NODES and name != '/**':
            warnings.append(f'site config {path}: no node is called "{name}" (typo?)')
    return [path], warnings


def node_parameter(files: list, node: str, name: str, default=None):
    """
    Return one parameter of a node as ROS would see it from these parameter files.

    Later files win, and in one file the node's own section wins over '/**'. Unreadable files
    are skipped (load_site_config already rejected broken site files).
    """
    value = default
    for path in files:
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError):
            continue
        if not isinstance(data, dict):
            continue
        for key in ('/**', node, f'/{node}'):
            section = data.get(key)
            params = section.get('ros__parameters') if isinstance(section, dict) else None
            if isinstance(params, dict) and name in params:
                value = params[name]
    return value


def expected_nodes(started: list[str], odometry: str) -> list[str]:
    """Return every node the health monitor must see: launched ones plus the odometry nodes."""
    estimator = {'fast_lio': ['fastlio_mapping_node'], 'leg_kilo': ['legkilo']}.get(odometry, [])
    extra = [*estimator, 'odometry_monitor_node', 'route_recorder_node', 'apriltag']
    return sorted({*started, *extra, 'health_monitor_node'})


def non_empty(overrides: dict) -> dict:
    """Drop empty overrides so an unset launch argument never hides the site configuration."""
    return {k: v for k, v in overrides.items() if v not in ('', None)}


def package_version(package_xml: str) -> str:
    """Return the <version> of a package.xml, or 'unknown'."""
    try:
        return ElementTree.parse(package_xml).getroot().findtext('version') or 'unknown'
    except (OSError, ElementTree.ParseError):
        return 'unknown'
