"""
Pure helpers of the bringup: argument validation and parameter files.

No launch or ROS imports, so everything here is unit-tested without a robot.
"""

from __future__ import annotations

import os
import re
import shutil
import tempfile

import yaml

from dharma_utils.config import ConfigError, build, entries
from dharma_utils.io import valid_name
from dharma_utils.robot import RobotProfile

DEFAULT_ROBOT_NS = 'robot'
ODOMETRY_METHODS = ('fast_lio', 'leg_kilo', 'external')  # external: simulator, own estimator
NAMESPACE_RE = re.compile(r'^[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*$')
# robot_driver: '<package>' or '<package>/<file>.launch.(py|xml|yaml)'
DRIVER_RE = re.compile(
    r'^([A-Za-z][A-Za-z0-9_]*)(?:/([A-Za-z0-9_.-]+\.launch\.(?:py|xml|yaml)))?$'
)
DEFAULT_DRIVER_LAUNCH = 'bringup.launch.py'
WILDCARD = '/**'
ROBOT_PREFIX = 'robot.'


class BringupError(ValueError):
    """A launch argument or a parameter file is invalid."""


def truthy(value) -> bool:
    """Interpret a launch argument as a bool."""
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def resolve_namespace(value: str) -> str:
    """Return the robot driver namespace without slashes around it, e.g. 'robot'."""
    ns = str(value).strip().strip('/') or DEFAULT_ROBOT_NS
    if not NAMESPACE_RE.match(ns):
        raise BringupError(f'robot_ns:={value!r} is not a valid ROS namespace')
    return ns


def resolve_odometry(value: str) -> str:
    """Return a valid odometry method (empty: fast_lio)."""
    method = str(value).strip().lower() or ODOMETRY_METHODS[0]
    if method not in ODOMETRY_METHODS:
        raise BringupError(f'odometry:={value!r}, use one of {", ".join(ODOMETRY_METHODS)}')
    return method


def resolve_detector(value: str) -> str:
    """Return the tag detector: apriltag or aruco."""
    detector = (value or 'apriltag').strip().lower()
    if detector not in ('apriltag', 'aruco'):
        raise BringupError(f'fiducials_detector: {value!r}, use apriltag or aruco')
    return detector


def resolve_site(value: str) -> str:
    """Return a site name, or '' for the one in the parameter files (or the last one)."""
    site = str(value).strip()
    if site and not valid_name(site):
        raise BringupError(f'site:={value!r}, use 1-48 of a-z, 0-9, _ and -')
    return site


def resolve_driver(value: str) -> tuple[str, str] | None:
    """Parse robot_driver '<package>[/<file>.launch.py]' -> (package, file); None when empty."""
    text = str(value).strip()
    if not text:
        return None
    match = DRIVER_RE.match(text)
    if not match:
        raise BringupError(
            f"robot_driver:={value!r}, use '<package>' or '<package>/<file>.launch.py'"
        )
    return match.group(1), match.group(2) or DEFAULT_DRIVER_LAUNCH


def existing_file(name: str, path: str) -> str:
    """Return the expanded path of a set file argument, '' when empty; fail when missing."""
    path = os.path.expanduser(str(path).strip())
    if path and not os.path.isfile(path):
        raise BringupError(f'{name}:={path} does not exist')
    return path


def load_params(path: str) -> dict:
    """Read a ROS 2 parameter file: {node or '/**': {'ros__parameters': {...}}}."""
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as error:
        raise BringupError(f'{path}: {error}') from error
    if not isinstance(data, dict):
        raise BringupError(f'{path}: expected a mapping of node names')
    for name, section in data.items():
        if not isinstance(section, dict) or not isinstance(section.get('ros__parameters'), dict):
            raise BringupError(f'{path}: "{name}" needs a ros__parameters mapping')
    return data


def flatten(values: dict, prefix: str = '') -> dict:
    """Return nested parameters as dotted names: {'a': {'b': 1}} -> {'a.b': 1}."""
    flat = {}
    for key, value in values.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            flat.update(flatten(value, f'{name}.'))
        else:
            flat[name] = value
    return flat


def robot_profile(path: str) -> RobotProfile:
    """Read and validate the robot profile (the robot.* parameters of the '/**' section)."""
    data = load_params(path)
    params = flatten(data.get(WILDCARD, {}).get('ros__parameters', {}))
    values = {k: v for k, v in params.items() if k.startswith(ROBOT_PREFIX)}
    if not values:
        raise BringupError(f'{path}: no robot.* parameters under "{WILDCARD}"')
    known = {entry.name for entry in entries(RobotProfile, ROBOT_PREFIX)}
    unknown = sorted(set(values) - known)
    if unknown:
        raise BringupError(f'{path}: unknown robot parameters {", ".join(unknown)} (typo?)')
    try:
        return build(RobotProfile, values, ROBOT_PREFIX)
    except ConfigError as error:
        raise BringupError(f'{path}: {error}') from error


def unknown_sections(data: dict, nodes: list[str]) -> list[str]:
    """Return the sections of a parameter file that match no started node (typos)."""
    known = {n.lstrip('/') for n in nodes}
    return [name for name in data if name != WILDCARD and name.lstrip('/') not in known]


def shared_parameters(data: dict) -> list[str]:
    """
    Return the '/**' parameters of a parameter file other than the robot profile.

    Foxy applies a node's own section after '/**' whatever the file order, so a '/**' value
    loses there against the same parameter in a package's defaults: put overrides under the
    node name.
    """
    params = flatten(data.get(WILDCARD, {}).get('ros__parameters', {}))
    return sorted(k for k in params if not k.startswith(ROBOT_PREFIX))


def merge(base: dict, extra: dict) -> dict:
    """Return base with extra merged in, recursively; extra wins (lists are replaced)."""
    out = dict(base)
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = merge(out[key], value)
        else:
            out[key] = value
    return out


def merge_files(paths: list[str]) -> dict:
    """Merge parameter files in order (later files win, parameter by parameter)."""
    data: dict = {}
    for path in paths:
        if path:
            data = merge(data, load_params(path))
    return data


def with_expected_nodes(data: dict, node: str, expected: list[str]) -> dict:
    """Add the started nodes to `nodes.expected` of the health monitor (keeps listed ones)."""
    params = data.get(node, {}).get('ros__parameters', {})
    listed = params.get('nodes', {}).get('expected', [])
    names = sorted({*(n for n in listed if n), *expected})
    return merge(data, {node: {'ros__parameters': {'nodes': {'expected': names}}}})


def with_sites_dir(data: dict, nodes, sites_dir: str) -> dict:
    """Give these nodes the sites directory of the launch (nothing when empty)."""
    if not sites_dir:
        return data
    for node in (nodes,) if isinstance(nodes, str) else nodes:
        data = merge(data, {node: {'ros__parameters': {'sites_dir': sites_dir}}})
    return data


def with_route(data: dict, node: str, route: str) -> dict:
    """Give the route loader the route of the launch, inside sites_dir (nothing when empty)."""
    if not route:
        return data
    return merge(data, {node: {'ros__parameters': {'route': route}}})


def seed_site(sites_dir: str, site: str, source: str) -> list[str]:
    """Copy the demo POIs and tours into a site that has none yet; return what was copied."""
    if not site or not os.path.isdir(source):
        return []
    root = os.path.join(os.path.expanduser(sites_dir), site)
    os.makedirs(root, exist_ok=True)
    copied = []
    for name in sorted(os.listdir(source)):
        target = os.path.join(root, name)
        if name.endswith('.json') and not os.path.exists(target):
            shutil.copyfile(os.path.join(source, name), target)
            copied.append(name)
    return copied


def package_version(package_xml: str) -> str:
    """Return the <version> of a package.xml, or 'unknown'."""
    try:
        with open(package_xml, encoding='utf-8') as f:
            match = re.search(r'<version>([^<]+)</version>', f.read())
    except OSError:
        return 'unknown'
    return match.group(1).strip() if match else 'unknown'


def write_params(data: dict, directory: str | None = None) -> str:
    """Write a merged parameter file for this launch; return its path."""
    fd, path = tempfile.mkstemp(prefix='molly_bringup_', suffix='.yaml', dir=directory)
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
    return path
