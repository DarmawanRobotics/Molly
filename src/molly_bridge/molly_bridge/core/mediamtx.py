"""
WebRTC video for the handheld: the MediaMTX server the bridge launch starts.

The handheld opens the video with WHEP (WebRTC over HTTP) on WEBRTC_PORT. MediaMTX then runs
video_stream_node (runOnDemand), which encodes the camera to H.264 and publishes it over RTSP on
the loopback interface; when nobody watches any more, MediaMTX stops it again. Reading needs the
teleop token (HTTP basic auth, user 'teleop'), publishing is only allowed from this computer.

No launch or ROS imports, so everything here is unit-tested without a robot.
"""

from __future__ import annotations

import base64
import hashlib
from importlib.util import find_spec
import os
import re
import shlex
import shutil
import subprocess

import yaml

PATH = 'camera'  # the MediaMTX path: WHEP at http://<robot>:WEBRTC_PORT/camera/whep
USER = 'teleop'  # basic auth user of the handheld, the password is the teleop token
WEBRTC_PORT = 8890  # TCP, WHEP signalling
ICE_PORT = 8190  # UDP, the WebRTC media
RTSP_PORT = 8654  # video_stream_node -> MediaMTX, loopback only
# Ports differ from the MediaMTX defaults, so a MediaMTX the robot already runs can stay.
INSTALL_DIR = '/opt/molly/mediamtx'  # script/setup.sh install webrtc
MIN_VERSION = (1, 11, 0)  # rtspTransports
MOQ_VERSION = (1, 19, 0)  # MoQ server, on by default from here: turned off


def find_mediamtx(explicit: str = ''):
    """Return the MediaMTX binary: explicit (MEDIAMTX) or the first one installed, else None."""
    if explicit.strip():
        candidates = [os.path.expanduser(explicit.strip())]
    else:
        candidates = [
            os.path.join(INSTALL_DIR, 'mediamtx'),
            shutil.which('mediamtx'),
            '/usr/local/bin/mediamtx',
            os.path.expanduser('~/mediamtx/mediamtx'),
        ]
    for path in candidates:
        if path and os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    return None


def mediamtx_version(binary: str):
    """Return the (major, minor, patch) version of a MediaMTX binary, or None."""
    try:
        result = subprocess.run(
            [binary, '--version'], capture_output=True, text=True, timeout=5, check=False
        )
    except (OSError, subprocess.SubprocessError):
        return None
    match = re.search(r'(\d+)\.(\d+)\.(\d+)', result.stdout + result.stderr)
    return tuple(int(part) for part in match.groups()) if match else None


def gstreamer_missing():
    """Return why video_stream_node cannot run here (no GStreamer for Python), or None."""
    if find_spec('gi') is None:
        return 'no GStreamer for Python (python3-gi)'
    return None


def password_hash(token: str) -> str:
    """Return the token as a MediaMTX sha256 credential (not plain text, any character)."""
    digest = hashlib.sha256(token.encode()).digest()
    return 'sha256:' + base64.b64encode(digest).decode()


def node_command(executable: str, params_files: list, rtsp_url: str) -> str:
    """Return the runOnDemand command line of video_stream_node (MediaMTX splits it like sh)."""
    args = [executable, '--ros-args']
    for path in params_files:
        args += ['--params-file', path]
    args += ['-p', f'rtsp_url:={rtsp_url}']
    return shlex.join(args)


def rtsp_url(port: int = RTSP_PORT) -> str:
    """Where video_stream_node publishes."""
    return f'rtsp://127.0.0.1:{port}/{PATH}'


def mediamtx_config(
    command: str,
    token: str,
    version: tuple,
    webrtc_port: int = WEBRTC_PORT,
    ice_port: int = ICE_PORT,
    rtsp_port: int = RTSP_PORT,
) -> dict:
    """
    Return the MediaMTX configuration: WebRTC out, RTSP in on loopback, nothing else.

    Without a token anyone on the network may watch, like the teleop link itself.
    """
    reader = {
        'user': USER if token else 'any',
        'pass': password_hash(token) if token else '',
        'ips': [],
        'permissions': [{'action': 'read', 'path': PATH}],
    }
    publisher = {
        'user': 'any',
        'pass': '',
        'ips': ['127.0.0.1', '::1'],
        'permissions': [{'action': 'publish', 'path': PATH}],
    }
    config = {
        'logLevel': 'info',
        'logDestinations': ['stdout'],
        'authMethod': 'internal',
        'authInternalUsers': [publisher, reader],
        'rtsp': True,
        'rtspTransports': ['tcp'],
        'rtspAddress': f'127.0.0.1:{rtsp_port}',
        'rtmp': False,
        'hls': False,
        'srt': False,
        'webrtc': True,
        'webrtcAddress': f':{webrtc_port}',
        'webrtcLocalUDPAddress': f':{ice_port}',
        'webrtcLocalTCPAddress': '',
        'webrtcIPsFromInterfaces': True,
        'paths': {
            PATH: {
                'runOnDemand': command,
                'runOnDemandRestart': True,
                'runOnDemandStartTimeout': '15s',
                'runOnDemandCloseAfter': '10s',
            }
        },
    }
    if tuple(version) >= MOQ_VERSION:
        config['moq'] = False
    return config


def write_config(config: dict, path: str) -> str:
    """Write the configuration readable by this user only (it holds the token hash)."""
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = f'{path}.tmp'
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w') as f:
        f.write('# Written by molly_bridge bridge.launch.py at every start, edits are lost.\n')
        yaml.safe_dump(config, f, sort_keys=False)
    os.chmod(tmp, 0o600)  # an older file with other permissions
    os.replace(tmp, path)
    return path


def version_text(version) -> str:
    """Return 'v1.20.1' or 'an unknown version'."""
    return 'v' + '.'.join(map(str, version)) if version else 'an unknown version'


def node_parameter(files: list, node: str, name: str, default=None):
    """
    Return one parameter of a node as ROS would see it from these parameter files.

    Later files win, and in one file the node's own section wins over '/**'. Unreadable files
    are skipped. Dotted names are looked up flat and nested.
    """
    value = default
    for path in files:
        try:
            with open(path, encoding='utf-8') as f:
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


def usable(explicit: str = '') -> tuple[str | None, tuple | None, str]:
    """Return (MediaMTX binary, version, '') or (None, None, why WebRTC video is off)."""
    found = find_mediamtx(explicit)
    if found is None:
        return None, None, 'MediaMTX not found (script/setup.sh install webrtc)'
    version = mediamtx_version(found)
    if version is None or version < MIN_VERSION:
        return None, None, f'{found} is {version_text(version)}, it needs v1.11 or newer'
    missing = gstreamer_missing()
    if missing:
        return None, None, f'{missing} (script/setup.sh install webrtc)'
    return found, version, ''
