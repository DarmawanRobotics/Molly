"""Tests of the MediaMTX setup for the handheld's WebRTC video."""

from __future__ import annotations

import base64
import hashlib
import os
import shlex
import stat

from molly_bridge.core import mediamtx as video
from molly_bridge.core.mediamtx import node_parameter
import yaml


def fake_mediamtx(directory, version='v1.20.1') -> str:
    """Write an executable that answers --version like MediaMTX."""
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, 'mediamtx')
    with open(path, 'w') as f:
        f.write(f'#!/bin/sh\necho {version}\n')
    os.chmod(path, 0o755)
    return path


def test_mediamtx_is_found_with_its_version(tmp_path):
    """MEDIAMTX names the binary; its version decides whether our configuration fits."""
    binary = fake_mediamtx(str(tmp_path))
    assert video.find_mediamtx(binary) == binary
    assert video.find_mediamtx(str(tmp_path / 'missing')) is None
    assert video.mediamtx_version(binary) == (1, 20, 1)
    assert video.mediamtx_version(fake_mediamtx(str(tmp_path / 'old'), 'v1.9.0')) < (1, 11, 0)
    assert video.mediamtx_version(fake_mediamtx(str(tmp_path / 'odd'), 'garbage')) is None
    assert video.mediamtx_version(str(tmp_path / 'missing')) is None
    assert video.version_text((1, 9, 0)) == 'v1.9.0'


def test_gstreamer_check(monkeypatch):
    """Without python3-gi the launch offers no WebRTC (the encoder could not start)."""
    monkeypatch.setattr(video, 'find_spec', lambda name: None)
    assert 'python3-gi' in video.gstreamer_missing()


def test_only_the_handheld_may_watch_and_only_this_computer_publish(tmp_path):
    """Reading needs the token (stored as a hash), publishing comes from the loopback only."""
    command = video.node_command(
        '/ws/install/molly_bridge/lib/molly_bridge/video_stream_node',
        ['/ws/my params.yaml', '/etc/molly/site.yaml'],
        video.rtsp_url(),
    )
    assert shlex.split(command) == [
        '/ws/install/molly_bridge/lib/molly_bridge/video_stream_node',
        '--ros-args',
        '--params-file',
        '/ws/my params.yaml',
        '--params-file',
        '/etc/molly/site.yaml',
        '-p',
        'rtsp_url:=rtsp://127.0.0.1:8654/camera',
    ]
    token = 'tok:en/%'  # characters MediaMTX refuses in a plain password
    config = video.mediamtx_config(command, token, (1, 20, 1))
    publisher, reader = config['authInternalUsers']
    digest = base64.b64encode(hashlib.sha256(token.encode()).digest()).decode()
    assert reader == {
        'user': 'teleop',
        'pass': f'sha256:{digest}',
        'ips': [],
        'permissions': [{'action': 'read', 'path': 'camera'}],
    }
    assert publisher['ips'] == ['127.0.0.1', '::1']
    assert publisher['permissions'] == [{'action': 'publish', 'path': 'camera'}]
    assert config['rtspAddress'] == '127.0.0.1:8654' and config['rtspTransports'] == ['tcp']
    assert not (config['rtmp'] or config['hls'] or config['srt'] or config['moq'])
    assert config['paths']['camera']['runOnDemand'] == command
    assert 'moq' not in video.mediamtx_config(command, token, (1, 18, 0))  # unknown key there
    anyone = video.mediamtx_config(command, '', (1, 20, 1))['authInternalUsers'][1]
    assert anyone['user'] == 'any' and anyone['pass'] == ''


def test_config_file_is_private(tmp_path):
    """The file holds the token hash: readable by the robot user only, rewritten in place."""
    config = video.mediamtx_config('true', 'secret', (1, 20, 1))
    path = str(tmp_path / 'home' / '.molly' / 'mediamtx.yml')
    for _ in range(2):
        assert video.write_config(config, path) == path
        assert stat.S_IMODE(os.stat(path).st_mode) == 0o600
        with open(path) as f:
            assert yaml.safe_load(f) == config
    assert os.listdir(os.path.dirname(path)) == ['mediamtx.yml']


def test_node_parameter_follows_ros_precedence(tmp_path):
    """Later files win; the node's own section wins over '/**'; missing files are skipped."""
    defaults = tmp_path / 'defaults.yaml'
    defaults.write_text('teleop_udp_node:\n  ros__parameters:\n    token: ""\n    port: 9870\n')
    site = tmp_path / 'site.yaml'
    site.write_text(
        'teleop_udp_node:\n  ros__parameters:\n    token: "change-me"\n'
        '/**:\n  ros__parameters:\n    token: everyone\n'
    )
    files = [str(defaults), str(site), str(tmp_path / 'missing.yaml')]
    assert node_parameter(files, 'teleop_udp_node', 'token') == 'change-me'
    assert node_parameter(files, 'teleop_udp_node', 'port') == 9870
    assert node_parameter(files, 'other_node', 'token') == 'everyone'
    assert node_parameter(files, 'other_node', 'port', 1) == 1
