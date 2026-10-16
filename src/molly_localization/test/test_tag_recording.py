"""Tag file helpers: atomic writes, empty files and merges keep the other tags."""

import json

import numpy as np
import pytest

tag_recording = pytest.importorskip('molly_localization.utils.tag_recording')


def test_create_empty_file_never_overwrites(tmp_path):
    """create_empty_file makes {} once and leaves an existing file alone."""
    path = tmp_path / 'a' / 'tags.json'
    tag_recording.create_empty_file(str(path))
    assert json.loads(path.read_text()) == {}
    path.write_text('{"keep": {"x": 1}}')
    tag_recording.create_empty_file(str(path))
    assert json.loads(path.read_text()) == {'keep': {'x': 1}}


def test_merge_keeps_other_tags_and_backs_up(tmp_path):
    """Merging a tag keeps the others, writes a .bak and leaves no temporary file."""
    path = tmp_path / 'tags.json'
    path.write_text(json.dumps({'old': {'x': 1.0, 'y': 0, 'z': 0, 'qx': 0, 'qy': 0, 'qz': 0,
                                        'qw': 1}}))
    m = np.eye(4)
    m[:3, 3] = (2.0, 3.0, 0.5)
    merged = tag_recording.merge_into_file(str(path), {'new': m})
    assert set(merged) == {'old', 'new'}
    assert json.loads(path.read_text())['new']['x'] == 2.0
    assert (tmp_path / 'tags.json.bak').exists()
    assert not (tmp_path / 'tags.json.tmp').exists()
