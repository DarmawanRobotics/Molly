"""
POIs and tours follow the map when a mapping reset archives a site.

A reset (dharma_mapping `mapping/start {reset: true}`) moves the site's map, tags and routes into
a new `<site>/backup-<time>/` and starts a new map frame. POIs are poses in the old frame: kept,
they would send the robot to wrong places, so they move into that backup too.
"""

from __future__ import annotations

import os
import shutil

from molly_guide.core.site.store import FILES

ARCHIVE_PREFIX = 'backup-'
RESET_MARKERS = ('map.yaml', 'map.pgm', 'tags.json')


def backups(root: str) -> set[str]:
    """Return the names of the backup directories of a site directory."""
    try:
        names = os.listdir(root)
    except OSError:
        return set()
    return {
        n for n in names if n.startswith(ARCHIVE_PREFIX) and os.path.isdir(os.path.join(root, n))
    }


def is_reset(backup_dir: str) -> bool:
    """Return True when a backup holds what a mapping reset archives (map or tags)."""
    return any(
        os.path.exists(os.path.join(backup_dir, name + suffix))
        for name in RESET_MARKERS
        for suffix in ('', '.bak')
    )


def new_resets(root: str, known: set[str]) -> list[str]:
    """Return the backups made since `known` was taken that come from a reset, oldest first."""
    fresh = sorted(backups(root) - set(known))
    return [n for n in fresh if is_reset(os.path.join(root, n))]


def archive_content(root: str, backup: str) -> list[str]:
    """Move the POI and tour files (and their .bak) into root/backup; return what moved."""
    moved = []
    target = os.path.join(root, backup)
    for name in FILES:
        for item in (name, name + '.bak'):
            source = os.path.join(root, item)
            if os.path.exists(source) and not os.path.exists(os.path.join(target, item)):
                shutil.move(source, os.path.join(target, item))
                moved.append(item)
    return moved
