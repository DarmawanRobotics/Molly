"""Taught routes the handheld lists and loads: CSV files inside the sites directory (no ROS)."""

from __future__ import annotations

import os

from dharma_utils.io import PathError, expand, resolve_inside

MAX_LISTED = 50


def list_routes(sites_dir: str, site: str) -> list[str]:
    """Return route files of the active site (else of every site), newest first, relative."""
    root = os.path.realpath(expand(sites_dir))
    top = os.path.join(root, site, 'routes') if site else root
    found = []
    for directory, _, files in os.walk(top):
        for name in files:
            if name.endswith('.csv'):
                path = os.path.join(directory, name)
                try:
                    found.append((os.path.getmtime(path), os.path.relpath(path, root)))
                except OSError:  # deleted while listing
                    continue
    return [path for _, path in sorted(found, reverse=True)[:MAX_LISTED]]


def route_path(sites_dir: str, file) -> str:
    """Return a route file from the client relative to sites_dir; only files inside it."""
    if not isinstance(file, str) or not file.strip() or not file.endswith('.csv'):
        raise ValueError('file must be a route CSV under the sites directory')
    root = os.path.realpath(expand(sites_dir))
    try:
        path = resolve_inside(root, file.strip())
    except PathError as error:
        raise ValueError(f'route must be a file under {sites_dir}') from error
    return os.path.relpath(path, root)
