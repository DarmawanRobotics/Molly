#!/usr/bin/env bash
# Development session: one tmux window per subsystem, from a yaml layout (tmuxp style).
#
#   scripts/start_tmux.sh [robot|sim|<layout.yaml>]    (default: robot)
#
# Layouts live in scripts/tmux/: session_name, environment (defaults, the caller's environment
# wins), setup (run in every pane first), windows -> panes -> shell_command; a pane with
# `when: VAR` runs only when VAR is set, otherwise it shows its `otherwise` hint. ${VAR} is
# replaced from the environment. Each pane runs one package launch with the bringup's robot
# profile and stack file; for exactly what the robot runs use `ros2 launch molly_bringup
# robot.launch.py`. Env: ROBOT_NS, ROBOT_DRIVER, ODOMETRY, SITE, SITES_DIR, DETACH=1 (do not
# attach), ROS_DISTRO (the one in /opt/ros, else humble).
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
installed=$(find /opt/ros -mindepth 1 -maxdepth 1 -type d -printf '%f\n' 2> /dev/null | sort | tail -n1)
export WS ROS_DISTRO="${ROS_DISTRO:-${installed:-humble}}"
LAYOUT="${1:-robot}"
[[ -f "$LAYOUT" ]] || LAYOUT="$WS/scripts/tmux/$LAYOUT.yaml"
if [[ ! -f "$LAYOUT" ]]; then
    layouts=("$WS"/scripts/tmux/*.yaml)
    layouts=("${layouts[@]##*/}")
    echo "no layout $LAYOUT (scripts/tmux: ${layouts[*]%.yaml})" >&2
    exit 1
fi
command -v tmux > /dev/null || { echo "tmux not found (scripts/setup.sh install tools)" >&2; exit 1; }

SESSION=$(
    python3 - "$LAYOUT" << 'PY'
"""Build the tmux session of a layout file; print its name."""

import os
import string
import subprocess
import sys

import yaml


def tmux(*args: str) -> str:
    return subprocess.run(['tmux', *args], check=True, capture_output=True, text=True).stdout.strip()


with open(sys.argv[1], encoding='utf-8') as f:
    layout = yaml.safe_load(f)
env = {k: str(v) for k, v in (layout.get('environment') or {}).items()}
env.update(os.environ)


def expand(text) -> str:
    return string.Template(str(text)).safe_substitute(env)


session = expand(layout['session_name'])
root = expand(layout.get('start_directory', env['WS']))
setup = expand(layout.get('setup', ''))
subprocess.run(['tmux', 'kill-session', '-t', session], capture_output=True, check=False)
for index, window in enumerate(layout['windows']):
    name = expand(window['window_name'])
    if index == 0:
        first = ['new-session', '-d', '-s', session, '-n', name, '-c', root]
    else:
        first = ['new-window', '-t', f'{session}:', '-n', name, '-c', root]
    pane = tmux(*first, '-P', '-F', '#{pane_id}')
    for number, spec in enumerate(window.get('panes') or [{}]):
        if number:
            pane = tmux('split-window', '-t', pane, '-c', root, '-P', '-F', '#{pane_id}')
            tmux('select-layout', '-t', pane, 'tiled')
        commands = spec.get('shell_command') or []
        commands = [commands] if isinstance(commands, str) else commands
        if spec.get('when') and not env.get(spec['when']):
            commands = [f'# {spec.get("otherwise", spec["when"] + " is not set")}']
        commands = [expand(c) for c in commands]
        notes = [c for c in commands if c.lstrip().startswith('#')]  # shown, not run
        run = ' && '.join(c for c in [setup, *(c for c in commands if c not in notes)] if c)
        for keys in [run, *notes] if run else notes:
            tmux('send-keys', '-t', pane, keys, 'C-m')
    tmux('select-layout', '-t', f'{session}:{name}', window.get('layout', 'tiled'))
tmux('select-window', '-t', f'{session}:{expand(layout["windows"][0]["window_name"])}')
print(session)
PY
)

echo "tmux session $SESSION ($LAYOUT)"
if [[ "${DETACH:-0}" != "1" ]]; then
    if [[ -n "${TMUX:-}" ]]; then tmux switch-client -t "$SESSION"; else tmux attach-session -t "$SESSION"; fi
fi
