#!/bin/bash
# Usage: script/start_tmux.sh [guide|mapping]   (development; production: molly.service)
#   guide:   the site's map, localization from AprilTags, path finder, tours, interaction
#   mapping: same, FAST-LIO keeps its PCD; start mapping with `mapping/start` (command pane)
# One pane per subsystem with the package default parameters (no site file); for exactly what the
# service runs use `ros2 launch molly robot.launch.py`. Settings come from
# /etc/molly/molly.env (script/env.sh) unless set in the environment:
# Env: MODE, SITE, SITES_DIR, ODOMETRY (fast_lio | leg_kilo), ROUTE, ROBOT_NS, ROBOT_BRINGUP
#      (robot package, <package>[/<file>.launch.py], empty: started elsewhere), ROBOT_PRESTART,
#      VISITORS, SPEECH, GREETER, CONVERSATION, WEB, RVIZ=0 (headless), DETACH=1 (do not
#      attach)
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
# shellcheck source=script/env.sh
source "$WS/script/env.sh" >/dev/null
set -u
MODE="${1:-${MODE:-guide}}"
case "$MODE" in patrol) MODE=guide ;; teach) MODE=mapping ;; esac   # Pashupati's names
SITE="${SITE:-}"
SITES_DIR="${SITES_DIR:-$WS/map}"
ODOMETRY="${ODOMETRY:-fast_lio}"
ROUTE="${ROUTE:-}"
ROBOT_NS="${ROBOT_NS:-robot}"
ROBOT_BRINGUP="${ROBOT_BRINGUP:-}"
VISITORS="${VISITORS:-true}"
SPEECH="${SPEECH:-true}"
GREETER="${GREETER:-true}"
CONVERSATION="${CONVERSATION:-false}"
WEB="${WEB:-true}"
RVIZ="${RVIZ:-1}"
DETACH="${DETACH:-0}"
SESSION=molly

if [[ "$MODE" != "guide" && "$MODE" != "mapping" ]]; then
    echo "usage: $0 [guide|mapping]" >&2
    exit 1
fi
SAVE_MAP=$([[ "$MODE" == "mapping" ]] && echo true || echo false)
truthy() { [[ "${1,,}" =~ ^(1|true|yes|on)$ ]]; }

# Robot-specific preparation, e.g. stopping a vendor autostart that would fight for the robot
if [[ -n "${ROBOT_PRESTART:-}" ]]; then
    bash -c "$ROBOT_PRESTART" || true
fi
tmux kill-session -t "$SESSION" 2>/dev/null || true

SETUP="source $WS/script/env.sh && cd $WS"

run() {  # run <pane> <command>
    tmux send-keys -t "$SESSION:$1" "$SETUP && $2" C-m
}

# Window 1: robot, odometry + localization, perception
tmux new-session -d -s "$SESSION" -n robot
tmux split-window -v -t "$SESSION:robot"
tmux split-window -h -t "$SESSION:robot.0"
tmux split-window -h -t "$SESSION:robot.2"
if [[ -n "$ROBOT_BRINGUP" ]]; then
    ROBOT_PKG="${ROBOT_BRINGUP%%/*}"
    ROBOT_LAUNCH="bringup.launch.py"
    [[ "$ROBOT_BRINGUP" == */* ]] && ROBOT_LAUNCH="${ROBOT_BRINGUP#*/}"
    run robot.0 "ros2 launch $ROBOT_PKG $ROBOT_LAUNCH namespace:=$ROBOT_NS"
else
    tmux send-keys -t "$SESSION:robot.0" "# ROBOT_BRINGUP unset: start the robot driver here" C-m
fi
sleep 3
run robot.1 "ros2 launch molly_odometry odometry.launch.py method:=$ODOMETRY save_map:=$SAVE_MAP \
localization:=true leg_odom_topic:=/$ROBOT_NS/odom sites_dir:=$SITES_DIR"
run robot.2 "ros2 launch molly_perception perception.launch.py visitors:=$VISITORS"
if [[ "$RVIZ" == "1" ]]; then
    run robot.3 "ros2 run rviz2 rviz2 -d \$(ros2 pkg prefix molly_rviz)/share/molly_rviz/rviz/robot.rviz \
--ros-args -r battery:=/$ROBOT_NS/battery -r cmd_vel:=/$ROBOT_NS/cmd_vel \
-r diagnostics:=/$ROBOT_NS/diagnostics"
fi
tmux select-layout -t "$SESSION:robot" tiled

# Window 2: sites and mapping, navigation with the path finder, bridge, command pane
tmux new-window -t "$SESSION" -n nav
tmux split-window -v -t "$SESSION:nav"
tmux split-window -h -t "$SESSION:nav.0"
tmux split-window -h -t "$SESSION:nav.2"
run nav.0 "ros2 launch molly_mapping mapping.launch.py sites_dir:=$SITES_DIR site:=$SITE"
run nav.1 "ros2 launch molly_navigation navigation.launch.py waypoints_file:=$ROUTE \
cmd_vel_topic:=/bridge/nav_cmd_vel battery_topic:=/$ROBOT_NS/battery"
run nav.2 "ros2 launch molly_bridge bridge.launch.py robot_ns:=$ROBOT_NS"
tmux send-keys -t "$SESSION:nav.3" "$SETUP" C-m
if [[ "$MODE" == "mapping" ]]; then
    tmux send-keys -t "$SESSION:nav.3" "# ros2 service call /mapping/start molly_interfaces/srv/StartMapping \"{site: '${SITE:-new_site}', reset: true}\"" C-m
else
    tmux send-keys -t "$SESSION:nav.3" "# ros2 service call /localization/start std_srvs/srv/Trigger" C-m
fi

# Window 3: guide, interaction, health and the dashboard's web endpoints
tmux new-window -t "$SESSION" -n guide
tmux split-window -v -t "$SESSION:guide"
tmux split-window -h -t "$SESSION:guide.0"
run guide.0 "ros2 launch molly_guide guide.launch.py"
run guide.1 "ros2 launch molly_interaction interaction.launch.py robot_ns:=$ROBOT_NS \
speech:=$SPEECH greeter:=$GREETER conversation:=$CONVERSATION"
run guide.2 "ros2 launch molly_health health.launch.py"
if truthy "$WEB"; then
    tmux split-window -h -t "$SESSION:guide.2"
    run guide.3 "ros2 launch rosbridge_server rosbridge_websocket_launch.xml & \
ros2 run web_video_server web_video_server"
fi

tmux select-window -t "$SESSION:robot"
if [[ "$DETACH" != "1" ]]; then
    tmux attach-session -t "$SESSION"
fi
