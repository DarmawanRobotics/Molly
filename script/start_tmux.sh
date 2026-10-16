#!/bin/bash
# Usage: script/start_tmux.sh [teach|patrol]   (development; production: molly.service)
#   teach:  odometry keeps the map, record a route with the RViz panel or the handheld
#   patrol: map off, route loaded, navigation ready (start it from the panel)
# One pane per subsystem with the package default parameters (no site file); for exactly what the
# service runs use `ros2 launch molly robot.launch.py`. Settings come from
# /etc/molly/molly.env (script/env.sh) unless set in the environment:
# Env: MODE, ODOMETRY (fast_lio | leg_kilo), ROUTE, TAGS, ROBOT_NS, ROBOT_BRINGUP (robot package,
#      <package>[/<file>.launch.py], empty: started elsewhere), ROBOT_PRESTART, PEOPLE, INSPECTION,
#      CC_URL + CC_TOKEN (command center), RVIZ=0 (headless), DETACH=1 (do not attach)
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
# shellcheck source=script/env.sh
source "$WS/script/env.sh" >/dev/null
set -u
MODE="${1:-${MODE:-patrol}}"
[[ "$MODE" == "mapping" ]] && MODE=teach   # old name
ODOMETRY="${ODOMETRY:-fast_lio}"
ROUTE="${ROUTE:-$WS/map/example/example_waypoint.csv}"
TAGS="${TAGS:-$WS/map/example/tag_config.json}"
ROBOT_NS="${ROBOT_NS:-robot}"
ROBOT_BRINGUP="${ROBOT_BRINGUP:-}"
CC_URL="${CC_URL:-}"
CC_TOKEN="${CC_TOKEN:-}"
PEOPLE="${PEOPLE:-true}"
INSPECTION="${INSPECTION:-true}"
RVIZ="${RVIZ:-1}"
DETACH="${DETACH:-0}"
SESSION=molly

if [[ "$MODE" != "teach" && "$MODE" != "patrol" ]]; then
    echo "usage: $0 [teach|patrol]" >&2
    exit 1
fi
SAVE_MAP=$([[ "$MODE" == "teach" ]] && echo true || echo false)
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

# Window 1: robot, odometry, perception
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
localization:=true tags_config_file:=$TAGS leg_odom_topic:=/$ROBOT_NS/odom"
run robot.2 "ros2 launch molly_perception perception.launch.py people:=$PEOPLE"
if [[ "$RVIZ" == "1" ]]; then
    run robot.3 "ros2 run rviz2 rviz2 -d \$(ros2 pkg prefix molly_rviz)/share/molly_rviz/rviz/robot.rviz \
--ros-args -r battery:=/$ROBOT_NS/battery -r cmd_vel:=/$ROBOT_NS/cmd_vel \
-r motor_temperature:=/$ROBOT_NS/motor_temperature -r diagnostics:=/$ROBOT_NS/diagnostics"
fi
tmux select-layout -t "$SESSION:robot" tiled

# Window 2: navigation + command pane
tmux new-window -t "$SESSION" -n nav
tmux split-window -v -t "$SESSION:nav"
if [[ "$MODE" == "patrol" ]]; then
    run nav.0 "ros2 launch molly_navigation navigation.launch.py waypoints_file:=$ROUTE \
cmd_vel_topic:=/bridge/nav_cmd_vel battery_topic:=/$ROBOT_NS/battery"
fi
tmux split-window -h -t "$SESSION:nav.0"
run nav.1 "ros2 launch molly_bridge bridge.launch.py robot_ns:=$ROBOT_NS"
tmux split-window -v -t "$SESSION:nav.1"
FLEET_ARGS="robot_ns:=$ROBOT_NS"
[[ -n "$CC_URL" ]] && FLEET_ARGS+=" server_url:=$CC_URL"
[[ -n "$CC_TOKEN" ]] && FLEET_ARGS+=" token:=$CC_TOKEN"
run nav.2 "ros2 launch robot_fleet fleet.launch.py $FLEET_ARGS"
tmux send-keys -t "$SESSION:nav.3" "$SETUP" C-m
tmux send-keys -t "$SESSION:nav.3" "# ros2 service call /localization/start std_srvs/srv/Trigger" C-m

# Window 3: health and, when enabled, on-robot inspection (Moondream)
tmux new-window -t "$SESSION" -n extra
run extra.0 "ros2 launch molly_health health.launch.py"
if truthy "$INSPECTION"; then
    tmux split-window -h -t "$SESSION:extra"
    run extra.1 "ros2 launch robot_inspection inspection.launch.py"
fi

tmux select-window -t "$SESSION:robot"
if [[ "$DETACH" != "1" ]]; then
    tmux attach-session -t "$SESSION"
fi
