# shellcheck shell=bash
# The environment of molly.service in an interactive shell (same RMW, domain, robot settings):
#
#   source ~/dev/Molly/script/env.sh      # e.g. alias molly='source ~/dev/Molly/script/env.sh'
#   ros2 topic list                            # sees the stack, not a vendor stack on another RMW
#   pnode molly_bridge teleop_udp_node bridge_params.yaml [ros args]
#
# pnode runs one node the way robot.launch.py does: package defaults, then SITE_CONFIG, then args.
# Env: MOLLY_ENV (/etc/molly/molly.env), ROS_DISTRO (humble)

MOLLY_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export MOLLY_WS
MOLLY_ENV="${MOLLY_ENV:-/etc/molly/molly.env}"
if [[ -r "$MOLLY_ENV" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "$MOLLY_ENV"
    set +a
fi
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}" OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# A ros2 daemon started from a shell with another RMW, domain or ROS_LOCALHOST_ONLY keeps that
# view (empty `ros2 topic list`): start it fresh in this environment.
ros2 daemon stop >/dev/null 2>&1 || true

if [[ -f "$MOLLY_WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$MOLLY_WS/install/setup.bash"
else
    echo "workspace not built yet: $MOLLY_WS/script/setup.sh install build" >&2
fi
# Same library order as run_robot.sh
if [[ -n "${LD_LIBRARY_PATH_FIRST:-}" ]]; then
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH_FIRST:${LD_LIBRARY_PATH:-}"
fi

pnode() {
    if [[ $# -lt 3 ]]; then
        echo "usage: pnode <package> <executable> <config.yaml> [ros args]" >&2
        return 1
    fi
    local pkg=$1 exe=$2 cfg=$3
    shift 3
    local site=()
    [[ -f "${SITE_CONFIG:-}" ]] && site=(--params-file "$SITE_CONFIG")
    ros2 run "$pkg" "$exe" --ros-args -r __node:="$exe" \
        --params-file "$MOLLY_WS/install/$pkg/share/$pkg/config/$cfg" "${site[@]}" "$@"
}

echo "molly env: $RMW_IMPLEMENTATION, domain $ROS_DOMAIN_ID, robot ns ${ROBOT_NS:-robot}"
