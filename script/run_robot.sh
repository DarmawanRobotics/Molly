#!/usr/bin/env bash
# Run the robot stack in the foreground (systemd ExecStart). Settings come from the environment,
# normally /etc/molly/molly.env; robot.launch.py reads every argument from it.
set -eo pipefail

WS="${WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

# Libraries found before ROS's own, e.g. a librealsense built from source in /usr/local/lib
# (the apt one under /opt/ros may not see a camera that the source build does).
if [[ -n "${LD_LIBRARY_PATH_FIRST:-}" ]]; then
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH_FIRST:${LD_LIBRARY_PATH:-}"
fi

# Without a battery-backed clock the computer boots with a wrong time and NTP steps it minutes
# later; odometry and TF stamps break on that jump. Wait for the first sync (TIME_SYNC_WAIT s,
# 0: don't wait), then start anyway: a robot without network keeps its own consistent time.
synced() { [[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" == yes ]]; }
if [[ "${TIME_SYNC_WAIT:-60}" -gt 0 ]] && command -v timedatectl >/dev/null && ! synced; then
    echo "waiting up to ${TIME_SYNC_WAIT:-60} s for the clock to synchronize (NTP)" >&2
    for ((i = 0; i < ${TIME_SYNC_WAIT:-60}; i++)); do
        synced && break
        sleep 1
    done
    synced && echo "clock synchronized: $(date)" >&2 \
        || echo "clock not synchronized, starting anyway: check timedatectl" >&2
fi

# Robot-specific preparation, e.g. stopping a vendor autostart that would fight for the robot.
if [[ -n "${ROBOT_PRESTART:-}" ]]; then
    bash -c "$ROBOT_PRESTART" || echo "ROBOT_PRESTART failed, starting anyway" >&2
fi

# numpy's OpenBLAS threads busy-wait between the small matrix ops the nodes run at
# 10-50 Hz and burn whole cores; one BLAS thread per process is plenty.
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}" OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"

export ROS_LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"
# Keep ROS logs from filling the disk: delete runs older than a week.
find "$ROS_LOG_DIR" -mindepth 1 -maxdepth 1 -mtime +7 -exec rm -rf {} + 2>/dev/null || true

# Version shown in the first log line and in the journal
MOLLY_GIT="$(git -C "$WS" describe --always --dirty --tags 2>/dev/null || echo unknown)"
export MOLLY_GIT

exec ros2 launch molly robot.launch.py
