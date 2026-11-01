#!/usr/bin/env bash
# Time synchronisation of a robot computer: chrony for the system clock, PTP for the LiDAR.
#
#   scripts/timesync.sh check [chrony|ptp]     # status, one line each (exit 0 = ok)
#   scripts/timesync.sh install [chrony|ptp]   # install and configure (default: both)
#
# Every stamp of the stack (LiDAR, IMU, camera, odometry, TF, the events sent to a server) must
# come from one clock, else TF is "too old", the odometry jumps and events land at the wrong time.
#
# chrony: the system clock follows TIME_SERVERS (space separated, e.g. the fleet server; empty:
#   the distribution's pool) and serves the robot's other computers in TIME_ALLOW (space
#   separated subnets, e.g. "192.168.123.0/24"). `local stratum 10` keeps serving them without
#   internet. Replaces systemd-timesyncd.
# ptp: ptp4l is the PTP master on the network interface of the LiDAR, so a Livox Mid-360 (PTPv2
#   over UDP) stamps its points with this computer's clock. The interface is PTP_IFACE, else the
#   one that routes to LIDAR_IP (or LIVOX_IP); without either, PTP is skipped.
#   PTP_TIMESTAMPING=software (default, any interface) or hardware (the interface's own clock,
#   kept on the system clock by phc2sys).
# TIME_MAX_OFFSET (0.05 s): the largest offset to the time source that check accepts.
# In a container both are the host's job: check says so, install does nothing.
set -euo pipefail

ETC="${TIMESYNC_ROOT:-}/etc"                  # TIMESYNC_ROOT: write into a test directory
SYSTEMD_DIR="$ETC/systemd/system"
CHRONY_CONF="$ETC/chrony/chrony.conf"
CHRONY_DROPIN="$ETC/chrony/conf.d/robot-timesync.conf"
PTP_CONF="$ETC/linuxptp/lidar-ptp.conf"
PTP_UNIT=lidar-ptp
PHC_UNIT=lidar-phc2sys
TIME_SERVERS="${TIME_SERVERS:-}"
TIME_ALLOW="${TIME_ALLOW:-}"
TIME_MAX_OFFSET="${TIME_MAX_OFFSET:-0.05}"
LIDAR_IP="${LIDAR_IP:-${LIVOX_IP:-}}"
PTP_IFACE="${PTP_IFACE:-}"
PTP_TIMESTAMPING="${PTP_TIMESTAMPING:-software}"
SUDO=$([[ "$(id -u)" -eq 0 || -n "${TIMESYNC_ROOT:-}" ]] && echo "" || echo "sudo")

in_container() { [[ -z "${TIMESYNC_ROOT:-}" && ( -f /.dockerenv || -f /run/.containerenv ) ]]; }
service() { $SUDO systemctl "$@"; }
write() { # write <file>: stdin into a root-owned file
    $SUDO mkdir -p "$(dirname "$1")"
    $SUDO tee "$1" >/dev/null
}

# ---------------------------------------------------------------- chrony

# `chronyc tracking` -> "<offset s, + = fast> <synchronised: 1/0> <reference>"
tracking() {
    chronyc tracking 2>/dev/null | awk -F': *' '
        /^Reference ID/ { ref = $2 }
        /^System time/ { split($2, a, " "); off = (a[3] == "slow") ? -a[1] : a[1] }
        /^Leap status/ { sync = ($2 ~ /Not synchronised/) ? 0 : 1 }
        END { if (off == "") exit 1; print off, sync, ref }'
}

check_chrony() {
    if in_container; then echo "the host's clock (container)"; return 0; fi
    command -v chronyc >/dev/null || { echo "chrony not installed"; return 1; }
    local line offset synced reference
    line=$(tracking) || { echo "chronyd not running"; return 1; }
    read -r offset synced reference <<<"$line"
    [[ "$synced" == 1 ]] || { echo "not synchronised yet (no reachable time source?)"; return 1; }
    if awk -v o="$offset" -v m="$TIME_MAX_OFFSET" 'BEGIN { exit !(o < -m || o > m) }'; then
        echo "offset ${offset} s to ${reference} (more than ${TIME_MAX_OFFSET} s)"
        return 1
    fi
    echo "offset ${offset} s to ${reference}"
}

chrony_dropin() {
    echo "# Written by scripts/timesync.sh: the robot's time source and the computers it serves."
    local server subnet
    for server in $TIME_SERVERS; do echo "server $server iburst prefer"; done
    for subnet in $TIME_ALLOW; do echo "allow $subnet"; done
    echo "local stratum 10"
}

install_chrony() {
    in_container && { echo "the host's clock (container): nothing to do"; return 0; }
    if [[ -z "${TIMESYNC_ROOT:-}" ]] && ! command -v chronyc >/dev/null; then
        $SUDO apt-get update -qq
        $SUDO apt-get install -y --no-install-recommends chrony
    fi
    chrony_dropin | write "$CHRONY_DROPIN"
    # chrony 3 (Ubuntu 20.04) has no confdir in its default configuration
    if [[ -f "$CHRONY_CONF" ]] && ! grep -qE '^(confdir|include) /etc/chrony/conf\.d' "$CHRONY_CONF"; then
        echo "include /etc/chrony/conf.d/*.conf" | $SUDO tee -a "$CHRONY_CONF" >/dev/null
    fi
    service enable chrony >/dev/null 2>&1 || true
    service restart chrony
    if [[ -z "${TIMESYNC_ROOT:-}" ]]; then # up to 30 s for the first synchronisation
        chronyc waitsync 6 0.1 0 5 >/dev/null 2>&1 || echo "chrony: not synchronised yet (time source reachable?)"
    fi
    echo "chrony: servers '${TIME_SERVERS:-distribution pool}', serving '${TIME_ALLOW:-nobody}'"
}

# ---------------------------------------------------------------- PTP

NO_LIDAR="no LiDAR address (LIDAR_IP / LIVOX_IP / PTP_IFACE)"

# The LiDAR's interface: PTP_IFACE, else the one that routes to LIDAR_IP ("" without either).
lidar_iface() {
    if [[ -n "$PTP_IFACE" ]]; then echo "$PTP_IFACE"; return 0; fi
    [[ -n "$LIDAR_IP" ]] || return 0
    local iface
    iface=$(ip route get "$LIDAR_IP" 2>/dev/null | awk '{ for (i = 1; i < NF; i++) if ($i == "dev") { print $(i + 1); exit } }')
    [[ -n "$iface" ]] || { echo "no interface routes to the LiDAR at $LIDAR_IP (set PTP_IFACE)" >&2; return 1; }
    echo "$iface"
}

check_ptp() {
    if in_container; then echo "the host's job (container)"; return 0; fi
    local iface
    iface=$(lidar_iface 2>&1) || { echo "$iface"; return 1; }
    [[ -n "$iface" ]] || { echo "$NO_LIDAR: skipped"; return 0; }
    command -v ptp4l >/dev/null || { echo "linuxptp not installed (LiDAR on $iface)"; return 1; }
    service is-active --quiet "$PTP_UNIT" || { echo "$PTP_UNIT not running (LiDAR on $iface)"; return 1; }
    echo "PTP master on $iface ($PTP_TIMESTAMPING timestamps)"
}

ptp_config() { # ptp_config <ptp4l major version>
    local only=masterOnly
    [[ "${1:-3}" -ge 4 ]] && only=serverOnly
    cat <<EOF
# Written by scripts/timesync.sh: PTP master for the LiDAR (Livox Mid-360: PTPv2 over UDP).
[global]
$only 1
priority1 127
domainNumber 0
network_transport UDPv4
delay_mechanism E2E
time_stamping $PTP_TIMESTAMPING
EOF
}

ptp_unit() { # ptp_unit <interface>
    cat <<EOF
[Unit]
Description=PTP master for the LiDAR on $1 (scripts/timesync.sh)
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/lidar-ptp.conf -i $1
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
}

phc_unit() { # phc_unit <interface>: the interface's clock follows the system clock
    cat <<EOF
[Unit]
Description=Clock of $1 follows the system clock for the LiDAR's PTP (scripts/timesync.sh)
After=$PTP_UNIT.service
Requires=$PTP_UNIT.service

[Service]
ExecStart=/usr/sbin/phc2sys -s CLOCK_REALTIME -c $1 -O 0
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
}

install_ptp() {
    in_container && { echo "the host's job (container): nothing to do"; return 0; }
    local iface major
    iface=$(lidar_iface) || return 1
    [[ -n "$iface" ]] || { echo "$NO_LIDAR: PTP skipped"; return 0; }
    case "$PTP_TIMESTAMPING" in
        software | hardware) ;;
        *) echo "PTP_TIMESTAMPING must be software or hardware" >&2; return 1 ;;
    esac
    if [[ -z "${TIMESYNC_ROOT:-}" ]] && ! command -v ptp4l >/dev/null; then
        $SUDO apt-get update -qq
        $SUDO apt-get install -y --no-install-recommends linuxptp
    fi
    major=$(ptp4l -v 2>/dev/null | cut -d. -f1)
    ptp_config "${major:-3}" | write "$PTP_CONF"
    ptp_unit "$iface" | write "$SYSTEMD_DIR/$PTP_UNIT.service"
    local units=("$PTP_UNIT")
    if [[ "$PTP_TIMESTAMPING" == hardware ]]; then
        phc_unit "$iface" | write "$SYSTEMD_DIR/$PHC_UNIT.service"
        units+=("$PHC_UNIT")
    else
        service disable --now "$PHC_UNIT" >/dev/null 2>&1 || true
        $SUDO rm -f "$SYSTEMD_DIR/$PHC_UNIT.service"
    fi
    service daemon-reload
    service enable "${units[@]}" >/dev/null 2>&1 || true
    service restart "${units[@]}"
    echo "PTP master on $iface ($PTP_TIMESTAMPING timestamps): the LiDAR follows this clock"
}

# ---------------------------------------------------------------- main

action="${1:-check}"
shift || true
parts=("$@")
[[ ${#parts[@]} -gt 0 ]] || parts=(chrony ptp)
status=0
for part in "${parts[@]}"; do
    case "$part" in chrony | ptp) ;; *) echo "unknown part: $part (chrony, ptp)" >&2; exit 2 ;; esac
    case "$action" in
        check) detail=$("check_$part") || status=1; echo "$part: $detail" ;;
        install) "install_$part" || status=1 ;;
        *) sed -n '2,5p' "$0"; exit 2 ;;
    esac
done
exit $status
