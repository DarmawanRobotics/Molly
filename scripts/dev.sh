#!/usr/bin/env bash
# Dev container helper (Linux / macOS): ROS 2 Humble, see compose.yaml.
#
#   scripts/dev.sh <command> [args]
#
#   up             import the Dharma stack if missing, build the image if needed, start it
#   shell          interactive shell inside (ROS and the workspace sourced)
#   build [args]   colcon build inside (args go to colcon, e.g. --packages-up-to molly_bringup)
#   test [args]    colcon test + colcon test-result inside
#   vnc            print the VNC / noVNC addresses (opens Screen Sharing on macOS)
#   down           stop and remove the container (build/install/log volumes are kept)
#   clean          down, then delete the build/install/log volumes
#   import [--update]  vcs import molly.repos into src/ and the submodules (--update: move
#                  existing checkouts to the pinned versions too); plain git without vcstool
#   doctor         check Docker, compose, CPU / memory, the architecture and GitHub access
#
# Env: VNC_PASSWORD (molly), VNC=0 (no desktop),
#      USER_UID / USER_GID (container user; default: yours on Linux, 1000 on macOS, where
#      Docker Desktop maps file ownership of the bind mount itself)
# Runs on the bash 3.2 of macOS: no empty "${array[@]}" under set -u, no bash 4 features.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

usage() {
    sed -n '2,/^set -/p' "$0" | sed -e '$d' -e 's/^# \{0,1\}//'
}

DISTRO=humble
VNC_PORT=5901 NOVNC_PORT=6080
SERVICE=dev
case "${1:-}" in -h | --help) usage; exit 0 ;; esac
if [[ "$(uname -s)" == Linux ]]; then
    # the bind-mounted workspace stays yours: the container user gets your uid / gid
    export USER_UID="${USER_UID:-$(id -u)}" USER_GID="${USER_GID:-$(id -g)}"
fi
compose() { docker compose "$@"; }

# Run a command in the container with ROS and the workspace sourced (arguments stay quoted).
in_container() {
    local tty=() quoted
    [[ -t 0 && -t 1 ]] || tty=(-T)
    printf -v quoted ' %q' "$@"
    compose exec ${tty[@]+"${tty[@]}"} "$SERVICE" bash -c \
        "source /opt/ros/$DISTRO/setup.bash && cd \"\$WS\" \
&& { [ ! -f install/setup.bash ] || source install/setup.bash; } &&$quoted"
}

# "path url version" for every repository of a .repos file (commented entries skipped)
repos_entries() {
    awk '
        /^[[:space:]]*#/ { next }
        /^  [^ #][^:]*:[[:space:]]*$/ { path = $1; sub(/:$/, "", path); url = ""; next }
        /^    url:/ { url = $2 }
        /^    version:/ { if (path != "" && url != "") print path, url, $2; path = "" }
    ' "$1"
}

# The same import with plain git (macOS rarely has vcstool): versions are branches or tags.
git_import() {
    local file="$1" update="$2" path url version
    while read -r path url version; do
        if [[ -d "src/$path/.git" ]]; then
            if [[ "$update" == 1 ]]; then
                git -C "src/$path" fetch -q origin "$version"
                git -C "src/$path" checkout -q FETCH_HEAD
                echo "=== src/$path -> $version"
            fi
            continue
        fi
        mkdir -p "src/$(dirname "$path")"
        git clone -q "$url" "src/$path"
        git -C "src/$path" checkout -q "$version"
        echo "=== src/$path ($version)"
    done < <(repos_entries "$file")
}

import_repos() {
    local flags=(--recursive --skip-existing) update=0 file
    if [[ "${1:-}" == "--update" ]]; then flags=(--recursive) update=1; fi
    git submodule update --init --recursive
    if command -v vcs > /dev/null; then
        vcs import "${flags[@]}" src < molly.repos
    else
        git_import molly.repos "$update"
    fi
}

# Is the machine ready for the dev container? Prints one line per check, exit 1 on a failure.
doctor() {
    local failed=0 cpus mem arch
    if ! docker info > /dev/null 2>&1; then
        echo "FAIL docker: not running (start Docker Desktop, OrbStack or colima)"
        return 1
    fi
    echo "ok   docker $(docker version --format '{{.Server.Version}}' 2> /dev/null)"
    if docker compose version > /dev/null 2>&1; then
        echo "ok   $(docker compose version | head -n1)"
    else
        echo "FAIL docker compose v2 missing"; failed=1
    fi
    arch="$(docker info --format '{{.Architecture}}' 2> /dev/null || echo unknown)"
    cpus="$(docker info --format '{{.NCPU}}' 2> /dev/null || echo 0)"
    mem="$(docker info --format '{{.MemTotal}}' 2> /dev/null || echo 0)"
    mem=$((mem / 1024 / 1024 / 1024))
    echo "ok   engine $arch, $cpus CPUs, ${mem} GiB (images are built natively, no emulation)"
    if [[ "$mem" -lt 8 ]]; then
        echo "WARN less than 8 GiB for Docker: raise it (Docker Desktop: Settings > Resources)"
        echo "     or build with fewer jobs: scripts/dev.sh build --parallel-workers 1"
    fi
    if command -v vcs > /dev/null; then echo "ok   vcs (vcstool)"; else echo "ok   git import (no vcstool)"; fi
    if git ls-remote -q "$(repos_entries molly.repos | awk 'NR == 1 { print $2 }')" > /dev/null 2>&1; then
        echo "ok   GitHub access to the dharma repositories"
    else
        echo "FAIL no access to the private dharma repositories: gh auth login (or an SSH key)"
        failed=1
    fi
    if [[ -d src/dharma ]]; then echo "ok   stack imported in src/dharma"; else echo "WARN stack not imported yet (up does it)"; fi
    return "$failed"
}

project_name() {
    echo "${COMPOSE_PROJECT_NAME:-$(sed -n 's/^name: *//p' compose.yaml)}"
}

case "${1:-}" in
    up)
        if [[ ! -d src/dharma ]]; then
            import_repos || echo "warning: the stack is not imported (scripts/dev.sh import)" >&2
        fi
        compose up -d --build "$SERVICE"
        echo "$SERVICE up: VNC localhost:$VNC_PORT, noVNC http://localhost:$NOVNC_PORT/vnc.html"
        echo "password: ${VNC_PASSWORD:-molly}"
        ;;
    shell) compose exec "$SERVICE" bash ;;
    build) shift; in_container colcon build "$@" ;;
    test)
        shift
        in_container colcon test "$@"
        in_container colcon test-result --verbose
        ;;
    vnc)
        echo "VNC   vnc://localhost:$VNC_PORT  (password: ${VNC_PASSWORD:-molly})"
        echo "noVNC http://localhost:$NOVNC_PORT/vnc.html?resize=remote&autoconnect=true"
        if [[ "$(uname -s)" == Darwin ]]; then open "vnc://localhost:$VNC_PORT"; fi
        ;;
    down) compose stop "$SERVICE" && compose rm -f "$SERVICE" ;;
    clean)
        compose stop "$SERVICE" && compose rm -f "$SERVICE"
        project="$(project_name)"
        for volume in build install log; do
            docker volume rm "${project}_${volume}" 2> /dev/null || true
        done
        echo "removed the build/install/log volumes"
        ;;
    import) shift; import_repos "$@" ;;
    doctor) doctor ;;
    *) usage; exit 1 ;;
esac
