#!/usr/bin/env bash
# Dev container helper (macOS / Linux).
#   script/dev.sh up            build the image if needed and start the container
#   script/dev.sh shell         shell inside the container
#   script/dev.sh build [args]  colcon build inside (args go to colcon)
#   script/dev.sh test [args]   colcon test + test-result inside
#   script/dev.sh vnc           open the desktop (Screen Sharing on macOS) and print the URLs
#   script/dev.sh down          stop the container (build/install volumes are kept)
#   script/dev.sh clean         stop and delete the build/install/log volumes
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

in_container() {
    docker compose exec dev bash -c "source /opt/ros/humble/setup.bash && cd \"\$WS\" && $*"
}

case "${1:-shell}" in
    up)
        git submodule update --init --recursive || echo "warning: submodule update failed"
        docker compose up -d --build
        echo "VNC   vnc://localhost:5901  (password: ${VNC_PASSWORD:-molly})"
        echo "noVNC http://localhost:6080/vnc.html"
        ;;
    shell) docker compose exec dev bash ;;
    build) shift; in_container "colcon build $*" ;;
    test) shift; in_container "colcon test --executor sequential $* && colcon test-result --verbose" ;;
    vnc)
        echo "TigerVNC Viewer: localhost:5901  (brew install --cask tigervnc-viewer)"
        echo "noVNC: http://localhost:6080/vnc.html?resize=remote&autoconnect=true"
        ;;
    down) docker compose down ;;
    clean) docker compose down -v ;;
    *) sed -n '2,10p' "$0"; exit 1 ;;
esac
