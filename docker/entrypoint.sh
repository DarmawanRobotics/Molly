#!/usr/bin/env bash
# Start the VNC desktop (Xvnc :1 + XFCE + noVNC) unless VNC=0, then run the command.
#   VNC_PASSWORD (dharma), VNC_RESOLUTION (1600x900)
set -e

start_desktop() {
    mkdir -p "$HOME/.vnc"
    echo "${VNC_PASSWORD:-dharma}" | vncpasswd -f > "$HOME/.vnc/passwd"
    chmod 600 "$HOME/.vnc/passwd"
    rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
    Xvnc :1 -geometry "${VNC_RESOLUTION:-1600x900}" -depth 24 -rfbport 5901 -localhost no \
        -SecurityTypes VncAuth -PasswordFile "$HOME/.vnc/passwd" -AlwaysShared \
        > /tmp/xvnc.log 2>&1 &
    for _ in $(seq 50); do
        [[ -e /tmp/.X11-unix/X1 ]] && break
        sleep 0.1
    done
    DISPLAY=:1 dbus-launch --exit-with-session startxfce4 > /tmp/xfce.log 2>&1 &
    websockify --web /usr/share/novnc 6080 localhost:5901 > /tmp/novnc.log 2>&1 &
}

if [[ "${VNC:-1}" != "0" ]]; then
    start_desktop
fi
exec "$@"
