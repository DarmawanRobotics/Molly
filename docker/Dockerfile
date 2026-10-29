# syntax=docker/dockerfile:1.6
# Molly dev container: ROS 2 Humble, every rosdep key of src/ (Molly, the Dharma stack from
# molly.repos, the Lite3 package), Livox-SDK2, the lint tools, and an XFCE desktop over VNC / noVNC
# for RViz. Built by compose.yaml (scripts/dev.sh up).
ARG ROS_DISTRO=humble

# Only the package manifests, so the rosdep layer is rebuilt when dependencies change, not code.
FROM alpine:3.20 AS manifests
COPY src /src
RUN find /src -type f ! -name package.xml ! -name '*_IGNORE' -delete \
    && find /src -depth -type d -empty -delete \
    && mkdir -p /src

FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    WS=/home/${USERNAME}/dev/Molly \
    DISPLAY=:1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    PIPX_HOME=/opt/pipx \
    PIPX_BIN_DIR=/usr/local/bin

# Desktop (TigerVNC + minimal XFCE + noVNC), dev tools, RViz and the Python packages of the stack.
RUN apt-get update && apt-get install -y --no-install-recommends \
        tigervnc-tools pipx python3-loguru tigervnc-standalone-server novnc websockify dbus-x11 \
        xfce4-session xfwm4 xfce4-panel xfdesktop4 xfce4-settings xfce4-terminal thunar \
        adwaita-icon-theme fonts-dejavu-core libgl1-mesa-dri mesa-utils \
        python3-colcon-common-extensions python3-vcstool python3-pip python3-pytest \
        python3-scipy python3-yaml \
        build-essential cmake git tmux sudo curl ca-certificates iputils-ping nano less \
        bash-completion shellcheck xclip \
        ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Lint: the same hooks as CI (pre-commit, on Python 3.9+ for the hooks) and ruff at the version of
# the hook
RUN pipx install "pre-commit>=3.5" \
    && pipx install "ruff==0.16.9"

# Sensor SDK through the same installer the robot uses
COPY scripts/setup.sh /opt/molly/setup.sh
RUN MOLLY_ENV=/nonexistent /opt/molly/setup.sh install livox

# ROS dependencies of every package in src/ (vendor packages that are not checked out: skipped)
COPY --from=manifests /src /tmp/src
RUN apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths /tmp/src --ignore-src -r -y --rosdistro "${ROS_DISTRO}" \
        --skip-keys "livox_ros_driver2 fast_lio legkilo python3-loguru" \
    && rm -rf /var/lib/apt/lists/* /tmp/src

# User with the host's uid (the noble images already have uid 1000: ubuntu). A gid the image
# already has is reused (macOS: gid 20 is staff there, dialout here).
RUN if id -u ubuntu > /dev/null 2>&1; then userdel -r ubuntu || true; fi \
    && if ! getent group "${USER_GID}" > /dev/null; then groupadd --gid "${USER_GID}" "${USERNAME}"; fi \
    && taken=$(getent passwd "${USER_UID}" | cut -d: -f1) \
    && if [ -n "$taken" ] && [ "${USER_UID}" != 0 ]; then userdel -r "$taken" || true; fi \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
    && mkdir -p "${WS}/build" "${WS}/install" "${WS}/log" "/home/${USERNAME}/.colcon" \
    && mkdir -p /tmp/.X11-unix && chmod 1777 /tmp/.X11-unix \
    && printf '%s\n' \
        "source /opt/ros/${ROS_DISTRO}/setup.bash" \
        '[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"' \
        'f=/usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; [ -f "$f" ] && source "$f"' \
        'cd "$WS"' >> "/home/${USERNAME}/.bashrc" \
    && chown -R "${USER_UID}:${USER_GID}" "/home/${USERNAME}"

# XFCE: no compositor (useless over VNC, slows software-rendered RViz)
RUN mkdir -p "/home/${USERNAME}/.config/xfce4/xfconf/xfce-perchannel-xml" \
    && printf '%s\n' \
        '<?xml version="1.0" encoding="UTF-8"?>' \
        '<channel name="xfwm4" version="1.0">' \
        '  <property name="general" type="empty">' \
        '    <property name="use_compositing" type="bool" value="false"/>' \
        '  </property>' \
        '</channel>' \
        > "/home/${USERNAME}/.config/xfce4/xfconf/xfce-perchannel-xml/xfwm4.xml" \
    && chown -R "${USER_UID}:${USER_GID}" "/home/${USERNAME}/.config"

COPY --chown=${USER_UID}:${USER_GID} docker/colcon_defaults.yaml /home/${USERNAME}/.colcon/defaults.yaml
COPY --chown=${USER_UID}:${USER_GID} docker/tmux/tmux.conf /home/${USERNAME}/.tmux.conf
COPY docker/entrypoint.sh /entrypoint.sh

USER ${USERNAME}
WORKDIR ${WS}
EXPOSE 5901 6080
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
