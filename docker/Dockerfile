# syntax=docker/dockerfile:1.6
# Molly dev container: ROS 2 Humble, Livox-SDK2, all rosdep keys, XFCE desktop over VNC/noVNC.
ARG ROS_DISTRO=humble

# Only the package manifests, so the rosdep layer is rebuilt when dependencies change, not code.
FROM alpine:3.20 AS manifests
COPY src /src
RUN find /src -type f ! -name package.xml ! -name '*_IGNORE' -delete \
    && find /src -depth -type d -empty -delete

FROM ros:${ROS_DISTRO}-ros-base-jammy
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
    QT_X11_NO_MITSHM=1

# Desktop (TigerVNC + minimal XFCE + noVNC) and dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
        tigervnc-standalone-server tigervnc-tools novnc websockify dbus-x11 \
        xfce4-session xfwm4 xfce4-panel xfdesktop4 xfce4-settings xfce4-terminal thunar \
        adwaita-icon-theme fonts-dejavu-core \
        libgl1-mesa-dri mesa-utils \
        python3-colcon-common-extensions python3-pip python3-pytest \
        build-essential cmake git tmux sudo curl iputils-ping nano less bash-completion \
        ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Sensor SDK through the same installer the robot uses (robot SDKs: see src/xtras/README.md)
COPY script/setup.sh /opt/molly/setup.sh
RUN /opt/molly/setup.sh install livox

# ROS dependencies of every package in src/
COPY --from=manifests /src /tmp/src
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/* /tmp/src

# User with the same workspace path as the robot
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && mkdir -p ${WS}/build ${WS}/install ${WS}/log /home/${USERNAME}/.colcon \
    && mkdir -p /tmp/.X11-unix && chmod 1777 /tmp/.X11-unix \
    && printf '%s\n' \
        "source /opt/ros/${ROS_DISTRO}/setup.bash" \
        '[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"' \
        'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' \
        'cd "$WS"' >> /home/${USERNAME}/.bashrc \
    && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

# XFCE: no compositor (useless over VNC, slows software-rendered RViz)
RUN mkdir -p /home/${USERNAME}/.config/xfce4/xfconf/xfce-perchannel-xml \
    && printf '%s\n' \
        '<?xml version="1.0" encoding="UTF-8"?>' \
        '<channel name="xfwm4" version="1.0">' \
        '  <property name="general" type="empty">' \
        '    <property name="use_compositing" type="bool" value="false"/>' \
        '  </property>' \
        '</channel>' \
        > /home/${USERNAME}/.config/xfce4/xfconf/xfce-perchannel-xml/xfwm4.xml \
    && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.config

COPY --chown=${USERNAME}:${USERNAME} docker/colcon_defaults.yaml /home/${USERNAME}/.colcon/defaults.yaml
COPY docker/entrypoint.sh /entrypoint.sh

USER ${USERNAME}
WORKDIR ${WS}
EXPOSE 5901 6080
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]