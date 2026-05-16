ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base AS base

# ✅ Re-déclarer après FROM
ARG ROS_DISTRO=humble
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG DEBIAN_FRONTEND=noninteractive

ENV PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:/usr/lib/aarch64-linux-gnu/pkgconfig:/usr/lib/x86_64-linux-gnu/pkgconfig

# ✅ Tout en ROOT : apt, rosdep, colcon, créer l'utilisateur
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config \
    python3-dev python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    wget curl ca-certificates \
    usbutils udev libusb-1.0-0-dev \
    libjpeg-dev nlohmann-json3-dev \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-xacro    \
    ros-${ROS_DISTRO}-trajectory-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-python \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true && rosdep update

# Créer l'utilisateur (toujours en root)
RUN groupadd -g ${GROUP_ID} rosdev && \
    useradd -u ${USER_ID} -g ${GROUP_ID} -m -s /bin/bash rosdev && \
    usermod -aG dialout rosdev

# Préparer le workspace avec les bons droits
RUN mkdir -p /mecarosmaster_ws && \
    chown -R ${USER_ID}:${GROUP_ID} /mecarosmaster_ws

COPY mecarosmaster_ws/src /mecarosmaster_ws/src
WORKDIR /mecarosmaster_ws

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    if [ \"\$(find src -name 'package.xml' | wc -l)\" -gt 0 ]; then \
        apt-get update && \
        rosdep install --from-paths src --ignore-src -r -y && \
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release; \
    else \
        echo '[INFO] Aucun package, build ignoré.'; \
    fi && \
    rm -rf /var/lib/apt/lists/* build log /root/.ros"

# Sourcing pour root (sessions debug)
RUN printf '%s\n' \
    "source /opt/ros/${ROS_DISTRO}/setup.bash" \
    "[ -f /mecarosmaster_ws/install/setup.bash ] && source /mecarosmaster_ws/install/setup.bash" \
    >> /root/.bashrc

# Sourcing pour rosdev
RUN printf '%s\n' \
    "source /opt/ros/${ROS_DISTRO}/setup.bash" \
    "[ -f /mecarosmaster_ws/install/setup.bash ] && source /mecarosmaster_ws/install/setup.bash" \
    >> /home/rosdev/.bashrc

# Donner les droits finaux sur tout le workspace à rosdev
RUN chown -R ${USER_ID}:${GROUP_ID} /mecarosmaster_ws

COPY docker/entrypoints/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# ✅ Basculer vers rosdev seulement à la toute fin
USER rosdev

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]