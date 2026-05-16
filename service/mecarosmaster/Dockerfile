ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base AS base

ARG DEBIAN_FRONTEND=noninteractive

ENV PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:/usr/lib/aarch64-linux-gnu/pkgconfig:/usr/lib/x86_64-linux-gnu/pkgconfig

# Dépendances système et ROS essentielles
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    python3-dev \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    curl \
    ca-certificates \
    usbutils \
    udev \
    libusb-1.0-0-dev \
    libjpeg-dev \
    nlohmann-json3-dev \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-trajectory-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-python \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Initialiser rosdep
RUN rosdep update

# Copier ton workspace dans l'image pour compilation
COPY mecarosmaster_ws /mecarosmaster_ws
WORKDIR /mecarosmaster_ws

# Installer dépendances déclarées dans package.xml
RUN rosdep install --from-paths src --ignore-src -r -y

# Compiler le workspace
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log /root/.ros"

# Sourcing automatique pour bash
RUN printf '%s\n' \
    "source /opt/ros/${ROS_DISTRO}/setup.bash" \
    "source /mecarosmaster_ws/install/setup.bash" \
    >> /root/.bashrc

COPY docker/entrypoints/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
