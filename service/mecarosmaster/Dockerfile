ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base AS base

ARG DEBIAN_FRONTEND=noninteractive

ENV PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:/usr/lib/aarch64-linux-gnu/pkgconfig:/usr/lib/x86_64-linux-gnu/pkgconfig

# Dépendances système minimales
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    python3-dev \
    python3-colcon-common-extensions \
    python3-pip \
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
    && rm -rf /var/lib/apt/lists/*

# Dépendances ROS utiles pour ton node
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-trajectory-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    && rm -rf /var/lib/apt/lists/*

# Initialiser rosdep
RUN rosdep update

# Préparer workspace
RUN mkdir -p /mecarosmaster_ws/src
WORKDIR /mecarosmaster_ws

CMD ["bash"]
