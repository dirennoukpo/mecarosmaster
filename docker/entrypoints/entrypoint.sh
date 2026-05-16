#!/bin/bash
set -e

# Charger ROS et ton workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f /mecarosmaster_ws/install/setup.bash ]; then
    source /mecarosmaster_ws/install/setup.bash
fi

exec "$@"
