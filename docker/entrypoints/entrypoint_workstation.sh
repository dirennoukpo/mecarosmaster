#!/bin/bash
# =============================================================================
# entrypoint_workstation.sh — Workstation développeur
# =============================================================================
set -e

# ✅ Source ROS base
source /opt/ros/${ROS_DISTRO}/setup.bash

# ✅ Source le workspace compilé si disponible
if [ -f /mecarosmaster_ws/install/setup.bash ]; then
    source /mecarosmaster_ws/install/setup.bash
fi

# ✅ Recompiler à chaud si le workspace est monté mais pas encore compilé
if [ -d /mecarosmaster_ws/src ] && [ ! -d /mecarosmaster_ws/install ]; then
    echo "[entrypoint-ws] Workspace non compilé — compilation en cours..."
    cd /mecarosmaster_ws
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    source /mecarosmaster_ws/install/setup.bash
fi

# ✅ FastDDS profile
if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE}" ] && [ -f "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE}"
fi

# ✅ DISPLAY : vérifier que X11 est accessible pour RViz2 / rqt
if [ -n "${DISPLAY}" ]; then
    echo "[entrypoint-ws] DISPLAY = ${DISPLAY} — RViz2/rqt disponibles"
else
    echo "[entrypoint-ws] ATTENTION : variable DISPLAY non définie — RViz2/rqt ne fonctionneront pas"
    echo "[entrypoint-ws] Lance 'xhost +local:docker' sur ta machine hôte si besoin"
fi

# ✅ Afficher le contexte ROS2 au démarrage
echo "[entrypoint-ws] ROS_DISTRO          = ${ROS_DISTRO}"
echo "[entrypoint-ws] RMW_IMPLEMENTATION  = ${RMW_IMPLEMENTATION}"
echo "[entrypoint-ws] ROS_DOMAIN_ID       = ${ROS_DOMAIN_ID}"
echo "[entrypoint-ws] ROS_MASTER_IP       = ${ROS_MASTER_IP:-non défini}"
echo "[entrypoint-ws] SLAM_MODE           = ${SLAM_MODE:-rtabmap}"
echo "[entrypoint-ws] MAP_FRAME           = ${MAP_FRAME:-map}"

exec "$@"
