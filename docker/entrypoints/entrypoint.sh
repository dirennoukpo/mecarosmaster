#!/bin/bash
# =============================================================================
# entrypoint.sh — Robot (mecarosmaster / base)
# =============================================================================
set -e

# ✅ Source ROS base
source /opt/ros/${ROS_DISTRO}/setup.bash

# ✅ Source le workspace compilé si disponible
if [ -f /mecarosmaster_ws/install/setup.bash ]; then
    source /mecarosmaster_ws/install/setup.bash
fi

# ✅ Recompiler à chaud si le workspace est monté mais pas encore compilé
# Utile en développement avec volume monté (sources fraîches de l'hôte)
if [ -d /mecarosmaster_ws/src ] && [ ! -d /mecarosmaster_ws/install ]; then
    echo "[entrypoint] Workspace non compilé détecté — compilation en cours..."
    cd /mecarosmaster_ws
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    source /mecarosmaster_ws/install/setup.bash
fi

# ✅ Appliquer le profil FastDDS si défini
if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE}" ] && [ -f "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE}"
fi

# ✅ Afficher le contexte ROS2 au démarrage (debug)
echo "[entrypoint] ROS_DISTRO        = ${ROS_DISTRO}"
echo "[entrypoint] RMW_IMPLEMENTATION = ${RMW_IMPLEMENTATION}"
echo "[entrypoint] ROS_DOMAIN_ID     = ${ROS_DOMAIN_ID}"
echo "[entrypoint] ROBOT_NAME        = ${ROBOT_NAME:-mecarosmaster}"

exec "$@"
