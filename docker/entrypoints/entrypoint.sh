#!/bin/bash
set -e

# ✅ Source ROS base
source /opt/ros/${ROS_DISTRO}/setup.bash

# ✅ Source le workspace compilé si disponible
if [ -f /mecarosmaster_ws/install/setup.bash ]; then
    source /mecarosmaster_ws/install/setup.bash
fi

# ✅ AJOUT : recompiler à chaud si le workspace est monté et pas encore compilé
# Utile quand tu développes avec le volume monté (sources fraîches de l'hôte)
if [ -d /mecarosmaster_ws/src ] && [ ! -d /mecarosmaster_ws/install ]; then
    echo "[entrypoint] Workspace non compilé détecté, compilation en cours..."
    cd /mecarosmaster_ws
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    source /mecarosmaster_ws/install/setup.bash
fi

exec "$@"