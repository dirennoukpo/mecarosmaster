# docker/ — Dockerfiles & entrypoints

## Structure

```
docker/
├── entrypoints/
│   ├── entrypoint.sh             # Entrypoint robot (mecarosmaster / base)
│   ├── entrypoint_workstation.sh # Entrypoint workstation développeur
│   └── README.md
└── scripts/
    └── README.md
```

## Entrypoints

Les entrypoints gèrent au démarrage du conteneur :

1. **Source ROS2** (`/opt/ros/$ROS_DISTRO/setup.bash`)
2. **Source workspace** si `/mecarosmaster_ws/install/setup.bash` existe
3. **Compilation à chaud** si les sources sont montées mais pas compilées
4. **Application profil FastDDS** si `FASTRTPS_DEFAULT_PROFILES_FILE` est défini
5. **Affichage du contexte ROS2** (distro, RMW, domain, robot name)

## Logique de build au démarrage

```bash
# Détection automatique dans l'entrypoint
if [ -d /mecarosmaster_ws/src ] && [ ! -d /mecarosmaster_ws/install ]; then
    colcon build ...
fi
```

Cela permet de travailler avec un volume monté sans reconstruire l'image
à chaque modification de source.
