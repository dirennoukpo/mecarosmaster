# service/mecarosmaster/ — Dockerfiles

## Dockerfile.base

Image de base pour le robot physique (arm64).

- Part de `ros:humble-ros-base`
- Installe : FastDDS, ros2-control, RPLidar, v4l2-camera, IMU tools,
  robot-localization, TF2, xacro, rosdep
- Configure le profil FastDDS `/etc/fastdds/fastdds_base.xml`
- Crée l'utilisateur `rosdev` (non-root)
- Compile le workspace si des sources sont présentes dans `src/`

## Dockerfile.mecarosmaster

Image service robot, hérite de `mecarosmaster-base:humble`.

- Surcharge les variables ENV si nécessaire
- Point d'extension pour des drivers customs ou outils terrain
- Entrypoint : `docker/entrypoints/entrypoint.sh`

## Dockerfile.workstation

Image poste développeur, indépendante (amd64).

- Part de `ros:humble-desktop` (inclut RViz2, rqt)
- Ajoute : SLAM (rtabmap, cartographer, slam_toolbox), RealSense,
  Nav2 complet, rosbag2, tf2-tools, teleop, debug tools (gdb, valgrind)
- Configure le profil FastDDS `/etc/fastdds/fastdds_workstation.xml`
- Entrypoint : `docker/entrypoints/entrypoint_workstation.sh`

## Variables ARG importantes

| ARG          | Défaut   | Description                              |
|--------------|----------|------------------------------------------|
| `ROS_DISTRO` | `humble` | Distribution ROS2                        |
| `USER_ID`    | `1000`   | UID utilisateur hôte (droits volumes)    |
| `GROUP_ID`   | `1000`   | GID utilisateur hôte                     |
| `REGISTRY`   | `local`  | Registre Docker (Dockerfile.mecarosmaster)|
