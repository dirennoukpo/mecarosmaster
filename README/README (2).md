# compose/ — Docker Compose

## Fichier

`docker-compose.yml` définit deux services isolés via des **profiles** :

| Profile        | Service              | Cible                              |
|----------------|----------------------|------------------------------------|
| `robot`        | `mecarosmaster`      | Raspberry Pi arm64 (robot physique)|
| `workstation`  | `workstation`        | Linux amd64 (poste développeur)    |

## Usage

```bash
# Robot uniquement
docker compose --profile robot up -d mecarosmaster

# Workstation uniquement
xhost +local:docker
docker compose --profile workstation up -d workstation

# Les deux
docker compose --profile robot --profile workstation up -d
```

## Pourquoi `network_mode: host` ?

FastDDS (rmw_fastrtps_cpp) utilise UDP unicast/multicast pour la découverte
DDS entre participants. Le NAT Docker empêche la découverte inter-conteneurs
sur le LAN. `network_mode: host` est la solution recommandée pour ROS2 en
production multi-machine.

## Volumes montés

| Volume                          | Conteneur              | Description                        |
|---------------------------------|------------------------|------------------------------------|
| `../mecarosmaster_ws`           | `/mecarosmaster_ws`    | Workspace ROS2 (robot + ws)        |
| `../maps`                       | `/maps`                | Cartes SLAM persistées (workstation)|
| `../bags`                       | `/bags`                | Bags rosbag (workstation)          |
| `/tmp/.X11-unix`                | `/tmp/.X11-unix`       | Socket X11 RViz2/rqt (workstation) |
