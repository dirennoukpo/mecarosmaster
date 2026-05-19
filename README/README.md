# mecarosmaster

Stack ROS2 Dockerisée pour robot YAHBOOM ROSMASTER X3 / Raspberry Pi.

## Architecture

```
mecarosmaster/
├── compose/              # docker-compose.yml (service robot + workstation)
├── dds/                  # Profils FastDDS XML (robot et workstation)
├── docker/
│   └── entrypoints/      # Scripts d'entrée des conteneurs
├── env/                  # Fichiers .env (commun + robot + workstation)
├── make/                 # Règles Makefile Docker
├── mecarosmaster_ws/     # Workspace ROS2 (monté en volume)
│   └── src/
│       ├── mecarosmaster_bringup/
│       ├── mecarosmaster_control/
│       └── straight_line_pid/
├── script/               # Scripts d'installation hôte (Raspberry Pi OS)
├── service/
│   └── mecarosmaster/
│       ├── Dockerfile.base           # Image commune robot
│       ├── Dockerfile.mecarosmaster  # Service robot physique
│       └── Dockerfile.workstation    # Poste développeur
└── Makefile
```

## Prérequis

| Hôte robot        | Raspberry Pi 4/5, Ubuntu 22.04 arm64 ou Raspberry Pi OS 64-bit |
|-------------------|-----------------------------------------------------------------|
| Hôte workstation  | Linux amd64, Ubuntu 22.04+                                     |
| Docker            | ≥ 24.x                                                         |
| Docker Compose    | ≥ 2.x (plugin, pas standalone)                                 |

## Démarrage rapide

### 1 — Préparer les fichiers `.env`

```bash
# Variables communes
cp env/.env.example env/.env

# Robot (Raspberry Pi)
cp env/.env.mecarosmaster.example env/.env.mecarosmaster
# → Renseigner SERIAL_PORT, ROBOT_IP

# Workstation (poste dev)
cp env/.env.workstation.example env/.env.workstation
# → Renseigner ROS_MASTER_IP, DISPLAY
```

### 2 — Construire les images

```bash
# Image base + service robot (sur le Pi ou en cross-compilation)
make build-robot

# Image workstation (sur le poste dev)
make build-workstation
```

### 3 — Lancer les services

```bash
# Sur le robot
make up-robot

# Sur la workstation (avec X11)
make up-workstation
```

### 4 — Accéder aux conteneurs

```bash
make shell-robot          # bash dans le robot
make shell-workstation    # bash dans la workstation
make logs-robot           # logs robot en direct
```

## Variables d'environnement clés

| Variable                          | Défaut                        | Description                              |
|-----------------------------------|-------------------------------|------------------------------------------|
| `ROS_DISTRO`                      | `humble`                      | Distribution ROS2                        |
| `ROS_DOMAIN_ID`                   | `0`                           | Domaine DDS partagé robot ↔ workstation  |
| `RMW_IMPLEMENTATION`              | `rmw_fastrtps_cpp`            | Middleware DDS                           |
| `FASTRTPS_DEFAULT_PROFILES_FILE`  | profil xml selon le service   | Profil FastDDS XML                       |
| `SERIAL_PORT`                     | `/dev/YB-ERF01-v3.0`          | Port série contrôleur moteurs            |
| `ROBOT_IP`                        | `192.168.1.100`               | IP fixe du robot sur le LAN             |
| `ROS_MASTER_IP`                   | `192.168.1.100`               | Idem vu depuis la workstation            |
| `SLAM_MODE`                       | `rtabmap`                     | Algorithme SLAM actif                    |
| `ENABLE_LIDAR`                    | `true`                        | Activer/désactiver RPLidar               |
| `ENABLE_CAMERA`                   | `true`                        | Activer/désactiver caméra embarquée      |
| `ENABLE_IMU`                      | `true`                        | Activer/désactiver IMU                   |
| `USE_SIM_TIME`                    | `false`                       | `true` pour replay rosbag               |

## DDS / FastRTPS

Les deux profils XML sont dans `dds/` et copiés dans `/etc/fastdds/` à l'intérieur des images :

- `fastdds_base.xml` → robot (buffers réduits, heartbeat fréquent, unicast)
- `fastdds_workstation.xml` → workstation (buffers larges, PointCloud / RGBD)

Le `ROS_DOMAIN_ID` doit être identique sur robot et workstation. Les deux services utilisent `network_mode: host` pour que DDS fonctionne correctement sur le LAN.

## Compilation à chaud

L'entrypoint détecte automatiquement si le workspace est compilé ou non :

- Si `/mecarosmaster_ws/install/setup.bash` existe → source direct
- Sinon → `colcon build` automatique au démarrage du conteneur

Cela permet de monter des sources fraîches via volume et de les compiler à chaud sans reconstruire l'image.
