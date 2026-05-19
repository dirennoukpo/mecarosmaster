# service/ — Dockerfiles des services

## Structure

```
service/
└── mecarosmaster/
    ├── Dockerfile.base           # Image commune — tout le hardware robot
    ├── Dockerfile.mecarosmaster  # Service robot — hérite de base
    ├── Dockerfile.workstation    # Service workstation — standalone amd64
    └── README.md
```

## Relation entre les images

```
ros:humble-ros-base (upstream)
    └── Dockerfile.base  →  mecarosmaster-base:humble
            └── Dockerfile.mecarosmaster  →  mecarosmaster:humble

ros:humble-desktop (upstream)
    └── Dockerfile.workstation  →  mecarosmaster-workstation:humble
```

`Dockerfile.base` contient tout le hardware (serial, LiDAR, caméra, IMU,
ros2-control, robot-localization). `Dockerfile.mecarosmaster` en hérite et
peut recevoir des surcharges spécifiques au service sans toucher à l'image base.

`Dockerfile.workstation` est entièrement indépendant et cible amd64.
