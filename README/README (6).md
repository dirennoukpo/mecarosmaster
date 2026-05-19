# docs/ — Documentation

## Contenu prévu

| Document                    | Description                                            |
|-----------------------------|--------------------------------------------------------|
| `architecture.md`           | Schéma des noeuds ROS2, topics, TF tree               |
| `network.md`                | Configuration réseau LAN, IP fixes, DDS discovery      |
| `slam.md`                   | Comparaison RTAB-Map / Cartographer / Slam Toolbox     |
| `nav2.md`                   | Configuration Nav2 (planners, costmap, lifecycle)      |
| `hardware.md`               | Câblage, ports série, RPLidar, caméra, IMU             |
| `troubleshooting.md`        | Problèmes fréquents et solutions                       |

## Noeuds ROS2 actifs

### Robot (mecarosmaster)
| Noeud                    | Package                  | Rôle                          |
|--------------------------|--------------------------|-------------------------------|
| `/driver_node`           | mecarosmaster_control    | Contrôle moteurs (serial)     |
| `/robot_state_publisher` | robot-state-publisher    | Publie TF depuis URDF         |
| `/joint_state_publisher` | joint-state-publisher    | États articulations           |
| `/rplidar_node`          | rplidar-ros              | LiDAR 2D LaserScan            |
| `/imu_filter_madgwick`   | imu-tools                | Filtre orientation IMU        |
| `/ekf_filter_node`       | robot-localization       | Fusion odom + IMU → EKF odom  |
| `/camera/camera`         | v4l2-camera              | Caméra embarquée              |

### Workstation
| Noeud                     | Package           | Rôle                            |
|---------------------------|-------------------|---------------------------------|
| `/rtabmap`                | rtabmap-ros       | SLAM RGB-D                      |
| `/rgbd_sync`              | rtabmap-ros       | Synchronisation flux RGBD       |
| `/cartographer_node`      | cartographer-ros  | SLAM LiDAR 2D                   |
| `/slam_toolbox`           | slam-toolbox      | SLAM lifelong                   |
| `/ekf_filter_node`        | robot-localization| EKF workstation (monitoring)    |
| `/teleop_twist_keyboard`  | teleop            | Téléopération clavier           |
