##
## straight_line_pid.launch.py for mecarosmaster [SSH: ROSMASTER-YAHBOOM]
##
## Made by dirennoukpo
## Login   <diren.noukpo@epitech.eu>
##
## Started on  Mon May 18 23:00:55 2026 dirennoukpo
## Last update Wed May 19 11:27:41 2026 dirennoukpo
##
## CHANGELOG v2 :
##   - Ajout des arguments ki et ki_max
##

"""
Launch straight_line_pid — contrôleur PI complet.

Flux des topics :
  teleop/nav2  →  /cmd_vel  (TwistStamped)
                      │
                      └── straight_line_pid  (PI sur yaw IMU)
                            cmd_vel_in  ← /cmd_vel
                            cmd_vel_out → /diff_drive_controller/cmd_vel
                                              │
                                              └── diff_drive_controller
                                                    → hw_cmd[] (rad/s)
                                                    → set_car_motion(vx, 0, wz)

Tuning rapide :
  ros2 launch straight_line_pid straight_line_pid.launch.py kp:=1.5 ki:=0.3 ki_max:=0.2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share   = FindPackageShare('straight_line_pid')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'pid_params.yaml'])

    # ── Arguments de lancement ────────────────────────────────────────────────
    kp_arg      = DeclareLaunchArgument('kp',      default_value='1.5',
                    description='Gain proportionnel (rad/s per rad)')
    ki_arg      = DeclareLaunchArgument('ki',      default_value='0.3',
                    description='Gain intégral (rad/s per rad·s)')
    ki_max_arg  = DeclareLaunchArgument('ki_max',  default_value='0.2',
                    description='Saturation anti-windup de l intégrale (rad/s)')
    max_wz_arg  = DeclareLaunchArgument('max_wz',  default_value='0.5',
                    description='Saturation totale de wz (rad/s)')
    db_arg      = DeclareLaunchArgument('deadband', default_value='0.017',
                    description='Seuil de deadband (rad, ~1°)')
    freq_arg    = DeclareLaunchArgument('freq',     default_value='50.0',
                    description='Fréquence de la boucle de contrôle (Hz)')
    timeout_arg = DeclareLaunchArgument('timeout',  default_value='5.0',
                    description='Timeout sans commande avant arrêt (s)')

    # ── Nœud PI ───────────────────────────────────────────────────────────────
    pid_node = Node(
        package='straight_line_pid',
        executable='straight_line_pid_node',
        name='straight_line_pid',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,   # valeurs par défaut depuis le yaml
            {              # overrides depuis les arguments de lancement
                'control_frequency': LaunchConfiguration('freq'),
                'cmd_vel_timeout':   LaunchConfiguration('timeout'),
                'kp':                LaunchConfiguration('kp'),
                'ki':                LaunchConfiguration('ki'),
                'ki_max':            LaunchConfiguration('ki_max'),
                'max_wz':            LaunchConfiguration('max_wz'),
                'deadband':          LaunchConfiguration('deadband'),
            }
        ],
        remappings=[
            ('cmd_vel_in',  '/cmd_vel'),
            ('cmd_vel_out', '/diff_drive_controller/cmd_vel'),
        ]
    )

    return LaunchDescription([
        kp_arg, ki_arg, ki_max_arg, max_wz_arg, db_arg, freq_arg, timeout_arg,
        pid_node,
    ])