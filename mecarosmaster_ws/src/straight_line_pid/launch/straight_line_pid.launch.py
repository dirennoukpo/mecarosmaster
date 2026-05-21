##
## straight_line_pid.launch.py for mecarosmaster [SSH: ROSMASTER-YAHBOOM]
##
## Made by dirennoukpo
## Login   <diren.noukpo@epitech.eu>
##
## Started on  Mon May 18 23:00:55 2026 dirennoukpo
## Last update Thu May 20 09:42:52 2026 dirennoukpo
##
## CHANGELOG v4 :
##   [BUG FIX CRITIQUE] Les valeurs du YAML étaient systématiquement écrasées
##   par les default_value des DeclareLaunchArgument.
##
##   Mécanisme du bug :
##     parameters=[params_file, {'kp': LaunchConfiguration('kp'), ...}]
##     → le second dict a TOUJOURS priorité sur le YAML.
##     → si l'utilisateur ne passe pas kp:=X, LaunchConfiguration('kp')
##       prend la default_value de DeclareLaunchArgument, PAS la valeur du YAML.
##     → résultat : deadband=0.017 au lieu de 0.010, timeout=5.0 au lieu de 0.5, etc.
##
##   Fix appliqué :
##     On NE passe PLUS les paramètres PI dans le bloc override.
##     Le YAML est la source de vérité unique pour tous les params PID.
##     Les arguments CLI (kp, ki, ...) sont uniquement documentés pour usage
##     avancé via ros2 param set APRÈS le démarrage, ou via un YAML custom.
##
##   Architecture correcte :
##     pid_params.yaml  → source de vérité (tous les paramètres PID)
##     CLI args         → SUPPRIMÉS du bloc override du Node
##                        (présents uniquement à titre documentaire si besoin)
##
##   Pour tuner en live :
##     ros2 param set /straight_line_pid kp 2.0
##     ros2 param set /straight_line_pid deadband 0.010
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

Source de vérité des paramètres :
  config/pid_params.yaml  — TOUJOURS chargé, jamais écrasé.

Tuning en cours d'exécution (sans redémarrer) :
  ros2 param set /straight_line_pid kp 2.0
  ros2 param set /straight_line_pid deadband 0.010
  ros2 param set /straight_line_pid ki_max 0.15
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share   = FindPackageShare('straight_line_pid')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'pid_params.yaml'])

    # ── Nœud PI ───────────────────────────────────────────────────────────────
    # IMPORTANT : on ne passe QUE le YAML dans parameters[].
    # Ne JAMAIS ajouter un second dict avec des LaunchConfiguration() pour les
    # paramètres PID : cela écraserait le YAML avec les default_value des args,
    # même si l'utilisateur n'a pas fourni ces arguments au CLI.
    pid_node = Node(
        package='straight_line_pid',
        executable='straight_line_pid_node',
        name='straight_line_pid',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,   # ← SEULE SOURCE DE VÉRITÉ — jamais écrasée
        ],
        remappings=[
            ('cmd_vel_in',  '/cmd_vel'),
            ('cmd_vel_out', '/diff_drive_controller/cmd_vel'),
        ]
    )

    return LaunchDescription([
        pid_node,
    ])