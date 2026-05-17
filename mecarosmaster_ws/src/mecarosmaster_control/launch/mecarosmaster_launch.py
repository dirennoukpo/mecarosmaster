##
## mecarosmaster_launch.py  —  ROS 2 Humble
##
## Made by dirennoukpo  <diren.noukpo@epitech.eu>
##
## Corrections v5 :
##   • BUG YAML RÉGLÉ : robot_description wrappé dans ParameterValue(..., str)
##     → évite l'erreur "Unable to parse the value of parameter robot_description as yaml"
##   • Reste des corrections v4 conservées (pas de robot_description dans
##     controller_manager, TimerAction pour la race condition RSP, etc.)
##

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = FindPackageShare("mecarosmaster_control")

    # ── Arguments ──────────────────────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/myserial",
        description="Port série du Mecarosmaster (ex: /dev/ttyUSB0)",
    )
    car_type_arg = DeclareLaunchArgument(
        "car_type",
        default_value="1",
        description="Type de châssis : 1=X3  2=X3_PLUS  4=X1  5=R2",
    )
    use_ros2_control_arg = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="true",
        description="true → ros2_control + mecanum_drive_controller ; "
                    "false → node autonome",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Utiliser l'horloge simulée (Gazebo / rosbag)",
    )

    # ── Robot description ─────────────────────────────────────────────────────
    # CORRECTION CRITIQUE : ParameterValue(..., value_type=str) force le type
    # string et empêche le système de paramètres ROS 2 de tenter un parse YAML
    # sur la sortie de xacro (qui contient des balises XML → erreur de parse).
    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([pkg, "urdf", "mecarosmaster.urdf.xacro"]),
            " serial_port:=", LaunchConfiguration("serial_port"),
            " car_type:=",    LaunchConfiguration("car_type"),
        ]),
        value_type=str,
    )

    # ── robot_state_publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # ── Chemin YAML controllers ───────────────────────────────────────────────
    controllers_yaml = PathJoinSubstitution(
        [pkg, "config", "mecarosmaster_controllers.yaml"]
    )

    # ── ros2_control_node ─────────────────────────────────────────────────────
    # Pas de robot_description ici : le CM lit uniquement le topic RSP.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )

    # ── Spawner joint_state_broadcaster (délai 2 s pour laisser RSP publier) ──
    spawn_jsb = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_ros2_control")),
            )
        ],
    )

    # ── Spawner mecanum_drive_controller (délai 4 s : 2s RSP + ~2s JSB) ──────
    spawn_mecanum = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "mecanum_drive_controller",
                    "--controller-manager", "/controller_manager",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_ros2_control")),
            )
        ],
    )

    # ── Node autonome (sans ros2_control) ────────────────────────────────────
    mecarosmaster_node = Node(
        package="mecarosmaster_control",
        executable="mecarosmaster_node",
        output="screen",
        parameters=[{
            "serial_port":      LaunchConfiguration("serial_port"),
            "car_type":         LaunchConfiguration("car_type"),
            "publish_rate":     50.0,
            "publish_tf":       True,
            "cmd_vel_timeout":  0.5,
            "use_sim_time":     LaunchConfiguration("use_sim_time"),
        }],
        condition=UnlessCondition(LaunchConfiguration("use_ros2_control")),
    )

    return LaunchDescription([
        serial_port_arg,
        car_type_arg,
        use_ros2_control_arg,
        use_sim_time_arg,
        robot_state_publisher,
        controller_manager,
        spawn_jsb,
        spawn_mecanum,
        mecarosmaster_node,
    ])