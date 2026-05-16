##
## mecamate_launch.py for mecamate_lib [SSH: ROSMASTER-YAHBOOM] in /home/rosmaster/mecamate_lib
##
## Made by dirennoukpo
## Login   <diren.noukpo@epitech.eu>
##
## Started on  Sat May 16 07:40:54 2026 dirennoukpo
## Last update Sun May 16 17:53:45 2026 dirennoukpo
##
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mecarosmaster_control")

    # ── Arguments ──────────────────────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="/dev/myserial",
        description="Serial port Mecarosmaster")

    car_type_arg = DeclareLaunchArgument(
        "car_type", default_value="1",
        description="1=X3 2=X3_PLUS 4=X1 5=R2")

    use_ros2_control_arg = DeclareLaunchArgument(
        "use_ros2_control", default_value="true",
        description="Use ros2_control hardware interface")

    # ── Robot description (xacro) ──────────────────────────────────────────────
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg, "urdf", "mecarosmaster.urdf.xacro"]),
        " serial_port:=", LaunchConfiguration("serial_port"),
        " car_type:=", LaunchConfiguration("car_type"),
    ])
    robot_description = {"robot_description": robot_description_content}

    # ── robot_state_publisher ──────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ── ros2_control path ──────────────────────────────────────────────────────
    controllers_yaml = PathJoinSubstitution(
        [pkg, "config", "mecarosmaster_controllers.yaml"]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )

    spawn_mecanum = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller",
                   "--controller-manager", "/controller_manager"],
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )

    delay_mecanum = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_mecanum],
        )
    )

    # ── Standalone node path (sans ros2_control) ───────────────────────────────
    mecarosmaster_node = Node(
        package="mecarosmaster_control",
        executable="mecarosmaster_node",
        output="screen",
        parameters=[{
            "serial_port":      LaunchConfiguration("serial_port"),
            "car_type":         1,
            "publish_rate":     50.0,
            "publish_tf":       True,
            "cmd_vel_timeout":  0.5,
        }],
        condition=UnlessCondition(LaunchConfiguration("use_ros2_control")),
    )

    return LaunchDescription([
        serial_port_arg,
        car_type_arg,
        use_ros2_control_arg,
        robot_state_publisher,
        controller_manager,
        spawn_jsb,
        delay_mecanum,
        mecarosmaster_node,
    ])
