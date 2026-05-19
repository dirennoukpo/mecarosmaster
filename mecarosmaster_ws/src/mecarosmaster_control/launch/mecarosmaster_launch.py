##
## mecarosmaster_launch.py  —  ROS 2 Humble
## Made by dirennoukpo  <diren.noukpo@epitech.eu>
##
## v7 : mecanum_drive_controller → diff_drive_controller
##      (Rosmaster X3 avec roues à crampons, pas mecanum)
##

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = FindPackageShare("mecarosmaster_control")

    # ── Arguments ──────────────────────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyUSB0",
        description="Port série (ex: /dev/ttyUSB0)",
    )
    car_type_arg = DeclareLaunchArgument(
        "car_type", default_value="1",
        description="Type châssis : 1=X3  2=X3_PLUS  4=X1  5=R2",
    )
    use_ros2_control_arg = DeclareLaunchArgument(
        "use_ros2_control", default_value="true",
        description="true → ros2_control + diff_drive_controller ; false → node autonome",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
    )

    # ── Robot description ─────────────────────────────────────────────────────
    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([pkg, "urdf", "mecarosmaster.urdf.xacro"]),
            " serial_port:=", LaunchConfiguration("serial_port"),
            " car_type:=",    LaunchConfiguration("car_type"),
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # ── robot_state_publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # ── controller_manager ────────────────────────────────────────────────────
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,   # passé en param direct (fix race condition QoS Humble)
            PathJoinSubstitution([pkg, "config", "mecarosmaster_controllers.yaml"]),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )

    # ── Spawner joint_state_broadcaster (délai 3 s) ───────────────────────────
    spawn_jsb = TimerAction(
        period=3.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_ros2_control")),
        )],
    )

    # ── Spawner diff_drive_controller (délai 5 s) ────────────────────────────
    spawn_diff = TimerAction(
        period=5.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_ros2_control")),
        )],
    )

    # ── Node autonome (sans ros2_control) ────────────────────────────────────
    mecarosmaster_node = Node(
        package="mecarosmaster_control",
        executable="mecarosmaster_node",
        output="screen",
        parameters=[{
            "serial_port":     LaunchConfiguration("serial_port"),
            "car_type":        LaunchConfiguration("car_type"),
            "publish_rate":    50.0,
            "publish_tf":      True,
            "cmd_vel_timeout": 0.5,
            "use_sim_time":    LaunchConfiguration("use_sim_time"),
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
        spawn_diff,
        mecarosmaster_node,
    ])