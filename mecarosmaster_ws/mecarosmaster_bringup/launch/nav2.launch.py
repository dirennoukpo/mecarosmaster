from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_pkg = FindPackageShare("nav2_bringup")
    bringup_pkg = FindPackageShare("mecarosmaster_bringup")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([
            bringup_pkg, "config", "nav2_params.yaml"])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([nav2_pkg, "launch", "bringup_launch.py"])
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
            }.items(),
        ),
    ])
