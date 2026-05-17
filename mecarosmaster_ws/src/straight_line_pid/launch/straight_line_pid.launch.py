"""
Launch the straight_line_pid node.

REMAPPING STRATEGY (no phantom topics):
  - Your teleop/Nav2 publishes to /cmd_vel (unchanged)
  - This node's input  (cmd_vel_in)  <- remapped from /cmd_vel
  - This node's output (cmd_vel_out) -> remapped to /diff_drive_controller/cmd_vel
  - The diff_drive_controller STOPS listening to /cmd_vel directly
    (its own cmd_vel subscription is remapped to an unused topic by the
     diff_drive_controller launch, or simply never gets a message because
     this node is the only publisher on /diff_drive_controller/cmd_vel)

Result: topic list looks exactly as before, zero new topics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('straight_line_pid')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'pid_params.yaml'])

    kp_arg   = DeclareLaunchArgument('kp',   default_value='1.0')
    ki_arg   = DeclareLaunchArgument('ki',   default_value='0.01')
    kd_arg   = DeclareLaunchArgument('kd',   default_value='0.1')
    freq_arg = DeclareLaunchArgument('freq', default_value='50.0')

    pid_node = Node(
        package='straight_line_pid',
        executable='straight_line_pid_node',
        name='straight_line_pid',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {
                'control_frequency': LaunchConfiguration('freq'),
                'pid.p':             LaunchConfiguration('kp'),
                'pid.i':             LaunchConfiguration('ki'),
                'pid.d':             LaunchConfiguration('kd'),
            }
        ],
        remappings=[
            # INPUT:  intercept the standard /cmd_vel
            ('cmd_vel_in',  '/cmd_vel'),
            # OUTPUT: publish corrected cmd directly to the controller
            ('cmd_vel_out', '/diff_drive_controller/cmd_vel'),
        ]
    )

    return LaunchDescription([kp_arg, ki_arg, kd_arg, freq_arg, pid_node])