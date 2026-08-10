import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_teleop = get_package_share_directory('uet_amr_teleop')
    joy_params_file = os.path.join(pkg_teleop, 'config', 'joy_teleop.yaml')

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[joy_params_file],
            remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
        ),
        Node(
            package='uet_amr_teleop',
            executable='joy_speed_adjust.py',
            name='joy_speed_adjust',
        ),
    ])
