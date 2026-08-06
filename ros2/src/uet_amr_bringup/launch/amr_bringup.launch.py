"""
Launch file for UET AMR real-hardware bringup.
Starts: robot_state_publisher, ros2_control_node (loading the
uet_amr_hardware/AmrHardwareInterface plugin over serial), and spawns
joint_state_broadcaster + diff_drive_controller.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    pkg_description = get_package_share_directory('uet_amr_description')
    pkg_hardware = get_package_share_directory('uet_amr_hardware')

    xacro_file = os.path.join(pkg_description, 'urdf', 'uet_amr.xacro')
    controller_params_file = os.path.join(pkg_hardware, 'config', 'ros2_control.yaml')

    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            'use_real_hardware': 'true',
            'serial_port': LaunchConfiguration('serial_port').perform(context),
            'baud_rate': LaunchConfiguration('baud_rate').perform(context),
        },
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_params_file],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )

    return [
        robot_state_publisher,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial device for the AMR base controller MCU'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='921600',
            description='Serial baud rate (firmware/amr_uart_bridge is fixed at 921600)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
