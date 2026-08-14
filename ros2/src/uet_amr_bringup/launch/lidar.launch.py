"""
RPLIDAR bringup. frame_id matches the lidar link published by
uet_amr_description/urdf/uet_amr.xacro.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 256000,
            'frame_id': 'front_lidar',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': '',
        }],
        output='screen',
    )

    return LaunchDescription([
        rplidar_node,
    ])
