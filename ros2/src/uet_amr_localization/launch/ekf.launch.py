import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('uet_amr_localization')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='imu/data',
        description='Actual IMU topic to fuse, remapped onto the ekf.yaml canonical imu0 name',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('imu/data', LaunchConfiguration('imu_topic')),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        imu_topic_arg,
        ekf_node,
    ])
