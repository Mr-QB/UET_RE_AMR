from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_config = PathJoinSubstitution(
        [FindPackageShare('uet_amr_localization'), 'config', 'ekf.yaml']
    )

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
