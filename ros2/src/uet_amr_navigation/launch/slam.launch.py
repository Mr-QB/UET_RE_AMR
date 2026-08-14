"""
Launch file for slam_toolbox (async, online) + RViz visualization.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_uet_amr_navigation = get_package_share_directory('uet_amr_navigation')
    pkg_uet_amr_description = get_package_share_directory('uet_amr_description')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    default_params_file = os.path.join(pkg_uet_amr_navigation, 'config', 'slam_params.yaml')
    default_rviz_config = os.path.join(pkg_uet_amr_description, 'config', 'gazebo.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation/Gazebo clock'),
        DeclareLaunchArgument('slam_params_file', default_value=default_params_file,
                              description='Full path to the slam_toolbox params file'),
        DeclareLaunchArgument('rviz_config_file', default_value=default_rviz_config,
                              description='Full path to the RViz config file'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Whether to start RViz'),
        slam_toolbox_launch,
        rviz_node,
    ])
