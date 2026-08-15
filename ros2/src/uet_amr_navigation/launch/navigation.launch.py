"""
Launch file for Nav2 localization (map_server + AMCL) and navigation
(planner/controller/bt_navigator) against a pre-built map, plus RViz
visualization.
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
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    default_params_file = os.path.join(pkg_uet_amr_navigation, 'config', 'nav2.yaml')
    default_map_file = os.path.join(pkg_uet_amr_navigation, 'maps', 'warehouse.yaml')
    default_rviz_config = os.path.join(pkg_uet_amr_description, 'config', 'gazebo.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
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
        DeclareLaunchArgument('map', default_value=default_map_file,
                              description='Full path to the map yaml file to load'),
        DeclareLaunchArgument('params_file', default_value=default_params_file,
                              description='Full path to the Nav2 params file'),
        DeclareLaunchArgument('rviz_config_file', default_value=default_rviz_config,
                              description='Full path to the RViz config file'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Whether to start RViz'),
        localization_launch,
        navigation_launch,
        rviz_node,
    ])
