"""
Launch file for UET AMR Gazebo simulation bringup.
Starts the Gazebo simulation (uet_amr_simulation/gazebo.launch.py), and
either slam_toolbox (uet_amr_navigation/slam.launch.py) or Nav2 localization
+ navigation against a pre-built map (uet_amr_navigation/navigation.launch.py)
depending on mode:=slam|nav, and, unless disabled with rviz:=false, an RViz2
window pre-configured (uet_amr_description/config/gazebo.rviz) to visualize it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='warehouse')
    default_map_file = PathJoinSubstitution([
        FindPackageShare('uet_amr_navigation'),
        'maps', 'warehouse.yaml'
    ])
    map_yaml_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')
    mode = LaunchConfiguration('mode')
    is_slam_mode = EqualsSubstitution(mode, 'slam')
    is_nav_mode = EqualsSubstitution(mode, 'nav')

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uet_amr_simulation'),
                'launch', 'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time, 'world': world}.items()
    )

    # slam_toolbox (online async mapping). Its own internal RViz is forced
    # off here so amr_simulation.launch.py's own 'rviz' arg stays the single
    # source of truth for whether RViz launches.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uet_amr_navigation'),
                'launch', 'slam.launch.py'
            ])
        ]),
        condition=IfCondition(is_slam_mode),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': 'false',
        }.items()
    )

    # Nav2 localization (map_server + AMCL) + navigation against a
    # pre-built map. Its own internal RViz is likewise forced off.
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uet_amr_navigation'),
                'launch', 'navigation.launch.py'
            ])
        ]),
        condition=IfCondition(is_nav_mode),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'use_rviz': 'false',
        }.items()
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('uet_amr_description'),
        'config', 'gazebo.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', rviz_config_file,
            # gazebo.rviz bakes in Fixed Frame: odom (correct when nothing
            # publishes map). Both slam and nav modes publish map->odom, so
            # point RViz at map in that case instead of editing the file.
            '-f', 'map',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='Gazebo world name'),
        DeclareLaunchArgument('map', default_value=default_map_file,
                              description="Full path to the map yaml file to load in mode:=nav"),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2 alongside Gazebo'),
        DeclareLaunchArgument('mode', default_value='slam',
                              choices=['slam', 'nav'],
                              description=(
                                  "'slam': run slam_toolbox to build a map online. "
                                  "'nav': run Nav2 localization (AMCL) + navigation against "
                                  "the map given by the 'map' argument."
                              )),
        gazebo_launch,
        slam_launch,
        nav_launch,
        rviz_node,
    ])
