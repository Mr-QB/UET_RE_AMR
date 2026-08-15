"""
Launch file for UET AMR real-hardware bringup: base (amr_bringup.launch.py)
+ sensors (sensors.launch.py) + either slam_toolbox or Nav2 localization +
navigation against a pre-built map, depending on mode:=slam|nav, plus an
RViz2 window scoped to that mode. Mirrors amr_simulation.launch.py's
structure with use_sim_time forced false.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    default_map_file = PathJoinSubstitution([
        FindPackageShare('uet_amr_navigation'), 'maps', 'warehouse.yaml'
    ])
    map_yaml_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')
    mode = LaunchConfiguration('mode')
    is_slam_mode = EqualsSubstitution(mode, 'slam')
    is_nav_mode = EqualsSubstitution(mode, 'nav')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uet_amr_bringup'), 'launch', 'amr_bringup.launch.py'])
        ]),
        launch_arguments={'serial_port': serial_port, 'baud_rate': baud_rate}.items()
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uet_amr_bringup'), 'launch', 'sensors.launch.py'])
        ])
    )

    # slam_toolbox (online async mapping). Its own internal RViz is forced
    # off here so this launch file's own 'rviz' arg stays the single source
    # of truth for whether RViz launches.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uet_amr_navigation'), 'launch', 'slam.launch.py'])
        ]),
        condition=IfCondition(is_slam_mode),
        launch_arguments={
            'use_sim_time': 'false',
            'use_rviz': 'false',
        }.items()
    )

    # Nav2 localization (map_server + AMCL) + navigation against a pre-built
    # map. Its own internal RViz is likewise forced off.
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uet_amr_navigation'), 'launch', 'navigation.launch.py'])
        ]),
        condition=IfCondition(is_nav_mode),
        launch_arguments={
            'use_sim_time': 'false',
            'map': map_yaml_file,
            'use_rviz': 'false',
        }.items()
    )

    slam_rviz_config = PathJoinSubstitution([
        FindPackageShare('uet_amr_navigation'), 'rviz', 'slam.rviz'
    ])
    nav_rviz_config = PathJoinSubstitution([
        FindPackageShare('uet_amr_navigation'), 'rviz', 'navigation.rviz'
    ])

    # Each RViz node needs mode:=<x> AND rviz:=true. Nesting the rviz:=true
    # IfCondition inside a mode-conditioned GroupAction ANDs the two using
    # two ordinary IfConditions, rather than hand-rolling string comparisons.
    slam_rviz_node = GroupAction(
        condition=IfCondition(is_slam_mode),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', slam_rviz_config],
                parameters=[{'use_sim_time': False}],
                condition=IfCondition(use_rviz),
                output='screen',
            ),
        ]
    )

    nav_rviz_node = GroupAction(
        condition=IfCondition(is_nav_mode),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', nav_rviz_config],
                parameters=[{'use_sim_time': False}],
                condition=IfCondition(use_rviz),
                output='screen',
            ),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='Serial device for the AMR base controller MCU'),
        DeclareLaunchArgument('baud_rate', default_value='921600',
                              description='Serial baud rate (firmware/amr_uart_bridge is fixed at 921600)'),
        DeclareLaunchArgument('map', default_value=default_map_file,
                              description="Full path to the map yaml file to load in mode:=nav"),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2 alongside the hardware bringup'),
        DeclareLaunchArgument('mode', default_value='slam',
                              choices=['slam', 'nav'],
                              description=(
                                  "'slam': run slam_toolbox to build a map online. "
                                  "'nav': run Nav2 localization (AMCL) + navigation against "
                                  "the map given by the 'map' argument."
                              )),
        bringup_launch,
        sensors_launch,
        slam_launch,
        nav_launch,
        slam_rviz_node,
        nav_rviz_node,
    ])
