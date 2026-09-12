"""
Launch file for UET AMR real-hardware bringup: robot_state_publisher,
ros2_control_node (loading the uet_amr_hardware/AmrHardwareInterface plugin
over serial), joint_state_broadcaster + diff_drive_controller spawners,
robot_localization's EKF (fusing wheel odometry with the D435i's IMU),
sensors (lidar + depth camera, uet_amr_bringup/launch/sensors.launch.py),
and, depending on mode:=slam|nav, either slam_toolbox or Nav2 localization +
navigation against a pre-built map, plus an RViz2 window scoped to that
mode. Mirrors uet_amr_simulation/simulation.launch.py's mode:=slam|nav
structure with use_sim_time forced false.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG_DESCRIPTION = FindPackageShare('uet_amr_description')


def generate_launch_description():
    default_map_file = PathJoinSubstitution([
        FindPackageShare('uet_amr_navigation'), 'maps', 'warehouse.yaml'
    ])

    xacro_file = PathJoinSubstitution([PKG_DESCRIPTION, 'urdf', 'uet_amr.xacro'])
    controller_params_file = PathJoinSubstitution([PKG_DESCRIPTION, 'config', 'controllers.yaml'])

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', xacro_file,
                ' use_real_hardware:=true',
                ' serial_port:=', LaunchConfiguration('serial_port'),
                ' baud_rate:=', LaunchConfiguration('baud_rate'),
            ]),
            value_type=str,
        )
    }

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
        parameters=[controller_params_file],
        remappings=[('~/robot_description', '/robot_description')],
        output='screen',
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster', 'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controller_params_file,
        ],
    )

    ekf_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uet_amr_localization'), 'launch', 'ekf.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'imu_topic': 'camera/camera/imu',
        }.items()
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uet_amr_bringup'), 'launch', 'sensors.launch.py'])
        ])
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_ws')),
    )

    mode = LaunchConfiguration('mode')
    is_slam_mode = EqualsSubstitution(mode, 'slam')
    is_nav_mode = EqualsSubstitution(mode, 'nav')
    use_rviz = LaunchConfiguration('rviz')
    use_rtabmap = LaunchConfiguration('use_rtabmap')

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

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uet_amr_navigation'), 'launch', 'navigation.launch.py'])
        ]),
        condition=IfCondition(is_nav_mode),
        launch_arguments={
            'use_sim_time': 'false',
            'map': LaunchConfiguration('map'),
            'use_rviz': 'false',
        }.items()
    )

    # rtabmap launch 
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'])
        ]),
        condition=IfCondition(use_rtabmap),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start --RGBD/LinearUpdate 0.0 --RGBD/AngularUpdate 0.0',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'base_footprint',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.05',
            'topic_queue_size': '100',
            'sync_queue_size': '100',
            'qos': '2',
            'viz': 'true',
            'use_sim_time': 'false',
        }.items()
    )

    slam_rviz_node = GroupAction(
        condition=IfCondition(is_slam_mode),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', PathJoinSubstitution(
                    [FindPackageShare('uet_amr_navigation'), 'rviz', 'slam.rviz'])],
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
                arguments=['-d', PathJoinSubstitution(
                    [FindPackageShare('uet_amr_navigation'), 'rviz', 'navigation.rviz'])],
                parameters=[{'use_sim_time': False}],
                condition=IfCondition(use_rviz),
                output='screen',
            ),
        ]
    )

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
        DeclareLaunchArgument('map', default_value=default_map_file,
                              description="Full path to the map yaml file to load in mode:=nav"),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz2 alongside the hardware bringup'),
        DeclareLaunchArgument('publish_ws', default_value='true',
                      description='Publish ROS topics/services over websocket via Foxglove Bridge'),
        DeclareLaunchArgument(
            'use_rtabmap',
            default_value='true',
            description='Launch RTAB-Map alongside bringup'
        ),
        DeclareLaunchArgument('mode', default_value='slam',
                              choices=['slam', 'nav'],
                              description=(
                                  "'slam': run slam_toolbox to build a map online. "
                                  "'nav': run Nav2 localization (AMCL) + navigation against "
                                  "the map given by the 'map' argument."
                              )),
        robot_state_publisher,
        controller_manager_node,
        controller_spawner,
        ekf_localization,
        sensors_launch,
        foxglove_bridge,
        slam_launch,
        nav_launch,
        rtabmap_launch,
        slam_rviz_node,
        nav_rviz_node,
    ])