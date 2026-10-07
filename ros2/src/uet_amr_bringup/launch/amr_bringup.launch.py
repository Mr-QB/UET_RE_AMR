"""
Launch file for UET AMR real-hardware bringup: robot_state_publisher,
ros2_control_node (loading the uet_amr_hardware/AmrHardwareInterface plugin
over serial), joint_state_broadcaster + diff_drive_controller spawners,
robot_localization's EKF (fusing wheel odometry with the D435i's IMU),
sensors (lidar + depth camera, uet_amr_bringup/launch/sensors.launch.py),
and an amr_mode_manager that starts SLAM or Nav2 and switches them at runtime
through /amr/set_mode. The selected mode can optionally start its RViz window.
Hardware, localization, and sensors stay alive during mode changes.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
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

    mode_manager = Node(
        package='uet_amr_bringup',
        executable='amr_mode_manager.py',
        name='amr_mode_manager',
        output='screen',
        parameters=[{
            'initial_mode': LaunchConfiguration('mode'),
            'initial_map': LaunchConfiguration('map'),
            'map_directory': LaunchConfiguration('map_directory'),
            'use_rviz': LaunchConfiguration('rviz'),
            'use_nvblox': LaunchConfiguration('use_nvblox'),
            'use_sim_time': False,
        }],
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
                              description="Map YAML to load at startup in mode:=nav"),
        DeclareLaunchArgument(
            'map_directory',
            default_value='',
            description='Root directory for maps saved by /amr/set_mode; each map gets its own subdirectory'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz2 alongside the hardware bringup'),
        DeclareLaunchArgument('use_nvblox', default_value='',
                              description='Override Nav2 use_nvblox (empty means auto-detect)'),
        DeclareLaunchArgument('publish_ws', default_value='true',
                      description='Publish ROS topics/services over websocket via Foxglove Bridge'),
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
        mode_manager,
    ])
