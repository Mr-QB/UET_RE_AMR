"""
Launch file for UET AMR real-hardware bringup: robot_state_publisher,
ros2_control_node (loading the uet_amr_hardware/AmrHardwareInterface plugin
over serial), joint_state_broadcaster + diff_drive_controller spawners,
robot_localization's EKF (fusing wheel odometry with the D435i's IMU),
sensors (lidar + depth camera, uet_amr_bringup/launch/sensors.launch.py),
and, depending on mode:=slam|nav, either slam_toolbox or Nav2 localization +
navigation against a pre-built map, plus an RViz2 window scoped to that
mode. Mirrors uet_amr_simulation/amr_simulation.launch.py's mode:=slam|nav
structure with use_sim_time forced false.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def launch_setup(context, *args, **kwargs):
    pkg_description = get_package_share_directory('uet_amr_description')

    xacro_file = os.path.join(pkg_description, 'urdf', 'uet_amr.xacro')
    # Shared with the simulation's ign_ros2_control plugin -- see the
    # use_sim_time override below for why that's safe on real hardware.
    controller_params_file = os.path.join(pkg_description, 'config', 'controllers.yaml')

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
        # controllers.yaml hardcodes use_sim_time: true for the simulated
        # ign_ros2_control plugin (which can't take a launch-time override);
        # force it back to false here since this is the real-hardware node.
        parameters=[robot_description, controller_params_file, {'use_sim_time': False}],
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

    pkg_uet_amr_localization = get_package_share_directory('uet_amr_localization')
    ekf_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_uet_amr_localization, 'launch', 'ekf.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'imu_topic': 'camera/camera/imu',
        }.items()
    )

    # Lidar + depth camera. The EKF's imu0 input above needs this running
    # for a live odom->base_footprint transform.
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uet_amr_bringup'), 'launch', 'sensors.launch.py'])
        ])
    )

    mode = LaunchConfiguration('mode')
    is_slam_mode = EqualsSubstitution(mode, 'slam')
    is_nav_mode = EqualsSubstitution(mode, 'nav')
    use_rviz = LaunchConfiguration('rviz')

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
            'map': LaunchConfiguration('map'),
            'use_rviz': 'false',
        }.items()
    )

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

    return [
        robot_state_publisher,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        ekf_localization,
        sensors_launch,
        slam_launch,
        nav_launch,
        slam_rviz_node,
        nav_rviz_node,
    ]


def generate_launch_description():
    default_map_file = PathJoinSubstitution([
        FindPackageShare('uet_amr_navigation'), 'maps', 'warehouse.yaml'
    ])

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
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2 alongside the hardware bringup'),
        DeclareLaunchArgument('mode', default_value='slam',
                              choices=['slam', 'nav'],
                              description=(
                                  "'slam': run slam_toolbox to build a map online. "
                                  "'nav': run Nav2 localization (AMCL) + navigation against "
                                  "the map given by the 'map' argument."
                              )),
        OpaqueFunction(function=launch_setup),
    ])
