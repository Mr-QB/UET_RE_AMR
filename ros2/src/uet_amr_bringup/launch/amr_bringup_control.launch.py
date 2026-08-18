"""
Control-only counterpart to amr_bringup.launch.py: robot_state_publisher,
ros2_control_node (loading the uet_amr_hardware/AmrHardwareInterface plugin
over serial), joint_state_broadcaster + diff_drive_controller spawners,
robot_localization's EKF (fusing wheel odometry with the D435i's IMU), and
sensors (lidar + depth camera, uet_amr_bringup/launch/sensors.launch.py).

No SLAM, no Nav2, no RViz -- for testing/running the hardware and control
loop in isolation from the rest of the navigation stack.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG_DESCRIPTION = FindPackageShare('uet_amr_description')


def generate_launch_description():
    xacro_file = PathJoinSubstitution([PKG_DESCRIPTION, 'urdf', 'uet_amr.xacro'])
    # Shared with the simulation's ign_ros2_control plugin -- see the
    # use_sim_time override below for why that's safe on real hardware.
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

    ekf_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uet_amr_localization'), 'launch', 'ekf.launch.py'])
        ]),
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
        robot_state_publisher,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        ekf_localization,
        sensors_launch,
    ])
