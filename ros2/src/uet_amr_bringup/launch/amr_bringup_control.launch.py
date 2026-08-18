"""
Minimal control-only bringup: robot_state_publisher and ros2_control_node
(loading the uet_amr_hardware/AmrHardwareInterface plugin over serial), plus
joint_state_broadcaster + diff_drive_controller. Nothing else -- no EKF, no
lidar, no depth camera, no SLAM/Nav2/RViz. For running the drive base by
itself, e.g. over teleop, with no other sensors attached.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG_DESCRIPTION = FindPackageShare('uet_amr_description')


def generate_launch_description():
    xacro_file = PathJoinSubstitution([PKG_DESCRIPTION, 'urdf', 'uet_amr.xacro'])
    # Shared with the simulation's ign_ros2_control plugin -- see
    # real_hardware_overrides.yaml for why that's safe on real hardware.
    controller_params_file = PathJoinSubstitution([PKG_DESCRIPTION, 'config', 'controllers.yaml'])
    real_hardware_overrides_file = PathJoinSubstitution(
        [PKG_DESCRIPTION, 'config', 'real_hardware_overrides.yaml'])

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
        # real_hardware_overrides_file forces use_sim_time back to false --
        # deliberately a plain params file, not a launch-time parameter dict
        # scoped via Node name=/namespace=; see that file for why (short
        # version: name=/namespace= makes launch_ros emit an unscoped
        # `-r __node:=... -r __ns:=...` remap that silently renames every
        # controller's own internal node too, breaking their parameter
        # overrides entirely).
        #
        # robot_description is NOT passed here (unlike robot_state_publisher
        # above): ros2_control_node logs passing it directly as deprecated in
        # favor of the '~/robot_description' topic robot_state_publisher
        # already publishes.
        parameters=[controller_params_file, real_hardware_overrides_file],
        # Without robot_description passed as a direct parameter (see above),
        # ros2_control_node instead subscribes to '~/robot_description', which
        # resolves to the private /controller_manager/robot_description -- but
        # robot_state_publisher only ever publishes the global /robot_description.
        # Remap so it actually receives it.
        remappings=[('~/robot_description', '/robot_description')],
        output='screen',
    )

    # Both controllers in a single spawner call, loaded sequentially -- two
    # concurrent spawner processes hitting controller_manager's single-threaded
    # service executor at the same moment stalls it indefinitely; every
    # load_controller/list_controllers call then times out until the spawners
    # give up.
    # -p/--param-file: without this, diff_drive_controller's own node relies on
    # picking up controllers.yaml as a process-wide global argument, which does
    # not reliably reach it -- it falls back to hard-coded defaults (e.g.
    # wheel_separation=0.0), which then fails diff_drive_controller's own
    # declare-time floating_point_range check (must be >0) and aborts init.
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster', 'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controller_params_file,
        ],
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
        controller_spawner,
    ])
