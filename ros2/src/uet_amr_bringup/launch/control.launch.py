# Copyright (c) 2026 Department of Robotics, University of Engineering and
#                    Technology, Vietnam National University, Hanoi (VNU).
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to
# deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

"""
Drive-base control: robot_state_publisher and ros2_control_node
(loading the uet_amr_hardware/AmrHardwareInterface plugin over serial), plus
joint_state_broadcaster + diff_drive_controller. Included by amr_bringup.launch.py; can also
be run on its own to drive the base (e.g. over teleop) with no sensors attached.
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
    controller_params_file = PathJoinSubstitution([FindPackageShare('uet_amr_bringup'), 'config', 'controllers.yaml'])

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', xacro_file,
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
        # robot_description is NOT passed here (unlike robot_state_publisher
        # above): ros2_control_node logs passing it directly as deprecated in
        # favor of the '~/robot_description' topic robot_state_publisher
        # already publishes.
        parameters=[controller_params_file],
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
            default_value='/dev/esp32',
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
