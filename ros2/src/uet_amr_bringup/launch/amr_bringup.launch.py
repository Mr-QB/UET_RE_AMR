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
Launch file for UET AMR real-hardware bringup: robot_state_publisher,
ros2_control_node (loading the uet_amr_hardware/AmrHardwareInterface plugin
over serial), joint_state_broadcaster + diff_drive_controller spawners,
robot_localization's EKF (fusing wheel odometry with the D435i's IMU),
sensors (lidar + depth camera, uet_amr_bringup/launch/sensors.launch.py),
and an amr_mode_manager that starts SLAM or Nav2 and switches them at runtime
through /amr/set_mode. Joystick and keyboard teleop plus their gate (uet_amr_teleop) are included too; the gate starts
with teleop off (teleop_source:=none) and /amr/set_teleop selects joy/keyboard/none. use_teleop:=false
leaves teleop out completely. foxglove:=true also starts Foxglove Bridge (off by default). The selected mode can
optionally start its RViz window.
Hardware, localization, and sensors stay alive during mode changes.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_map_file = PathJoinSubstitution([
        FindPackageShare('uet_amr_maps'), 'maps', 'gd2', 'gd2.yaml'
    ])

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uet_amr_bringup'), 'launch', 'control.launch.py'])
        ]),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
        }.items()
    )

    ekf_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uet_amr_localization'), 'launch', 'ekf.launch.py'])
        ]),
        launch_arguments={
            'imu_topic': 'camera/camera/imu',
        }.items()
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uet_amr_bringup'), 'launch', 'sensors.launch.py'])
        ])
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uet_amr_teleop'), 'launch', 'teleop.launch.py'])
        ]),
        launch_arguments={'initial_source': LaunchConfiguration('teleop_source')}.items(),
        condition=IfCondition(LaunchConfiguration('use_teleop')),
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'max_qos_depth': 300,              # default 25 truncates /tf history
            # Wi-Fi: drop what the link cannot carry instead of building a growing backlog (default 10 MB = seconds of lag)
            'send_buffer_limit': 4000000,
            'use_compression': True,           # websocket deflate: costmaps, grids and the mesh markers shrink a lot
            'best_effort_qos_topic_whitelist': ['/scan', '/nvblox_node/.*pointcloud', '/camera/.*'],
            # Not offered to Foxglove, so one click cannot saturate the link: raw/depth/theora image streams (use
            # .../image_raw/compressed), nvblox's raw Mesh and voxel-layer markers (use /nvblox_node/mesh_marker, which
            # sends only changed blocks), the 10 Hz ESDF point clouds and the people-segmentation masks/tensors (/unet, uncompressed). Remove the pattern to get a topic back.
            'topic_whitelist': [r'^(?!/nvblox_node/(mesh|tsdf_layer|tsdf_layer_marker|color_layer|color_layer_marker|'
                                r'static_esdf_pointcloud|pessimistic_.*|back_projected_depth/.*)$)'
                                r'(?!.*/(image_raw|image_rect_raw)$)(?!.*/(compressedDepth|theora)$)'
                                # PeopleSemSegNet internals; the colour mask is an uncompressed Image (1.6 MB x ~22 Hz = 35 MB/s), so only its
                                # JPEG version (/unet/colored_segmentation_mask/compressed) is offered
                                r'(?!/unet/(?!colored_segmentation_mask/compressed$))(?!/(dnn_image_encoder|tensor_pub|tensor_sub)(/|$))(?!.*/image_tensor$).*'],
        }],
        arguments=['--ros-args', '--log-level', 'foxglove_bridge:=warn'],  # drop per-subscription/channel INFO spam
        condition=IfCondition(LaunchConfiguration('foxglove')),
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
            'use_rviz': LaunchConfiguration('use_rviz'),
            # Strings because amr_mode_manager parses the override as "true"/"false".
            'use_detection': ParameterValue(
                LaunchConfiguration('use_detection'), value_type=str),
            'use_sim_time': False,
        }],
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
        DeclareLaunchArgument('map', default_value=default_map_file,
                              description="Map YAML to load at startup in mode:=nav"),
        DeclareLaunchArgument(
            'map_directory',
            default_value='auto',
            description='Root directory for maps saved by /amr/save_map; each map gets its own subdirectory'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Launch RViz2 alongside the hardware bringup'),
        DeclareLaunchArgument('use_detection', default_value='false',
                              description='nav mode: leave people out of the nvblox map using PeopleSemSegNet '
                                          '(needs the model, see docs/setup.md)'),
        DeclareLaunchArgument('foxglove', default_value='false',
                              description='Start Foxglove Bridge (websocket on port 8765) for Foxglove Studio; off by default'),
        DeclareLaunchArgument('use_teleop', default_value='true',
                              description='Start the joystick and keyboard teleop nodes and the teleop gate (/amr/set_teleop); false leaves teleop out'),
        DeclareLaunchArgument('teleop_source', default_value='none',
                              choices=['none', 'joy', 'keyboard'],
                              description='Teleop source allowed to drive at startup; none until enabled via /amr/set_teleop'),
        DeclareLaunchArgument('mode', default_value='slam',
                              choices=['slam', 'nav'],
                              description=(
                                  "'slam': run slam_toolbox to build a map online. "
                                  "'nav': run Nav2 localization (AMCL) + navigation against "
                                  "the map given by the 'map' argument."
                              )),
        control_launch,
        ekf_localization,
        sensors_launch,
        teleop_launch,
        foxglove_bridge,
        mode_manager,
    ])
