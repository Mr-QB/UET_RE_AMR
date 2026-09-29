# Copyright (c) 2026 Department of Robotics, University of Engineering and
#                     Technology, Vietnam National University, Hanoi (VNU).
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
Launch file for navigation mode of UET AMR.
"""
import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def _has_nvidia_gpu():
    """Mirrors tools/common.sh's has_nvidia_gpu(): used only to pick the
    default value of the use_nvblox launch argument below -- still
    overridable on the command line either way."""
    if os.path.exists('/etc/nv_tegra_release'):
        return True
    try:
        subprocess.run(['nvidia-smi'], stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL, check=True)
        return True
    except (OSError, subprocess.CalledProcessError):
        return False


def generate_launch_description():
    default_use_nvblox = 'true' if _has_nvidia_gpu() else 'false'

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    use_nvblox = LaunchConfiguration('use_nvblox')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Picks nav2_nvblox.yaml (nvblox-backed local costmap) or nav2.yaml (the
    # CPU voxel_layer fallback) based on use_nvblox -- evaluated at launch
    # time, so an explicit `use_nvblox:=` override on the CLI still works.
    default_params_filename = PythonExpression(
        ["'nav2_nvblox.yaml' if '", use_nvblox, "' == 'true' else 'nav2.yaml'"])
    default_params_file = PathJoinSubstitution([
        FindPackageShare('uet_amr_navigation'), 'config', default_params_filename
    ])
    default_map_file = PathJoinSubstitution([
        FindPackageShare('uet_amr_navigation'), 'maps', 'warehouse.yaml'
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare('uet_amr_navigation'), 'rviz', 'navigation.rviz'
    ])

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
        }.items()
    )

    # nav2_bringup's navigation_launch.py hardcodes velocity_smoother's final
    # output remap as ('cmd_vel_smoothed', 'cmd_vel'). Override just that one
    # rule (a global SetRemap takes priority over a node's own remappings=[])
    # so Nav2's final commanded velocity lands directly on the topic
    # diff_drive_controller actually listens on, instead of the unused
    # '/cmd_vel'. Scoped to this group only, so it doesn't leak into
    # localization_launch/rviz_node below.
    navigation_launch = GroupAction(
        actions=[
            SetRemap(src='cmd_vel_smoothed', dst='/diff_drive_controller/cmd_vel_unstamped'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                }.items()
            ),
        ]
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

    # Only runs inside the Isaac ROS Docker container (nvblox_ros needs CUDA
    # Toolkit + NITROS, not available natively) -- see docs/isaac_ros_setup.md.
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('uet_amr_navigation'), 'launch', 'nvblox.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_nvblox),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation/Gazebo clock'),
        DeclareLaunchArgument('map', default_value=default_map_file,
                              description='Full path to the map yaml file to load'),
        DeclareLaunchArgument('use_nvblox', default_value=default_use_nvblox,
                              description='Use isaac_ros_nvblox for the local costmap '
                                          '(default: auto-detected NVIDIA GPU presence)'),
        DeclareLaunchArgument('params_file', default_value=default_params_file,
                              description='Full path to the Nav2 params file'),
        DeclareLaunchArgument('rviz_config_file', default_value=default_rviz_config,
                              description='Full path to the RViz config file'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Whether to start RViz'),
        nvblox_launch,
        localization_launch,
        navigation_launch,
        rviz_node,
    ])
