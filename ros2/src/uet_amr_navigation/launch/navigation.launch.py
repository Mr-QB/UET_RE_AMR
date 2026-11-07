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
from nav2_common.launch import RewrittenYaml


def _has_nvidia_gpu():
    """Mirrors scripts/common.sh's has_nvidia_gpu(): used only to pick the
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
    log_level = LaunchConfiguration('log_level')
    use_detection = LaunchConfiguration('use_detection')

    # Picks nav2_nvblox.yaml (nvblox-backed local costmap) or nav2.yaml (the
    # CPU voxel_layer fallback) based on use_nvblox -- evaluated at launch
    # time, so an explicit `use_nvblox:=` override on the CLI still works.
    default_params_filename = PythonExpression(
        ["'nav2_nvblox.yaml' if '", use_nvblox, "' == 'true' else 'nav2.yaml'"])
    default_params_file = PathJoinSubstitution([
        FindPackageShare('uet_amr_navigation'), 'config', default_params_filename
    ])
    # The default NavigateToPose tree spins before it backs up, which can't work against a wall. Ours
    # backs up first. bt_navigator needs the tree's absolute path, so it's rewritten into the params.
    bt_xml = PathJoinSubstitution([
        FindPackageShare('uet_amr_navigation'), 'config', 'behavior_trees', 'nav_to_pose_backup_first.xml'
    ])
    params_with_bt = RewrittenYaml(
        source_file=params_file, root_key='',
        param_rewrites={'default_nav_to_pose_bt_xml': bt_xml}, convert_types=True)
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
            'log_level': log_level,
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
                    'params_file': params_with_bt,
                    'log_level': log_level,
                }.items()
            ),
        ]
    )

    # AMCL only corrects the pose after the robot has moved; a stalled robot (door ramp) drifts and never recovers. See
    # scripts/amcl_nomotion_updater.py.
    amcl_nomotion_updater = Node(
        package='uet_amr_navigation',
        executable='amcl_nomotion_updater.py',
        name='amcl_nomotion_updater',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # behavior_server (spin/backup/drive_on_heading) publishes on /cmd_vel, which nothing consumed: recoveries ran
    # without moving the robot. See scripts/cmd_vel_relay.py.
    cmd_vel_relay = Node(
        package='uet_amr_navigation',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        output='screen',
    )

    # Applies dashboard zones as Nav2 keepout/speed filter masks for this map; see scripts/zones_manager.py.
    zones_manager = Node(
        package='uet_amr_navigation',
        executable='zones_manager.py',
        name='zones_manager',
        parameters=[{'map': map_yaml_file}],
        output='screen',
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
            'use_detection': use_detection,
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
        DeclareLaunchArgument('use_detection', default_value='false',
                              description='nvblox: leave people out of the map using PeopleSemSegNet (install the model '
                                          'once, see docs/isaac_ros_setup.md; options in nvblox.launch.py)'),
        DeclareLaunchArgument('log_level', default_value='warn',
                              choices=['debug', 'info', 'warn', 'error', 'fatal'],
                              description='Log level of the Nav2 / AMCL / map_server nodes (their INFO lifecycle and '
                                          'filter-mask chatter is noise); info shows it all again'),
        nvblox_launch,
        localization_launch,
        navigation_launch,
        cmd_vel_relay,
        amcl_nomotion_updater,
        zones_manager,
        rviz_node,
    ])
