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

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _ensure_map_directory(context):
    os.makedirs(LaunchConfiguration('map_directory').perform(context), exist_ok=True)
    return []


def generate_launch_description():
    current_dir = os.path.dirname(os.path.realpath(__file__))
    workspace_src_maps = os.path.abspath(os.path.join(current_dir, '..', 'maps'))

    map_directory = LaunchConfiguration('map_directory')
    map_filename = LaunchConfiguration('map_filename')

    declare_map_directory_cmd = DeclareLaunchArgument(
        'map_directory',
        default_value=workspace_src_maps,
        description='Directory in which map and pose graph files are saved')
    declare_map_filename_cmd = DeclareLaunchArgument(
        'map_filename',
        default_value='my_map',
        description='Map file name (without extension)')

    full_path = PathJoinSubstitution([map_directory, map_filename])

    # 1. Save the map image (.yaml + .pgm) for Nav2
    save_map_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
             '-f', full_path,
             # map_saver_cli gives up on /map after 2 s by default; DDS discovery here (zenoh bridge,
             # several /map publishers) is sometimes slower, which made saving fail intermittently.
             '--ros-args', '-p', 'free_thresh:=0.25', '-p', 'occupied_thresh:=0.65',
             '-p', 'save_map_timeout:=10.0'],
        output='screen'
    )

    # 2. Clean lidar speckle/rays out of the saved .pgm (never fails the save)
    clean_map_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'uet_amr_maps', 'clean_map.py', [full_path, '.pgm']],
        output='screen'
    )

    # 3. Call slam_toolbox's serialize_map service to write the .posegraph + .data pair
    serialize_map_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/slam_toolbox/serialize_map',
             'slam_toolbox/srv/SerializePoseGraph',
             ['{filename: "', full_path, '"}']],
        output='screen'
    )

    # Clean and serialize only after map_saver_cli has exited successfully. All
    # steps use the same filename prefix and must finish before this launch
    # process exits.
    clean_after_map_save = RegisterEventHandler(
        OnProcessExit(
            target_action=save_map_cmd,
            on_exit=lambda event, context: [clean_map_cmd]
            if event.returncode == 0
            else [LogInfo(msg=f'map_saver_cli failed with exit code {event.returncode}; not serializing pose graph')],
        )
    )

    serialize_after_clean = RegisterEventHandler(
        OnProcessExit(
            target_action=clean_map_cmd,
            on_exit=[serialize_map_cmd],
        )
    )

    return LaunchDescription([
        declare_map_directory_cmd,
        declare_map_filename_cmd,
        OpaqueFunction(function=_ensure_map_directory),
        clean_after_map_save,
        serialize_after_clean,
        save_map_cmd,
    ])
