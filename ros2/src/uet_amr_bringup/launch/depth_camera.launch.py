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
Generic depth camera bringup — wraps realsense2_camera's rs_launch.py with
the profile settings this robot uses.

Publishes the synced IMU on camera/camera/imu, which
uet_amr_bringup/launch/amr_bringup.launch.py's EKF expects as its imu0 input.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch', 'rs_launch.py'
            ])
        ),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': 'camera',
            # IMU for the EKF; depth + color go straight to nvblox, which does its own filtering.
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            # Deliver depth and color as matched pairs (measured: color stamped ~60 ms behind depth without it).
            'enable_sync': 'true',
            # Depth re-projected into the colour camera (/camera/camera/aligned_depth_to_color/*, colour intrinsics). Needed so a
            # people-segmentation mask made from the COLOUR image lines up with the depth nvblox integrates (use_detection:=true).
            # Costs ~5% of a core at 30 Hz; unaligned depth keeps publishing for the normal costmap path.
            'align_depth.enable': 'true',
        }.items()
    )

    return LaunchDescription([
        depth_camera_launch,
    ])