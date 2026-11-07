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
RPLIDAR bringup. frame_id matches the lidar link published by
uet_amr_description/urdf/uet_amr.xacro.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            # Stable symlink from /etc/udev/rules.d/rplidar.rules (matches on
            # the RPLidar's CP210x vendor/product ID)
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 256000,
            'frame_id': 'front_lidar',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': '',
        }],
        output='screen',
        # rplidar_node exits if the lidar isn't powered/enumerated yet; retry.
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        rplidar_node,
    ])
