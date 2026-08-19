# Copyright (c) 2026 UET Robotics Club, University of Engineering and
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
Launch file for controller teleoperation of UET AMR.
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_config_path = PathJoinSubstitution(
        [FindPackageShare('uet_amr_teleop'), 'config', 'joy_teleop.yaml']
    )

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[joy_config_path],
            remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
        ),
    ])
