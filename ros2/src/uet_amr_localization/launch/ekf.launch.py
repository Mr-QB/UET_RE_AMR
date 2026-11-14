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
Launch file for robot localization of UET AMR.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_config = PathJoinSubstitution(
        [FindPackageShare('uet_amr_localization'), 'config', 'ekf.yaml']
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='imu/data',
        description='Raw IMU topic; bias-corrected by imu_bias_corrector and the EKF fuses imu/corrected',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': False},
        ],
        remappings=[
            ('imu/data', 'imu/corrected'),
        ],
    )

    # Subtracts the gyro's at-rest bias (learned only while the wheels are stopped) so the EKF's yaw does not creep.
    imu_bias_corrector = Node(
        package='uet_amr_localization',
        executable='imu_bias_corrector',
        name='imu_bias_corrector',
        output='screen',
        parameters=[{
            'imu_in': LaunchConfiguration('imu_topic'),
            'imu_out': 'imu/corrected',
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        imu_topic_arg,
        imu_bias_corrector,
        ekf_node,
    ])
