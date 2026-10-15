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
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
        }.items()
    )

    return LaunchDescription([
        depth_camera_launch,
    ])