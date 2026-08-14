"""
Generic depth camera bringup — wraps realsense2_camera's rs_launch.py with
the profile/filter settings this robot uses.

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
            'pointcloud.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'align_depth.enable': 'true',
            'depth_module.depth_profile': '848x480x30',
            'rgb_camera.color_profile': '848x480x30',
            'enable_sync': 'true',
            'clip_distance': '3.0',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
            'decimation_filter.enable': 'true',
        }.items()
    )

    return LaunchDescription([
        depth_camera_launch,
    ])
