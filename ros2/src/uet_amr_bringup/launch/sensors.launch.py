"""
Brings up all onboard sensors: the RPLIDAR and the depth camera.

The raw RPLIDAR /scan is also run through scan_deskew_node +
pointcloud_to_laserscan to produce /scan_corrected, a motion-compensated
scan that slam_toolbox/Nav2 consume instead of the raw one -- see
scan_deskew_node.cpp for why. Simulation's Gazebo lidar plugin publishes
/scan_corrected directly (see uet_amr_simulation/launch/gazebo.launch.py);
it has no per-scan sweep-time distortion to remove.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('uet_amr_bringup')

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'lidar.launch.py'])
        )
    )

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'depth_camera.launch.py'])
        )
    )

    scan_deskew_node = Node(
        package='uet_amr_bringup',
        executable='scan_deskew_node',
        name='scan_deskew_node',
        parameters=[{'fixed_frame': 'odom'}],
        output='screen',
    )

    # Converts the deskewed point cloud back into a LaserScan (relative to
    # the lidar's current pose) since slam_toolbox/AMCL/costmaps only
    # consume sensor_msgs/LaserScan, not PointCloud2.
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in', 'scan_deskewed_points'),
            ('scan', 'scan_corrected'),
        ],
        parameters=[{
            'target_frame': 'front_lidar',
            'transform_tolerance': 0.02,
            'min_height': -0.5,
            'max_height': 0.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            # Matches uet_amr_description's front_lidar ray_count (800
            # samples/rev) -- retune if the RPLidar's actual scan_mode
            # sample count differs.
            'angle_increment': 0.00785,
            'range_min': 0.15,
            'range_max': 12.0,
            'use_inf': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        lidar_launch,
        depth_camera_launch,
        scan_deskew_node,
        pointcloud_to_laserscan_node,
    ])
