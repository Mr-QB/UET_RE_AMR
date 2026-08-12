import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 1. Khai báo đường dẫn tới package realsense2_camera
    realsense_pkg_dir = get_package_share_directory('realsense2_camera')
    realsense_launch_file = os.path.join(realsense_pkg_dir, 'launch', 'rs_launch.py')

    # 2. Cấu hình IncludeLaunchDescription cho Realsense D435i
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'pointcloud.enable': 'false',  # RTAB-Map builds its own cloud from rgb+depth; this was wasted CPU
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
            'decimation_filter.enable': 'true', # Thêm bộ lọc này để giảm tải cho Jetson
        }.items()
    )

    # 3. Khai báo đường dẫn tới package rtabmap_launch
    rtabmap_pkg_dir = get_package_share_directory('rtabmap_launch')
    rtabmap_launch_file = os.path.join(rtabmap_pkg_dir, 'launch', 'rtabmap.launch.py')

    # 4. Cấu hình IncludeLaunchDescription cho RTAB-Map
    rtabmap_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_file),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start Kp/MaxFeatures=200 Rtabmap/DetectionRate=1 Grid/Sensor=1 Reg/Force3DoF=true', # Cập nhật args nhẹ hơn
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'camera_link',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.05',
            'topic_queue_size': '10', # Giảm từ 100 xuống 10
            'sync_queue_size': '10',  # Giảm từ 100 xuống 10
            'qos': '2',
            'rtabmap_viz': 'true',   # Tắt viz trên Jetson
        }.items()
    )

    # 5. Trả về LaunchDescription bao gồm cả 2 node để chạy song song
    return LaunchDescription([
        realsense_node,
        rtabmap_node
    ])