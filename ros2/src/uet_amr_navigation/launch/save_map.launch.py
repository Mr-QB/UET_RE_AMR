import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    current_dir = os.path.dirname(os.path.realpath(__file__))
    workspace_src_maps = os.path.abspath(os.path.join(current_dir, '..', 'maps'))
    
    if not os.path.exists(workspace_src_maps):
        os.makedirs(workspace_src_maps)

    map_filename = LaunchConfiguration('map_filename')
    
    declare_map_filename_cmd = DeclareLaunchArgument(
        'map_filename',
        default_value='my_map',
        description='Tên file bản đồ (không bao gồm phần mở rộng)')

    full_path = PathJoinSubstitution([workspace_src_maps, map_filename])

    # 1. Lệnh lưu ảnh map (.yaml + .pgm) cho Nav2
    save_map_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', 
             '-f', full_path, 
             '--ros-args', '-p', 'free_thresh:=0.25', '-p', 'occupied_thresh:=0.65'],
        output='screen'
    )

    # 2. Lệnh gọi service Serialize của SLAM Toolbox để tạo 2 file (.posegraph + .data)
    serialize_map_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/slam_toolbox/serialize_map', 
             'slam_toolbox/srv/SerializePoseGraph', 
             ['{filename: "', full_path, '"}']],
        output='screen'
    )

    return LaunchDescription([
        declare_map_filename_cmd,
        save_map_cmd,
        serialize_map_cmd
    ])