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
        description='Tên file bản đồ (không bao gồm phần mở rộng)')

    full_path = PathJoinSubstitution([map_directory, map_filename])

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

    # Serialize only after map_saver_cli has exited successfully. Both
    # operations use the same filename prefix and must finish before this
    # launch process exits.
    serialize_after_map_save = RegisterEventHandler(
        OnProcessExit(
            target_action=save_map_cmd,
            on_exit=lambda event, context: [serialize_map_cmd]
            if event.returncode == 0
            else [LogInfo(msg=f'map_saver_cli failed with exit code {event.returncode}; not serializing pose graph')],
        )
    )

    return LaunchDescription([
        declare_map_directory_cmd,
        declare_map_filename_cmd,
        OpaqueFunction(function=_ensure_map_directory),
        serialize_after_map_save,
        save_map_cmd,
    ])
