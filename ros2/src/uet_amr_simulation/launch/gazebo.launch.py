import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
from os.path import join


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rbot = get_package_share_directory('uet_amr_description')
    pkg_uet_amr_sim = get_package_share_directory('uet_amr_simulation')
    pkg_uet_amr_localization = get_package_share_directory('uet_amr_localization')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_house',
        description="World to load (file stem under worlds/): 'warehouse' or 'turtlebot3_house'"
    )
    x_pose_arg = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Spawn X position'
    )
    y_pose_arg = DeclareLaunchArgument(
        # turtlebot3_house's walls span roughly x:[-7.5,7.5] y:[-5.5,5.5];
        # -12 lands well outside the building envelope, on open ground, so
        # spawning can't clip a wall/doorway/furniture piece in either world.
        'y_pose', default_value='-12.0',
        description='Spawn Y position'
    )
    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'uet_amr.xacro')
    ros_gz_bridge_config = os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')
    world_models_path = os.path.join(pkg_uet_amr_sim, 'worlds', 'models')

    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    load_diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

   
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

   
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": [
                "-r -v 4 ",
                os.path.join(pkg_uet_amr_sim, "worlds", ""),
                LaunchConfiguration("world"),
                ".sdf",
            ]
        }.items()
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                "-topic", "/robot_description",
                "-name", "uet_amr",
                "-allow_renaming", "false",
                "-x", LaunchConfiguration('x_pose'),
                "-y", LaunchConfiguration('y_pose'),
                "-z", "0.32",
                "-Y", "0.0"
            ],
            output='screen'
        )]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        arguments=[
            # Bridged directly onto /scan_corrected (not /scan): Gazebo's lidar
            # plugin computes a whole scan in one physics step, so it has none
            # of the real RPLidar's ~100-200ms sweep-time motion distortion
            # that /scan_corrected exists to remove on real hardware (see
            # uet_amr_bringup/src/scan_deskew_node.cpp) -- slam.yaml/nav2.yaml
            # expect /scan_corrected either way.
            '/scan_corrected@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ekf_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_uet_amr_localization, 'launch', 'ekf.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    description_resource_path = os.path.dirname(pkg_ros_gz_rbot)
    gz_resource_path = os.pathsep.join([description_resource_path, world_models_path])

    return LaunchDescription([
        world_arg,
        x_pose_arg,
        y_pose_arg,
        SetEnvironmentVariable('IGN_IP', '127.0.0.1'),
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        robot_state_publisher,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        ekf_localization,
    ])
