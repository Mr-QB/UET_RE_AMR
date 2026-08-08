import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from os.path import join

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rbot = get_package_share_directory('uet_amr_description')
    pkg_uet_amr_sim = get_package_share_directory('uet_amr_simulation')

    # Set IGN_GAZEBO_RESOURCE_PATH so Gazebo resolves model://uet_amr_description meshes
    share_dir = os.path.abspath(os.path.join(pkg_ros_gz_rbot, '..'))
    src_dir = os.path.abspath(os.path.join(pkg_ros_gz_rbot, '../../../../src/UET_RE_AMR/ros2/src'))
    
    ign_resource_path = f"{share_dir}:{src_dir}"
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        ign_resource_path = f"{ign_resource_path}:{os.environ['IGN_GAZEBO_RESOURCE_PATH']}"
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path = f"{ign_resource_path}:{os.environ['GZ_SIM_RESOURCE_PATH']}"
    else:
        gz_resource_path = ign_resource_path

    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = ign_resource_path
    os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path

    set_ign_resource = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path)
    set_gz_resource = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path)

    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'uet_amr.xacro')
    ros_gz_bridge_config = os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')
    world_file = os.path.join(pkg_uet_amr_sim, 'worlds', 'warehouse.sdf')
    
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    controllers_file = os.path.join(pkg_ros_gz_rbot, 'config', 'controllers.yaml')

    load_controllers = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "-c", "/controller_manager",
                    "-t", "joint_state_broadcaster/JointStateBroadcaster",
                    "-p", controllers_file,
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "diff_drive_controller",
                    "-c", "/controller_manager",
                    "-t", "diff_drive_controller/DiffDriveController",
                    "-p", controllers_file,
                ],
            ),
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v 4 {world_file}"}.items()
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
                "-x", "0.0",
                "-y", "0.0",
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
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/world/test_place/model/uet_amr/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/model/uet_amr/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        set_ign_resource,
        set_gz_resource,
        gazebo,
        robot_state_publisher,
        ros_gz_bridge,
        spawn_robot,
        load_controllers,
    ])
