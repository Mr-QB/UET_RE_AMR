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
Launch file for UET AMR Gazebo simulation bringup.
Starts Gazebo, spawns the robot, the ros_gz bridge, robot_state_publisher,
ros2_control (joint_state_broadcaster + diff_drive_controller), and the EKF,
then either slam_toolbox (uet_amr_navigation/slam.launch.py) or Nav2
localization + navigation against a pre-built map
(uet_amr_navigation/navigation.launch.py) depending on mode:=slam|nav, and,
unless disabled with rviz:=false, an RViz2 window scoped to that mode
(uet_amr_navigation/rviz/slam.rviz or navigation.rviz) to visualize it.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='warehouse')
    default_map_file = PathJoinSubstitution([
        FindPackageShare('uet_amr_navigation'),
        'maps', 'warehouse.yaml'
    ])
    map_yaml_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')
    mode = LaunchConfiguration('mode')
    is_slam_mode = EqualsSubstitution(mode, 'slam')
    is_nav_mode = EqualsSubstitution(mode, 'nav')

    pkg_ros_gz_rbot = get_package_share_directory('uet_amr_description')
    pkg_uet_amr_sim = get_package_share_directory('uet_amr_simulation')

    x_pose_arg = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Spawn X position'
    )
    # turtlebot3_house's walls span roughly x:[-7.5,7.5] y:[-5.5,5.5], so a
    # spawn needs to clear that whole band to avoid clipping a wall/doorway/
    # furniture piece there. Our own small 'warehouse' scene puts all its
    # action (a wall, a shelf) near the origin, well inside that same band --
    # so the two worlds can't share one safe-and-close default. Pick per-world
    # instead of compromising on either.
    default_y_pose = PythonExpression([
        "'-2.0' if '", world, "' == 'warehouse' else '-12.0'"
    ])
    y_pose_arg = DeclareLaunchArgument(
        'y_pose', default_value=default_y_pose,
        description='Spawn Y position'
    )

    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'uet_amr.xacro')
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
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            "gz_args": [
                "-r -v 4 ",
                os.path.join(pkg_uet_amr_sim, "worlds", ""),
                world,
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
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/camera/camera/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ekf_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uet_amr_localization'),
                'launch', 'ekf.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'imu_topic': 'camera/camera/imu',
        }.items()
    )

    description_resource_path = os.path.dirname(pkg_ros_gz_rbot)
    gz_resource_path = os.pathsep.join([description_resource_path, world_models_path])

    # slam_toolbox (online async mapping). Its own internal RViz is forced
    # off here so simulation.launch.py's own 'rviz' arg stays the single
    # source of truth for whether RViz launches.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uet_amr_navigation'),
                'launch', 'slam.launch.py'
            ])
        ]),
        condition=IfCondition(is_slam_mode),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': 'false',
        }.items()
    )

    # Nav2 localization (map_server + AMCL) + navigation against a
    # pre-built map. Its own internal RViz is likewise forced off.
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uet_amr_navigation'),
                'launch', 'navigation.launch.py'
            ])
        ]),
        condition=IfCondition(is_nav_mode),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'use_rviz': 'false',
        }.items()
    )

    slam_rviz_config = PathJoinSubstitution([
        FindPackageShare('uet_amr_navigation'), 'rviz', 'slam.rviz'
    ])
    nav_rviz_config = PathJoinSubstitution([
        FindPackageShare('uet_amr_navigation'), 'rviz', 'navigation.rviz'
    ])

    # Each RViz node needs mode:=<x> AND rviz:=true. Nesting the rviz:=true
    # IfCondition inside a mode-conditioned GroupAction ANDs the two using
    # two ordinary IfConditions, rather than hand-rolling string comparisons.
    slam_rviz_node = GroupAction(
        condition=IfCondition(is_slam_mode),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', slam_rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(use_rviz),
                output='screen',
            ),
        ]
    )

    nav_rviz_node = GroupAction(
        condition=IfCondition(is_nav_mode),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', nav_rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(use_rviz),
                output='screen',
            ),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='warehouse',
                              description="World to load (file stem under worlds/): 'warehouse' or 'turtlebot3_house'"),
        x_pose_arg,
        y_pose_arg,
        DeclareLaunchArgument('map', default_value=default_map_file,
                              description="Full path to the map yaml file to load in mode:=nav"),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2 alongside Gazebo'),
        DeclareLaunchArgument('mode', default_value='slam',
                              choices=['slam', 'nav'],
                              description=(
                                  "'slam': run slam_toolbox to build a map online. "
                                  "'nav': run Nav2 localization (AMCL) + navigation against "
                                  "the map given by the 'map' argument."
                              )),
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
        slam_launch,
        nav_launch,
        slam_rviz_node,
        nav_rviz_node,
    ])
