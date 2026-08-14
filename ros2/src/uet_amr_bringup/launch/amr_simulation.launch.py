"""
Launch file for UET AMR Gazebo simulation bringup.
Starts the Gazebo simulation (uet_amr_simulation/gazebo.launch.py) and,
unless disabled with rviz:=false, an RViz2 window pre-configured
(uet_amr_description/config/gazebo.rviz) to visualize it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='warehouse')
    use_rviz = LaunchConfiguration('rviz')

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uet_amr_simulation'),
                'launch', 'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('uet_amr_description'),
        'config', 'gazebo.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='Gazebo world name'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2 alongside Gazebo'),
        gazebo_launch,
        rviz_node,
    ])
