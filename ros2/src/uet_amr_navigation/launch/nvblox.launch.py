# Copyright (c) 2026 Department of Robotics, University of Engineering and
#                     Technology, Vietnam National University, Hanoi (VNU).
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
Launch file for the isaac_ros_nvblox reconstruction node, feeding the D435i
depth stream into a GPU-accelerated costmap consumed by nav2_nvblox's
NvbloxCostmapLayer (see config/nav2_nvblox.yaml). Only meant to be run inside
the Isaac ROS Docker dev container (see docs/isaac_ros_setup.md) -- nvblox_ros
needs CUDA Toolkit + NVIDIA's NITROS transport, which aren't available
natively. Included conditionally by navigation.launch.py's use_nvblox arg.
"""
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def _load_ros_parameters(path):
    """The `ros__parameters` of a params file, with or without a node-name/wildcard key.

    Passing the file straight to the ComposableNode left nvblox_node on its defaults here (use_lidar True,
    ESDF slice 0-1 m, which puts the floor into the Nav2 costmap), so the values are loaded explicitly.
    """
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    params = {}
    for node_params in data.values():
        if isinstance(node_params, dict):
            params.update(node_params.get('ros__parameters', {}))
    return params


def _launch_nvblox(context):
    params = _load_ros_parameters(LaunchConfiguration('nvblox_params_file').perform(context))
    params['use_sim_time'] = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    # camera_0/{depth,color} are nvblox_node's fixed input topic names (see
    # isaac_ros_nvblox's nvblox_ros/include/nvblox_ros/nvblox_node.hpp) --
    # remapped here to this robot's actual D435i topics (camera_name/
    # camera_namespace both 'camera', set in uet_amr_bringup's
    # depth_camera.launch.py).
    remappings = [
        ('camera_0/depth/image', '/camera/camera/depth/image_rect_raw'),
        ('camera_0/depth/camera_info', '/camera/camera/depth/camera_info'),
        ('camera_0/color/image', '/camera/camera/color/image_raw'),
        ('camera_0/color/camera_info', '/camera/camera/color/camera_info'),
    ]

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=[params],
    )

    return [ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[nvblox_node],
        output='screen',
    )]


def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare('uet_amr_navigation'), 'config', 'nvblox.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation/Gazebo clock'),
        # Not 'params_file': navigation.launch.py includes this file and declares its own params_file (the Nav2
        # params), which an included file inherits, so nvblox was silently given nav2_nvblox.yaml and ran on defaults.
        DeclareLaunchArgument('nvblox_params_file', default_value=default_params_file,
                              description='Full path to the nvblox params file'),
        DeclareLaunchArgument('container_name', default_value='nvblox_container',
                              description='Name of the composable node container to create'),
        OpaqueFunction(function=_launch_nvblox),
    ])
