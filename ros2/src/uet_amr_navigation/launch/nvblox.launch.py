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
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node, SetRemap
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


def _people_segmentation(context, mask_image_topic):
    """NVIDIA's own isaac_ros_unet PeopleSemSegNet launch (the Isaac ROS quickstart), fed with the colour image.

    It runs in its own container and publishes the raw mask on /unet/raw_segmentation_mask (mono8, 960x544).
    """
    engine = LaunchConfiguration('engine_file_path').perform(context)
    if not os.path.isfile(engine):
        raise RuntimeError(
            f'PeopleSemSegNet engine not found: {engine}\n'
            f'Install it once inside the container (downloads the model from NGC; --eula is your acceptance of '
            f'NVIDIA\'s terms):\n  ros2 run isaac_ros_peoplesemseg_models_install '
            f'install_peoplesemsegnet_vanilla.sh --eula')
    if mask_image_topic != '/unet/raw_segmentation_mask':
        raise RuntimeError('mask_image_topic must stay /unet/raw_segmentation_mask with run_segmentation:=true '
                           '(the official isaac_ros_unet launch publishes there)')
    unet_launch = os.path.join(get_package_share_directory('isaac_ros_unet'), 'launch',
                               'isaac_ros_unet_tensor_rt.launch.py')
    return [GroupAction([
        # The official launch reads absolute /image and /camera_info.
        SetRemap(src='/image', dst=LaunchConfiguration('color_image_topic').perform(context)),
        SetRemap(src='/camera_info', dst=LaunchConfiguration('color_info_topic').perform(context)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(unet_launch),
            launch_arguments={
                'input_image_width': LaunchConfiguration('color_width').perform(context),
                'input_image_height': LaunchConfiguration('color_height').perform(context),
                'engine_file_path': engine,
                'input_binding_names': '["input_1:0"]',
                'output_binding_names': '["argmax_1"]',
                'input_tensor_formats': '["nitros_tensor_list_nchw_rgb_f32"]',
                'output_tensor_formats': '["nitros_tensor_list_nhwc_rgb_f32"]',
                'network_output_type': 'argmax',
            }.items(),
        ),
    ])]


def _launch_nvblox(context):
    params = _load_ros_parameters(LaunchConfiguration('nvblox_params_file').perform(context))
    params['use_sim_time'] = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    use_detection = LaunchConfiguration('use_detection').perform(context).lower() == 'true'
    run_segmentation = LaunchConfiguration('run_segmentation').perform(context).lower() == 'true'

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

    if use_detection:
        # nvblox leaves masked pixels out of the static map (people stop leaving ghosts). It then waits for a mask with
        # EVERY frame, so only turn this on together with a segmentation network publishing one (isaac_ros_unet with
        # PeopleSemSegNet): point mask_image_topic / mask_info_topic at its output.
        params['use_segmentation'] = True
        # The mask comes from the COLOUR image, so it only lines up with depth that has been re-projected into the colour
        # camera (the driver's align_depth, set in uet_amr_bringup's depth_camera.launch.py). The unaligned depth is
        # ~79 deg wide against the colour camera's ~56 deg, so those pixels would not match. The price: nvblox (and the
        # costmap) now see only the colour camera's narrower field of view.
        remappings = [r for r in remappings if r[0] not in ('camera_0/depth/image', 'camera_0/depth/camera_info')] + [
            ('camera_0/depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('camera_0/depth/camera_info', '/camera/camera/aligned_depth_to_color/camera_info'),
            ('camera_0/mask/image', LaunchConfiguration('mask_image_topic').perform(context)),
            ('camera_0/mask/camera_info', LaunchConfiguration('mask_info_topic').perform(context)),
        ]

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=remappings,
        parameters=[params],
    )

    # Clears nvblox obstacle cells inside the robot's own footprint before the local costmap reads the slice, so stale or
    # smeared cells under the robot (e.g. after wheel slip on the door threshold) cannot put the footprint in collision.
    footprint_filter = Node(
        package='uet_amr_navigation',
        executable='nvblox_footprint_filter.py',
        name='nvblox_footprint_filter',
        output='screen',
        parameters=[{'use_sim_time': params['use_sim_time']}],
    )

    actions = []
    if use_detection and run_segmentation:
        actions += _people_segmentation(context, LaunchConfiguration('mask_image_topic').perform(context))
    if use_detection:
        # The mask's camera_info: the colour camera_info scaled to the mask's size (the U-Net works on a resized image).
        actions.append(Node(
            package='uet_amr_navigation',
            executable='mask_camera_info.py',
            name='mask_camera_info',
            output='screen',
            parameters=[{
                'use_sim_time': params['use_sim_time'],
                'mask_topic': LaunchConfiguration('mask_image_topic').perform(context),
                'mask_camera_info_topic': LaunchConfiguration('mask_info_topic').perform(context),
            }],
        ))

    return actions + [ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[nvblox_node],
        output='screen',
    ), footprint_filter]


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
        DeclareLaunchArgument('use_detection', default_value='false',
                              description='Leave people out of the static map using a PeopleSemSegNet mask; nvblox then '
                                          'waits for a mask with every frame. Switches nvblox to the aligned depth '
                                          '(narrower field of view). Needs the model installed once, see '
                                          'docs/isaac_ros_setup.md'),
        DeclareLaunchArgument('run_segmentation', default_value='true',
                              description="With use_detection: start NVIDIA's isaac_ros_unet PeopleSemSegNet launch here. "
                                          'false = another node publishes mask_image_topic'),
        DeclareLaunchArgument('color_image_topic', default_value='/camera/camera/color/image_raw',
                              description='Colour image the people mask is computed from'),
        DeclareLaunchArgument('color_info_topic', default_value='/camera/camera/color/camera_info',
                              description='camera_info of color_image_topic'),
        DeclareLaunchArgument('color_width', default_value='640', description='Width of the colour image'),
        DeclareLaunchArgument('color_height', default_value='480', description='Height of the colour image'),
        DeclareLaunchArgument('engine_file_path',
                              default_value=os.path.join(
                                  os.environ.get('ISAAC_ROS_WS', '/workspaces/isaac_ros-dev'), 'isaac_ros_assets/models',
                                  'peoplesemsegnet/deployable_quantized_vanilla_unet_onnx_v2.0/1/model.plan'),
                              description='TensorRT engine made by install_peoplesemsegnet_vanilla.sh'),
        DeclareLaunchArgument('mask_image_topic', default_value='/unet/raw_segmentation_mask',
                              description='mono8 mask nvblox reads as camera_0/mask/image'),
        DeclareLaunchArgument('mask_info_topic', default_value='/nvblox_mask/camera_info',
                              description='camera_info of the mask, read as camera_0/mask/camera_info; published by '
                                          'mask_camera_info.py (colour camera_info scaled to the mask size)'),
        OpaqueFunction(function=_launch_nvblox),
    ])
