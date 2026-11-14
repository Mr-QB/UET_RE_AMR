#!/usr/bin/env python3
# Copyright (c) 2026 Department of Robotics, University of Engineering and
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

"""
camera_info for the people-segmentation mask.

The U-Net works on a resized copy of the colour image, so its mask has a different size (e.g. 960x544) than the camera
(640x480), but nvblox reads camera_0/mask/camera_info next to the mask and needs intrinsics that match the mask's pixels.
This node takes the camera_info of the image the mask was made from (the colour camera_info), scales K and P to the size of
the latest mask and republishes it with the original header, so it carries the same stamp as the mask. (The network stage
here is assumed to resize the whole frame; if it crops or letterboxes, K needs an offset too and this is not enough.)

Parameters
  camera_info_topic       colour camera_info in   (default /camera/camera/color/camera_info)
  mask_topic              the mask, read only for its size (default /unet/raw_segmentation_mask)
  mask_camera_info_topic  output                  (default /nvblox_mask/camera_info)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def scale_camera_info(info, width, height):
    """Copy of info for an image resized (not cropped) to width x height."""
    sx, sy = width / info.width, height / info.height
    out = CameraInfo()
    out.header = info.header
    out.width, out.height = int(width), int(height)
    out.distortion_model = info.distortion_model
    out.d = list(info.d)
    k, p = list(info.k), list(info.p)
    for i in (0, 2):                       # fx, cx
        k[i] *= sx
    for i in (4, 5):                       # fy, cy (k[4] is fy, k[5] is cy)
        k[i] *= sy
    p[0] *= sx; p[2] *= sx; p[3] *= sx     # fx, cx, Tx
    p[5] *= sy; p[6] *= sy                 # fy, cy
    out.k, out.r, out.p = k, list(info.r), p
    out.binning_x, out.binning_y = info.binning_x, info.binning_y
    return out


class MaskCameraInfo(Node):
    def __init__(self):
        super().__init__('mask_camera_info')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('mask_topic', '/unet/raw_segmentation_mask')
        self.declare_parameter('mask_camera_info_topic', '/nvblox_mask/camera_info')
        p = self.get_parameter
        self.size = None
        self.pub = self.create_publisher(CameraInfo, p('mask_camera_info_topic').value, qos_profile_sensor_data)
        self.create_subscription(Image, p('mask_topic').value, self._on_mask, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p('camera_info_topic').value, self._on_info, qos_profile_sensor_data)
        self.get_logger().info(f'{p("camera_info_topic").value} scaled to {p("mask_topic").value} -> '
                               f'{p("mask_camera_info_topic").value}')

    def _on_mask(self, m):
        if self.size != (m.width, m.height):
            self.size = (m.width, m.height)
            self.get_logger().info(f'mask size {m.width}x{m.height}')

    def _on_info(self, info):
        if self.size is not None and info.width > 0:
            self.pub.publish(scale_camera_info(info, *self.size))


def main():
    rclpy.init()
    node = MaskCameraInfo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
