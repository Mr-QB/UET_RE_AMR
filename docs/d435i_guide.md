# Intel RealSense D435i SLAM & Mapping Guide

This guide provides step-by-step instructions for installing drivers, optimizing data streams, and integrating the RTAB-Map SLAM algorithm to build high-resolution 3D point cloud maps (Map Cloud) using the **Intel RealSense D435i** camera on **ROS2 Humble**.

---

## 1. Hardware Requirements

- **Device**: Intel RealSense D435i.
- **Connection**: Must be connected to a **USB 3.0 port or higher**
- **Environment**: Point the camera at well-lit areas rich in geometric features (Rich Texture) so the SLAM algorithm can extract sufficient feature tracking points. Avoid blank white walls, featureless ceilings, or highly reflective/mirror-like surfaces.

---

## 2. Driver and ROS2 Humble Wrapper Installation

Whether you are running natively on Ubuntu/Fedora or inside a container environment (such as Distrobox/Docker), ensure the following packages are installed:

```bash
# Install the official Intel RealSense SDK and tools
sudo apt-get update
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev -y

# Install the ROS2 Wrapper for RealSense (Humble)
sudo apt-get install ros-humble-realsense2-camera -y

# Install the RTAB-Map SLAM packages for Humble
sudo apt-get install ros-humble-rtabmap-ros -y
```

---

## 3. Step by step Mapping Execution

### Step 1: Launch the Camera

Open the first terminal and launch the camera node. This configuration uses the optimal resolution for the infrared depth sensors (848x480), forces system clock synchronization (enable_sync), enables the IMU, and activates hardware post-processing filters to smooth out the point cloud:

```bash
ros2 launch realsense2_camera rs_launch.py \
    pointcloud.enable:=true \
    enable_gyro:=true \
    enable_accel:=true \
    unite_imu_method:=2 \
    align_depth.enable:=true \
    depth_module.depth_profile:=848x480x30 \
    rgb_camera.color_profile:=848x480x30 \
    enable_sync:=true \
    clip_distance:=3.0 \
    spatial_filter.enable:=true \
    temporal_filter.enable:=true
```

### Step 2: Verify the Data Frequency (Recommended)

Before starting the slam node, open a new terminal and verify that the camera is publishing data properly at healthy framerate:

```bash 
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

*Requirement: The rate should be stable between 15 - 30 FPS. If it is significantly lower (e.g., ~5 FPS), the SLAM tracking will time out.*

### Step 3: Start RTAB-Map SLAM

In a new terminal, run the SLAM node configured to handle the Realsense topics:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    frame_id:=camera_link \
    approx_sync:=true \
    approx_sync_max_interval:=0.05 \
    topic_queue_size:=100 \
    sync_queue_size:=100 \
    qos:=2 \
    viz:=true
```

