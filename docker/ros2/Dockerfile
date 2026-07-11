FROM ros:humble

LABEL maintainer="UET Robotics Club <robotics@uet.vnu.edu.vn>"
LABEL description="UET AMR ROS2 development environment"

ARG DEBIAN_FRONTEND=noninteractive

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-nav2-bringup \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-slam-toolbox \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-realsense2-camera \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Setup workspace
WORKDIR /ros2_ws
COPY ros2/src ./src

# Install deps and build
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# Entrypoint
COPY docker/ros2/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
