#!/usr/bin/env bash
set -euo pipefail

case "${ROS_DISTRO:-}" in
    foxy|humble) ;;
    *)
        echo "Source /opt/ros/foxy/setup.bash or /opt/ros/humble/setup.bash first." >&2
        exit 1
        ;;
esac

sudo apt-get install -y \
    cmake g++ pkg-config unzip python3-colcon-common-extensions python3-wheel \
    libopencv-dev libpcl-dev pcl-tools libyaml-cpp-dev libepoxy-dev \
    libgflags-dev libgoogle-glog-dev libtbb-dev \
    libgl1-mesa-dev libegl1-mesa-dev libglew-dev libeigen3-dev \
    libx11-dev libwayland-dev libxkbcommon-dev wayland-protocols \
    "ros-${ROS_DISTRO}-pcl-conversions" \
    "ros-${ROS_DISTRO}-rosbag2" \
    "ros-${ROS_DISTRO}-ament-cmake-auto" \
    "ros-${ROS_DISTRO}-rosidl-default-generators"
