#!/usr/bin/env bash
set -euo pipefail
case "${ROS_DISTRO:-}" in
    foxy|humble) ;;
    *) echo "Source /opt/ros/foxy/setup.bash (Ubuntu 20.04) or /opt/ros/humble/setup.bash (Ubuntu 22.04) first." >&2; exit 1 ;;
esac
packages=(
    cmake g++ ccache pkg-config unzip python3-colcon-common-extensions python3-wheel
    libopencv-dev libpcl-dev pcl-tools libyaml-cpp-dev libepoxy-dev
    libgflags-dev libgoogle-glog-dev libtbb-dev
    libgl1-mesa-dev libegl1-mesa-dev libglew-dev libeigen3-dev
    libx11-dev libwayland-dev libxkbcommon-dev wayland-protocols
    ros-${ROS_DISTRO}-pcl-conversions ros-${ROS_DISTRO}-rosbag2 ros-${ROS_DISTRO}-rosbag2-storage-default-plugins
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp ros-${ROS_DISTRO}-ros2run ros-${ROS_DISTRO}-ros2bag ros-${ROS_DISTRO}-ros2service
    ros-${ROS_DISTRO}-rosidl-default-generators ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-message-filters
)
missing=()
for package in "${packages[@]}"; do
    if [[ $(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true) != 'install ok installed' ]]; then
        missing+=("$package")
    fi
done
if ((${#missing[@]})); then
    sudo apt-get update
    sudo apt-get install -y "${missing[@]}"
else
    echo "All build and ROS dependencies are installed."
fi
