#!/usr/bin/env bash
set -euo pipefail
usage() {
    echo "Usage: bash scripts/install_dep.sh [--check | --download-uris | --offline DEB_DIRECTORY]"
}
mode=${1:-online}
case "$mode" in
    online) [[ $# -eq 0 ]] || { usage >&2; exit 2; } ;;
    --check|--download-uris) [[ $# -eq 1 ]] || { usage >&2; exit 2; } ;;
    --offline)
        [[ $# -eq 2 && -d $2 ]] || { usage >&2; exit 2; }
        package_cache=$(cd -- "$2" && pwd)
        ;;
    --help|-h) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
esac
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
    case "$mode" in
        --check)
            printf 'Missing dependency: %s\n' "${missing[@]}" >&2
            exit 1
            ;;
        --download-uris)
            # Resolve on the target using its package indexes and installed state.
            # An empty cache also lists packages already cached elsewhere on AOS.
            uri_cache=$(mktemp -d)
            trap 'rm -rf -- "$uri_cache"' EXIT
            apt-get -qq --print-uris --yes --download-only --no-remove \
                -o "Dir::Cache::archives=$uri_cache" install "${missing[@]}"
            ;;
        --offline)
            sudo apt-get --no-download --no-remove \
                -o "Dir::Cache::archives=$package_cache" install -y "${missing[@]}"
            ;;
        online)
            sudo apt-get update
            sudo apt-get install -y "${missing[@]}"
            ;;
    esac
else
    echo "All build and ROS dependencies are installed." >&2
fi
