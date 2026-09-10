#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
build_dir="$repo_dir/libraryf-build"
mkdir -p "$build_dir"
echo "Building run_slam_offline with $(nproc) CPUs. Logs: $build_dir/{configure,build}.log"

# Use the installed Humble environment without inheriting another ROS overlay.
env -i HOME="$HOME" USER="${USER:-ubuntu}" PATH=/usr/local/bin:/usr/bin:/bin \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/humble/setup.bash
    cmake -S "$1" -B "$2" -DCMAKE_BUILD_TYPE=Release \
      -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
      > "$2/configure.log" 2>&1
    cmake --build "$2" --target run_slam_offline --parallel "$(nproc)" \
      > "$2/build.log" 2>&1
  ' bash "$repo_dir" "$build_dir"

echo "Built: $repo_dir/bin/run_slam_offline"
