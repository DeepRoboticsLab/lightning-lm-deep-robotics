#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
build_dir="$repo_dir/libraryf-build"
bag_path=${1:-"$repo_dir/../dataset/m20_lidar_data/libraryf/libraryf_0.db3"}
if [[ $# -gt 1 || "$bag_path" == --help ]]; then
  echo "Usage: $0 [path/to/rosbag.db3]"
  exit 0
fi
if [[ ! -x "$repo_dir/bin/run_slam_offline" || ! -f "$build_dir/liblightning.libs.so" ]]; then
  echo "Build first: bash $repo_dir/scripts/build_libraryf.sh" >&2
  exit 1
fi
if [[ ! -f "$bag_path" ]]; then
  echo "Bag does not exist: $bag_path" >&2
  exit 1
fi
bag_path=$(realpath -- "$bag_path")
mkdir -p "$repo_dir/outputs"
run_dir=$(mktemp -d "$repo_dir/outputs/libraryf-$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")
cp "$repo_dir/config/libraryf_march18_3d.yaml" "$run_dir/config.yaml"
cpus=$(/usr/bin/python3 -c 'import os; print(",".join(map(str, sorted(os.sched_getaffinity(0))[:8])))')
git -C "$repo_dir" rev-parse HEAD > "$run_dir/revision.txt"
git -C "$repo_dir" diff HEAD --binary > "$run_dir/source.patch"
sha256sum "$repo_dir/bin/run_slam_offline" "$build_dir"/*.so > "$run_dir/runtime.sha256"
echo "Output directory: $run_dir"
echo "Live visualization enabled; trajectory height is unconstrained."
echo "Runtime CPUs: $cpus; OMP threads: 4."

# A fresh working directory prevents the application's data/new_map output
# from replacing a previous map. These thread settings match the experiments.
cd "$run_dir"
env -i HOME="$HOME" USER="${USER:-ubuntu}" PATH=/usr/local/bin:/usr/bin:/bin \
  DISPLAY="${DISPLAY:-:1}" \
  XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}" \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}" \
  PANGOLIN_WINDOW_URI=x11:// \
  OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=87 ROS_LOG_DIR="$run_dir/ros-logs" \
  LD_LIBRARY_PATH="$build_dir" \
  /usr/bin/time -v -o "$run_dir/resources.txt" \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/humble/setup.bash
    exec taskset -c "$4" "$1/bin/run_slam_offline" \
      --input_bag "$2" --config "$3/config.yaml" --log_dir "$3"
  ' bash "$repo_dir" "$bag_path" "$run_dir" "$cpus" 2>&1 | tee "$run_dir/run.log"

echo "Saved map: $run_dir/data/new_map/global.pcd"
