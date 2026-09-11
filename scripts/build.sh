#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_dir"
case "${ROS_DISTRO:-}" in
    foxy|humble) ;;
    *) echo "Source /opt/ros/foxy/setup.bash (Ubuntu 20.04) or /opt/ros/humble/setup.bash (Ubuntu 22.04) first." >&2; exit 1 ;;
esac
build_type=${CMAKE_BUILD_TYPE:-Release}
jobs=${CMAKE_BUILD_PARALLEL_LEVEL:-$(nproc)}
export CMAKE_BUILD_PARALLEL_LEVEL="$jobs"
export MAKEFLAGS="-j$jobs"
# Use ccache when available; an explicit launcher (including an empty one) wins.
compiler_cache=$(command -v ccache || true)
launcher_args=(
    "-DCMAKE_C_COMPILER_LAUNCHER=${CMAKE_C_COMPILER_LAUNCHER-$compiler_cache}"
    "-DCMAKE_CXX_COMPILER_LAUNCHER=${CMAKE_CXX_COMPILER_LAUNCHER-$compiler_cache}"
)
unzip -nq thirdparty/Pangolin-0.9.3.zip -d thirdparty
cmake -S thirdparty/Pangolin-0.9.3 -B build-pangolin \
    -DCMAKE_BUILD_TYPE="$build_type" -DCMAKE_INSTALL_PREFIX="$repo_dir/.deps" \
    -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF \
    -DBUILD_PANGOLIN_LIBOPENEXR=OFF "${launcher_args[@]}"
cmake --build build-pangolin --parallel "$jobs"
cmake --install build-pangolin
export CMAKE_PREFIX_PATH="$repo_dir/.deps${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
colcon build --base-paths . --packages-select lightning --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE="$build_type" "${launcher_args[@]}" \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
