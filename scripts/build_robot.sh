#!/usr/bin/env bash
set -euo pipefail

# CMake invokes this mode for each compilation. A separate lock for each CPU
# distributes workers even when firmware disables scheduler load balancing.
if [[ ${1:-} == --compiler ]]; then
    lock_dir=$2
    cpu_list=$3
    shift 3
    mkdir -p -- "$lock_dir"
    IFS=, read -r -a cores <<< "$cpu_list"
    while :; do
        for core in "${cores[@]}"; do
            exec {slot}>"$lock_dir/$core.lock"
            if flock -n "$slot"; then
                taskset -c "$core" "$@"
                exit $?
            fi
            exec {slot}>&-
        done
        sleep 0.1
    done
fi

usage() {
    cat <<'EOF'
Usage: bash scripts/build_robot.sh [--offline DEB_DIRECTORY | --check]

Install transferred dependencies and build/install on the robot. With no
arguments, require dependencies to be installed already. No network downloads.
Source /opt/ros/foxy/setup.bash or /opt/ros/humble/setup.bash first.

--check only checks dependencies and prints the selected build settings.
CMAKE_BUILD_PARALLEL_LEVEL overrides the default, which is at most four jobs.
ROBOT_BUILD_CPUS overrides CPU selection, e.g. 4-7 or 4,6.
EOF
}
mode=${1:-build}
case "$mode" in
    build) [[ $# == 0 ]] || { usage >&2; exit 2; } ;;
    --check) [[ $# == 1 ]] || { usage >&2; exit 2; } ;;
    --offline)
        [[ $# == 2 && -d $2 ]] || { usage >&2; exit 2; }
        deb_dir=$(cd -- "$2" && pwd)
        ;;
    --help|-h) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
esac
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_dir"
if [[ $mode == --offline ]]; then
    bash scripts/install_dep.sh --offline "$deb_dir"
else
    bash scripts/install_dep.sh --check
fi

# RK3588's four A76 cores are often isolated by M20 firmware. Set affinity only
# on our compiler children, leaving firmware services and CPU isolation intact.
cpu_list=$(/usr/bin/python3 - <<'PY'
import os
from pathlib import Path

requested = os.environ.get('ROBOT_BUILD_CPUS')
if requested:
    try:
        cpus = set()
        for part in requested.split(','):
            ends = [int(x) for x in part.split('-')]
            if len(ends) not in (1, 2) or min(ends) < 0 or ends[0] > ends[-1]:
                raise ValueError()
            cpus.update(range(ends[0], ends[-1] + 1))
    except ValueError:
        raise SystemExit('ROBOT_BUILD_CPUS must be a CPU list such as 4-7 or 4,6.')
    cpus = sorted(cpus)
else:
    compatible = Path('/proc/device-tree/compatible')
    rk3588 = compatible.exists() and b'rockchip,rk3588' in compatible.read_bytes()
    # Prefer the less occupied A76 cores for trailing compiler jobs on standard firmware.
    cpus = [7, 6, 4, 5] if rk3588 else sorted(os.sched_getaffinity(0))
print(','.join(str(cpu) for cpu in cpus))
PY
)
IFS=, read -r -a cores <<< "$cpu_list"
for core in "${cores[@]}"; do
    taskset -c "$core" true || { echo "Cannot use build CPU $core." >&2; exit 1; }
done

# Reserve 2 GiB of currently available memory for other activity. Debug symbols
# need more compiler memory. This is a startup estimate, not a system memory limit.
compiler_mib=2816
[[ ${CMAKE_BUILD_TYPE:-Release} == Release ]] || compiler_mib=4608
available_kib=$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)
memory_jobs=$(( (available_kib - 2 * 1024 * 1024) / (compiler_mib * 1024) ))
limit=${#cores[@]}
(( limit <= memory_jobs )) || limit=$memory_jobs
(( limit > 0 )) || { echo "Not enough available RAM for a robot build; free memory first." >&2; exit 1; }
default_jobs=$limit
(( default_jobs <= 4 )) || default_jobs=4
jobs=${CMAKE_BUILD_PARALLEL_LEVEL:-$default_jobs}
[[ $jobs =~ ^[1-9][0-9]*$ ]] || { echo "CMAKE_BUILD_PARALLEL_LEVEL must be a positive integer." >&2; exit 2; }
(( jobs <= limit )) || { echo "Requested $jobs jobs; available RAM and CPUs permit at most $limit." >&2; exit 1; }
echo "Robot build: $jobs jobs; compiler CPUs $cpu_list; $((available_kib / 1024)) MiB available RAM."
[[ $mode != --check ]] || exit 0

cache=$(command -v ccache || true)
c_launcher=${CMAKE_C_COMPILER_LAUNCHER-$cache}
cxx_launcher=${CMAKE_CXX_COMPILER_LAUNCHER-$cache}
affinity_launcher="bash;$repo_dir/scripts/build_robot.sh;--compiler;$repo_dir/build-robot-locks;$cpu_list"
export CMAKE_C_COMPILER_LAUNCHER="$affinity_launcher${c_launcher:+;$c_launcher}"
export CMAKE_CXX_COMPILER_LAUNCHER="$affinity_launcher${cxx_launcher:+;$cxx_launcher}"
export CMAKE_BUILD_PARALLEL_LEVEL="$jobs"
nice -n 10 bash scripts/build.sh
echo "Build and installation complete. Run: source scripts/setup.bash"
