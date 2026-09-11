# Source this file in every terminal before running Lightning-LM.
_lightning_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! -f "$_lightning_repo/install/setup.bash" ]]; then
    echo "Build Lightning-LM with scripts/build.sh first." >&2
    unset _lightning_repo
    return 1
fi
_lightning_distro_file="$_lightning_repo/install/lightning/share/lightning/ros_distro"
if [[ ! -r "$_lightning_distro_file" ]]; then
    echo "Rebuild with scripts/build.sh to record the ROS distribution." >&2
    unset _lightning_repo _lightning_distro_file
    return 1
fi
read -r _lightning_distro < "$_lightning_distro_file"
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != "$_lightning_distro" ]]; then
    echo "This build uses ROS $_lightning_distro; open a fresh terminal instead of mixing it with ROS $ROS_DISTRO." >&2
    unset _lightning_repo _lightning_distro_file _lightning_distro
    return 1
fi
# Colcon's setup chain sources the ROS underlay used to build this installation.
source "$_lightning_repo/install/setup.bash" || return 1
export LD_LIBRARY_PATH="$_lightning_repo/.deps/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-$_lightning_repo/config/fastdds.xml}
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-4}
export OMP_WAIT_POLICY=${OMP_WAIT_POLICY:-PASSIVE}
export OPENBLAS_NUM_THREADS=${OPENBLAS_NUM_THREADS:-1}
export PANGOLIN_WINDOW_URI=${PANGOLIN_WINDOW_URI:-x11://}
unset _lightning_repo _lightning_distro_file _lightning_distro
