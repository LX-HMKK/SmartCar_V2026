#!/usr/bin/env bash
# Reliable native Ubuntu entry point for SmartCar simulation.
set -eo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [ -f "${script_dir}/../package.xml" ]; then
    default_workspace=$(cd "${script_dir}/../../.." && pwd)
else
    default_workspace=$(cd "${script_dir}/../../../../.." && pwd)
fi
workspace=${SMARTCAR_WS:-${default_workspace}}
headless=true
use_rviz=true
clean_processes=true
skip_cleanup=false
launch_args=()

while [ "$#" -gt 0 ]; do
    case "$1" in
        --gui) headless=false ;;
        --headless) headless=true ;;
        --rviz) use_rviz=true ;;
        --no-rviz) use_rviz=false ;;
        # A caller preserving an existing local ROS/Gazebo session must skip
        # both this wrapper's process cleanup and sim.launch.py's stale-file
        # cleanup. The launch still requires the caller to isolate DDS/Gazebo.
        --no-clean) clean_processes=false; skip_cleanup=true ;;
        --) shift; launch_args+=("$@"); break ;;
        *) launch_args+=("$1") ;;
    esac
    shift
done

source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
source "${script_dir}/sim_env.sh"
set -u

if [ "$clean_processes" = true ]; then
    bash "${script_dir}/sim_cleanup.sh" --kill-processes
fi

if [ "$skip_cleanup" = true ]; then
    launch_args+=("skip_cleanup:=true")
fi

echo "[sim] RMW=${RMW_IMPLEMENTATION:-ros-default}, localhost=${ROS_LOCALHOST_ONLY}"
echo "[sim] headless=${headless}, rviz=${use_rviz}"

exec ros2 launch smartcar_sim sim.launch.py \
    headless:="$headless" \
    use_rviz:="$use_rviz" \
    "${launch_args[@]}"
