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
launch_args=()

while [ "$#" -gt 0 ]; do
    case "$1" in
        --gui) headless=false ;;
        --headless) headless=true ;;
        --rviz) use_rviz=true ;;
        --no-rviz) use_rviz=false ;;
        --no-clean) clean_processes=false ;;
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

echo "[sim] RMW=${RMW_IMPLEMENTATION}, localhost=${ROS_LOCALHOST_ONLY}"
echo "[sim] headless=${headless}, rviz=${use_rviz}"

exec ros2 launch smartcar_sim sim.launch.py \
    headless:="$headless" \
    use_rviz:="$use_rviz" \
    "${launch_args[@]}"
