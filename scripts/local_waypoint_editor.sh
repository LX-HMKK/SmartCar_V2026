#!/usr/bin/env bash
# Native Ubuntu entry point for the motion-disabled simulation waypoint editor.
set -eo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

if [ ! -f "${workspace}/install/setup.bash" ]; then
    echo "[waypoint-editor] Missing ${workspace}/install/setup.bash. Run scripts/setup_local_sim.sh first." >&2
    exit 2
fi

unset LD_LIBRARY_PATH
unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u

exec ros2 launch smartcar_tools sim_waypoint_editor.launch.py \
    use_rviz:=false \
    use_sim_time:=false \
    "$@"
