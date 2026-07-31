#!/usr/bin/env bash
# Native Ubuntu entry point for the Gazebo simulation only.
set -eo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

if [ ! -f "${workspace}/install/setup.bash" ]; then
    echo "[local-sim] Missing ${workspace}/install/setup.bash. Run scripts/setup_local_sim.sh first." >&2
    exit 2
fi

# Isaac/Conda overlays in an interactive desktop shell can replace system
# ncurses, Python, and Gazebo libraries. The simulation uses system ROS 2.
unset LD_LIBRARY_PATH
unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u
export SMARTCAR_WS="${workspace}"

exec bash "${workspace}/install/smartcar_sim/share/smartcar_sim/scripts/sim_start.sh" "$@"
