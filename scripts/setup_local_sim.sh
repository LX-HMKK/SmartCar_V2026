#!/usr/bin/env bash
# Bootstrap the native Ubuntu development simulator for this checkout.
set -eo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "[setup] ROS 2 Humble is required at /opt/ros/humble." >&2
    exit 2
fi

# Avoid resolving or loading libraries from an active Conda/Isaac environment.
unset LD_LIBRARY_PATH
unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

sudo apt-get update
sudo apt-get install --yes \
    ignition-fortress \
    fonts-noto-cjk \
    python3-matplotlib \
    python3-pyqt5 \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-robot-localization

source /opt/ros/humble/setup.bash
set -u

# Limit rosdep to packages required by the simulator. Hardware-only camera
# drivers are deliberately outside this native simulation bootstrap.
rosdep install --from-paths \
    "${workspace}/src/smartcar_interfaces" \
    "${workspace}/src/origincar/3rdparty/ackermann_msgs-ros2" \
    "${workspace}/src/smartcar_safety" \
    "${workspace}/src/smartcar_nav2" \
    "${workspace}/src/smartcar_task" \
    "${workspace}/src/smartcar_tools" \
    "${workspace}/src/smartcar_sim" \
    --ignore-src --rosdistro humble -y \
    --skip-keys "smartcar_vision smartcar_speech ament_python"

cd "${workspace}"
colcon build --symlink-install --packages-up-to smartcar_sim \
    --allow-overriding ackermann_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source "${workspace}/install/setup.bash"
ros2 pkg prefix ros_gz_sim >/dev/null
ros2 pkg prefix ros_gz_bridge >/dev/null
ros2 pkg prefix robot_localization >/dev/null
command -v ign >/dev/null

echo "[setup] Native simulator is ready. Start it with:"
echo "  bash ${workspace}/scripts/local_sim.sh --headless --rviz"
