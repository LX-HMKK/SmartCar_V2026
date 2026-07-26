#!/usr/bin/env bash
# Shared ROS/DDS environment for every SmartCar simulation CLI process.

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1

# NAT + ROS_LOCALHOST_ONLY is the supported WSL setup. Clear custom locator
# profiles so every participant uses the same default Fast DDS transports.
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_DEFAULT_PROFILES_FILE
unset RMW_FASTRTPS_USE_QOS_FROM_XML
unset CYCLONEDDS_URI

if grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null; then
    export DISPLAY="${DISPLAY:-:0}"
    export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
    export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
fi

if [ -d /root/ros2_ws/install/smartcar_sim/share/smartcar_sim/models ]; then
    SIM_MODEL_PATH=/root/ros2_ws/install/smartcar_sim/share/smartcar_sim/models
    export IGN_GAZEBO_RESOURCE_PATH="${SIM_MODEL_PATH}${IGN_GAZEBO_RESOURCE_PATH:+:${IGN_GAZEBO_RESOURCE_PATH}}"
fi

unset SIM_MODEL_PATH
