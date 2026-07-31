#!/usr/bin/env bash
# Shared ROS/DDS environment for every SmartCar simulation CLI process.

sim_env_script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [ -f "${sim_env_script_dir}/../package.xml" ]; then
    sim_env_default_workspace=$(cd "${sim_env_script_dir}/../../.." && pwd)
else
    sim_env_default_workspace=$(cd "${sim_env_script_dir}/../../../../.." && pwd)
fi
sim_env_workspace=${SMARTCAR_WS:-${sim_env_default_workspace}}

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1

# Clear custom locator profiles so every local participant uses the same
# default Fast DDS transports.
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset FASTDDS_DEFAULT_PROFILES_FILE
unset RMW_FASTRTPS_USE_QOS_FROM_XML
unset CYCLONEDDS_URI

if [ -d "${sim_env_workspace}/install/smartcar_sim/share/smartcar_sim/models" ]; then
    SIM_MODEL_PATH="${sim_env_workspace}/install/smartcar_sim/share/smartcar_sim/models"
    export IGN_GAZEBO_RESOURCE_PATH="${SIM_MODEL_PATH}${IGN_GAZEBO_RESOURCE_PATH:+:${IGN_GAZEBO_RESOURCE_PATH}}"
fi

unset SIM_MODEL_PATH
unset sim_env_workspace
unset sim_env_script_dir
unset sim_env_default_workspace
