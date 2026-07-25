#!/bin/bash
# sim_start.sh — 简化启动：手动顺序启动，避开 ros2 launch 的复杂时序
set -e
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

export DISPLAY=:0
export IGN_RELAY=1
export IGN_GAZEBO_RESOURCE_PATH=/root/ros2_ws/install/smartcar_sim/share/smartcar_sim/models:$IGN_GAZEBO_RESOURCE_PATH
WORLD=/root/ros2_ws/install/smartcar_sim/share/smartcar_sim/worlds/track.world
PKG_SIM=/root/ros2_ws/install/smartcar_sim/share/smartcar_sim
PKG_NAV2=/root/ros2_ws/install/smartcar_nav2/share/smartcar_nav2
PKG_SAFETY=/root/ros2_ws/install/smartcar_safety/share/smartcar_safety
PKG_TASK=/root/ros2_ws/install/smartcar_task/share/smartcar_task

pkill -9 ign ros2 2>/dev/null || true; sleep 2
rm -f /tmp/sim_manual.log

echo "=== MANUAL SIM START $(date) ===" | tee /tmp/sim_manual.log

# 1) Gazebo server (headless)
echo "[1] Starting Gazebo..." | tee -a /tmp/sim_manual.log
IGN_RELAY=1 ign gazebo -r -s -v 1 "$WORLD" >> /tmp/gz_manual.log 2>&1 &
GZ_PID=$!
sleep 8
echo "  Gazebo PID=$GZ_PID" | tee -a /tmp/sim_manual.log

# 2) Static TFs
echo "[2] Static TFs..." | tee -a /tmp/sim_manual.log
ros2 run tf2_ros static_transform_publisher 0.0841 0 0.03 0 0 0 base_footprint base_link --ros-args -p use_sim_time:=true &
ros2 run tf2_ros static_transform_publisher -0.05 0 0.23 0 0 0 base_link laser_link --ros-args -p use_sim_time:=true &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 origincar/base_footprint base_footprint --ros-args -p use_sim_time:=true &
sleep 1

# 3) ros_gz_bridge
echo "[3] Bridge..." | tee -a /tmp/sim_manual.log
IGN_RELAY=1 ros2 run ros_gz_bridge parameter_bridge --ros-args -p use_sim_time:=true -p config_file:="$PKG_SIM/config/gz_bridge.yaml" >> /tmp/bridge_manual.log 2>&1 &
sleep 2
echo "  Bridge started" | tee -a /tmp/sim_manual.log

# 4) EKF
echo "[4] EKF..." | tee -a /tmp/sim_manual.log
ros2 run robot_localization ekf_node --ros-args -r __node:=robot_ekf \
  --params-file "$PKG_SIM/config/sim_ekf.yaml" -p use_sim_time:=true >> /tmp/ekf_manual.log 2>&1 &
sleep 3
echo "  EKF started" | tee -a /tmp/sim_manual.log

# 5) Direction guard + safety
echo "[5] Safety nodes..." | tee -a /tmp/sim_manual.log
ros2 run smartcar_safety direction_guard_node --ros-args -r __node:=direction_guard -p use_sim_time:=true >> /tmp/dg_manual.log 2>&1 &
ros2 run smartcar_safety safety_node_cpp --ros-args -r __node:=safety_node \
  --params-file "$PKG_SAFETY/config/safety.yaml" -p use_sim_time:=true -p use_safety_ackermann:=false -p emergency_stop_on_start:=false -p minimum_voltage:=0.0 >> /tmp/safety_manual.log 2>&1 &
sleep 2

# 6) Nav2 lifecycle nodes
echo "[6] Nav2 nodes..." | tee -a /tmp/sim_manual.log
ros2 run nav2_controller controller_server --ros-args -r __node:=controller_server \
  --params-file "$PKG_NAV2/config/nav2_params_fixed.yaml" -p use_sim_time:=true \
  -r cmd_vel:=cmd_vel_nav >> /tmp/nav2_manual.log 2>&1 &
ros2 run nav2_planner planner_server --ros-args -r __node:=planner_server \
  --params-file "$PKG_NAV2/config/nav2_params_fixed.yaml" -p use_sim_time:=true >> /tmp/nav2_manual.log 2>&1 &
ros2 run nav2_bt_navigator bt_navigator --ros-args -r __node:=bt_navigator \
  --params-file "$PKG_NAV2/config/nav2_params_fixed.yaml" -p use_sim_time:=true >> /tmp/nav2_manual.log 2>&1 &
ros2 run nav2_velocity_smoother velocity_smoother --ros-args -r __node:=velocity_smoother \
  --params-file "$PKG_NAV2/config/nav2_params_fixed.yaml" -p use_sim_time:=true \
  -r cmd_vel:=cmd_vel_nav -r cmd_vel_smoothed:=cmd_vel_candidate >> /tmp/nav2_manual.log 2>&1 &
sleep 5

# 7) Lifecycle manager
echo "[7] Lifecycle manager..." | tee -a /tmp/sim_manual.log
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_navigation \
  -p use_sim_time:=true -p autostart:=true \
  -p node_names:="['controller_server','planner_server','bt_navigator','velocity_smoother']" >> /tmp/lifecycle_manual.log 2>&1 &
sleep 10

# 8) Check status
echo "[8] Status..." | tee -a /tmp/sim_manual.log
echo "  ROS nodes:" | tee -a /tmp/sim_manual.log
ros2 node list 2>/dev/null | head -20 | tee -a /tmp/sim_manual.log
echo "  /odom:" | tee -a /tmp/sim_manual.log
timeout 3 ros2 topic echo /odom --once 2>/dev/null | head -2 | tee -a /tmp/sim_manual.log || echo "  NO /odom" | tee -a /tmp/sim_manual.log
echo "  /odom_combined:" | tee -a /tmp/sim_manual.log
timeout 3 ros2 topic echo /odom_combined --once 2>/dev/null | head -2 | tee -a /tmp/sim_manual.log || echo "  NO /odom_combined" | tee -a /tmp/sim_manual.log

echo "=== SIM READY ===" | tee -a /tmp/sim_manual.log
echo "Sim running. Send goal with:"
echo "  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: odom_combined}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'"
