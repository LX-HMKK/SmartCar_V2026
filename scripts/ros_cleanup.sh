#!/bin/bash
# ============================================================
# SmartCar ROS2 全量清理脚本
# 部署: cp 到 /usr/local/bin/ros_cleanup && chmod +x
# 用法: bash /usr/local/bin/ros_cleanup
# ============================================================
set -euo pipefail

echo "=== SmartCar ROS2 全量清理 ==="

# 1. 杀掉所有 ROS 相关进程（按可执行文件名匹配）
PROCS=(
  ros2 ros2-daemon
  ekf_node safety_node_cpp safety_node
  controller_server planner_server behavior_server bt_navigator
  waypoint_follower velocity_smoother lifecycle_manager
  origincar_base_node ydlidar_ros2_driver_node
  static_transform_publisher robot_state_publisher
  task_node field_reference_node waypoint_viz waypoint_editor_node
  waypoint_drag_editor
  rviz2 zbar_ros barcode_reader hobot_usb_cam
  nav_test full_test go_test run_test
)
for name in "${PROCS[@]}"; do
  pkill -9 -f "$name" 2>/dev/null || true
done
sleep 1

# 2. 清理孤儿 Python 进程
pkill -9 -f "ros2/ros2" 2>/dev/null || true
pkill -9 -f "python3.*launch" 2>/dev/null || true

# 3. 停 daemon
ros2 daemon stop 2>/dev/null || true
sleep 1

# 4. 清理共享内存
rm -f /dev/shm/fastrtps_port* /dev/shm/fastdds_port* 2>/dev/null || true

# 5. 重开 daemon
ros2 daemon start 2>/dev/null || true
sleep 1

# 6. 验证
LEFT=$(ps aux | grep -cE "ekf_node|safety_node|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|lifecycle_manager|task_node|origincar_base|ydlidar|static_transform|rviz2|field_ref|waypoint_viz" 2>/dev/null || echo 0)
echo "Remaining: $LEFT (expect 0-2 for daemon only)"
echo "=== Done ==="
