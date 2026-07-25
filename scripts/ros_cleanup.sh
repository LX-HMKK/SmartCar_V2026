#!/bin/bash
# ============================================================
# SmartCar ROS2 全量清理脚本
# 部署: cp scripts/ros_cleanup.sh /usr/local/bin/ros_cleanup && chmod +x
# 用法: bash /usr/local/bin/ros_cleanup
#
# 安全: 可从任意上下文调用，不依赖 ros2 CLI 存在
# ============================================================

echo "=== SmartCar ROS2 全量清理 ==="

# ---- 1. 停 daemon（在杀进程之前，防止自动重启） ----
if command -v ros2 &>/dev/null; then
  ros2 daemon stop 2>/dev/null || true
  echo "  ros2 daemon stopped"
else
  echo "  ros2 CLI unavailable, skipping daemon stop"
fi

# ---- 2. 杀掉所有 ROS 相关进程 ----
# 2a. 按可执行文件名精确匹配（常用 ROS2 节点）
PROCS=(
  # ROS2 core
  ros2 ros2-daemon ros2_daemon
  # Nav2
  controller_server planner_server behavior_server bt_navigator
  waypoint_follower velocity_smoother lifecycle_manager
  map_server costmap_filter_info_server
  # SmartCar
  safety_node_cpp safety_node direction_guard
  task_node field_reference_node waypoint_viz waypoint_editor_node
  waypoint_drag_editor voltage_monitor odom_diag
  # Sensors / platform
  origincar_base_node origincar_base
  ydlidar_ros2_driver_node ydlidar_ros2_driver
  obstacle_extractor obstacle_detector_2
  # TF / state publishing
  static_transform_publisher robot_state_publisher joint_state_publisher
  # Vision / camera
  zbar_ros barcode_reader hobot_usb_cam usb_cam_node
  v4l2_camera_node camera_node
  # Speech
  tts_node tts_consumer
  # Visualization
  rviz2 rviz
  # RF2O laser odometry
  rf2o_laser_odometry_node rf2o_node
  # EKF
  ekf_node ekf_localization_node robot_localization_node
)
for name in "${PROCS[@]}"; do
  pkill -9 -x "$name" 2>/dev/null || true
done

# 2b. 按命令行模糊匹配（catch 通过 ros2 run / launch 启动的进程和 Python 脚本）
PATTERNS=(
  "ros2/ros2"
  "ros2 run"
  "ros2 launch"
  "python3.*ros2"
  "python3.*launch"
  "python3.*smartcar"
  "python3.*origincar"
  "python3.*nav2"
  "ros2_daemon"
  "fastrtps"
  "fastdds"
)
for pat in "${PATTERNS[@]}"; do
  pkill -9 -f "$pat" 2>/dev/null || true
done

sleep 1

# ---- 3. 清理共享内存端口 ----
rm -f /dev/shm/fastrtps_port*   2>/dev/null || true
rm -f /dev/shm/fastdds_port*    2>/dev/null || true
rm -f /dev/shm/*ros2*           2>/dev/null || true
rm -f /dev/shm/*fast*           2>/dev/null || true
echo "  SHM ports cleaned"

# ---- 4. 清理 ros2 daemon 缓存与临时文件 ----
rm -rf /tmp/ros2_daemon_*       2>/dev/null || true
rm -rf /tmp/launch_params_*     2>/dev/null || true
rm -rf /tmp/tmpx*launch*        2>/dev/null || true
rm -f  /tmp/ros2_*              2>/dev/null || true
rm -f  /tmp/*.xml_              2>/dev/null || true
# DDS/RMW leftover temp files
rm -f  /tmp/fastrtps_*          2>/dev/null || true
rm -f  /tmp/fastdds_*           2>/dev/null || true
echo "  temp files cleaned"

# ---- 5. 清理孤儿 ros2 daemon 进程与 socket 文件 ----
# 某些版本的 ros2 daemon 使用 Unix socket
rm -f /tmp/.ros2_daemon_socket* 2>/dev/null || true
rm -f /tmp/.ros2_socket_*       2>/dev/null || true

# ---- 6. 重开 daemon ----
if command -v ros2 &>/dev/null; then
  ros2 daemon start 2>/dev/null || true
  echo "  ros2 daemon started"
fi
sleep 1

# ---- 7. 验证 ----
COUNT_CMD="ps aux | grep -v grep | grep -ciE"
REMAINING=$($COUNT_CMD "ekf_node|safety_node|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|lifecycle_manager|task_node|origincar_base|ydlidar|static_transform|rviz2|field_ref|waypoint_viz" 2>/dev/null || printf "0")
echo "Remaining ROS processes: ${REMAINING:-0} (expect 0)"
echo "=== Done ==="
