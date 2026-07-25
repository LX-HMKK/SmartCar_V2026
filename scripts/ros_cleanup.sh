#!/bin/bash
# ============================================================
# SmartCar ROS2 全量清理脚本（PID 模式，避免 SSH 自杀）
# 部署: cp scripts/ros_cleanup.sh /usr/local/bin/ros_cleanup && chmod +x
# 用法: bash /usr/local/bin/ros_cleanup
#
# 安全保证:
#   1. 用 ps -C + pgrep 收集 PID，然后 kill -9，不用 pkill -f
#   2. pkill -f 'ros2 launch' 会匹配 SSH 会话自身 → 连接断开，后续 kill 全不执行
#   3. 保护调用者自身、父 shell 和 SSH 会话链，不被误杀
# ============================================================

echo "=== SmartCar ROS2 全量清理 ==="

# ---- 0. 保护树：找到自身、父 shell 和 SSH 会话链 ----
PROTECTED_PIDS="$$"
# 沿 PPID 链向上，收集所有祖先 PID，直到 init(1)
PARENT=$PPID
while [ "$PARENT" -gt 1 ] 2>/dev/null; do
  PROTECTED_PIDS="$PROTECTED_PIDS $PARENT"
  PARENT=$(ps -o ppid= -p "$PARENT" 2>/dev/null | tr -d ' ')
done
# 额外保护：当前 shell 的所有子进程（如 grep/ps 本身可能被误杀）
for pid in $(pgrep -P "$$" 2>/dev/null || true); do
  PROTECTED_PIDS="$PROTECTED_PIDS $pid"
done
echo "  protected PIDs: $PROTECTED_PIDS"

# ---- 1. 停 daemon（在杀进程之前，防止自动重启） ----
if command -v ros2 &>/dev/null; then
  ros2 daemon stop 2>/dev/null || true
  echo "  ros2 daemon stopped"
else
  echo "  ros2 CLI unavailable, skipping daemon stop"
fi

# ---- 2. 收集所有 ROS 相关进程 PID ----
TARGET_PIDS=""

# 2a. C++ 编译的可执行文件 — 进程名即二进制名，ps -C 精确匹配
CXX_BINS=(
  # SmartCar
  safety_node_cpp direction_guard_node
  # Nav2
  controller_server planner_server bt_navigator
  velocity_smoother lifecycle_manager
  # Sensors / platform
  origincar_base_node
  ydlidar_ros2_driver_node ydlidar_ros2_driver
  obstacle_extractor_node
  # RF2O laser odometry
  rf2o_laser_odometry_node
  # TF / state publishing
  static_transform_publisher robot_state_publisher joint_state_publisher
  # ROS2 daemon (in case ros2 daemon stop didn't work)
  ros2_daemon
)
BIN_LIST=$(IFS=,; echo "${CXX_BINS[*]}")
NEW_PIDS=$(ps -C "$BIN_LIST" -o pid= --no-headers 2>/dev/null || true)
if [ -n "$NEW_PIDS" ]; then
  TARGET_PIDS="$TARGET_PIDS $(echo "$NEW_PIDS" | tr '\n' ' ')"
fi

# 2b. Python 脚本 — 进程名是 python3，需 grep cmdline
#     只在 cmdline 中匹配 ROS/SmartCar/Nav2 关键词，避免误杀
PY_PATTERNS=(
  "ros2 run"
  "ros2 launch"
  "python3.*ros2"
  "python3.*launch"
  "python3.*smartcar"
  "python3.*origincar"
  "python3.*nav2"
)
for pat in "${PY_PATTERNS[@]}"; do
  for pid in $(pgrep -f "$pat" 2>/dev/null || true); do
    TARGET_PIDS="$TARGET_PIDS $pid"
  done
done

# 2c. 可视化
for bin in rviz2 rviz; do
  NEW_PIDS=$(ps -C "$bin" -o pid= --no-headers 2>/dev/null || true)
  [ -n "$NEW_PIDS" ] && TARGET_PIDS="$TARGET_PIDS $(echo "$NEW_PIDS" | tr '\n' ' ')"
done

# ---- 3. 从目标中移除受保护 PID ----
FILTERED=""
for pid in $TARGET_PIDS; do
  pid=$(echo "$pid" | tr -d ' ')
  [ -z "$pid" ] && continue
  IS_PROTECTED=0
  for pp in $PROTECTED_PIDS; do
    if [ "$pid" = "$pp" ]; then
      IS_PROTECTED=1
      break
    fi
  done
  if [ "$IS_PROTECTED" -eq 0 ]; then
    FILTERED="$FILTERED $pid"
  fi
done
TARGET_PIDS=$(echo "$FILTERED" | tr ' ' '\n' | sort -u | tr '\n' ' ')

# ---- 4. 按 PID 杀进程 ----
if [ -z "$(echo "$TARGET_PIDS" | tr -d ' ')" ]; then
  echo "  no ROS processes to kill"
else
  echo "  killing PIDs: $TARGET_PIDS"
  kill -9 $TARGET_PIDS 2>/dev/null || true
  sleep 1
  # 二次确认：已杀 PIDs 中仍存活的，再杀
  STILL_ALIVE=""
  for pid in $TARGET_PIDS; do
    if kill -0 "$pid" 2>/dev/null; then
      STILL_ALIVE="$STILL_ALIVE $pid"
    fi
  done
  if [ -n "$STILL_ALIVE" ]; then
    echo "  retry killing: $STILL_ALIVE"
    kill -9 $STILL_ALIVE 2>/dev/null || true
    sleep 1
  fi
fi

# ---- 5. 清理共享内存端口 ----
rm -f /dev/shm/fastrtps_port*   2>/dev/null || true
rm -f /dev/shm/fastdds_port*    2>/dev/null || true
rm -f /dev/shm/*ros2*           2>/dev/null || true
rm -f /dev/shm/*fast*           2>/dev/null || true
echo "  SHM ports cleaned"

# ---- 6. 清理临时文件 ----
rm -rf /tmp/ros2_daemon_*       2>/dev/null || true
rm -rf /tmp/launch_params_*     2>/dev/null || true
rm -rf /tmp/tmpx*launch*        2>/dev/null || true
rm -f  /tmp/ros2_*              2>/dev/null || true
rm -f  /tmp/*.xml_              2>/dev/null || true
rm -f  /tmp/fastrtps_*          2>/dev/null || true
rm -f  /tmp/fastdds_*           2>/dev/null || true
rm -f  /tmp/.ros2_daemon_socket* 2>/dev/null || true
rm -f  /tmp/.ros2_socket_*       2>/dev/null || true
echo "  temp files cleaned"

# ---- 7. 重开 daemon ----
if command -v ros2 &>/dev/null; then
  ros2 daemon start 2>/dev/null || true
  echo "  ros2 daemon started"
fi
sleep 1

# ---- 8. 验证 ----
COUNT_CMD="ps aux | grep -v grep | grep -ciE"
REMAINING=$($COUNT_CMD "ekf_node|safety_node|controller_server|planner_server|bt_navigator|waypoint_follower|velocity_smoother|lifecycle_manager|task_node|origincar_base|ydlidar|static_transform|rviz2|field_ref|waypoint_viz|direction_guard" 2>/dev/null || printf "0")
echo "Remaining ROS processes: ${REMAINING:-0} (expect 0)"
echo "=== Done ==="
