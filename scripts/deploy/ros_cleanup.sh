#!/bin/bash
# ============================================================
# SmartCar ROS2 全量清理脚本 (v3 — PID 模式，防 SSH 自杀)
# 部署: cp /root/ros2_ws/scripts/ros_cleanup.sh /usr/local/bin/ros_cleanup && chmod +x
# 用法: bash /usr/local/bin/ros_cleanup
#
# 安全保证:
#   1. ps -C + pgrep 收集 PID → kill，不再用 pkill -f 模糊匹配
#   2. pkill -f 'ros2 launch' 会匹配 SSH 会话自身 → 连接断开
#   3. 保护调用者 PPID 链到 sshd，不被误杀
#   4. 两阶段终止: SIGTERM (优雅) → 等待 → SIGKILL (强制)
# ============================================================
set -euo pipefail

echo ""
echo "=== SmartCar ROS2 全量清理 ==="
echo ""

# ---- 0. 保护树 ----
PROTECTED_PIDS="$$ $PPID"
PARENT=$PPID
while [ "$PARENT" -gt 1 ] 2>/dev/null; do
  PROTECTED_PIDS="$PROTECTED_PIDS $PARENT"
  PARENT=$(ps -o ppid= -p "$PARENT" 2>/dev/null | tr -d ' ')
done
for pid in $(pgrep -P "$$" 2>/dev/null || true); do
  PROTECTED_PIDS="$PROTECTED_PIDS $pid"
done

is_protected() {
  for pp in $PROTECTED_PIDS; do
    [ "$1" = "$pp" ] && return 0
  done
  return 1
}

kill_pid() {
  local sig="$1" pid="$2"
  is_protected "$pid" && return 1
  kill "-${sig}" "$pid" 2>/dev/null || true
}

# ---- 收集所有 ROS 进程 PID ----
collect_pids() {
  local all=""

  # C++ 二进制: ps -C 精确匹配
  local cxx_bins=(
    safety_node_cpp direction_guard_node
    controller_server planner_server bt_navigator
    velocity_smoother lifecycle_manager
    origincar_base_node
    ydlidar_ros2_driver_node ydlidar_ros2_driver
    obstacle_extractor_node
    rf2o_laser_odometry_node
    static_transform_publisher robot_state_publisher joint_state_publisher
    ros2_daemon
  )
  local bin_list=$(IFS=,; echo "${cxx_bins[*]}")
  local new_pids=$(ps -C "$bin_list" -o pid= --no-headers 2>/dev/null || true)
  [ -n "$new_pids" ] && all="$all $(echo "$new_pids" | tr '\n' ' ')"

  # Python: pgrep -f 匹配 cmdline 关键词
  local py_pats=(
    "ros2 run"
    "ros2 launch"
    "python3.*ros2"
    "python3.*launch"
    "python3.*smartcar"
    "python3.*origincar"
    "python3.*nav2"
  )
  for pat in "${py_pats[@]}"; do
    for pid in $(pgrep -f "$pat" 2>/dev/null || true); do
      all="$all $pid"
    done
  done

  # 可视化
  for bin in rviz2 rviz; do
    new_pids=$(ps -C "$bin" -o pid= --no-headers 2>/dev/null || true)
    [ -n "$new_pids" ] && all="$all $(echo "$new_pids" | tr '\n' ' ')"
  done

  # 过滤保护 PID + 去重
  local filtered=""
  for pid in $all; do
    pid=$(echo "$pid" | tr -d ' ')
    [ -z "$pid" ] && continue
    is_protected "$pid" && continue
    filtered="$filtered $pid"
  done
  echo "$filtered" | tr ' ' '\n' | sort -u | tr '\n' ' '
}

# ---- 1. 停 daemon ----
echo "[1/6] 停止 ros2 daemon"
ros2 daemon stop 2>/dev/null || true
sleep 1
# 补充 kill daemon 进程
for pid in $(ps -C ros2_daemon -o pid= --no-headers 2>/dev/null || true); do
  kill_pid 9 "$pid" || true
done
echo "  daemon stopped"

# ---- 2. 收集目标 PID ----
echo "[2/6] 收集 ROS 进程 PID"
TARGET_PIDS=$(collect_pids)
if [ -z "$(echo "$TARGET_PIDS" | tr -d ' ')" ]; then
  echo "  无 ROS 进程"
else
  echo "  目标 PID: $TARGET_PIDS"
fi

# ---- 3. SIGTERM 优雅终止 ----
echo "[3/6] 发送 SIGTERM"
TERM_COUNT=0
for pid in $TARGET_PIDS; do
  kill_pid 15 "$pid" && ((TERM_COUNT=TERM_COUNT+1)) || true
done
echo "  TERM: $TERM_COUNT 进程"
sleep 2

# ---- 4. SIGKILL 强制终止 ----
echo "[4/6] 发送 SIGKILL (强制)"
KILL_COUNT=0
for pid in $TARGET_PIDS; do
  if kill -0 "$pid" 2>/dev/null; then
    kill_pid 9 "$pid" && ((KILL_COUNT=KILL_COUNT+1)) || true
  fi
done
echo "  KILL: $KILL_COUNT 进程"
sleep 1

# ---- 5. 清理 DDS/临时文件 ----
echo "[5/6] 清理 DDS 共享内存 + 临时文件"
SHM_CLEANED=0
for pattern in fastrtps dds ros2 eprosima cyclone iceoryx rmw; do
  for item in /dev/shm/*"${pattern}"*; do
    [ -e "$item" ] || continue
    rm -rf "$item" 2>/dev/null && ((SHM_CLEANED=SHM_CLEANED+1)) || true
  done
done
echo "  共享内存: $SHM_CLEANED 项"

rm -rf /tmp/ros2_daemon_*       2>/dev/null || true
rm -rf /tmp/launch_params_*     2>/dev/null || true
rm -rf /tmp/tmpx*launch*        2>/dev/null || true
rm -f  /tmp/ros2_*              2>/dev/null || true
rm -f  /tmp/*.xml_              2>/dev/null || true
rm -f  /tmp/fastrtps_*          2>/dev/null || true
rm -f  /tmp/fastdds_*           2>/dev/null || true
rm -f  /tmp/.ros2_daemon_socket* 2>/dev/null || true
rm -f  /tmp/.ros2_socket_*       2>/dev/null || true
echo "  临时文件已清理"

# ---- 6. 重开 daemon + 验证 ----
echo "[6/6] 重开 daemon + 验证"
ros2 daemon start 2>/dev/null || {
  echo "  ! daemon start 失败，重试..."
  sleep 2
  ros2 daemon start 2>/dev/null || echo "  ! daemon 启动失败"
}
sleep 1

REMAINING_FILTER="ekf_node|safety_node|controller_server|planner_server|bt_navigator|waypoint_follower|velocity_smoother|lifecycle_manager|task_node|origincar_base|ydlidar|static_transform|rviz2|field_ref|waypoint_viz|direction_guard|barcode_reader"
REMAINING=$(ps aux | grep -v grep | grep -ciE "$REMAINING_FILTER" 2>/dev/null || printf "0")
if [ "${REMAINING:-0}" -gt 0 ]; then
  echo "  ! 仍有 ${REMAINING} 个 ROS 进程存活:"
  ps aux | grep -E "$REMAINING_FILTER" | grep -v grep | head -10
else
  echo "  OK: 无 ROS 进程残留"
fi

echo ""
echo "=== 清理完成, 系统就绪 ==="
