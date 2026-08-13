#!/usr/bin/env bash
# Stop the SmartCar navigation stack without matching the SSH caller.
set -u

STATE_DIR=/tmp/smartcar_nav

stop_saved_process() {
  local pid_file=$1
  local expected=$2
  local pid cmdline

  [ -r "$pid_file" ] || return
  pid=$(cat "$pid_file")
  case "$pid" in
    ''|*[!0-9]*) rm -f "$pid_file"; return ;;
  esac
  [ -r "/proc/$pid/cmdline" ] || { rm -f "$pid_file"; return; }
  cmdline=$(tr '\0' ' ' < "/proc/$pid/cmdline")
  if [[ "$cmdline" != *"$expected"* ]]; then
    rm -f "$pid_file"
    return
  fi

  echo "  stopping PID $pid ($expected)"
  kill -TERM "$pid" 2>/dev/null || true
  for _ in 1 2 3 4 5; do
    kill -0 "$pid" 2>/dev/null || break
    sleep 1
  done
  kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
  rm -f "$pid_file"
}

collect_by_name() {
  local name
  for name in "$@"; do
    pgrep -x "$name" 2>/dev/null || true
  done
}

collect_by_command() {
  local pattern
  for pattern in "$@"; do
    pgrep -f "$pattern" 2>/dev/null || true
  done
}

remove_stale_fastdds_ports() {
  local port

  command -v fuser >/dev/null 2>&1 || return
  for port in /dev/shm/fastrtps_port* /dev/shm/fastdds_port*; do
    [ -e "$port" ] || continue
    # Never unlink a port held by a still-running DDS participant.
    fuser -s "$port" >/dev/null 2>&1 && continue
    rm -f -- "$port"
  done
}

echo "=== SmartCar cleanup ==="
stop_saved_process "$STATE_DIR/launch.pid" \
  "ros2 launch smartcar_bringup smartcar_system.launch.py"
stop_saved_process "$STATE_DIR/rviz.pid" "navigation.rviz"
rmdir "$STATE_DIR" 2>/dev/null || true

TARGET_PIDS=$( {
  collect_by_name \
    ekf_node controller_server planner_server behavior_server bt_navigator \
    velocity_smoother lifecycle_manager safety_node_cpp direction_guard_node \
    origincar_base_node aurora930_node task_node depth_pointcloud_relay \
    pointcloud_to_laserscan_node
  collect_by_command \
    '/nav2_lifecycle_manager/lifecycle_manager' \
    '/tf2_ros/static_transform_publisher.*__node:=(base_to_link|base_to_gyro|link_to_laser|link_to_depth_camera_sensor)' \
    '/smartcar_tools/(field_reference_node|waypoint_viz)' \
    '/smartcar_(task|safety|vision|nav2|bringup)/' \
    '/origincar_base/' \
    'pointcloud_to_laserscan' \
    'deptrum-ros-driver-aurora930' \
    'smartcar_tools/rviz/navigation.rviz'
} | sort -un)

if [ -n "$TARGET_PIDS" ]; then
  echo "  stopping remaining PIDs: $(tr '\n' ' ' <<< "$TARGET_PIDS")"
  kill -TERM $TARGET_PIDS 2>/dev/null || true
  sleep 2
  TARGET_PIDS=$(printf '%s\n' "$TARGET_PIDS" | while read -r pid; do
    kill -0 "$pid" 2>/dev/null && printf '%s\n' "$pid"
  done)
  [ -z "$TARGET_PIDS" ] || kill -KILL $TARGET_PIDS 2>/dev/null || true
fi

remove_stale_fastdds_ports
echo "=== Done ==="
