#!/bin/bash
# ============================================================
# SmartCar 纯导航安全测试（DUBIN 全正向路线 + 方向门）
# 用法: bash /root/nav_test.sh [--no-rviz] [--depth-camera] [--p-to-a|--p-to-c1] [--supervised-p-to-a|--supervised-p-to-c1] [--reset-origin]
# ============================================================
set -uo pipefail

SOURCE_ENV=/root/source_env.sh
WORKSPACE=/root/ros2_ws
LOG=/tmp/bringup.log
GEOM=/root/ros2_ws/src/smartcar_tools/config/routes/field_geometry.yaml
WP=/root/ros2_ws/src/smartcar_nav2/config/waypoints/nav_only.yaml

# TROS humble setup.bash has unbound AMENT_TRACE_SETUP_FILES — disable
# nounset around the source to avoid script exit under set -uo pipefail.
set +u
source "$SOURCE_ENV"
set -u
cd "$WORKSPACE"

# ---- 参数解析 ----
NO_RVIZ=false
END_SEGMENT_ID=""
RESET_ORIGIN=false
SUPERVISED_P_TO_A=false
SUPERVISED_P_TO_C1=false
DEPTH_CAMERA=false
for arg in "$@"; do
  case "$arg" in
    --no-rviz)   NO_RVIZ=true ;;
    --p-to-a)    END_SEGMENT_ID="p_to_qr" ;;
    --p-to-c1)   END_SEGMENT_ID="qr_to_vlm" ;;
    --supervised-p-to-a) SUPERVISED_P_TO_A=true ;;
    --supervised-p-to-c1) SUPERVISED_P_TO_C1=true ;;
    --reset-origin) RESET_ORIGIN=true ;;
    --depth-camera) DEPTH_CAMERA=true ;;
    *)
      echo "未知选项: $arg"
      echo "用法: bash /root/nav_test.sh [--no-rviz] [--depth-camera] [--p-to-a|--p-to-c1] [--supervised-p-to-a|--supervised-p-to-c1] [--reset-origin]"
      exit 1
      ;;
  esac
done

if $SUPERVISED_P_TO_A && [ "$END_SEGMENT_ID" != "p_to_qr" ]; then
  echo "✗ --supervised-p-to-a 必须同时指定 --p-to-a"
  exit 1
fi
if $SUPERVISED_P_TO_C1 && [ "$END_SEGMENT_ID" != "qr_to_vlm" ]; then
  echo "✗ --supervised-p-to-c1 必须同时指定 --p-to-c1"
  exit 1
fi
if $SUPERVISED_P_TO_A && $SUPERVISED_P_TO_C1; then
  echo "✗ 一次只能选择一个受看护导航前缀"
  exit 1
fi
if $SUPERVISED_P_TO_A || $SUPERVISED_P_TO_C1; then
  # A watched P-to-A measurement is only meaningful when the vehicle's
  # current, manually aligned heading is made the localization origin.
  RESET_ORIGIN=true
fi
if $DEPTH_CAMERA && { $SUPERVISED_P_TO_A || $SUPERVISED_P_TO_C1; }; then
  echo "✗ 深度相机尚未完成外参与障碍物标定，不能用于受看护运动前缀"
  exit 1
fi

export DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0

banner() { echo ""; echo "=== $* ==="; }

die()  { echo "✗ $*"; exit 1; }

require_parameter() {
  local node=$1
  local parameter=$2
  local expected=$3
  local result
  local attempt

  for attempt in 1 2 3; do
    # After the parameter service is visible, a persistent CLI daemon keeps
    # its Fast DDS graph between successive parameter checks.  A fresh
    # --no-daemon process is retained as a bounded fallback for a stale graph.
    result=$(timeout 15s ros2 param get "$node" "$parameter" 2>&1) || true
    if [[ "$result" == *"$expected"* ]]; then
      echo "  ✓ $node $parameter = $expected"
      return 0
    fi
    result=$(timeout 15s ros2 param get --no-daemon "$node" "$parameter" 2>&1) || true
    if [[ "$result" == *"$expected"* ]]; then
      echo "  ✓ $node $parameter = $expected"
      return 0
    fi
    if [ "$attempt" -lt 3 ]; then
      sleep 2
    fi
  done

  echo "  ✗ $node 参数 $parameter 不是 $expected"
  echo "    实际: $result"
  return 1
}

wait_for_parameter_service() {
  local node=$1
  local service_name="${node}/get_parameters"
  local services
  local attempt

  for attempt in 1 2 3 4 5; do
    services=$(timeout 15s ros2 service list --no-daemon 2>&1) || true
    if [[ "$services" == *"$service_name"* ]]; then
      echo "  ✓ 参数服务已发现: $service_name"
      return 0
    fi
    if [ "$attempt" -lt 5 ]; then
      sleep 2
    fi
  done

  echo "  ✗ 未发现参数服务 $service_name"
  echo "    实际: $services"
  return 1
}

require_topic_sample() {
  local topic=$1
  local message_type=$2

  if ! timeout 8s ros2 topic echo --no-daemon --once --qos-reliability best_effort \
      "$topic" "$message_type" >/dev/null 2>&1; then
    echo "  ✗ 8 秒内未收到 $topic ($message_type)"
    return 1
  fi
  echo "  ✓ 收到 $topic"
}

require_latched_topic_sample() {
  local topic=$1
  local message_type=$2

  # The map server publishes these configuration messages once with
  # transient-local durability. A volatile late subscriber would falsely
  # report the already-active keepout stack as unavailable.
  if ! timeout 8s ros2 topic echo --no-daemon --once --qos-reliability reliable \
      --qos-durability transient_local "$topic" "$message_type" >/dev/null 2>&1; then
    echo "  ✗ 8 秒内未收到锁存话题 $topic ($message_type)"
    return 1
  fi
  echo "  ✓ 收到锁存话题 $topic"
}

require_latched_status() {
  local topic=$1
  local expected=$2
  local result

  result=$(timeout 12s ros2 topic echo --no-daemon --once \
    --qos-reliability reliable --qos-durability transient_local \
    "$topic" std_msgs/msg/String 2>&1) || true
  if [[ "$result" != *"data: $expected"* ]]; then
    echo "  ✗ $topic 状态不是 $expected"
    echo "    实际: $result"
    return 1
  fi
  echo "  ✓ $topic = $expected"
}

verify_obstacle_avoidance() {
  banner "避障与禁区验证"
  local costmap
  local expected_sources="String value is: scan"
  if $DEPTH_CAMERA; then
    expected_sources="String value is: depth_points"
  fi
  for costmap in /local_costmap/local_costmap /global_costmap/global_costmap; do
    wait_for_parameter_service "$costmap" || return 1
    require_parameter "$costmap" "obstacle_layer.enabled" "Boolean value is: True" || return 1
    require_parameter "$costmap" "obstacle_layer.observation_sources" "$expected_sources" || return 1
    require_parameter "$costmap" "inflation_layer.enabled" "Boolean value is: True" || return 1
    require_parameter "$costmap" "keepout_filter.enabled" "Boolean value is: True" || return 1
    require_parameter "$costmap" "keepout_filter.filter_info_topic" "String value is: /keepout_filter_info" || return 1
    if $DEPTH_CAMERA; then
      require_parameter "$costmap" "obstacle_layer.depth_points.topic" "String value is: /smartcar/depth/points" || return 1
      require_parameter "$costmap" "obstacle_layer.depth_points.data_type" "String value is: PointCloud2" || return 1
      require_parameter "$costmap" "obstacle_layer.depth_points.observation_persistence" "Double value is: 0.0" || return 1
      require_parameter "$costmap" "obstacle_layer.depth_points.marking" "Boolean value is: True" || return 1
      require_parameter "$costmap" "obstacle_layer.depth_points.clearing" "Boolean value is: True" || return 1
    else
      require_parameter "$costmap" "obstacle_layer.scan.topic" "String value is: /scan" || return 1
      require_parameter "$costmap" "obstacle_layer.scan.data_type" "String value is: LaserScan" || return 1
      require_parameter "$costmap" "obstacle_layer.scan.observation_persistence" "Double value is: 0.0" || return 1
      require_parameter "$costmap" "obstacle_layer.scan.min_obstacle_height" "Double value is: 0.05" || return 1
      require_parameter "$costmap" "obstacle_layer.scan.max_obstacle_height" "Double value is: 0.5" || return 1
      require_parameter "$costmap" "obstacle_layer.scan.inf_is_valid" "Boolean value is: False" || return 1
    fi
  done
  if $DEPTH_CAMERA; then
    require_topic_sample /smartcar/depth/points sensor_msgs/msg/PointCloud2 || return 1
    require_latched_status /smartcar/depth_obstacles/status depth_points_active || return 1
    wait_for_parameter_service /safety_node || return 1
    require_parameter /safety_node "require_depth_points" "Boolean value is: True" || return 1
    require_parameter /safety_node "require_scan" "Boolean value is: False" || return 1
    require_parameter /safety_node "depth_points_topic" "String value is: /smartcar/depth/points" || return 1
  else
    require_topic_sample /scan sensor_msgs/msg/LaserScan || return 1
  fi
  require_latched_topic_sample /keepout_filter_mask nav_msgs/msg/OccupancyGrid || return 1
  require_latched_topic_sample /keepout_filter_info nav2_msgs/msg/CostmapFilterInfo || return 1
  require_topic_sample /local_costmap/costmap_raw nav2_msgs/msg/Costmap || return 1
  require_topic_sample /global_costmap/costmap_raw nav2_msgs/msg/Costmap || return 1
  require_topic_sample /local_costmap/costmap nav_msgs/msg/OccupancyGrid || return 1
  require_topic_sample /global_costmap/costmap nav_msgs/msg/OccupancyGrid || return 1
}

# ---- 1. 清理 ----
banner "[1/6] 彻底清理"
if [ -x /usr/local/bin/ros_cleanup ]; then
  bash /usr/local/bin/ros_cleanup
else
  bash /root/ros2_ws/scripts/ros_cleanup.sh
fi
echo "  ✓ 清理完成"

# ---- 2. 构建 ----
banner "[2/6] 构建"
colcon build --symlink-install \
  --packages-select smartcar_interfaces smartcar_safety smartcar_nav2 smartcar_task smartcar_bringup smartcar_vision \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --allow-overriding smartcar_interfaces smartcar_safety smartcar_nav2 smartcar_task smartcar_bringup 2>&1 | tail -8 \
  || die "构建失败"
echo "  ✓ 构建完成"

# ---- 3. 参数验证 ----
banner "[3/6] 参数验证"
FIXED_YAML="$WORKSPACE/install/smartcar_nav2/share/smartcar_nav2/config/nav2_params_fixed.yaml"
if [ -f "$FIXED_YAML" ]; then
  echo "  motion_model: $(grep motion_model_for_search "$FIXED_YAML" | xargs)"
  echo "  allow_reversing: $(grep allow_reversing "$FIXED_YAML" | xargs)"
  echo "  yaw_goal_tolerance: $(grep yaw_goal_tolerance "$FIXED_YAML" | xargs)"
  echo "  min_velocity: $(grep 'min_velocity:' "$FIXED_YAML" | xargs)"
  echo "  current_goal_checker: $(grep current_goal_checker "$FIXED_YAML" | xargs)"
else
  echo "  ⚠ nav2_params_fixed.yaml 未找到，将使用默认参数"
fi

# ---- 4. 启动系统 ----
BANNER_MSG="DUBIN + 全正向路线，急停锁存"
EXTRA_ARGS="autostart_mission:=false safety_emergency_stop_on_start:=true"
CAMERA_ARGS="use_camera:=false use_vision:=false camera_driver:=usb use_depth_camera:=false"
LIDAR_ARGS="use_lidar:=true"
if $DEPTH_CAMERA; then
  BANNER_MSG="DUBIN + Aurora 深度障碍感知，急停锁存"
  CAMERA_ARGS="use_camera:=false use_vision:=false camera_driver:=aurora use_depth_camera:=true"
  LIDAR_ARGS="use_lidar:=false"
fi
if [ -n "$END_SEGMENT_ID" ]; then
  EXTRA_ARGS="$EXTRA_ARGS navigation_test_end_segment_id:=$END_SEGMENT_ID"
fi
if $SUPERVISED_P_TO_A || $SUPERVISED_P_TO_C1; then
  # The task node permits only one fixed pure-navigation prefix.  This is an
  # explicit operator attestation for a watched field test, never a default
  # or a full-route authorization.
  SUPERVISED_ARG="supervised_p_to_a_only:=true"
  if $SUPERVISED_P_TO_C1; then
    SUPERVISED_ARG="supervised_p_to_c1_only:=true"
  fi
  EXTRA_ARGS="$EXTRA_ARGS $SUPERVISED_ARG waypoints_calibrated:=true extrinsics_calibrated:=true steering_calibrated:=true emergency_stop_ready:=true operator_approved:=true"
fi
if $NO_RVIZ; then
  VISUALIZATION_ARG="use_visualization:=false"
else
  VISUALIZATION_ARG="use_visualization:=true"
fi
banner "[4/6] 启动系统 ($BANNER_MSG)"
true > "$LOG"
ros2 launch smartcar_bringup smartcar_system.launch.py \
  use_base:=true $LIDAR_ARGS \
  use_laser_odometry:=false use_safety:=true use_nav:=true \
  nav_autostart:=true $CAMERA_ARGS \
  use_task:=true \
  $EXTRA_ARGS $VISUALIZATION_ARG waypoints_file:="$WP" \
  >> "$LOG" 2>&1 &
LAUNCH_PID=$!
echo "  Launch PID: $LAUNCH_PID"

# ---- 5. 等就绪 ----
banner "[5/6] 等待就绪"
READY=0
READY_TIMEOUT_SEC=120
for i in $(seq 1 $((READY_TIMEOUT_SEC / 10))); do
  if grep -q "Managed nodes are active" "$LOG" 2>/dev/null; then
    READY=1; echo "  Nav2 就绪 (${i}0s)"; break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "  ✗ Launch 异常退出"; tail -30 "$LOG"; exit 1
  fi
  sleep 10
done
[ "$READY" -eq 0 ] && {
  echo "  ✗ Nav2 lifecycle ${READY_TIMEOUT_SEC}s 内未全部 active"
  tail -60 "$LOG"
  exit 1
}
sleep 5
echo "  ✓ 全部 lifecycle active"
verify_obstacle_avoidance || die "避障感知未就绪；急停保持锁存，禁止解除急停或发车"

if $RESET_ORIGIN; then
  banner "定位原点复位"
  RESET_RESULT=$(timeout 12s ros2 service call /smartcar/task/reset \
    std_srvs/srv/Trigger "{}" 2>&1) || {
      echo "$RESET_RESULT"
      die "定位复位调用失败；急停保持锁存"
    }
  if [[ "$RESET_RESULT" != *"success=True"* ]]; then
    echo "$RESET_RESULT"
    die "定位复位未通过原点验证；急停保持锁存"
  fi
  echo "  ✓ 已将人工放置的 P 点当前位置和航向写入定位原点"
fi

# ---- 6. RViz ----
if $NO_RVIZ; then
  banner "[6/6] 跳过 RViz"
  echo "  - RViz 已禁用 (--no-rviz)"
else
  OLD_RVIZ=$(ps -C rviz2 -o pid= --no-headers 2>/dev/null || true)
  [ -n "$OLD_RVIZ" ] && kill -9 $OLD_RVIZ 2>/dev/null || true
  sleep 1
  banner "[6/6] RViz + 等待人工发车"
  rviz2 -d "$WORKSPACE/src/smartcar_tools/rviz/navigation.rviz" &
  sleep 3
  echo "  ✓ RViz 已启动"
fi

echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║  系统已就绪，急停仍锁存，车辆不会自动发车    ║"
echo "║  本脚本不授予运动门禁                          ║"
echo "║  监控: ros2 topic echo /smartcar/task/state  ║"
echo "║  日志: tail -f /tmp/bringup.log               ║"
echo "║  急停: ros2 service call /smartcar/safety/emergency_stop std_srvs/srv/SetBool '{data: true}' ║"
echo "║  硬杀: bash /usr/local/bin/ros_cleanup        ║"
echo "╚══════════════════════════════════════════════╝"
