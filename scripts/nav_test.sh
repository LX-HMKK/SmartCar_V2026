#!/bin/bash
# ============================================================
# SmartCar 纯导航安全测试（DUBIN 虚拟倒车 + 方向门）
# 用法: bash /root/nav_test.sh [--autostart] [--no-rviz]
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
AUTOSTART=false
NO_RVIZ=false
for arg in "$@"; do
  case "$arg" in
    --autostart) AUTOSTART=true ;;
    --no-rviz)   NO_RVIZ=true ;;
    *)
      echo "未知选项: $arg"
      echo "用法: bash /root/nav_test.sh [--autostart] [--no-rviz]"
      exit 1
      ;;
  esac
done

export DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0

banner() { echo ""; echo "=== $* ==="; }

die()  { echo "✗ $*"; exit 1; }

# ---- 1. 清理 ----
banner "[1/7] 彻底清理"
if [ -x /usr/local/bin/ros_cleanup ]; then
  bash /usr/local/bin/ros_cleanup
else
  bash /root/ros2_ws/scripts/ros_cleanup.sh
fi
echo "  ✓ 清理完成"

# ---- 2. 构建 ----
banner "[2/7] 构建"
colcon build --symlink-install \
  --packages-select smartcar_interfaces smartcar_safety smartcar_nav2 smartcar_task smartcar_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --allow-overriding smartcar_interfaces smartcar_safety smartcar_nav2 smartcar_task smartcar_bringup 2>&1 | tail -8 \
  || die "构建失败"
echo "  ✓ 构建完成"

# ---- 3. 参数验证 ----
banner "[3/7] 参数验证"
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

# ---- 4. 先验地图 + 航点 ----
banner "[4/7] 可视化"
ros2 run smartcar_tools field_reference_node --ros-args -r __ns:=/smartcar &>/tmp/field_ref.log &
VIZ_PID1=$!
ros2 run smartcar_tools waypoint_viz --ros-args -p waypoints_file:="$WP" &>/tmp/waypoint_viz.log &
VIZ_PID2=$!
sleep 2
echo "  ✓ field_reference + waypoint_viz 已启动 (PIDs: $VIZ_PID1 $VIZ_PID2)"

# ---- 5. 启动系统 ----
if $AUTOSTART; then
  BANNER_MSG="DUBIN + 强制倒车，自动发车"
  EXTRA_ARGS="autostart_mission:=true safety_emergency_stop_on_start:=false"
else
  BANNER_MSG="DUBIN + 强制倒车，急停锁存"
  EXTRA_ARGS="autostart_mission:=false safety_emergency_stop_on_start:=true"
fi
banner "[5/7] 启动系统 ($BANNER_MSG)"
true > "$LOG"
ros2 launch smartcar_bringup smartcar_system.launch.py \
  use_base:=true use_lidar:=true use_obstacle:=false \
  use_laser_odometry:=false use_safety:=true use_nav:=true \
  nav_autostart:=true use_camera:=false use_vision:=false \
  use_task:=true camera_driver:=usb \
  $EXTRA_ARGS \
  waypoints_calibrated:=true extrinsics_calibrated:=true \
  steering_calibrated:=true emergency_stop_ready:=true \
  operator_approved:=true waypoints_file:="$WP" \
  >> "$LOG" 2>&1 &
LAUNCH_PID=$!
echo "  Launch PID: $LAUNCH_PID"

# ---- 6. 等就绪 ----
banner "[6/7] 等待就绪"
READY=0
for i in $(seq 1 60); do
  if grep -q "Managed nodes are active" "$LOG" 2>/dev/null; then
    READY=1; echo "  Nav2 就绪 (${i}0s)"; break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "  ✗ Launch 异常退出"; tail -30 "$LOG"; exit 1
  fi
  sleep 10
done
[ "$READY" -eq 0 ] && { echo "  ✗ 超时"; tail -30 "$LOG"; exit 1; }
sleep 5
echo "  ✓ 全部 lifecycle active"

# ---- 7. RViz ----
if $NO_RVIZ; then
  banner "[7/7] 跳过 RViz"
  echo "  - RViz 已禁用 (--no-rviz)"
else
  OLD_RVIZ=$(ps -C rviz2 -o pid= --no-headers 2>/dev/null || true)
  [ -n "$OLD_RVIZ" ] && kill -9 $OLD_RVIZ 2>/dev/null || true
  sleep 1
  if $AUTOSTART; then
    banner "[7/7] RViz + 自动发车"
  else
    banner "[7/7] RViz + 等待人工发车"
  fi
  rviz2 -d "$WORKSPACE/src/smartcar_tools/rviz/navigation.rviz" &
  sleep 3
  echo "  ✓ RViz 已启动"
fi

echo ""
echo "╔══════════════════════════════════════════════╗"
if $AUTOSTART; then
  echo "║  系统已就绪，任务将自动开始                    ║"
else
  echo "║  系统已就绪，急停仍锁存，车辆不会自动发车    ║"
  echo "║  确认后手动 reset、解除急停、start            ║"
fi
echo "║  监控: ros2 topic echo /smartcar/task/state  ║"
echo "║  日志: tail -f /tmp/bringup.log               ║"
echo "║  急停: ros2 service call /smartcar/safety/emergency_stop std_srvs/srv/SetBool '{data: true}' ║"
echo "║  硬杀: bash /usr/local/bin/ros_cleanup        ║"
echo "╚══════════════════════════════════════════════╝"
