#!/bin/bash
# ============================================================
# SmartCar 纯导航全流程测试（含先验地图+航点可视化+REEDS_SHEPP 倒车）
# 用法: bash /root/nav_test.sh
# ============================================================
set -euo pipefail

SOURCE_ENV=/root/source_env.sh
WORKSPACE=/root/ros2_ws
LOG=/tmp/bringup.log
GEOM=/root/ros2_ws/src/smartcar_tools/config/routes/field_geometry.yaml
WP=/root/ros2_ws/src/smartcar_nav2/config/waypoints/nav_only.yaml

source "$SOURCE_ENV"
cd "$WORKSPACE"
export DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0

banner() { echo ""; echo "=== $* ==="; }

# ---- 1. 清理 ----
banner "[1/7] 彻底清理"
bash /usr/local/bin/ros_cleanup 2>/dev/null || true
sleep 1

# ---- 2. 构建 ----
banner "[2/7] 构建"
colcon build --symlink-install \
  --packages-select smartcar_nav2 smartcar_task \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --allow-overriding smartcar_nav2 smartcar_task 2>&1 | tail -5
echo "  ✓ 构建完成"

# ---- 3. 参数验证 ----
banner "[3/7] 参数验证"
FIXED_YAML="$WORKSPACE/install/smartcar_nav2/share/smartcar_nav2/config/nav2_params_fixed.yaml"
echo "  motion_model: $(grep motion_model_for_search "$FIXED_YAML" | xargs)"
echo "  allow_reversing: $(grep allow_reversing "$FIXED_YAML" | xargs)"
echo "  yaw_goal_tolerance: $(grep yaw_goal_tolerance "$FIXED_YAML" | xargs)"
echo "  min_velocity: $(grep 'min_velocity:' "$FIXED_YAML" | xargs)"

# ---- 4. 先验地图 + 航点 ----
banner "[4/7] 可视化"
ros2 run smartcar_tools field_reference_node --ros-args -p geometry_file:="$GEOM" &
ros2 run smartcar_tools waypoint_viz --ros-args -p waypoints_file:="$WP" &
sleep 2
echo "  ✓ field_reference + waypoint_viz 已启动"

# ---- 5. 启动系统 ----
banner "[5/7] 启动系统 (REEDS_SHEPP + 倒车)"
true > "$LOG"
ros2 launch smartcar_bringup smartcar_system.launch.py \
  use_base:=true use_lidar:=true use_obstacle:=false \
  use_laser_odometry:=false use_safety:=true use_nav:=true \
  nav_autostart:=true use_camera:=false use_vision:=false \
  use_task:=true autostart_mission:=false camera_driver:=usb \
  safety_emergency_stop_on_start:=false \
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

# ---- 7. RViz + 发车 ----
banner "[7/7] RViz + 发车"
pkill -9 -f rviz2 2>/dev/null || true
sleep 1
rviz2 -d "$WORKSPACE/src/smartcar_tools/rviz/navigation.rviz" &
sleep 3
echo "  ✓ RViz 已启动"

ros2 service call /smartcar/task/reset std_srvs/srv/Trigger "{}" 2>&1 | grep "success"
ros2 service call /smartcar/safety/emergency_stop std_srvs/srv/SetBool "{data: false}" 2>&1 | grep "success"
ros2 service call /smartcar/task/start std_srvs/srv/Trigger "{}" 2>&1 | grep "success"

echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║  导航任务已启动 (REEDS_SHEPP + 倒车)         ║"
echo "║  监控: ros2 topic echo /smartcar/task/state  ║"
echo "║  日志: tail -f /tmp/bringup.log               ║"
echo "║  急停: pkill -9 -f 'ros2 launch'             ║"
echo "╚══════════════════════════════════════════════╝"
