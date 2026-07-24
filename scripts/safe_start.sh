#!/bin/bash
# ============================================================
# SmartCar 安全启动 —— 急停锁存，仅可视化，不驱动
# 用法: bash /root/safe_start.sh
#   1. 开启 RViz（场地参考层 + 航点标记）
#   2. 启动系统（急停锁存）
#   3. 等待 Nav2 就绪
#   4. 提示用户检查 RViz 中定位是否正确
#   5. 用户确认后手动解除急停并发车
# ============================================================
set -euo pipefail
source /root/source_env.sh

BLOG=/tmp/bringup_safe.log
GEOM=/root/ros2_ws/src/smartcar_tools/config/routes/field_geometry.yaml
WP=/root/ros2_ws/src/smartcar_nav2/config/waypoints/default_waypoints.yaml
export DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0

echo "=== [1/5] 清理 ==="
bash /usr/local/bin/ros_cleanup 2>/dev/null || true
sleep 1

echo "=== [2/5] 构建 ==="
cd /root/ros2_ws
colcon build --symlink-install \
  --packages-select smartcar_interfaces smartcar_safety smartcar_nav2 smartcar_task smartcar_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --allow-overriding smartcar_interfaces smartcar_safety smartcar_nav2 smartcar_task smartcar_bringup 2>&1 | tail -8

echo "=== [3/5] 可视化节点 ==="
ros2 run smartcar_tools field_reference_node --ros-args -p geometry_file:="$GEOM" &
ros2 run smartcar_tools waypoint_viz --ros-args -p waypoints_file:="$WP" &
sleep 2
rviz2 -d /root/ros2_ws/src/smartcar_tools/rviz/navigation.rviz &
sleep 3
echo "RViz 已启动——检查场地参考层和航点"

echo "=== [4/5] 启动系统（急停锁存） ==="
true > "$BLOG"
ros2 launch smartcar_bringup smartcar_system.launch.py \
  use_base:=true use_lidar:=true use_obstacle:=false \
  use_laser_odometry:=false use_safety:=true use_nav:=true \
  nav_autostart:=true use_camera:=false use_vision:=false \
  use_task:=true autostart_mission:=false camera_driver:=usb \
  safety_emergency_stop_on_start:=true \
  waypoints_calibrated:=true extrinsics_calibrated:=true \
  steering_calibrated:=true emergency_stop_ready:=true \
  operator_approved:=true waypoints_file:="$WP" \
  >> "$BLOG" 2>&1 &
LPID=$!

echo "=== [5/5] 等待就绪 ==="
for i in $(seq 1 60); do
  grep -q "Managed nodes are active" "$BLOG" 2>/dev/null && { echo "Nav2 就绪 (${i}0s)"; break; }
  kill -0 $LPID 2>/dev/null || { echo "启动失败"; tail -20 "$BLOG"; exit 1; }
  sleep 10
done
sleep 10  # 稳定期

echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║  系统已就绪，急停锁存中                      ║"
echo "║  请在 RViz 中检查：                          ║"
echo "║    - 场地参考层位置正确                      ║"
echo "║    - 航点标记位置正确                        ║"
echo "║    - /odom_combined 定位无漂移                ║"
echo "║                                              ║"
echo "║  确认无误后，手动发车：                       ║"
echo "║    ros2 service call /smartcar/safety/emergency_stop std_srvs/srv/SetBool \"{data: false}\""
echo "║    ros2 service call /smartcar/task/start std_srvs/srv/Trigger \"{}\""
echo "║                                              ║"
echo "║  紧急停车：                                   ║"
echo "║    ros2 service call /smartcar/safety/emergency_stop std_srvs/srv/SetBool \"{data: true}\""
echo "║    pkill -9 -f 'ros2 launch'                 ║"
echo "║  日志: tail -f /tmp/bringup_safe.log          ║"
echo "╚══════════════════════════════════════════════╝"

wait $LPID 2>/dev/null
