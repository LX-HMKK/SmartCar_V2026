#!/bin/bash
# ============================================================
# SmartCar 纯导航全流程测试（无视觉任务）
# 用法: bash /root/nav_test.sh
# ============================================================
set -o pipefail

SOURCE_ENV=/root/source_env.sh
WORKSPACE=/root/ros2_ws
LOG=/tmp/bringup.log
WAYPOINTS_FILE=/root/ros2_ws/src/smartcar_nav2/config/waypoints/nav_only.yaml

source "$SOURCE_ENV"
cd "$WORKSPACE"

banner() { echo ""; echo "=== $* ==="; }

# ---- 1. 全杀 ----
banner "[1/6] 彻底清理"
python3 -c "
import os, subprocess
my_pid = os.getpid()
parent = os.getppid()
skip_pids = {my_pid, parent}
procs = subprocess.check_output(['ps', 'aux']).decode()
killed = 0
for line in procs.splitlines():
    if 'grep' in line: continue
    cols = line.split()
    if int(cols[1]) in skip_pids: continue   # 不杀自己
    if any(k in line for k in [
        'ekf_node','safety_node','controller_server','planner_server',
        'behavior_server','bt_navigator','waypoint_follower',
        'velocity_smoother','lifecycle_manager','task_node',
        'origincar_base','ydlidar_ros2_driver','static_transform_publisher',
        'rviz2','zbar_ros','barcode_reader','hobot_usb_cam',
        'ros2 launch','ros2 daemon','start_rviz',
    ]):
        try: os.kill(int(cols[1]), 9); killed += 1
        except: pass
print(f'  killed={killed}')
" 2>/dev/null
sleep 2
LEFT=$(ps aux | grep -E "(ekf|safety|controller|planner|navigator|ydlidar|rviz|zbar|task_node|origincar)" | grep -v grep | wc -l)
echo "  残留进程: $LEFT"

# ---- 2. 构建 ----
banner "[2/6] 构建"
colcon build --symlink-install \
  --packages-select smartcar_nav2 smartcar_task \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --allow-overriding smartcar_nav2 smartcar_task 2>&1 | grep -E "(Finished|Error|warning)" | head -5
echo "  ✓ 构建完成"

# ---- 3. 启动 ----
banner "[3/6] 启动系统 (无视觉 纯导航)"
true > "$LOG"
ros2 launch smartcar_bringup smartcar_system.launch.py \
  use_base:=true \
  use_lidar:=true \
  use_obstacle:=false \
  use_laser_odometry:=false \
  use_safety:=true \
  use_nav:=true \
  nav_autostart:=true \
  use_camera:=false \
  use_vision:=false \
  use_task:=true \
  autostart_mission:=false \
  camera_driver:=usb \
  safety_emergency_stop_on_start:=false \
  waypoints_calibrated:=true \
  extrinsics_calibrated:=true \
  steering_calibrated:=true \
  emergency_stop_ready:=true \
  operator_approved:=true \
  waypoints_file:="$WAYPOINTS_FILE" \
  >> "$LOG" 2>&1 &
LAUNCH_PID=$!
echo "  Launch PID: $LAUNCH_PID"

# ---- 4. 等就绪 ----
banner "[4/6] 等待就绪"
READY=0
for i in $(seq 1 60); do
  if grep -q "Managed nodes are active" "$LOG" 2>/dev/null; then
    READY=1; break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "  ✗ Launch 异常退出"; tail -30 "$LOG"; exit 1
  fi
  printf "  ... %ds\r" $((i * 2))
  sleep 2
done
echo ""
[ "$READY" -eq 0 ] && { echo "  ✗ 超时"; tail -30 "$LOG"; exit 1; }
echo "  ✓ 全部 lifecycle active"

# ---- 5. RViz ----
banner "[5/6] RViz"
cat > /tmp/start_rviz.sh << 'RVEOF'
#!/bin/bash
source /root/source_env.sh
export DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0
exec rviz2 -d /root/ros2_ws/src/smartcar_tools/rviz/navigation.rviz
RVEOF
chmod +x /tmp/start_rviz.sh
# 杀掉旧 RViz
pkill -9 -f rviz2 2>/dev/null || true
sleep 1
nohup /tmp/start_rviz.sh > /tmp/rviz_out.log 2>&1 &
RViz_PID=$!
sleep 3
if kill -0 "$RViz_PID" 2>/dev/null || ps aux | grep -q "[r]viz2"; then
  echo "  ✓ RViz 已启动"
else
  echo "  ⚠ RViz 启动失败，见 /tmp/rviz_out.log"
fi

# ---- 6. 发车 ----
banner "[6/6] 发车"
ros2 service call /smartcar/task/reset std_srvs/srv/Trigger "{}" 2>&1 | grep -o "success=[^,]*"
ros2 service call /smartcar/safety/emergency_stop std_srvs/srv/SetBool "{data: false}" 2>&1 | grep -o "success=[^,]*"
ros2 service call /smartcar/task/start std_srvs/srv/Trigger "{}" 2>&1 | grep -o "success=[^,]*"

echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║  导航任务已启动 (纯导航模式)                 ║"
echo "║  监控: python3 /root/monitor_mission.py       ║"
echo "║  日志: tail -f /tmp/bringup.log               ║"
echo "╚══════════════════════════════════════════════╝"
