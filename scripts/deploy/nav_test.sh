#!/bin/bash
# ============================================================
# SmartCar 纯导航全流程测试 (v2)
# 用法: bash /root/nav_test.sh
#
# 改进点 vs v1:
#   - 自动启动 field_reference_node (场地参考叠加层)
#   - 自动启动 waypoint_viz (航点 MarkerArray 可视化)
#   - 分离日志: /tmp/nav_test_logs/{bringup,field_ref,viz,rviz,health}.log
#   - 等待所有 lifecycle 节点激活（逐个验证，不只等一条日志）
#   - 发车前验证 yaw_goal_tolerance 参数
#   - 发车前执行 TF/odom/scan 预检
#   - lifecycle 就绪后额外 8s 稳定延迟
# ============================================================
set -o pipefail

SOURCE_ENV=/root/source_env.sh
WORKSPACE=/root/ros2_ws
LOG_DIR=/tmp/nav_test_logs
WAYPOINTS_FILE=/root/ros2_ws/src/smartcar_nav2/config/waypoints/nav_only.yaml
FIELD_GEOMETRY=/root/ros2_ws/src/smartcar_tools/config/routes/field_geometry.yaml
NAV_PARAMS=/root/ros2_ws/src/smartcar_nav2/config/nav2_params_fixed.yaml

source "$SOURCE_ENV"
cd "$WORKSPACE"
mkdir -p "$LOG_DIR"

TIMESTAMP() { date '+%Y-%m-%d %H:%M:%S'; }
banner()   { echo ""; echo "=== [$TIMESTAMP] $* ==="; }
log()      { echo "[$TIMESTAMP] $*" | tee -a "$LOG_DIR/main.log"; }
warn()     { echo "[$TIMESTAMP] WARN: $*" | tee -a "$LOG_DIR/main.log" >&2; }
die()      { echo "[$TIMESTAMP] FATAL: $*" | tee -a "$LOG_DIR/main.log" >&2; exit 1; }
check_proc() { kill -0 "$1" 2>/dev/null; }

# ── 1. 清理 ──────────────────────────────────────────────────
banner "[1/9] 清理环境"
bash /usr/local/bin/ros_cleanup 2>&1 | tee "$LOG_DIR/cleanup.log" || true
sleep 2

# ── 2. 构建 ──────────────────────────────────────────────────
banner "[2/9] 构建"
colcon build --symlink-install \
  --packages-select smartcar_nav2 smartcar_task \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --allow-overriding smartcar_nav2 smartcar_task \
  2>&1 | tee "$LOG_DIR/build.log"
grep -q "Finished" "$LOG_DIR/build.log" 2>/dev/null || \
  warn "构建可能未成功完成，检查 $LOG_DIR/build.log"

for f in "$WAYPOINTS_FILE" "$NAV_PARAMS"; do
    [ -f "$f" ] || die "缺少配置文件: $f"
done
echo "  ✓ 配置文件校验通过"

# ── 3. 启动主系统 ────────────────────────────────────────────
banner "[3/9] 启动主系统 (无视觉 纯导航)"
true > "$LOG_DIR/bringup.log"

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
  >> "$LOG_DIR/bringup.log" 2>&1 &
LAUNCH_PID=$!
echo "  Launch PID: $LAUNCH_PID"

# ── 4. 等待 lifecycle 节点激活 ───────────────────────────────
banner "[4/9] 等待 lifecycle 节点激活"

EXPECTED_NODES=(
    "controller_server"
    "planner_server"
    "behavior_server"
    "bt_navigator"
    "waypoint_follower"
)

READY=0
for i in $(seq 1 90); do
    check_proc "$LAUNCH_PID" || \
      die "Launch 进程异常退出 (PID $LAUNCH_PID)，最后日志: $(tail -5 "$LOG_DIR/bringup.log")"

    ALL_ACTIVE=true
    NODE_STATUS=""
    for node in "${EXPECTED_NODES[@]}"; do
        state=$(ros2 lifecycle get "/$node" 2>/dev/null | grep -oP 'State \(\K[0-9]+' || echo "?")
        [ "$state" != "3" ] && ALL_ACTIVE=false
        NODE_STATUS="$NODE_STATUS  /$node=$state"
    done

    if $ALL_ACTIVE; then
        echo "  ✓ 全部 lifecycle 节点已激活 (${i}s):$NODE_STATUS"
        READY=1
        break
    fi

    printf "  [%2d/90] 等待中...%s\r" "$i" "$NODE_STATUS"
    sleep 1
done
echo ""

# 后备: lifecycle_manager 日志模式
if [ "$READY" -eq 0 ]; then
    if grep -q "Managed nodes are active" "$LOG_DIR/bringup.log" 2>/dev/null; then
        log "后备: lifecycle_manager 报告全部 active"
        READY=1
    fi
fi

[ "$READY" -eq 0 ] && die "lifecycle 节点等待超时 (90s)"

# ── 5. 预检 ──────────────────────────────────────────────────
banner "[5/9] 发车前预检"
PREFLIGHT_OK=true

echo -n "  /odom_combined ... "
timeout 5 ros2 topic echo /odom_combined --once &>/dev/null && echo "OK" || { echo "FAIL"; PREFLIGHT_OK=false; }

echo -n "  /scan ... "
timeout 5 ros2 topic echo /scan --once &>/dev/null && echo "OK" || { echo "FAIL"; PREFLIGHT_OK=false; }

echo -n "  TF odom_combined->base_footprint ... "
if timeout 3 ros2 topic echo /tf --once 2>/dev/null | grep -q "odom_combined"; then
    echo "OK"
else
    echo "FAIL"
    PREFLIGHT_OK=false
fi

echo -n "  TF static ... "
timeout 3 ros2 topic echo /tf_static --once &>/dev/null && echo "OK" || echo "WARN (可能不影响)"

$PREFLIGHT_OK && echo "  ✓ 预检全部通过" || warn "预检有失败项，继续但请注意"

# ── 6. 启动 field_reference + waypoint_viz ───────────────────
banner "[6/9] 启动可视化叠加层"

echo -n "  启动 field_reference_node ... "
ros2 run smartcar_tools field_reference_node --ros-args \
  -p geometry_file:="$FIELD_GEOMETRY" \
  >> "$LOG_DIR/field_ref.log" 2>&1 &
FIELD_REF_PID=$!
sleep 1
check_proc "$FIELD_REF_PID" && echo "OK (PID $FIELD_REF_PID)" || warn "field_reference_node 启动失败"

echo -n "  启动 waypoint_viz ... "
ros2 run smartcar_tools waypoint_viz --ros-args \
  -p waypoints_file:="$WAYPOINTS_FILE" \
  >> "$LOG_DIR/waypoint_viz.log" 2>&1 &
VIZ_PID=$!
sleep 1
check_proc "$VIZ_PID" && echo "OK (PID $VIZ_PID)" || warn "waypoint_viz 启动失败"

# ── 7. RViz ──────────────────────────────────────────────────
banner "[7/9] 启动 RViz"
cat > /tmp/start_rviz.sh << 'RVEOF'
#!/bin/bash
source /root/source_env.sh
export DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0
exec rviz2 -d /root/ros2_ws/src/smartcar_tools/rviz/navigation.rviz
RVEOF
chmod +x /tmp/start_rviz.sh

pkill -9 -f rviz2 2>/dev/null || true
sleep 1
nohup /tmp/start_rviz.sh > "$LOG_DIR/rviz.log" 2>&1 &
RVIZ_PID=$!
sleep 4
check_proc "$RVIZ_PID" || ps aux | grep -q "[r]viz2" \
  && echo "  ✓ RViz 已启动" \
  || warn "RViz 启动可能失败, 检查 DISPLAY"

# ── 8. 参数验证 + 稳定等待 ───────────────────────────────────
banner "[8/9] 参数验证与稳定等待"

echo -n "  yaw_goal_tolerance ... "
YAW_TOL=$(ros2 param get /controller_server yaw_goal_tolerance 2>/dev/null | grep -oP '[0-9.]+' | head -1 || echo "?")
if [ -n "$YAW_TOL" ] && [ "$YAW_TOL" != "?" ]; then
    echo "$YAW_TOL rad"
    awk "BEGIN {exit !($YAW_TOL > 0.5)}" 2>/dev/null && \
      warn "yaw_goal_tolerance=$YAW_TOL 偏大 (>0.5)，可能兜圈"
else
    warn "无法读取 yaw_goal_tolerance"
fi

STABILITY_DELAY=8
echo "  等待 ${STABILITY_DELAY}s 稳定 (代价地图填充+传感器同步) ..."
for s in $(seq $STABILITY_DELAY -1 1); do
    printf "  %ds ...\r" "$s"
    sleep 1
done
echo "  稳定期结束"

# ── 9. 发车 ──────────────────────────────────────────────────
banner "[9/9] 发车"

echo -n "  reset 任务状态 ... "
ros2 service call /smartcar/task/reset std_srvs/srv/Trigger "{}" 2>&1 | grep -o "success=[^,]*" || echo "?"

echo -n "  release emergency_stop ... "
ros2 service call /smartcar/safety/emergency_stop std_srvs/srv/SetBool "{data: false}" 2>&1 | grep -o "success=[^,]*" || echo "?"

echo -n "  start 任务 ... "
ros2 service call /smartcar/task/start std_srvs/srv/Trigger "{}" 2>&1 | grep -o "success=[^,]*" || echo "?"

echo ""
echo "╔════════════════════════════════════════════════╗"
echo "║  导航任务已启动 (纯导航, 含场地参考+航点可视化)  ║"
echo "║                                                ║"
echo "║  日志目录: $LOG_DIR                             ║"
echo "║  主日志:   tail -f $LOG_DIR/bringup.log         ║"
echo "║  监控:     python3 /root/monitor_mission.py      ║"
echo "║  紧急停车:                                       ║"
echo "║    ros2 service call /smartcar/safety/           ║"
echo "║      emergency_stop std_srvs/srv/SetBool         ║"
echo '║      "{data: true}"                              ║'
echo '║  或: pkill -9 -f "ros2 launch"                    ║'
echo "╚════════════════════════════════════════════════╝"
