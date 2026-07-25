#!/bin/bash
# ============================================================
# SmartCar 安全启动脚本 (v1)
# 用法: bash /root/safe_start.sh
#
# 与 nav_test.sh 的区别:
#   - 系统启动时 emergency_stop 锁存为 ON (不解除)
#   - 不发车——用户手动释放急停并启动作业
#   - 包含场地参考层和航点可视化
#
# 适用场景:
#   - 首次部署或重新标定后确认 TF/odom/航点
#   - 场地变动后验证定位
#   - 测试新航点坐标
# ============================================================
set -o pipefail

SOURCE_ENV=/root/source_env.sh
WORKSPACE=/root/ros2_ws
LOG_DIR=/tmp/safe_start_logs
WAYPOINTS_FILE=/root/ros2_ws/src/smartcar_nav2/config/waypoints/nav_only.yaml
FIELD_GEOMETRY=/root/ros2_ws/src/smartcar_tools/config/routes/field_geometry.yaml
NAV_PARAMS=/root/ros2_ws/src/smartcar_nav2/config/nav2_params_fixed.yaml

source "$SOURCE_ENV"
cd "$WORKSPACE"
mkdir -p "$LOG_DIR"

TIMESTAMP() { date '+%Y-%m-%d %H:%M:%S'; }
banner()   { echo ""; echo "=== [$(TIMESTAMP)] $* ==="; }
log()      { echo "[$(TIMESTAMP)] $*" | tee -a "$LOG_DIR/main.log"; }
warn()     { echo "[$(TIMESTAMP)] WARN: $*" | tee -a "$LOG_DIR/main.log" >&2; }
die()      { echo "[$(TIMESTAMP)] FATAL: $*" | tee -a "$LOG_DIR/main.log" >&2; exit 1; }
check_proc() { kill -0 "$1" 2>/dev/null; }

# ── 1. 清理 ──────────────────────────────────────────────────
banner "[1/8] 清理环境"
bash /usr/local/bin/ros_cleanup 2>&1 | tee "$LOG_DIR/cleanup.log" || true
sleep 2

# ── 2. 构建 ──────────────────────────────────────────────────
banner "[2/8] 构建"
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
echo "  OK 配置文件校验通过"

# ── 3. 启动主系统 (急停锁存 ON) ──────────────────────────────
banner "[3/8] 安全启动主系统"
echo "  ! 急停已锁存——车辆不会移动"
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
  safety_emergency_stop_on_start:=true \
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
banner "[4/8] 等待 lifecycle 节点激活"

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
      die "Launch 进程异常退出 (PID $LAUNCH_PID)"

    ALL_ACTIVE=true
    NODE_STATUS=""
    for node in "${EXPECTED_NODES[@]}"; do
        state=$(ros2 lifecycle get "/$node" 2>/dev/null | grep -oP 'State \(\K[0-9]+' || echo "?")
        [ "$state" != "3" ] && ALL_ACTIVE=false
        NODE_STATUS="$NODE_STATUS  /$node=$state"
    done

    if $ALL_ACTIVE; then
        echo "  OK 全部 lifecycle 节点已激活 (${i}s):$NODE_STATUS"
        READY=1
        break
    fi

    printf "  [%2d/90] 等待中...%s\r" "$i" "$NODE_STATUS"
    sleep 1
done
echo ""

if [ "$READY" -eq 0 ]; then
    grep -q "Managed nodes are active" "$LOG_DIR/bringup.log" 2>/dev/null && {
        log "后备: lifecycle_manager 报告全部 active"
        READY=1
    }
fi

[ "$READY" -eq 0 ] && die "lifecycle 节点等待超时 (90s)"

# ── 5. 预检 ──────────────────────────────────────────────────
banner "[5/8] 发车前预检"
PREFLIGHT_OK=true
ISSUES=""

echo -n "  /odom_combined ... "
timeout 5 ros2 topic echo /odom_combined --once &>/dev/null && echo "OK" || {
    echo "FAIL"; PREFLIGHT_OK=false
    ISSUES="$ISSUES  - /odom_combined 无数据\n"
}

echo -n "  /scan ... "
timeout 5 ros2 topic echo /scan --once &>/dev/null && echo "OK" || {
    echo "FAIL"; PREFLIGHT_OK=false
    ISSUES="$ISSUES  - /scan 无数据\n"
}

echo -n "  TF odom_combined->base_footprint ... "
timeout 3 ros2 topic echo /tf --once 2>/dev/null | grep -q "odom_combined" && echo "OK" || {
    echo "FAIL"; PREFLIGHT_OK=false
    ISSUES="$ISSUES  - odom_combined->base_footprint TF 缺失\n"
}

echo -n "  TF static ... "
timeout 3 ros2 topic echo /tf_static --once &>/dev/null && echo "OK" || echo "WARN (可能不影响)"

if ! $PREFLIGHT_OK; then
    echo ""
    echo "  ! 预检发现问题:"
    printf "%b" "$ISSUES"
    echo "  继续启动可视化, 但请勿解除急停直到问题解决"
else
    echo "  OK 预检全部通过"
fi

# ── 6. 启动 field_reference + waypoint_viz ───────────────────
banner "[6/8] 启动可视化叠加层"

echo -n "  启动 field_reference_node ... "
ros2 run smartcar_tools field_reference_node --ros-args \
  -p geometry_file:="$FIELD_GEOMETRY" \
  >> "$LOG_DIR/field_ref.log" 2>&1 &
FIELD_REF_PID=$!
sleep 1
check_proc "$FIELD_REF_PID" && echo "OK (PID $FIELD_REF_PID)" || warn "启动失败"

echo -n "  启动 waypoint_viz ... "
ros2 run smartcar_tools waypoint_viz --ros-args \
  -p waypoints_file:="$WAYPOINTS_FILE" \
  >> "$LOG_DIR/waypoint_viz.log" 2>&1 &
VIZ_PID=$!
sleep 1
check_proc "$VIZ_PID" && echo "OK (PID $VIZ_PID)" || warn "启动失败"

# ── 7. RViz ──────────────────────────────────────────────────
banner "[7/8] 启动 RViz"
cat > /tmp/start_rviz.sh << 'RVEOF'
#!/bin/bash
source /root/source_env.sh
export DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0
exec rviz2 -d /root/ros2_ws/src/smartcar_tools/rviz/navigation.rviz
RVEOF
chmod +x /tmp/start_rviz.sh

OLD_RVIZ=$(ps -C rviz2 -o pid= --no-headers 2>/dev/null || true)
[ -n "$OLD_RVIZ" ] && kill -9 $OLD_RVIZ 2>/dev/null || true
sleep 1
nohup /tmp/start_rviz.sh > "$LOG_DIR/rviz.log" 2>&1 &
RVIZ_PID=$!
sleep 4
check_proc "$RVIZ_PID" || ps aux | grep -q "[r]viz2" \
  && echo "  OK RViz 已启动" \
  || { warn "RViz 启动可能失败"; echo "  检查: export DISPLAY=:0; echo \$XAUTHORITY"; }

# ── 8. 稳定等待 + 用户检查清单 ──────────────────────────────
banner "[8/8] 稳定等待"

STABILITY_DELAY=8
echo "  等待 ${STABILITY_DELAY}s 稳定 (代价地图填充+传感器同步) ..."
for s in $(seq $STABILITY_DELAY -1 1); do
    printf "  %ds ...\r" "$s"
    sleep 1
done
echo "  稳定期结束"

YAW_TOL=$(ros2 param get /controller_server yaw_goal_tolerance 2>/dev/null | grep -oP '[0-9.]+' | head -1 || echo "?")

echo ""
echo "+============================================================+"
echo "|         系统已安全启动 -- 急停锁存 (车辆不动)               |"
echo "+============================================================+"
echo "|                                                            |"
echo "|  RViz 检查清单:                                             |"
echo "|  +----------------------------------------------------+    |"
echo "|  | [] robot 位置在 P 点原点附近 (odom_combined)       |    |"
echo "|  | [] 场地参考层 (ABC区/走廊/C环) 与真实场地对齐      |    |"
echo "|  | [] 航点标记 (彩色球+箭头) 位于合理位置             |    |"
echo "|  | [] LiDAR /scan 点云与场地边界一致                  |    |"
echo "|  | [] 无异常 TF 跳变或 odom 漂移                      |    |"
echo "|  +----------------------------------------------------+    |"
echo "|                                                            |"
echo "|  参数: yaw_goal_tolerance = $YAW_TOL rad"
echo "|                                                            |"
echo "|  日志: $LOG_DIR"
echo "|  主日志: tail -f $LOG_DIR/bringup.log"
echo "|                                                            |"
echo "|  确认无误后, 手动发车:                                      |"
echo "|  +----------------------------------------------------+    |"
echo "|  | # 解除急停                                           |    |"
echo '|  | ros2 service call /smartcar/safety/emergency_stop   |    |'
echo '|  |   std_srvs/srv/SetBool "{data: false}"              |    |'
echo "|  |                                                      |    |"
echo "|  | # 重置并启动任务                                      |    |"
echo '|  | ros2 service call /smartcar/task/reset \            |    |'
echo '|  |   std_srvs/srv/Trigger "{}"                         |    |'
echo '|  | ros2 service call /smartcar/task/start \            |    |'
echo '|  |   std_srvs/srv/Trigger "{}"                         |    |'
echo "|  +----------------------------------------------------+    |"
echo "|                                                            |"
echo "|  紧急停车:                                                  |"
echo '|    ros2 service call /smartcar/safety/emergency_stop \    |'
echo '|      std_srvs/srv/SetBool "{data: true}"                  |'
echo '|  或: bash /usr/local/bin/ros_cleanup                        |'
echo "+============================================================+"
