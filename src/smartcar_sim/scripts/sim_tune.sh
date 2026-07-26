#!/bin/bash
# ============================================================================
# sim_tune.sh —— 仿真调参循环脚本
# 用法：
#   bash sim_tune.sh              # 构建→启动仿真→跑路线→记录结果
#   bash sim_tune.sh --no-build   # 跳过构建（仅改 YAML 时）
#   bash sim_tune.sh --headless   # 无 GUI，纯数据收集
#   bash sim_tune.sh --loop N     # 循环 N 次，每次随机微调参数
# ============================================================================
set -e

WS="${HOME}/ros2_ws"
SRC="/mnt/d/StudyWorks/3.2/SmartCar/src"
SIM_PKG="smartcar_sim"
LOG_DIR="${WS}/tune_logs"
RUN_ID="run_$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/${RUN_ID}.log"
HEADLESS=false
NO_BUILD=false
LOOP_COUNT=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-build) NO_BUILD=true; shift ;;
        --headless) HEADLESS=true; shift ;;
        --loop) LOOP_COUNT="$2"; shift 2 ;;
        *) echo "Unknown: $1"; exit 1 ;;
    esac
done

mkdir -p "$LOG_DIR"

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG_FILE"; }

cleanup() {
    log "Cleaning up..."
    pkill -9 gz sim 2>/dev/null || true
    pkill -9 ros2 rviz2 2>/dev/null || true
    sleep 2
}

# ── Step 1: Clean ──
cleanup

# ── Step 2: Source ──
log "Sourcing ROS2..."
source /opt/ros/humble/setup.bash

# ── Step 3: Sync source ──
if [ ! -d "$WS/src/smartcar_sim" ]; then
    log "Syncing source to WSL workspace..."
    mkdir -p "$WS/src"
    rsync -a --exclude='build/' --exclude='install/' --exclude='log/' --exclude='.git/' \
        "$SRC/" "$WS/src/"
fi
source "${WS}/src/smartcar_sim/scripts/sim_env.sh"

# ── Step 4: Build ──
if [ "$NO_BUILD" = false ]; then
    log "Building (packages: smartcar_interfaces smartcar_safety smartcar_nav2 smartcar_task smartcar_sim)..."
    cd "$WS"
    colcon build --symlink-install \
        --packages-select smartcar_interfaces smartcar_safety smartcar_nav2 smartcar_task smartcar_sim \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo 2>&1 | tail -20 | tee -a "$LOG_FILE"
    log "Build complete"
fi

# ── Step 5: Source workspace ──
source "$WS/install/setup.bash"

# ── Step 6: Launch simulation ──
HEADLESS_ARG="false"
[ "$HEADLESS" = true ] && HEADLESS_ARG="true"

log "Launching simulation (headless=$HEADLESS_ARG)..."
ros2 launch smartcar_sim sim.launch.py \
    headless:="$HEADLESS_ARG" \
    use_rviz:="true" \
    2>&1 | tee -a "$LOG_FILE" &
SIM_PID=$!
log "Simulation PID: $SIM_PID"

# ── Step 7: Wait for Nav2 ready ──
log "Waiting for Nav2 lifecycle..."
TIMEOUT=120
ELAPSED=0
while [ $ELAPSED -lt $TIMEOUT ]; do
    if ros2 node list 2>/dev/null | grep -q "controller_server"; then
        log "Nav2 ready after ${ELAPSED}s"
        break
    fi
    sleep 2
    ELAPSED=$((ELAPSED + 2))
done

if [ $ELAPSED -ge $TIMEOUT ]; then
    log "ERROR: Nav2 not ready after ${TIMEOUT}s"
    kill $SIM_PID 2>/dev/null
    exit 1
fi

# ── Step 8: Collect key params snapshot ──
log "=== PARAMS SNAPSHOT ==="
PARAMS_FILE="$WS/src/smartcar_nav2/config/nav2_params.yaml"
for key in motion_model_for_search minimum_turning_radius yaw_goal_tolerance xy_goal_tolerance \
    inflation_radius lookahead_dist desired_linear_vel allow_reversing; do
    val=$(grep "$key" "$PARAMS_FILE" 2>/dev/null | head -1 | awk '{print $2}' || echo "N/A")
    log "  $key = $val"
done

# ── Step 9: Start mission ──
log "Starting mission..."
ros2 service call /smartcar/safety/emergency_stop std_srvs/srv/SetBool '{data: false}' 2>/dev/null || true
sleep 1
ros2 service call /smartcar/task/reset std_srvs/srv/Trigger 2>/dev/null || true
sleep 1
ros2 service call /smartcar/task/start std_srvs/srv/Trigger 2>/dev/null || true
log "Mission started"

# ── Step 10: Monitor progress ──
log "Monitoring mission (Ctrl+C to stop monitoring, sim continues)..."
python3 "$WS/src/smartcar_sim/scripts/monitor_mission.py" 2>&1 | tee -a "$LOG_FILE" || true

# ── Step 11: Report ──
log "=== RUN COMPLETE: $RUN_ID ==="
log "Log file: $LOG_FILE"
log "To relaunch: bash sim_tune.sh --no-build"

# Keep sim running for inspection
log "Simulation still running. Press Ctrl+C to stop."
wait $SIM_PID 2>/dev/null || true
