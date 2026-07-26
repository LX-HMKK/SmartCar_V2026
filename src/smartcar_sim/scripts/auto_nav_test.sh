#!/bin/bash
# auto_nav_test.sh — 全自动导航测试 + 兜圈子检测
set -e
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "${SCRIPT_DIR}/sim_env.sh"
export DISPLAY=:0
export IGN_GAZEBO_RESOURCE_PATH=/root/ros2_ws/install/smartcar_sim/share/smartcar_sim/models:$IGN_GAZEBO_RESOURCE_PATH

LOG=/tmp/auto_nav.log
rm -f $LOG

echo "=== AUTO NAV TEST $(date) ===" | tee $LOG

# Kill stale
pkill -9 ign ros2 2>/dev/null || true
sleep 2

# Start sim
echo "[1] Starting sim..." | tee -a $LOG
ros2 launch smartcar_sim sim.launch.py headless:=true use_rviz:=false > /tmp/sim_output.log 2>&1 &
SIM_PID=$!

# Wait for Nav2 to be configured (check log, not ros2 CLI)
echo "[2] Waiting for Nav2 configure..." | tee -a $LOG
for i in $(seq 1 40); do
    sleep 3
    if grep -q "Controller Server has" /tmp/sim_output.log 2>/dev/null; then
        echo "  Nav2 configured after $((i*3))s" | tee -a $LOG
        break
    fi
    [ $i -eq 40 ] && { echo "  TIMEOUT - Nav2 never configured" | tee -a $LOG; cat /tmp/sim_output.log | tail -20 >> $LOG; kill $SIM_PID; exit 1; }
done

# Extra wait for activation
sleep 10

# Verify EKF is publishing by checking for odom_combined
echo "[3] Checking odom_combined..." | tee -a $LOG
ODE=$(timeout 10 ros2 topic echo /odom_combined --once 2>/dev/null | head -3 || echo "NO_DATA")
echo "  $ODE" | tee -a $LOG
if echo "$ODE" | grep -q "NO_DATA"; then
    echo "  WARNING: no odom_combined, trying anyway..." | tee -a $LOG
fi

# Send goal to QR waypoint
echo "[4] Sending goal to QR (forward)..." | tee -a $LOG
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom_combined}, pose: {position: {x: 3.127, y: 0.977, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.2588, w: 0.9659}}}}" \
  --feedback 2>&1 > /tmp/goal_result.txt &
GOAL_PID=$!

# Monitor position for 45 seconds - detect circling
echo "[5] Monitoring 45s..." | tee -a $LOG
LAST_X=0; LAST_Y=0; MAX_DIST=0; CIRCLING=0; DIR_CHANGES=0; LAST_DIR=""
for i in $(seq 1 45); do
    sleep 1
    POS=$(timeout 2 ros2 topic echo /odom_combined --once 2>/dev/null | grep "position:" -A2 | grep -E "x:|y:" | tr -d " xy:" | tr "\n" "," | sed 's/,$//' || echo "?,?")
    X=$(echo $POS | cut -d, -f1)
    Y=$(echo $POS | cut -d, -f2)
    if [ "$X" != "?" ] 2>/dev/null; then
        DIST=$(echo "sqrt(($X - 0)^2 + ($Y - 0)^2)" | bc 2>/dev/null || echo "0")
        DX=$(echo "$X - $LAST_X" | bc 2>/dev/null || echo "0")
        DY=$(echo "$Y - $LAST_Y" | bc 2>/dev/null || echo "0")
        # Detect direction reversal (sign of progress toward goal)
        PROGRESS=$(echo "$DX * (3.127 - $LAST_X) + $DY * (0.977 - $LAST_Y)" | bc 2>/dev/null || echo "0")
        if [ "$PROGRESS" != "0" ] 2>/dev/null; then
            if [ "$PROGRESS" != "$LAST_DIR" ] 2>/dev/null; then
                DIR_CHANGES=$((DIR_CHANGES + 1))
                echo "  t=${i}s dir_change #${DIR_CHANGES} pos=($X,$Y)" | tee -a $LOG
            fi
            LAST_DIR=$PROGRESS
        fi
        MAX_DIST=$DIST
        LAST_X=$X; LAST_Y=$Y
    fi
done

echo "[6] Results:" | tee -a $LOG
echo "  Max distance from origin: $MAX_DIST m" | tee -a $LOG
echo "  Direction changes: $DIR_CHANGES" | tee -a $LOG
cat /tmp/goal_result.txt 2>/dev/null | head -10 | tee -a $LOG

if [ $MAX_DIST != "0" ] && [ $(echo "$MAX_DIST < 1.5" | bc 2>/dev/null || echo 0) -eq 1 ] && [ $DIR_CHANGES -gt 3 ]; then
    echo "*** CIRCLING DETECTED: robot not progressing, many direction changes ***" | tee -a $LOG
elif [ "$MAX_DIST" != "0" ] && [ $(echo "$MAX_DIST > 2.5" | bc 2>/dev/null || echo 0) -eq 1 ]; then
    echo "*** OK: robot making progress toward goal ***" | tee -a $LOG
else
    echo "*** UNCLEAR: robot may be stuck or odom not flowing ***" | tee -a $LOG
fi

kill $GOAL_PID $SIM_PID 2>/dev/null || true
pkill -9 ign ros2 2>/dev/null || true
echo "=== DONE ===" | tee -a $LOG
cat $LOG
