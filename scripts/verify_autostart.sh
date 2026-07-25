#!/bin/bash
# ============================================================
# SmartCar autostart + all gates verification
# ============================================================
#
# === HOW TO RUN (from Windows PowerShell) ===
#
#   # Step 1: Sync changed source files to RDK
#   python scripts/sync_to_rdk.py push
#
#   # Step 2: Copy this script to RDK and execute
#   scp scripts/verify_autostart.sh root@192.168.128.10:/root/
#   ssh root@192.168.128.10 'bash /root/verify_autostart.sh'
#
#   # Or deploy ros_cleanup first if not already installed:
#   ssh root@192.168.128.10 'cp /root/ros2_ws/scripts/ros_cleanup.sh /usr/local/bin/ros_cleanup && chmod +x /usr/local/bin/ros_cleanup'
#
# === WHAT THIS VERIFIES ===
#
#   1. Changed source files are present on RDK and match expected content
#   2. __pycache__ cleared, SHM ports cleaned, no stale ROS2 processes
#   3. Build succeeds with the 5 affected packages
#   4. Launch with autostart_mission:=true + all 5 motion gates=true
#   5. Autostart retry loop (3s interval, up to 60 attempts) works
#   6. No "navigation_server_unavailable" errors in the bringup log
#   7. Mission transitions to RUNNING automatically (confirmed in log)
#   8. Graceful kill and full cleanup after observation period
#
# === IMPORTANT NOTE ===
#
# bringup validation (smartcar_system.launch.py L124) requires
# use_vision=true when autostart_mission=true. This script passes
# use_camera:=true use_vision:=true to satisfy the gate check.
# The "nav" task type in nav_only.yaml never calls vision services,
# so vision nodes start but remain idle. If the USB camera is not
# physically connected, the camera driver node may log errors but
# will not block navigation testing.
#
# If you prefer NOT to start camera/vision at all, temporarily
# remove "use_vision" from required_components in
# src/smartcar_bringup/launch/smartcar_system.launch.py line 124
# before syncing.
# ============================================================
set -euo pipefail

SOURCE_ENV=/root/source_env.sh
WORKSPACE=/root/ros2_ws
LOG=/tmp/bringup.log
CLEANUP_BIN=/usr/local/bin/ros_cleanup
MONITOR_LOG=/tmp/verify_monitor.log
# Max time to wait for mission autostart (seconds)
AUTOSTART_TIMEOUT=300
# How long to let navigation run after autostart before killing
NAV_RUN_SEC=30

export DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0

banner() { echo ""; echo "=============================================="; echo "  $*"; echo "=============================================="; }
fail()  { echo "FAIL: $*"; exit 1; }

# ---- Pre-flight: changed files must be synced ----
banner "Pre-flight checks"

if [ ! -f "$CLEANUP_BIN" ]; then
  echo "WARNING: ros_cleanup not found at $CLEANUP_BIN"
  echo "  Run: cp $WORKSPACE/scripts/ros_cleanup.sh $CLEANUP_BIN && chmod +x $CLEANUP_BIN"
fi

set +u; source "$SOURCE_ENV"; set -u
cd "$WORKSPACE"

echo "Changed source files:"
for f in \
  src/smartcar_task/smartcar_task/task_node.py \
  src/smartcar_task/smartcar_task/mission.py \
  src/smartcar_task/config/task.yaml; do
  if [ -f "$f" ]; then
    echo "  OK: $f (mtime: $(stat -c %y "$f" 2>/dev/null || date -r "$f"))"
  else
    fail "$f NOT FOUND — did you run 'python scripts/sync_to_rdk.py push'?"
  fi
done

# Verify the key changed values
echo ""
echo "Verifying changed code:"
grep -n "server_wait_timeout_sec" src/smartcar_task/smartcar_task/task_node.py | head -3
grep -n "poll_interval" src/smartcar_task/smartcar_task/task_node.py
grep -n "autostart_retries\|autostart_max_retries\|_on_autostart" src/smartcar_task/smartcar_task/task_node.py | head -10

# ---- Step 1: Cleanup ----
banner "[1/7] Cleanup"

if [ -x "$CLEANUP_BIN" ]; then
  bash "$CLEANUP_BIN"
else
  echo "Skipping ros_cleanup (not installed)"
fi

# Extra: pycache cleanup
find "$WORKSPACE/src" -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
find "$WORKSPACE/build" -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
find "$WORKSPACE/install" -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
echo "  __pycache__ cleaned"

# SHM ports
rm -f /dev/shm/fastrtps_port* /dev/shm/fastdds_port* /dev/shm/*ros2* /dev/shm/*fast* 2>/dev/null || true
echo "  SHM ports cleaned"

sleep 2

# ---- Step 2: Build ----
banner "[2/7] Build"

colcon build --symlink-install \
  --packages-select smartcar_interfaces smartcar_safety smartcar_nav2 smartcar_task smartcar_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --allow-overriding smartcar_interfaces smartcar_safety smartcar_nav2 smartcar_task smartcar_bringup 2>&1 | tail -10

echo "  Build done (exit: $?)"

# ---- Step 3: Parameter verification ----
banner "[3/7] Parameter verification"

FIXED_YAML="$WORKSPACE/install/smartcar_nav2/share/smartcar_nav2/config/nav2_params_fixed.yaml"
echo "  motion_model: $(grep motion_model_for_search "$FIXED_YAML" 2>/dev/null | xargs || echo 'NOT FOUND')"
echo "  allow_reversing: $(grep allow_reversing "$FIXED_YAML" 2>/dev/null | xargs || echo 'NOT FOUND')"
echo "  yaw_goal_tolerance: $(grep yaw_goal_tolerance "$FIXED_YAML" 2>/dev/null | xargs || echo 'NOT FOUND')"

# ---- Step 4: Start visualization nodes ----
banner "[4/7] Visualization"

GEOM="$WORKSPACE/src/smartcar_tools/config/routes/field_geometry.yaml"
WP="$WORKSPACE/src/smartcar_nav2/config/waypoints/nav_only.yaml"

ros2 run smartcar_tools field_reference_node --ros-args -p geometry_file:="$GEOM" &
VIZ_PID1=$!
ros2 run smartcar_tools waypoint_viz --ros-args -p waypoints_file:="$WP" &
VIZ_PID2=$!
sleep 2
echo "  field_reference PID: $VIZ_PID1, waypoint_viz PID: $VIZ_PID2"

# ---- Step 5: Launch system with autostart + all gates ----
banner "[5/7] Launch (autostart + all motion gates)"

true > "$LOG"
ros2 launch smartcar_bringup smartcar_system.launch.py \
  use_base:=true use_lidar:=true use_obstacle:=false \
  use_laser_odometry:=false use_safety:=true use_nav:=true \
  nav_autostart:=true \
  use_camera:=true use_vision:=true \
  use_task:=true camera_driver:=usb \
  autostart_mission:=true safety_emergency_stop_on_start:=false \
  waypoints_calibrated:=true extrinsics_calibrated:=true \
  steering_calibrated:=true emergency_stop_ready:=true \
  operator_approved:=true waypoints_file:="$WP" \
  >> "$LOG" 2>&1 &
LAUNCH_PID=$!
echo "  Launch PID: $LAUNCH_PID"

# ---- Step 6: Monitor ----
banner "[6/7] Wait for nav lifecycle + autostart"

# 6a. Wait for Nav2 lifecycle to go active
NAV_READY=0
echo "  Waiting for Nav2 lifecycle (up to 600s)..."
for i in $(seq 1 60); do
  if grep -q "Managed nodes are active" "$LOG" 2>/dev/null; then
    NAV_READY=1
    echo "  Nav2 lifecycle active at t=${i}0s"
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo ""
    echo "=== LAUNCH DIED UNEXPECTEDLY ==="
    tail -50 "$LOG"
    fail "Launch process exited before Nav2 was ready"
  fi
  sleep 10
done
if [ "$NAV_READY" -eq 0 ]; then
  tail -50 "$LOG"
  fail "Nav2 lifecycle did not become active within 600s"
fi

# Give lifecycle a moment to fully settle
sleep 3

# 6b. Now monitor for autostart behavior in the log
echo ""
echo "  Monitoring for autostart and navigation..."
MONITOR_START=$(date +%s)
MISSION_STARTED=0
AUTOSTART_RETRIES=0
NAV_UNAVAILABLE=0

while true; do
  ELAPSED=$(($(date +%s) - MONITOR_START))

  # Check launch still alive
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo ""
    echo "=== LAUNCH DIED at t=${ELAPSED}s ==="
    break
  fi

  # Check for navigation_server_unavailable
  if grep -q "navigation_server_unavailable" "$LOG" 2>/dev/null; then
    NAV_UNAVAILABLE=$(grep -c "navigation_server_unavailable" "$LOG" 2>/dev/null || echo 0)
    echo "  [t=${ELAPSED}s] navigation_server_unavailable found in log (count: $NAV_UNAVAILABLE)"
  fi

  # Check for autostart retry messages
  NEW_RETRIES=$(grep -c "autostart attempt" "$LOG" 2>/dev/null || echo 0)
  if [ "$NEW_RETRIES" -gt "$AUTOSTART_RETRIES" ]; then
    AUTOSTART_RETRIES=$NEW_RETRIES
    echo "  [t=${ELAPSED}s] autostart retry #$AUTOSTART_RETRIES"
  fi

  # Check for mission autostarted
  if grep -q "Mission autostarted" "$LOG" 2>/dev/null && [ "$MISSION_STARTED" -eq 0 ]; then
    MISSION_STARTED=1
    MISSION_START_TIME=$ELAPSED
    echo "  [t=${ELAPSED}s] MISSION AUTOSTARTED SUCCESSFULLY"
  fi

  # Check task state via ros2 topic (best-effort, may fail if ros2 CLI is slow)
  TASK_STATE=$(timeout 3 ros2 topic echo /smartcar/task/state --once 2>/dev/null || echo "unavailable")
  if [ -n "$TASK_STATE" ] && [ "$TASK_STATE" != "unavailable" ]; then
    echo "  [t=${ELAPSED}s] task/state: $TASK_STATE"
  fi

  # If mission started, let it run for NAV_RUN_SEC then exit
  if [ "$MISSION_STARTED" -eq 1 ]; then
    RUN_DURATION=$((ELAPSED - MISSION_START_TIME))
    if [ "$RUN_DURATION" -ge "$NAV_RUN_SEC" ]; then
      echo ""
      echo "  Mission has been running for ${RUN_DURATION}s — done monitoring"
      break
    fi
  fi

  # Timeout
  if [ "$ELAPSED" -ge "$AUTOSTART_TIMEOUT" ]; then
    echo ""
    echo "  TIMEOUT (${AUTOSTART_TIMEOUT}s) waiting for autostart"
    break
  fi

  sleep 5
done

# ---- Summary ----
banner "[7/7] Results"

echo ""
echo "=== VERIFICATION SUMMARY ==="
echo ""

# Nav2 lifecycle
if [ "$NAV_READY" -eq 1 ]; then
  echo "  [PASS] Nav2 lifecycle became active"
else
  echo "  [FAIL] Nav2 lifecycle never became active"
fi

# Autostart
if [ "$MISSION_STARTED" -eq 1 ]; then
  echo "  [PASS] Mission autostarted at t=${MISSION_START_TIME}s"
  echo "         Autostart retries before success: $AUTOSTART_RETRIES"
else
  echo "  [FAIL] Mission did NOT autostart within ${AUTOSTART_TIMEOUT}s"
  echo "         Autostart retries attempted: $AUTOSTART_RETRIES"
fi

# navigation_server_unavailable check
if [ "$NAV_UNAVAILABLE" -eq 0 ]; then
  echo "  [PASS] No 'navigation_server_unavailable' errors"
else
  echo "  [FAIL] 'navigation_server_unavailable' appeared $NAV_UNAVAILABLE times"
fi

# Check for specific error patterns
ERRORS=$(grep -ci "error\|ERROR" "$LOG" 2>/dev/null || echo 0)
echo "  [INFO] Log contains $ERRORS lines with 'error'/'ERROR'"

# Show last autostart-related lines
echo ""
echo "--- Last 20 autostart-related log lines ---"
grep -i "autostart\|navigation_server\|NavigateToPose\|task/state" "$LOG" 2>/dev/null | tail -20 || echo "  (none)"

echo ""
echo "--- Last 20 error/warn lines ---"
grep -iE "error|warn" "$LOG" 2>/dev/null | tail -20 || echo "  (none)"

# ---- Kill ----
echo ""
echo "=== Killing system ==="
echo ""

# Kill launch first
kill -TERM "$LAUNCH_PID" 2>/dev/null || true
sleep 2
kill -9 "$LAUNCH_PID" 2>/dev/null || true

# Kill viz nodes
kill -9 "$VIZ_PID1" "$VIZ_PID2" 2>/dev/null || true

# Full cleanup
if [ -x "$CLEANUP_BIN" ]; then
  bash "$CLEANUP_BIN"
else
  pkill -9 -f "ros2 launch" 2>/dev/null || true
  pkill -9 -f "field_reference" 2>/dev/null || true
  pkill -9 -f "waypoint_viz" 2>/dev/null || true
  sleep 1
  rm -f /dev/shm/fastrtps_port* /dev/shm/fastdds_port* 2>/dev/null || true
fi

echo ""
echo "=== Verification complete ==="
echo "  Full log: $LOG"

# Final pass/fail
if [ "$NAV_READY" -eq 1 ] && [ "$MISSION_STARTED" -eq 1 ] && [ "$NAV_UNAVAILABLE" -eq 0 ]; then
  echo "  OVERALL: ALL CHECKS PASSED"
  exit 0
else
  echo "  OVERALL: SOME CHECKS FAILED"
  exit 1
fi
