#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Full cleanup
ros2 daemon stop 2>/dev/null || true
sleep 1
bash /root/ros2_ws/install/smartcar_sim/share/smartcar_sim/scripts/sim_cleanup.sh --kill-processes 2>/dev/null || true
rm -f /tmp/auto_train_results.json /tmp/sim_run.log
ros2 daemon start 2>/dev/null || true
echo "Cleanup done, launching simulation..."

# Launch simulation
setsid ros2 launch smartcar_sim sim.launch.py headless:=true use_rviz:=true run_route:=true > /tmp/sim_run.log 2>&1 &
LAUNCH_PID=$!
echo "Launched PID=$LAUNCH_PID"

# Poll for results
for i in $(seq 1 120); do
  sleep 5
  if [ -f /tmp/auto_train_results.json ]; then
    echo "RESULTS_READY"
    cat /tmp/auto_train_results.json
    exit 0
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "LAUNCH_DIED pid=$LAUNCH_PID at iteration $i"
    echo "=== TAIL OF SIM LOG ==="
    tail -80 /tmp/sim_run.log
    echo "=== LAST 30 LINES (ALL) ==="
    tail -30 /tmp/sim_run.log
    exit 1
  fi
done

echo "TIMEOUT after 600s"
tail -50 /tmp/sim_run.log
exit 1
