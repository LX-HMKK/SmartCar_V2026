#!/usr/bin/env bash
# Targeted short segment test: c_corner_2 -> b_corridor_return
set -eo pipefail

WS=/root/ros2_ws
INSTALL=$WS/install
RESULT_FILE=/tmp/short_test_results.json
LOG_FILE=/tmp/short_test_$(date +%H%M%S).log

source /opt/ros/humble/setup.bash
source $INSTALL/setup.bash
source $INSTALL/smartcar_sim/share/smartcar_sim/scripts/sim_env.sh

# Cleanup
bash $INSTALL/smartcar_sim/share/smartcar_sim/scripts/sim_cleanup.sh --kill-processes

rm -f "$RESULT_FILE"

echo "[test] Starting short segment: c_corner_2 -> b_corridor_return"
ros2 launch smartcar_sim sim.launch.py \
    headless:=true \
    use_rviz:=false \
    run_route:=true \
    start_goal_id:=c_corner_2 \
    end_goal_id:=b_corridor_return \
    results_file:=$RESULT_FILE \
    > $LOG_FILE 2>&1 &
SIM_PID=$!

# Wait for results (max 6 min)
for i in $(seq 1 180); do
    if [ -s "$RESULT_FILE" ]; then
        echo "[test] Results ready after ${i}s"
        break
    fi
    if ! kill -0 $SIM_PID 2>/dev/null; then
        echo "[test] Sim exited at ${i}s"
        break
    fi
    sleep 2
done

# Stop sim
kill -TERM $SIM_PID 2>/dev/null || true
sleep 3
kill -KILL $SIM_PID 2>/dev/null || true

# Cleanup
bash $INSTALL/smartcar_sim/share/smartcar_sim/scripts/sim_cleanup.sh --kill-processes

# Print results
if [ -s "$RESULT_FILE" ]; then
    python3 -c "
import json
d = json.load(open('$RESULT_FILE'))
print(f'overall: {d[\"overall_outcome\"]}')
for r in d['results']:
    print(f'  {r[\"id\"]:30s} {r[\"outcome\"]:16s} {r[\"duration_sec\"]:6.1f}s  err={r[\"goal_error_m\"]}m  yaw={r[\"goal_yaw_error_rad\"]}rad  travel={r[\"travel_m\"]}m')
"
else
    echo "[test] No results. Log:"
    tail -30 $LOG_FILE
fi
