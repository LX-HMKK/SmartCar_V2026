#!/usr/bin/env bash
# Validate NavigateThroughPoses for the forward segment in local simulation.
#
# Usage (inside the repository checkout):
#   bash through_poses_test.sh [--rviz]

set -o pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=${SMARTCAR_WS:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}
INSTALL=$WS/install
RESULT_FILE=/tmp/through_poses_test_results.json
LOG_FILE="/tmp/through_poses_test_$(date +%H%M%S).log"
SIM_PID=""
TEST_PID=""

_cleanup() {
    kill -TERM "$TEST_PID" 2>/dev/null || true
    kill -TERM "$SIM_PID" 2>/dev/null || true
    sleep 2
    kill -KILL "$TEST_PID" 2>/dev/null || true
    kill -KILL "$SIM_PID" 2>/dev/null || true
    bash "$INSTALL/smartcar_sim/share/smartcar_sim/scripts/sim_cleanup.sh" \
        --kill-processes 2>/dev/null || true
}

source /opt/ros/humble/setup.bash
source "$INSTALL/setup.bash"
source "$INSTALL/smartcar_sim/share/smartcar_sim/scripts/sim_env.sh"

USE_RVIZ=false
if [ "${1:-}" = "--rviz" ]; then
    USE_RVIZ=true
fi

# ── Cleanup ──
bash "$INSTALL/smartcar_sim/share/smartcar_sim/scripts/sim_cleanup.sh" --kill-processes
rm -f "$RESULT_FILE"

echo "============================================"
echo " ThroughPoses Validation Test"
echo " Mode: hybrid (single reverse + ThroughPoses forward)"
echo " RViz: $USE_RVIZ"
echo " Log:  $LOG_FILE"
echo "============================================"

# ── Launch simulation ──
ros2 launch smartcar_sim sim.launch.py \
    headless:=true \
    use_rviz:=$USE_RVIZ \
    run_route:=false \
    > "$LOG_FILE" 2>&1 &
SIM_PID=$!

echo "[setup] Simulation PID: $SIM_PID"
echo "[setup] Waiting for Nav2 (max 180s)..."

WAIT_START=$(date +%s)
FOUND_BT=false
while true; do
    ELAPSED=$(($(date +%s) - WAIT_START))
    if ros2 node list 2>/dev/null | grep -q "bt_navigator"; then
        echo "[setup] bt_navigator detected after ${ELAPSED}s"
        FOUND_BT=true
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[setup] ERROR: simulation died after ${ELAPSED}s"
        tail -50 "$LOG_FILE"
        _cleanup
        exit 1
    fi
    if [ "$ELAPSED" -ge 180 ]; then
        echo "[setup] ERROR: timeout (180s) waiting for bt_navigator"
        tail -50 "$LOG_FILE"
        _cleanup
        exit 1
    fi
    sleep 2
done

# Wait for lifecycle to fully activate (all nodes active)
echo "[setup] Waiting for lifecycle activation..."
for i in $(seq 1 30); do
    if tail -5 "$LOG_FILE" 2>/dev/null | grep -q "Managed nodes are active"; then
        echo "[setup] Lifecycle active after ~$((i*2))s"
        break
    fi
    sleep 2
done
sleep 3  # extra settle

# ── Run the hybrid test ──
NAV2_SHARE="$INSTALL/smartcar_nav2/share/smartcar_nav2"

echo "[test] Starting hybrid test..."
ros2 run smartcar_sim test_through_poses.py --ros-args \
    -p use_sim_time:=true \
    -p waypoints_file:="$NAV2_SHARE/config/waypoints/nav_only.yaml" \
    -p through_poses_bt:="$NAV2_SHARE/config/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml" \
    -p forward_behavior_tree:="$NAV2_SHARE/config/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml" \
    -p precise_behavior_tree:="$NAV2_SHARE/config/behavior_trees/navigate_to_pose_precise_w_replanning_and_recovery.xml" \
    -p reverse_behavior_tree:="$NAV2_SHARE/config/behavior_trees/navigate_to_pose_reverse_w_replanning_and_recovery.xml" \
    -p reverse_handoff_behavior_tree:="$NAV2_SHARE/config/behavior_trees/navigate_to_pose_reverse_handoff_w_replanning_and_recovery.xml" \
    -p nav2_params_file:="$NAV2_SHARE/config/nav2_params_fixed.yaml" \
    -p results_file:="$RESULT_FILE" \
    -p settle_delay_sec:=10.0 \
    -p goal_timeout_sec:=300.0 \
    >> "$LOG_FILE" 2>&1 &
TEST_PID=$!

# Wait for test (max 10 minutes)
echo "[test] Waiting for results (PID $TEST_PID, max 600s)..."
TEST_START=$(date +%s)
while true; do
    ELAPSED=$(($(date +%s) - TEST_START))
    if [ -s "$RESULT_FILE" ]; then
        echo "[test] Results ready after ${ELAPSED}s"
        break
    fi
    if ! kill -0 "$TEST_PID" 2>/dev/null; then
        echo "[test] Test process exited after ${ELAPSED}s (checking for partial results)"
        sleep 1
        break
    fi
    if [ "$ELAPSED" -ge 600 ]; then
        echo "[test] Timeout (600s)"
        break
    fi
    sleep 2
done

# ── Print results ──
echo ""
echo "============================================"
echo " Results"
echo "============================================"

if [ -s "$RESULT_FILE" ]; then
    python3 - "$RESULT_FILE" << 'PYEOF'
import json, sys
d = json.load(open(sys.argv[1]))
print(f"overall: {d['overall_outcome']}")
print(f"mode: {d['mode']}")
print()
for r in d['results']:
    if r.get('mode') == 'through_poses':
        rid = r['id']
        if len(rid) > 80:
            rid = rid[:77] + '...'
        print(f"[through] {rid}")
        print(f"  outcome={r['outcome']}  dur={r['duration_sec']}s  travel={r['travel_m']}m")
        print(f"  final_err={r.get('final_goal_error_m','?')}m  yaw_err={r.get('final_yaw_error_rad','?')}rad")
        for wp in r.get('waypoints_passed', []):
            print(f"    {wp['id']:30s} min_dist={wp['min_distance_m']}m")
    elif r.get('mode') == 'single':
        print(f"[single] {r['id']:30s} {r['outcome']:16s} {r['duration_sec']:6.1f}s  err={r.get('goal_error_m','?')}m  yaw={r.get('goal_yaw_error_rad','?')}rad  travel={r.get('travel_m','?')}m")
    else:
        print(f"[{r.get('mode','?')}] {r.get('id','?')}: {r.get('outcome','?')}")
PYEOF
else
    echo "No results file."
fi

# ── Cleanup ──
_cleanup
