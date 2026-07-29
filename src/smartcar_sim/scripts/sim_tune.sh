#!/usr/bin/env bash
# Build the current parameters and run the complete simulated route.
set -eo pipefail

workspace=${SMARTCAR_WS:-/root/ros2_ws}
repo_root=${SMARTCAR_REPO_ROOT:-/mnt/d/StudyWorks/3.2/SmartCar}
source_root=${SMARTCAR_SRC:-${repo_root}/src}
log_dir=${SMARTCAR_TUNE_LOG_DIR:-${workspace}/tune_logs}
headless=false
loop_count=1
no_build_requested=false
sim_pid=""

while [ "$#" -gt 0 ]; do
    case "$1" in
        --headless)
            headless=true
            shift
            ;;
        --loop)
            loop_count=${2:?--loop requires a count}
            shift 2
            ;;
        --no-build)
            no_build_requested=true
            shift
            ;;
        *)
            echo "Unknown option: $1" >&2
            exit 2
            ;;
    esac
done

if ! [[ "$loop_count" =~ ^[1-9][0-9]*$ ]]; then
    echo "--loop must be a positive integer" >&2
    exit 2
fi

if [ ! -f "${source_root}/smartcar_sim/package.xml" ] || \
        [ ! -f "${source_root}/smartcar_nav2/config/nav2_params.yaml" ] || \
        [ ! -f "${source_root}/smartcar_tools/config/routes/route_planning.yaml" ]; then
    echo "Invalid SMARTCAR_SRC (missing simulation, nav2 params, or route planning config): ${source_root}" >&2
    exit 2
fi

mkdir -p "$log_dir" "${workspace}/src"
lock_file=${SMARTCAR_TUNE_LOCK_FILE:-${workspace}/.sim_tune.lock}
exec 9>"$lock_file"
if ! flock -n 9; then
    echo "Another sim_tune.sh run is active for ${workspace}." >&2
    exit 2
fi
source /opt/ros/humble/setup.bash

echo "[tune] Regenerating simulation keepout map from shared route_planning.yaml..."
PYTHONPATH="${source_root}/smartcar_tools${PYTHONPATH:+:${PYTHONPATH}}" \
    python3 "${source_root}/smartcar_sim/scripts/generate_field_map.py" \
        --geometry "${source_root}/smartcar_tools/config/routes/field_geometry.yaml" \
        --route-planning-config "${source_root}/smartcar_tools/config/routes/route_planning.yaml" \
        --maps-dir "${source_root}/smartcar_sim/maps"
python3 "${source_root}/smartcar_sim/scripts/sync_route_planning.py" \
    --route-planning-config "${source_root}/smartcar_tools/config/routes/route_planning.yaml" \
    --nav2-params "${source_root}/smartcar_nav2/config/nav2_params.yaml" \
    --keepout-overlay "${source_root}/smartcar_sim/config/nav2_keepout_filter.yaml"

rsync -a \
    --delete \
    --exclude='build/' \
    --exclude='install/' \
    --exclude='log/' \
    --exclude='__pycache__/' \
    "${source_root}/" "${workspace}/src/"

source "${workspace}/src/smartcar_sim/scripts/sim_env.sh"
cleanup_script="${workspace}/src/smartcar_sim/scripts/sim_cleanup.sh"
result_validator="${workspace}/src/smartcar_sim/scripts/validate_sim_results.py"

cleanup() {
    if [ -n "$sim_pid" ] && kill -0 "$sim_pid" 2>/dev/null; then
        kill -TERM "$sim_pid" 2>/dev/null || true
        for _ in $(seq 1 50); do
            kill -0 "$sim_pid" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 "$sim_pid" 2>/dev/null; then
            kill -KILL "$sim_pid" 2>/dev/null || true
        fi
        wait "$sim_pid" 2>/dev/null || true
    fi
    sim_pid=""
    bash "$cleanup_script" --kill-processes >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

if [ "$no_build_requested" = true ]; then
    echo "[tune] --no-build ignored: nav2_params_fixed.yaml must be regenerated."
fi

echo "[tune] Building current navigation and simulation sources..."
(
    cd "$workspace"
    colcon build --symlink-install \
        --packages-up-to smartcar_sim \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
)
source "${workspace}/install/setup.bash"

overall_failed=0
for run_index in $(seq 1 "$loop_count"); do
    run_id="run_$(date +%Y%m%d_%H%M%S)_${run_index}"
    log_file="${log_dir}/${run_id}.log"
    json_file="${log_dir}/${run_id}.json"
    snapshot_dir="${log_dir}/${run_id}_inputs"
    result_file="/tmp/${run_id}_auto_train_results.json"
    rm -f "$result_file"
    cleanup

    mkdir -p "$snapshot_dir"
    cp "${source_root}/smartcar_nav2/config/nav2_params.yaml" "$snapshot_dir/"
    cp "${workspace}/install/smartcar_nav2/share/smartcar_nav2/config/nav2_params_fixed.yaml" \
        "$snapshot_dir/"
    cp "${source_root}/smartcar_nav2/config/waypoints/nav_only.yaml" "$snapshot_dir/"
    cp "${source_root}/smartcar_tools/config/routes/route_planning.yaml" "$snapshot_dir/"
    cp "${source_root}/smartcar_sim/maps/field_map.pgm" "$snapshot_dir/"
    cp "${source_root}/smartcar_sim/maps/field_map.yaml" "$snapshot_dir/"
    cp "${source_root}/smartcar_nav2/config/behavior_trees/"*.xml "$snapshot_dir/"

    if [ "$headless" = true ]; then
        gazebo_headless=true
        use_rviz=false
    else
        gazebo_headless=false
        use_rviz=true
    fi

    echo "[tune] Starting ${run_id} (headless=${gazebo_headless})"
    run_started_epoch=$(date +%s)
    ros2 launch smartcar_sim sim.launch.py \
        headless:="$gazebo_headless" \
        use_rviz:="$use_rviz" \
        run_route:=true \
        results_file:="$result_file" \
        >"$log_file" 2>&1 &
    sim_pid=$!

    elapsed=0
    timeout_sec=1500
    while [ "$elapsed" -lt "$timeout_sec" ]; do
        if [ -s "$result_file" ]; then
            break
        fi
        if ! kill -0 "$sim_pid" 2>/dev/null; then
            echo "[tune] Simulation exited before producing results." >&2
            break
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done

    if [ ! -s "$result_file" ]; then
        echo "[tune] ${run_id} produced no result within ${timeout_sec}s." >&2
        overall_failed=1
        cleanup
        continue
    fi

    cp "$result_file" "$json_file"
    if python3 "$result_validator" "$json_file" \
            --started-after "$run_started_epoch"
    then
        echo "[tune] ${run_id} completed. Results: ${json_file}"
    else
        overall_failed=1
        echo "[tune] ${run_id} failed. Results: ${json_file}" >&2
    fi
    cleanup
done

exit "$overall_failed"
