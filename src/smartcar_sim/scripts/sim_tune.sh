#!/usr/bin/env bash
# Build the current parameters and run the complete simulated route.
set -eo pipefail

workspace=${SMARTCAR_WS:-/root/ros2_ws}
# The WSL workspace is the runtime source of truth. In particular, do not
# replace user-edited waypoints merely because this script was started from a
# Windows-mounted checkout.
source_root="${workspace}/src"
windows_repo_root=${SMARTCAR_REPO_ROOT:-/mnt/d/StudyWorks/3.2/SmartCar}
windows_source_root=${SMARTCAR_WINDOWS_SRC:-${windows_repo_root}/src}
log_dir=${SMARTCAR_TUNE_LOG_DIR:-${workspace}/tune_logs}
headless=false
loop_count=1
no_build_requested=false
sync_from_windows=false
sync_only=false
sim_pid=""

usage() {
    cat <<'EOF'
Usage: sim_tune.sh [options]

Build and run the simulation using SMARTCAR_WS/src (default: /root/ros2_ws/src).

Options:
  --headless                 Run Gazebo without RViz.
  --loop COUNT               Run the complete route COUNT times.
  --no-build                 Accepted for compatibility; the build still runs.
  --sync-from-windows        Explicitly copy the Windows source into SMARTCAR_WS/src.
  --windows-src PATH         Windows source root used with --sync-from-windows.
  --sync-only                Finish after the explicit source sync; do not build or run.
  -h, --help                 Show this help text.

The Windows source defaults to SMARTCAR_WINDOWS_SRC, or
SMARTCAR_REPO_ROOT/src (/mnt/d/StudyWorks/3.2/SmartCar/src by default).
Before a differing editable Nav2 YAML is overwritten, the WSL copy is backed
up under SMARTCAR_TUNE_LOG_DIR/manual_backups.
EOF
}

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
        --sync-from-windows)
            sync_from_windows=true
            shift
            ;;
        --windows-src)
            windows_source_root=${2:?--windows-src requires a path}
            shift 2
            ;;
        --sync-only)
            sync_only=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
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

if [ "$sync_only" = true ] && [ "$sync_from_windows" != true ]; then
    echo "--sync-only requires --sync-from-windows." >&2
    exit 2
fi

validate_source_root() {
    local candidate=$1
    local label=$2
    if [ ! -f "${candidate}/smartcar_sim/package.xml" ] || \
            [ ! -f "${candidate}/smartcar_nav2/config/nav2_params.yaml" ] || \
            [ ! -f "${candidate}/smartcar_nav2/config/waypoints/nav_only.yaml" ] || \
            [ ! -f "${candidate}/smartcar_tools/config/routes/route_planning.yaml" ]; then
        echo "Invalid ${label} (missing simulation, nav2 params, nav_only waypoints, or route planning config): ${candidate}" >&2
        return 2
    fi
}

file_sha256() {
    sha256sum "$1" | awk '{print $1}'
}

backup_wsl_file_before_sync() {
    local source_file=$1
    local workspace_file=$2
    local file_label=$3
    local windows_hash
    local wsl_hash
    local backup_dir
    local backup_prefix
    local backup_file
    local suffix=1

    windows_hash=$(file_sha256 "$source_file")
    echo "[tune] Windows ${file_label} SHA256: ${windows_hash}"
    if [ ! -f "$workspace_file" ]; then
        echo "[tune] WSL ${file_label} does not exist yet; no backup is needed."
        return
    fi

    wsl_hash=$(file_sha256 "$workspace_file")
    echo "[tune] WSL ${file_label} SHA256 before sync: ${wsl_hash}"
    if [ "$windows_hash" = "$wsl_hash" ]; then
        echo "[tune] ${file_label} matches; no backup is needed."
        return
    fi

    backup_dir="${log_dir}/manual_backups"
    mkdir -p "$backup_dir"
    backup_prefix="${backup_dir}/${file_label%.yaml}.before-windows-sync-$(date +%Y%m%d_%H%M%S)"
    backup_file="${backup_prefix}.yaml"
    while [ -e "$backup_file" ]; do
        backup_file="${backup_prefix}_${suffix}.yaml"
        suffix=$((suffix + 1))
    done
    cp --preserve=mode,timestamps "$workspace_file" "$backup_file"
    echo "[tune] Backed up WSL ${file_label} before sync: ${backup_file}"
}

sync_windows_source() {
    local windows_waypoints="${windows_source_root}/smartcar_nav2/config/waypoints/nav_only.yaml"
    local wsl_waypoints="${source_root}/smartcar_nav2/config/waypoints/nav_only.yaml"
    local windows_nav2_params="${windows_source_root}/smartcar_nav2/config/nav2_params.yaml"
    local wsl_nav2_params="${source_root}/smartcar_nav2/config/nav2_params.yaml"

    validate_source_root "$windows_source_root" "Windows source"
    mkdir -p "$source_root"
    backup_wsl_file_before_sync "$windows_waypoints" "$wsl_waypoints" "nav_only.yaml"
    backup_wsl_file_before_sync "$windows_nav2_params" "$wsl_nav2_params" "nav2_params.yaml"

    echo "[tune] Explicit Windows-to-WSL source sync: ${windows_source_root} -> ${source_root}"
    # Never delete WSL-only files. This is an explicit overwrite sync, but
    # local investigation artifacts and saved files remain recoverable.
    rsync -a \
        --exclude='build/' \
        --exclude='install/' \
        --exclude='log/' \
        --exclude='__pycache__/' \
        "${windows_source_root}/" "${source_root}/"

    if [ "$(file_sha256 "$windows_waypoints")" != "$(file_sha256 "$wsl_waypoints")" ]; then
        echo "[tune] ERROR: nav_only.yaml did not match Windows source after sync." >&2
        return 1
    fi
    if [ "$(file_sha256 "$windows_nav2_params")" != "$(file_sha256 "$wsl_nav2_params")" ]; then
        echo "[tune] ERROR: nav2_params.yaml did not match Windows source after sync." >&2
        return 1
    fi
    echo "[tune] WSL nav_only.yaml SHA256 after sync: $(file_sha256 "$wsl_waypoints")"
    echo "[tune] WSL nav2_params.yaml SHA256 after sync: $(file_sha256 "$wsl_nav2_params")"
}

mkdir -p "$log_dir" "${workspace}/src"
lock_file=${SMARTCAR_TUNE_LOCK_FILE:-${workspace}/.sim_tune.lock}
exec 9>"$lock_file"
if ! flock -n 9; then
    echo "Another sim_tune.sh run is active for ${workspace}." >&2
    exit 2
fi

if [ -n "${SMARTCAR_SRC:-}" ]; then
    echo "[tune] SMARTCAR_SRC is ignored; using WSL source: ${source_root}" >&2
fi

if [ "$sync_from_windows" = true ]; then
    sync_windows_source
fi
validate_source_root "$source_root" "WSL source"
waypoints_file="${source_root}/smartcar_nav2/config/waypoints/nav_only.yaml"

if [ "$sync_only" = true ]; then
    echo "[tune] Explicit Windows-to-WSL source sync completed without build/run."
    exit 0
fi

source /opt/ros/humble/setup.bash

echo "[tune] WSL source: ${source_root}"
echo "[tune] Runtime nav_only.yaml: ${waypoints_file}"
echo "[tune] Runtime nav_only.yaml SHA256: $(file_sha256 "$waypoints_file")"
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
    cp "$waypoints_file" "$snapshot_dir/nav_only.yaml"
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
        waypoints_file:="$snapshot_dir/nav_only.yaml" \
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
            --started-after "$run_started_epoch" \
            --waypoints-file "$snapshot_dir/nav_only.yaml"
    then
        echo "[tune] ${run_id} completed. Results: ${json_file}"
    else
        overall_failed=1
        echo "[tune] ${run_id} failed. Results: ${json_file}" >&2
    fi
    cleanup
done

exit "$overall_failed"
