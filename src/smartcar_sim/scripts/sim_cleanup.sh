#!/bin/bash
# ============================================================================
# sim_cleanup.sh - simulation cleanup
#
# With no arguments this is safe to run inside ros2 launch and only removes
# stale files. --kill-processes is for sim_start.sh before ros2 launch exists.
# ============================================================================

set -u

kill_processes=false
if [ "${1:-}" = "--kill-processes" ]; then
    kill_processes=true
fi

echo "[cleanup] Starting simulation cleanup..."

if [ "$kill_processes" = true ]; then
    protected_pids="$$"
    parent_pid=$PPID
    while [ "$parent_pid" -gt 1 ] 2>/dev/null; do
        protected_pids="${protected_pids} ${parent_pid}"
        parent_pid=$(ps -o ppid= -p "$parent_pid" 2>/dev/null | tr -d ' ')
        [ -n "$parent_pid" ] || break
    done

    timeout 3s ros2 daemon stop >/dev/null 2>&1 || true

    target_pids=""
    process_names=(
        rviz2 parameter_bridge static_transform_publisher
        controller_server planner_server bt_navigator velocity_smoother
        lifecycle_manager ros2_daemon
    )
    for process_name in "${process_names[@]}"; do
        matches=$(ps -C "$process_name" -o pid= --no-headers 2>/dev/null || true)
        target_pids="${target_pids} ${matches}"
    done

    process_patterns=(
        "ign gazebo"
        "ignition-gazebo"
        "ros2 launch smartcar_sim"
        "/smartcar_sim/"
        "smartcar_sim/launch/sim.launch.py"
        # These Python tools are started by the simulation launch but live in
        # smartcar_tools, so the generic /smartcar_sim/ pattern does not find
        # them after an interrupted run.  Leaving them behind creates duplicate
        # marker publishers and consumes enough CPU to make Gazebo/RViz appear
        # to lag behind the scan and odometry graph.
        "/smartcar_tools/lib/smartcar_tools/waypoint_viz"
        "/smartcar_tools/lib/smartcar_tools/field_reference_node"
        # The keepout stack uses nav2_map_server executables, which are not
        # covered by the core Nav2 process names above. Match their dedicated
        # simulation node names instead of killing unrelated map servers.
        "/nav2_map_server/map_server.*__node:=keepout_mask_server"
        "/nav2_map_server/costmap_filter_info_server.*__node:=keepout_filter_info_server"
    )
    for process_pattern in "${process_patterns[@]}"; do
        matches=$(pgrep -f "$process_pattern" 2>/dev/null || true)
        target_pids="${target_pids} ${matches}"
    done

    filtered_pids=""
    for target_pid in $target_pids; do
        if ! kill -0 "$target_pid" 2>/dev/null; then
            continue
        fi
        is_protected=false
        for protected_pid in $protected_pids; do
            if [ "$target_pid" = "$protected_pid" ]; then
                is_protected=true
                break
            fi
        done
        if [ "$is_protected" = false ]; then
            filtered_pids="${filtered_pids} ${target_pid}"
        fi
    done

    filtered_pids=$(echo "$filtered_pids" | tr ' ' '\n' | sed '/^$/d' | sort -un | tr '\n' ' ')
    if [ -n "$filtered_pids" ]; then
        echo "[cleanup] Killing stale simulation PIDs:${filtered_pids}"
        kill -9 $filtered_pids 2>/dev/null || true
        # Do not relaunch Ogre2 while the previous Gazebo process is still
        # releasing its rendering context. A new server can otherwise expose
        # /scan with only minimum-range samples and no Ackermann odometry.
        # Wait for the exact stale PID set rather than a blind one-second
        # pause, then leave a short scheduler handoff for their descendants.
        remaining_pids="$filtered_pids"
        for _ in $(seq 1 50); do
            next_remaining=""
            for target_pid in $remaining_pids; do
                if kill -0 "$target_pid" 2>/dev/null; then
                    next_remaining="${next_remaining} ${target_pid}"
                fi
            done
            remaining_pids="$next_remaining"
            [ -z "${remaining_pids// /}" ] && break
            sleep 0.1
        done
        if [ -n "${remaining_pids// /}" ]; then
            echo "[cleanup] Warning: stale PIDs survived kill:${remaining_pids}" >&2
        fi
        sleep 0.5
    else
        echo "[cleanup] No stale simulation processes found"
    fi
fi

# ---- 1. Clean DDS shared-memory transport files ----
# FastDDS SHM 传输使用 /dev/shm/ 下的文件作为共享内存对象。
# 前缀包括：fastdds_（官方）、fastrtps_（旧版）。
# 修复原因：shm_only DDS 配置下，残留段名冲突 → participant 创建失败。
shopt -s nullglob 2>/dev/null || true
shm_cleaned=0
for f in /dev/shm/fastdds_* /dev/shm/fastrtps_*; do
    rm -f "$f" && ((shm_cleaned++))
done
shopt -u nullglob 2>/dev/null || true
echo "[cleanup] Removed ${shm_cleaned} DDS SHM file(s)"

# ---- 2. Clean DDS POSIX semaphores ----
# FastDDS 使用命名信号量（sem. 前缀）进行 SHM 段同步。
# 残留信号量会导致新 participant 的 SHM 初始化失败（EACCES/EINVAL）。
shopt -s nullglob 2>/dev/null || true
sem_cleaned=0
for f in /dev/shm/sem.fastdds_* /dev/shm/sem.fastrtps_*; do
    rm -f "$f" && ((sem_cleaned++))
done
shopt -u nullglob 2>/dev/null || true
echo "[cleanup] Removed ${sem_cleaned} DDS semaphore(s)"

# ---- 3. Clean simulation temp files ----
shopt -s nullglob 2>/dev/null || true
tmp_cleaned=0
for f in /tmp/gazebo-* /tmp/ign-* /tmp/fastdds* /tmp/fastrtps* /tmp/ros2_daemon_*; do
    rm -rf "$f" && ((tmp_cleaned++))
done
shopt -u nullglob 2>/dev/null || true
echo "[cleanup] Removed ${tmp_cleaned} tmp file(s)"

# ---- 4. Clean the Gazebo pending lock ----
# Gazebo 的 .gz 锁文件和 master 注册表在 --verbose 模式下可能残留
if [ -d "$HOME/.gz" ]; then
    rm -rf "$HOME/.gz/server/pending.lock" 2>/dev/null || true
    echo "[cleanup] Gazebo lock files cleaned"
fi

echo "[cleanup] Done."
