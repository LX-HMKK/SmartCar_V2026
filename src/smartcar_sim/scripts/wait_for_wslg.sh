#!/usr/bin/env bash
# Wait for WSLg after a WSL restart, then replace this process with RViz.
set -euo pipefail

if [ "$#" -ne 1 ]; then
    echo "usage: wait_for_wslg.sh <rviz-config>" >&2
    exit 2
fi

rviz_config=$1

if grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null; then
    runtime_dir=${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}
    wayland_display=${WAYLAND_DISPLAY:-wayland-0}
    wayland_socket="${runtime_dir}/${wayland_display}"

    export DISPLAY="${DISPLAY:-:0}"
    export XDG_RUNTIME_DIR="$runtime_dir"
    export WAYLAND_DISPLAY="$wayland_display"

    echo "[rviz] Waiting for WSLg socket: ${wayland_socket}"
    for attempt in $(seq 1 60); do
        if [ -S "$wayland_socket" ]; then
            echo "[rviz] WSLg ready after $((attempt - 1))s"
            break
        fi
        sleep 1
    done

    if [ ! -S "$wayland_socket" ]; then
        echo "[rviz] WSLg did not become ready within 60s" >&2
        exit 1
    fi
fi

exec rviz2 -d "$rviz_config" --ros-args -p use_sim_time:=true
