#!/usr/bin/env bash
# Native Ubuntu entry point for the motion-disabled simulation waypoint editor.
set -eo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

if [ ! -f "${workspace}/install/setup.bash" ]; then
    echo "[waypoint-editor] Missing ${workspace}/install/setup.bash. Run scripts/setup_local_sim.sh first." >&2
    exit 2
fi

unset LD_LIBRARY_PATH
unset LD_PRELOAD
unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

# A terminal launched from Snap VS Code carries GTK/GIO/XDG paths rooted in
# /snap/code. Qt can otherwise load a GTK module whose dependencies resolve
# against /snap/core20 instead of the host glibc. Preserve the display/session
# variables so the editor still opens on the current desktop.
is_snap_terminal=false
if [ -n "${SNAP:-}" ] || [ -n "${SNAP_NAME:-}" ] || \
        [ -n "${SNAP_USER_DATA:-}" ]; then
    is_snap_terminal=true
fi

if [ "$is_snap_terminal" = true ]; then
    desktop_home=${SNAP_REAL_HOME:-${HOME}}
    export HOME="$desktop_home"
    export XDG_DATA_HOME="${HOME}/.local/share"
    export XDG_CONFIG_HOME="${HOME}/.config"
    export XDG_CACHE_HOME="${HOME}/.cache"
    export XDG_STATE_HOME="${HOME}/.local/state"
    export XDG_DATA_DIRS="${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-/usr/local/share:/usr/share}"
    export XDG_CONFIG_DIRS="${XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG:-/etc/xdg}"

    unset GTK_DATA_PREFIX GTK_EXE_PREFIX GTK_PATH GTK_IM_MODULE_FILE
    unset GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR
    unset GIO_EXTRA_MODULES GIO_MODULE_DIR GIO_LAUNCHED_DESKTOP_FILE
    unset GIO_LAUNCHED_DESKTOP_FILE_PID GSETTINGS_SCHEMA_DIR GI_TYPELIB_PATH
    unset XDG_DATA_DIRS_VSCODE_SNAP_ORIG XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG
fi

unset SNAP SNAP_ARCH SNAP_COMMON SNAP_CONTEXT SNAP_COOKIE SNAP_DATA SNAP_EUID
unset SNAP_INSTANCE_KEY SNAP_INSTANCE_NAME SNAP_LAUNCHER_ARCH_TRIPLET
unset SNAP_LIBRARY_PATH SNAP_NAME SNAP_REAL_HOME SNAP_REEXEC SNAP_REVISION
unset SNAP_UID SNAP_USER_COMMON SNAP_USER_DATA SNAP_VERSION

source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u

exec ros2 launch smartcar_tools sim_waypoint_editor.launch.py \
    use_rviz:=false \
    use_sim_time:=false \
    "$@"
