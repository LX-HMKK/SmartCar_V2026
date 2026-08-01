#!/usr/bin/env bash
# Native Ubuntu entry point for the Gazebo simulation only.
set -eo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

if [ ! -f "${workspace}/install/setup.bash" ]; then
    echo "[local-sim] Missing ${workspace}/install/setup.bash. Run scripts/setup_local_sim.sh first." >&2
    exit 2
fi

# Isaac/Conda overlays in an interactive desktop shell can replace system
# ncurses, Python, and Gazebo libraries. The simulation uses system ROS 2.
unset LD_LIBRARY_PATH
unset LD_PRELOAD
unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

# A terminal launched from Snap VS Code carries GTK/GIO/XDG paths rooted in
# /snap/code. Even after LD_LIBRARY_PATH is cleared, RViz can load a GTK module
# from that runtime, whose dependencies resolve against /snap/core20. Restore
# native desktop locations while deliberately preserving DISPLAY, WAYLAND_DISPLAY,
# XAUTHORITY, XDG_RUNTIME_DIR, and DBUS_SESSION_BUS_ADDRESS.
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

    # VS Code records the host values before it adds its own Snap paths. Use
    # those when available; otherwise remove only known Snap path prefixes.
    if [ -n "${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-}" ]; then
        export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
    else
        export XDG_DATA_DIRS="/usr/local/share:/usr/share"
    fi
    if [ -n "${XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG:-}" ]; then
        export XDG_CONFIG_DIRS="$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG"
    else
        export XDG_CONFIG_DIRS="/etc/xdg"
    fi

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
export SMARTCAR_WS="${workspace}"

exec bash "${workspace}/install/smartcar_sim/share/smartcar_sim/scripts/sim_start.sh" "$@"
