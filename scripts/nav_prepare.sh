#!/bin/bash
# Rebuild the physical navigation stack only after source/config changes.
set -euo pipefail

SOURCE_ENV=/root/source_env.sh
WORKSPACE=/root/ros2_ws
COLCON_PARALLEL_WORKERS=8
BUILD_FINGERPRINT="$WORKSPACE/.smartcar_nav_prepare_fingerprint"

source_fingerprint() {
  # Hash every package build input. Runtime launch and cleanup scripts are
  # deployed independently and do not require a seven-package rebuild.
  find \
    "$WORKSPACE/src/smartcar_common" \
    "$WORKSPACE/src/smartcar_interfaces" \
    "$WORKSPACE/src/smartcar_safety" \
    "$WORKSPACE/src/smartcar_nav2" \
    "$WORKSPACE/src/smartcar_task" \
    "$WORKSPACE/src/smartcar_bringup" \
    "$WORKSPACE/src/smartcar_vision" \
    -type f \
    ! -path '*/__pycache__/*' \
    ! -name '*.pyc' \
    ! -name '*.bak' \
    ! -name '*.orig' \
    -print0 \
    | sort -z \
    | xargs -0r sha256sum \
    | sha256sum \
    | awk '{print $1}'
}

set +u
source "$SOURCE_ENV"
set -u
cd "$WORKSPACE"

bash "$WORKSPACE/scripts/ros_cleanup.sh"

colcon build --symlink-install \
  --parallel-workers "$COLCON_PARALLEL_WORKERS" \
  --packages-select smartcar_common smartcar_interfaces smartcar_safety smartcar_nav2 smartcar_task smartcar_bringup smartcar_vision \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --allow-overriding smartcar_interfaces smartcar_safety smartcar_nav2 smartcar_task smartcar_bringup smartcar_vision

source_fingerprint > "$BUILD_FINGERPRINT"
echo "Prepared fingerprint: $(cat "$BUILD_FINGERPRINT")"
