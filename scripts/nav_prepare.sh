#!/bin/bash
# Rebuild the physical navigation stack only after source/config changes.
set -euo pipefail

SOURCE_ENV=/root/source_env.sh
WORKSPACE=/home/sunrise/ros2_ws
COLCON_PARALLEL_WORKERS=8
BUILD_FINGERPRINT="$WORKSPACE/.smartcar_nav_prepare_fingerprint"

source_fingerprint() {
  # A fresh workspace must not depend on packages left in a previous overlay.
  # Hash the full source tree so every dependency change requires preparation.
  find "$WORKSPACE/src" \
    -type f \
    ! -path '*/.git/*' \
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
  --packages-up-to smartcar_bringup smartcar_vision smartcar_tools \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --allow-overriding ackermann_msgs smartcar_interfaces smartcar_safety smartcar_nav2 smartcar_task smartcar_bringup smartcar_vision smartcar_tools

source_fingerprint > "$BUILD_FINGERPRINT"
echo "Prepared fingerprint: $(cat "$BUILD_FINGERPRINT")"
