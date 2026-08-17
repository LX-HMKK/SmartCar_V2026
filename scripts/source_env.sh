#!/usr/bin/env bash
# RDK X5 单一工作空间环境入口
# 部署：python scripts/sync_to_rdk.py setup  （复制到 RDK 工作空间 scripts/）
COMPETITION_ENV_SNAPSHOT=/home/sunrise/ros2_ws/.smartcar_competition_env.sh

# nav_prepare.sh writes this small, post-build environment snapshot. It avoids
# repeatedly evaluating every colcon package hook during competition startup.
# A missing snapshot deliberately falls back to the normal setup path.
if [[ "${SMARTCAR_COMPETITION_FAST_ENV:-}" == "1" && \
      -r "$COMPETITION_ENV_SNAPSHOT" ]]; then
  source "$COMPETITION_ENV_SNAPSHOT"
elif [ -f /home/sunrise/ros2_ws/install/setup.bash ]; then
  source /home/sunrise/ros2_ws/install/setup.bash
else
  source /opt/tros/humble/setup.bash
fi
