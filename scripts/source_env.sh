#!/usr/bin/env bash
# RDK X5 单一工作空间环境入口
# 部署：python scripts/sync_to_rdk.py setup  （复制到 RDK 工作空间 scripts/）
source /opt/tros/humble/setup.bash
if [ -f /home/sunrise/ros2_ws/install/setup.bash ]; then
  source /home/sunrise/ros2_ws/install/setup.bash
fi
