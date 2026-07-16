#!/usr/bin/env bash
# RDK X5 单一工作空间环境入口
# 部署：python scripts/sync_to_rdk.py setup  （scp 到 RDK ~/source_env.sh）
source /opt/tros/humble/setup.bash
[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash
