# 已知问题与避坑指南

> 源文件：`CLAUDE.md` → 本文件（低频查阅，为 CLAUDE.md 减负）
> 最后更新：2026-07-26

## WSL2 Gazebo 仿真

- **mirrored 网络破坏 Fast DDS discovery**：当前 WSL 2.6.3 环境中，
  `networkingMode=mirrored` 会创建 `loopback0` 和额外策略路由；Fast DDS
  `ros2 node list` 可能为空或经 daemon 卡死，Nav2 lifecycle manager 会持续等待
  `controller_server/get_state`。将 `%USERPROFILE%\.wslconfig` 改为
  `networkingMode=nat`，执行 `wsl.exe --shutdown` 后恢复。不要注入旧
  `FASTRTPS_DEFAULT_PROFILES_FILE` loopback XML。
- **WSLg 重建竞态**：WSL 重启后 RViz 必须等待
  `/mnt/wslg/runtime-dir/wayland-0`。使用 `sim_start.sh --headless --rviz`，不要用
  固定 sleep 代替 socket 检查。
- **仿真残留进程**：日常使用 `sim_start.sh` 自动清理；手动恢复使用
  `sim_cleanup.sh --kill-processes`。完整说明见
  [`../deployment/wsl-simulation.md`](../deployment/wsl-simulation.md)。

## 硬件与传感器

- **LiDAR 物理朝向**：2D LiDAR 物理安装转了 90°（Y 朝后/X 朝左），`laser_yaw=1.5708` 已修复。若更换/重新安装 LiDAR 必须重新确认朝向。
- **`cmd_vel_to_ackermann_drive` 竞争**：直接往 `/ackermann_cmd` pub 时，必须先 kill 该节点，否则 safety_node 20Hz 零值覆盖你的指令（车不动或抖动）。
- **2D LiDAR 无法区分高度**：锥桶上窄下宽——雷达看到的是上半窄截面，车体下半蹭宽底。通过非对称 footprint（后轴原点）+ `obstacle_min_range=0.25` 滤车身 + 膨胀半径补偿，不能根本解决。物理上需 3D 感知或更谨慎的锥桶摆放。

## 导航与运动

- **Ackermann 终点兜圈**：无法原地旋转。`xy_goal_tolerance=0.25, yaw_goal_tolerance=0.50` 已修复。
- **0.30 m/s 运动异常未定因**：原始日志表明 EKF 最严重的更新超期发生在路线启动前；`odom0_config` 也不融合原始 pose，因此旧版"IntegrationClock 导致 EKF pose/速度冲突"的推断已撤回。现已消除 EKF 非零 TF 等待、降低 BT tick 负载并关闭 RF2O 逐帧 INFO；复测前仍按 0.15 m/s 已验证上限管理，详见 `docs/review/odometry-speed-analysis.md`。
- **倒车导航实现 (2026-07-25)**：QR→VLM 段必须倒车。每个航点独立发送 `NavigateToPose`；QR 方向切换点使用 `navigate_to_pose_precise_w_replanning_and_recovery.xml`（`0.12 m / 0.15 rad` 精确 goal checker），Smac 近似终点 `tolerance=0.0`；reverse goal 使用 `navigate_to_pose_reverse_w_replanning_and_recovery.xml`（`ComputeReversePathToPose` 自定义 BT：yaw 加 π → 唯一 DUBIN planner → yaw 恢复 π → 严格验证）。`allow_reversing=true` 只允许 RPP 沿既有反向路径输出负速度，不会把 planner 改成 Reeds-Shepp，cusp 也会被拒绝。方向门倒车 `angular.z` 翻转是用户现场 A/B 确认有效的执行器链补偿；`minimum_turning_radius=0.55, curvature_tolerance=0.20` 已恢复，后段历史 `curvature_exceeded` 仍待本轮完整路线复测。详见 `docs/troubleshooting/nav-startup-issues.md`。
- **方向门倒车转向翻转 (2026-07-24)**：用户在实体车上观察到倒车转向打反；`direction_guard` 的 `on_candidate()` 和 `evaluate()` 在 `MotionDirection::Reverse` 时翻转 `candidate[5]`（angular.z）后，同一首个倒车 goal 转向正确并成功。该行为记录当前执行器链的实测约定，不再归因于通用 RPP 公式。

## Nav2 配置与行为树

- **Nav2 参数链路 (2026-07-23 修复)**：Nav2 1.1.20 (TROS Humble) 的 `RewrittenYaml` chain 存在 bug——`ParameterFile` 输出被 YDLIDAR 驱动参数覆盖，导致 `controller_server` 加载默认 DWB 而非 RPP。修复方案：`CMakeLists.txt` 自动生成 `nav2_params_fixed.yaml`（BT 路径硬编码），`navigation_launch.py` 直接使用该文件。任何对 `nav2_params.yaml` 的修改会在 `colcon build` 时自动同步到 fixed 文件。不可手动创建/编辑 `nav2_params_fixed.yaml`。
- **Nav2 启动封装遗留接口**：`smartcar_nav2.launch.py` 仍暴露并转发 `use_waypoint_follower`，上层也仍传入该参数，但当前运行时不会启动 `waypoint_follower`。同一封装中的 `params_file`、BT XML 和 `waypoints_file` 参数部分已被固定参数链路旁路；不得把这些旧参数当作有效运行时配置。后续应连同 `nav2_waypoint_follower` 包依赖和未使用的 `FollowWaypoints` 结果分类器一起清理并更新合同测试。
- **行为树已移除 backup/wait recovery (2026-07-23)**：两个 BT XML 文件不再包含 `BackUp` 和 `Wait` 恢复动作，仅保留 `ClearEntireCostmap` + 重规划。阿克曼底盘不可引入原地旋转或后退恢复。

## ROS2 生命周期与运维

- **`velocity_smoother` 生命周期卡死**：激活后偶发 `get_state` 服务无响应。解决：`ros2 daemon stop` → `kill -9` 所有 ROS 进程 → `ros2 daemon start` → 重启 launch。
- **重启必须彻底杀旧进程**：多次 kill/restart 循环会残留旧 launch 子进程（YDLIDAR、obstacle_extractor、task_node 等），占用 `/dev/ttyUSB0` 串口并消耗 CPU。**推荐使用 `bash scripts/ros_cleanup.sh` 一键清理**；手动方式：`pkill -9` 全部 ROS 节点可执行文件 → `ros2 daemon stop` → `sleep 1` → `ros2 daemon start` → `ros2 launch`。仅靠 `ctrl-c` 或部分 pkill 不可靠。
- **ROS2 CLI 卡死备用方案**：`ros2 service call` 在 lifecycle 异常后可能无限等待。紧急停车可用 `pkill -9 -f "ros2 launch"` 直接杀 launch 进程，STM32 超时自动发送停止指令。日常清理推荐 `bash scripts/ros_cleanup.sh`。

## 运维检查清单

### 构建后参数验证

`colcon build` 成功后，必须确认 `nav2_params_fixed.yaml` 中的关键参数：

```bash
grep motion_model_for_search /root/ros2_ws/install/smartcar_nav2/share/smartcar_nav2/config/nav2_params_fixed.yaml
grep allow_reversing /root/ros2_ws/install/smartcar_nav2/share/smartcar_nav2/config/nav2_params_fixed.yaml
grep current_goal_checker /root/ros2_ws/install/smartcar_nav2/share/smartcar_nav2/config/nav2_params_fixed.yaml
```

期望：`DUBIN`、`true`、`"goal_checker"`，且 `min_velocity` 含负值（倒车）。

### 发车前检查

```bash
grep "Managed nodes are active" /tmp/bringup.log            # Nav2 生命周期全部 active
ros2 node list | grep -E "safety_node|task_node|controller_server|direction_guard"
ros2 service list | grep -E "smartcar/(safety/emergency_stop|task/start)"
```

### SSH 远程操作

- 有线 `192.168.128.10` 优先于无线 `172.16.24.193`
- 所有 ROS2 CLI 前必须 `source /root/source_env.sh`
- 后台启动用 `nohup` + 输出重定向，不依赖 SSH 会话保持
- 日志追踪优先于 topic echo（`tail -f /tmp/bringup.log | grep ...` 比 `ros2 topic echo` 更可靠）
- 关键 grep 关键词：`NavigateToPose`、`reached`、`ERROR`、`FAILED`、`reverse`

### 应急操作

| 优先级 | 操作 |
|--------|------|
| 1 | `ros2 service call /smartcar/safety/emergency_stop std_srvs/srv/SetBool '{data: true}'` |
| 2 | 若 CLI 卡死：`pkill -9 -f "ros2 launch"`（STM32 超时自动发停止） |
| 3 | 物理急停按钮 |

完全重建：`bash /usr/local/bin/ros_cleanup && rm -f /tmp/bringup.log && nohup bash /root/nav_test.sh > /tmp/nav_test_output.log 2>&1 &`

### 症状速查

| 症状 | 根因 | 修复 |
|------|------|------|
| 脚本首行就退出 | TROS `unbound variable` | `set +u` 包裹 source |
| cleanup 后脚本消失 | ros_cleanup kill 自己 | 从 kill 列表移除脚本名 |
| `goal_checker ... does not exist` | BT FollowPath 缺 goal_checker_id | 添加 `goal_checker_id="goal_checker"` |
| `ros2 service call` 挂起 | daemon 状态不一致 | `ros2 daemon stop && ros2 daemon start` |
| `reset_not_terminal` | 任务未在终态 | 正常，直接 start 即可 |

## 已修复项（保持警惕）

- **TROS/ros_cleanup/Forward BT 三修复 (2026-07-24)**：① `set -euo pipefail` 脚本 source TROS setup.bash 前须包裹 `set +u`/`set -u`；② `ros_cleanup.sh` pkill 列表不得含 `nav_test`（会自杀）；③ 正向 BT `FollowPath` 须指定 `goal_checker_id="goal_checker"`。——均已修复并保持。
- **direction_guard 时序过紧 (2026-07-24)**：`raw_odom_timeout_sec=0.25, stop_settle_sec=0.25` 在 prepare→activate 间隙中偶发 `raw_odom_not_stopped`。已调整为 `0.50/0.15`（`config/direction_guard.yaml`）。

## 任务与电源

- **任务起点航点 (2026-07-23 修复)**：`mission.py` 在构建导航段时跳过 `task=start` 的航点，避免 planner 对零长度路径 (0,0)→(0,0) 规划失败。发车前必须将车辆手动置于 P 点原点，车头朝 +X。
- **纯导航任务类型 `nav` (2026-07-23)**：`waypoints.py` 新增 `"nav"` 任务类型，可用于替代 `"qr"` 和 `"vlm"` 进行无视觉纯导航测试。`"nav"` 在状态机中等效于 qr/vlm 的位置约束，但不触发任何视觉服务调用，导航段不拆分直通下一航点。见 `nav_only.yaml`。
- **电压监控 (2026-07-24)**：`voltage_monitor` 工具订阅 `/PowerVoltage`（STM32 串口 byte 20-21，mV→V），记录到 `/tmp/voltage_history.log`（含时间戳，上限 10 万行自动轮转）。安全节点 `minimum_voltage: 10.0`（`safety.yaml`）——低于 10.0V 锁止运动。当前电池 3S 18650 LiPo，满电 12.6V。
