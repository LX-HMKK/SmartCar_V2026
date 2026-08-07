# 已知问题与避坑指南

> 源文件：`CLAUDE.md` → 本文件（低频查阅，为 CLAUDE.md 减负）
> 最后更新：2026-08-03

## Ubuntu 本机 Gazebo 仿真

- **DDS 环境不一致**：不同终端若加载不同的 Conda/Isaac overlay 或自定义
  `FASTRTPS_DEFAULT_PROFILES_FILE`，`ros2 node list` 可能为空，Nav2 lifecycle
  manager 也会持续等待 `controller_server/get_state`。统一使用
  `scripts/local_sim.sh` 或加载 `sim_env.sh`，不要注入自定义 DDS profile。
- **仿真残留进程**：日常使用 `sim_start.sh` 自动清理；手动恢复使用
  `sim_cleanup.sh --kill-processes`。完整说明见
  [`../deployment/local-simulation.md`](../deployment/local-simulation.md)。
- **Gazebo 与 RViz 感知不同步（2026-08-02 已加门禁）**：仿真现在以 Gazebo ground-truth
  `/odom` 为唯一 TF/里程计源，并在发车前检查 scan 时间戳可查询 TF、odom/TF 对齐、A 区
  锥桶回波以及 local/global raw costmap。检查结果发布到
  `/smartcar/sim_perception_ready` 并写进路线结果 JSON；未 ready 时不会发送目标。它证明
  仿真感知链已连通，不替代实体传感器验收。

## 硬件与传感器

- **LiDAR 物理朝向**：2D LiDAR 物理安装转了 90°（Y 朝后/X 朝左），`laser_yaw=1.5708` 已修复。若更换/重新安装 LiDAR 必须重新确认朝向。
- **`/ackermann_cmd` 直发不受支持**：实车必须经 `smartcar_safety` 发布 Ackermann 指令。保持 `use_safety_ackermann:=true`，不要停止安全节点或绕过方向门。
- **2D LiDAR 无法区分高度**：锥桶上窄下宽——雷达看到的是上半窄截面，车体下半蹭宽底。通过非对称 footprint（后轴原点）+ `obstacle_min_range=0.25` 滤车身 + 膨胀半径补偿，不能根本解决。物理上需 3D 感知或更谨慎的锥桶摆放。
- **转向 0.7 rad 命令被固件折算 (2026-08-02)**：`/ackermann_cmd` 发 `steering_angle=0.7` 时，下位机 `Move_Z` 显示 `0.322`（`MINI_AKM_MIN_TURN_RADIUS=0.35`）或 `0.453`（调低该常量后）——不是 ROS 丢值，而是 STM32 固件把协议帧 `tx[7:8]` 当角速度 ω、经 `Vz_to_Akm_Angle` 自行车模型反算前轮转角并被最小转弯半径钳制。修复：将固件 `Vz_to_Akm_Angle`（`HARDWARE/usartx.c`，用户本地 Keil 工程）改为直接透传 `return Vz;`，全固件 4 条接收路径（USART1/3/5 + CAN）同时生效；实车确认 `Move_Z=0.700`。0.7 rad 最大转向画圆卷尺实测 **R_min ≈ 0.2 m**（理论 0.09 m，工程偏差正常）。ROS 运行链已同步 `steering_command_scale=1.0`、offset `0.0` 和 `0.70 rad` 上限。完整推导见 `docs/review/steering-command-chain-fix-2026-08-02.md`。

## 导航与运动

- **Ackermann 终点兜圈**：无法原地旋转。当前全正向路线的 A 与 C1 使用精确终点树
  （`0.12 m / 0.15 rad`），回 P 使用常规 `goal_checker`（`0.35 m / 0.50 rad`）。本机 Gazebo
  已复核 C1 终态 `0.039 m / 0.113 rad`；这些配置尚未完成当前路线的实体到达验收。
- **P→A 前进碰撞恢复（2026-08-03）**：历史上 RPP 在高位紧右弯触及右侧
  `50 mm` 保护包络后，精确仿真树的内层 `FollowPath` recovery 只清 local costmap 并重发
  旧路径，导致控制器反复碰撞而没有及时取得新路径。现已移除该旧路径重试：碰撞或 patience
  超时会直接进入外层 recovery，依次清 local/global costmap、重新 `ComputePathToPose`，再
  重新跟随。两次定向 P→A Gazebo 运行均成功，末端位置误差均为 `0.102 m`，航向误差分别为
  `0.085 rad`、`0.073 rad`；第二次明确记录了碰撞超时、双 costmap 清理和新路径后成功到达。
  前进段不执行短退。后续完整三阶段 Gazebo 路线对应先前含倒车段的 YAML；该证据不覆盖当前全正向路线、实体传感器、方向门或底盘运动。
- **0.30 m/s 实体复验待完成**：运行配置已同步为 `0.30 m/s`，但尚无新的实体运行验收记录。原始日志表明 EKF 最严重的更新超期发生在路线启动前；`odom0_config` 也不融合原始 pose，因此旧版"IntegrationClock 导致 EKF pose/速度冲突"的推断已撤回，详见 `docs/review/odometry-speed-analysis.md`。
- **当前全正向路线 (2026-08-07)**：`default_waypoints.yaml` 与 `nav_only.yaml` 以四个 explicit planning segment 串联 14 个全正向航点约束；A/C1 使用 `precise` profile，P 使用 `standard` profile。离线几何预检及本机 Gazebo `run_20260807_170954_1` 均已通过，RDK 与实体车辆验收仍未完成，且 YAML 仍为 `calibrated: false`。
- **历史倒车导航实现 (2026-08-03，未激活)**：旧 QR→VLM 倒车路线使用 `navigate_to_pose_precise_w_replanning_and_recovery.xml`（`0.12 m / 0.15 rad` 精确 goal checker），普通 reverse 与锁定 yaw 的 `reverse_handoff` 使用 `ComputeReverseFreeHeadingPath*`：虚拟 yaw 加 π 调用唯一 DUBIN planner，恢复后严格验证。倒车多目标允许普通 `via` 后接一个作为末端、锁定航向的 `reverse_handoff`，并使用 reverse-locked ThroughPoses 树。`allow_reversing=true` 只允许沿既有反向路径输出负速度，不作为 `FollowPath` BT 端口；不会把 planner 改成 Reeds-Shepp，cusp 也会被拒绝。该链路及其 Gazebo 记录不验证当前全正向路线或实体运动。
- **方向门倒车转向翻转 (2026-07-24，未激活)**：历史实体链曾观察到倒车转向打反；`direction_guard` 的 `on_candidate()` 和 `evaluate()` 在 `MotionDirection::Reverse` 时翻转 `candidate[5]`（angular.z）后，同一首个倒车 goal 转向正确并成功。该行为是保留的反向执行器约定，不适用于当前全正向路线。

## Nav2 配置与行为树

- **Nav2 参数链路 (2026-07-23 修复)**：Nav2 1.1.20 (TROS Humble) 的 `RewrittenYaml` chain 存在 bug——`ParameterFile` 输出被 YDLIDAR 驱动参数覆盖，导致 `controller_server` 加载默认 DWB 而非 RPP。修复方案：`CMakeLists.txt` 自动生成 `nav2_params_fixed.yaml`（BT 路径硬编码），`navigation_launch.py` 直接使用该文件。任何对 `nav2_params.yaml` 的修改会在 `colcon build` 时自动同步到 fixed 文件。不可手动创建/编辑 `nav2_params_fixed.yaml`。
- **Nav2 启动入口收敛**：`navigation_launch.py` 直接启动 Nav2 节点，默认使用构建生成的 `nav2_params_fixed.yaml`。航点由任务节点加载；旧的 `use_waypoint_follower`、BT XML 覆盖和 Nav2 航点文件入口均不存在。需要仿真覆盖时仅传入已解析的 `params_overlay_file`。
- **行为树已移除 backup/wait recovery (2026-07-23)**：活动前进树不包含 `BackUp`、`Wait`、Spin
  或直接倒退恢复；前进碰撞经 local/global `ClearEntireCostmap` 后重新规划。仅未激活的反向规划候选在
  清图后仍穷尽时，才可在完整足迹检查通过后执行一次受限的 `AckermannReverseRetreat`。

## ROS2 生命周期与运维

- **`velocity_smoother` 生命周期卡死**：激活后偶发 `get_state` 服务无响应。先执行物理急停，再使用 `/usr/local/bin/ros_cleanup`（已部署时）或 `/root/ros2_ws/scripts/ros_cleanup.sh` 清理后重启。
- **重启必须彻底清理旧进程**：残留 launch 子进程会占用串口并消耗 CPU。清理脚本按 PID 终止 ROS 进程并保护 SSH 调用链，不能用 `pkill -f` 替代。
- **ROS2 CLI 卡死备用方案**：`ros2 service call` 在 lifecycle 异常后可能无限等待。紧急时先使用物理急停；随后运行 `/usr/local/bin/ros_cleanup`（已部署时）或 `/root/ros2_ws/scripts/ros_cleanup.sh`。

## 运维检查清单

### 构建后参数验证

`colcon build` 成功后，必须确认 `nav2_params_fixed.yaml` 中的关键参数：

```bash
grep motion_model_for_search /root/ros2_ws/install/smartcar_nav2/share/smartcar_nav2/config/nav2_params_fixed.yaml
grep allow_reversing /root/ros2_ws/install/smartcar_nav2/share/smartcar_nav2/config/nav2_params_fixed.yaml
grep current_goal_checker /root/ros2_ws/install/smartcar_nav2/share/smartcar_nav2/config/nav2_params_fixed.yaml
```

期望：`DUBIN`、`true`、`"goal_checker"`。负 `min_velocity` 和 `allow_reversing=true` 为保留的反向基础设施配置，当前活动路线不使用它们。

### 发车前检查

```bash
grep "Managed nodes are active" /tmp/bringup.log            # Nav2 生命周期全部 active
ros2 node list | grep -E "safety_node|task_node|controller_server|direction_guard"
ros2 service list | grep -E "smartcar/(safety/emergency_stop|task/start)"
```

### SSH 远程操作

- 使用现场 DHCP 地址 `root@<RDK_IP>`；`192.168.128.10` 仅为有线备用地址。
- 所有 ROS2 CLI 前必须 `source /root/source_env.sh`
- 后台启动用 `nohup` + 输出重定向，不依赖 SSH 会话保持
- 日志追踪优先于 topic echo（`tail -f /tmp/bringup.log | grep ...` 比 `ros2 topic echo` 更可靠）
- 关键 grep 关键词：`NavigateToPose`、`reached`、`ERROR`、`FAILED`、`reverse`

### 应急操作

| 优先级 | 操作 |
|--------|------|
| 1 | 物理急停按钮 |
| 2 | `ros2 service call /smartcar/safety/emergency_stop std_srvs/srv/SetBool '{data: true}'`（CLI 可用时） |
| 3 | `/usr/local/bin/ros_cleanup`（已部署时）或 `/root/ros2_ws/scripts/ros_cleanup.sh` |

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

- **TROS/ros_cleanup/Forward BT 三修复 (2026-07-24)**：① `set -euo pipefail` 脚本 source TROS setup.bash 前须包裹 `set +u`/`set -u`；② `ros_cleanup.sh` 的清理目标不得包含 `nav_test`（会自杀）；③ 标准前进树的 `FollowPath` 须指定 `goal_checker_id="goal_checker"`，P→A 精确树使用 `precise_goal_checker`。——均已修复并保持。
- **direction_guard 时序过紧 (2026-07-24)**：`raw_odom_timeout_sec=0.25, stop_settle_sec=0.25` 在 prepare→activate 间隙中偶发 `raw_odom_not_stopped`。已调整为 `0.50/0.15`（`config/direction_guard.yaml`）。

## 任务与电源

- **任务起点航点 (2026-07-23 修复)**：`mission.py` 在构建导航段时跳过 `task=start` 的航点，避免 planner 对零长度路径 (0,0)→(0,0) 规划失败。发车前必须将车辆手动置于 P 点原点，车头朝 +X。
- **纯导航任务类型 `nav` (2026-07-23)**：`waypoints.py` 新增 `"nav"` 任务类型，可用于替代 `"qr"` 和 `"vlm"` 进行无视觉纯导航测试。`"nav"` 在状态机中等效于 qr/vlm 的位置约束，但不触发任何视觉服务调用，导航段不拆分直通下一航点。见 `nav_only.yaml`。
- **电压监控 (2026-07-24)**：`voltage_monitor` 工具订阅 `/PowerVoltage`（STM32 串口 byte 20-21，mV→V），记录到 `/tmp/voltage_history.log`（含时间戳，上限 10 万行自动轮转）。安全节点 `minimum_voltage: 10.0`（`safety.yaml`）——低于 10.0V 锁止运动。当前电池 3S 18650 LiPo，满电 12.6V。
