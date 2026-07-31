# WSL2 Gazebo 仿真运行手册

本文档说明如何在 Windows 11 + WSL2 Ubuntu-22.04 中运行
`smartcar_sim`。仿真环境为 ROS2 Humble、Ignition Gazebo 6.18 和 WSLg。

## 1. 适用范围与安全边界

仿真复用真实 Nav2 planner、controller、BT 和航点，但使用 Gazebo 里程计，
并把 `/cmd_vel_candidate` 直接桥接到 Gazebo `/cmd_vel`。仿真不会启动实体
相机、串口底盘或 RDK 驱动，也不会经过实车 `direction_guard` 和
`smartcar_safety`。

`sim.launch.py` 只有在显式传入 `run_route:=true` 时，才会在启动约 20 秒后运行
`auto_train.py`。该 runner 从 `nav_only.yaml` 读取全部 10 个非起点航点，按
`forward`、`precise`、`reverse` 选择对应 BT，并向 Gazebo 模型发送非零速度。
该入口只能在 WSL 仿真工作区中使用，不得部署为实车启动入口。

## 2. WSL 网络配置

仿真要求 WSL2 使用 NAT 网络。当前 Windows/WSL 版本的 mirrored 模式会创建
额外的 `loopback0` 和策略路由，导致 Fast DDS participant 发现不完整：
`ros2 node list` 可能为空或经 daemon 卡死，Nav2 lifecycle manager 也可能持续
等待 `controller_server/get_state`。

编辑 Windows `%USERPROFILE%\.wslconfig`：

```ini
[wsl2]
networkingMode=nat
dnsTunneling=true
autoProxy=true
```

应用配置：

```powershell
wsl.exe --shutdown
```

重新进入 Ubuntu 后检查：

```bash
ip -br address
ip route show
```

正常 NAT 环境只有 `lo` 和一个 NAT 网段的 `eth0`，不应存在 `loopback0`。
`sim_start.sh` 会主动检测 `loopback0`；发现 mirrored 模式时直接退出并提示修改
`.wslconfig`。

仿真统一使用：

```text
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_LOCALHOST_ONLY=1
```

不要设置 `FASTRTPS_DEFAULT_PROFILES_FILE`、`FASTDDS_DEFAULT_PROFILES_FILE` 或
`CYCLONEDDS_URI`。NAT + `ROS_LOCALHOST_ONLY=1` 使用默认 Fast DDS transport 即可；
旧 loopback XML 会让多进程 discovery 分裂成互相不可见的节点集合。

## 3. 构建

Windows 仓库的 `src/` 是权威源。同步到 WSL `/root/ros2_ws/src` 后构建：

```bash
source /opt/ros/humble/setup.bash
cd /root/ros2_ws
colcon build --symlink-install \
  --packages-up-to smartcar_sim \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source /root/ros2_ws/install/setup.bash
```

只修改 `smartcar_sim` 时可使用 `--packages-select smartcar_sim` 增量构建。

## 4. 推荐启动方式

在 Windows PowerShell 中启动 WSL 命令：

```powershell
wsl.exe -d Ubuntu-22.04 -u root bash -lc '
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
exec /root/ros2_ws/install/smartcar_sim/share/smartcar_sim/scripts/sim_start.sh --headless --rviz
'
```

`sim_start.sh` 会完成以下步骤：

1. 加载统一 Fast DDS、localhost、WSLg 和 Gazebo model 环境。
2. 检查 WSL 是否误用 mirrored 网络。
3. 按 PID 清理残留 Gazebo、RViz、Nav2、ROS daemon 和 DDS 临时文件。
4. 启动 `sim.launch.py`。
5. RViz 子进程等待 WSLg Wayland socket 就绪后再启动。

可用选项：

| 选项 | 作用 |
|---|---|
| `--headless` | Gazebo server-only，默认值 |
| `--gui` | 同时启动 Gazebo GUI |
| `--rviz` | 启动 RViz，默认值 |
| `--no-rviz` | 不启动 RViz |
| `--no-clean` | 跳过进程清理，仅用于已确认环境干净的调试 |

启动完整自动路线时，在 `sim_start.sh` 后追加 `run_route:=true`；不传该参数时
仿真只启动 Gazebo、里程计、Nav2 和 RViz，不会自动发导航目标。

直接调用 `ros2 launch smartcar_sim sim.launch.py` 时不会执行启动前的进程清理，
因此日常运行优先使用 `sim_start.sh`。

## 5. 手动清理

仿真异常退出或需要重新构建前执行：

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
bash /root/ros2_ws/install/smartcar_sim/share/smartcar_sim/scripts/sim_cleanup.sh \
  --kill-processes
```

脚本保护自身和父进程，不使用会匹配调用 shell 的宽泛 `pkill -f`。若 WSLg 或
网络栈本身异常，再从 PowerShell 执行 `wsl.exe --shutdown`。

## 6. 验证

另开一个 WSL shell，并加载与仿真一致的环境：

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
source /root/ros2_ws/install/smartcar_sim/share/smartcar_sim/scripts/sim_env.sh
```

### DDS 与 Nav2

```bash
timeout 3s ros2 node list --no-daemon
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother
```

`ros2 node list` 必须在 3 秒内返回，四个 lifecycle 节点必须为 `active [3]`。
启动日志应在 20 秒内包含 `Managed nodes are active`。

### TF 与 RViz

```bash
test -S /mnt/wslg/runtime-dir/wayland-0 && echo "WSLg ready"
timeout 4s ros2 run tf2_ros tf2_echo \
  odom_combined origincar/laser_link/lidar_sensor
```

RViz 使用 `odom_combined` fixed frame，激光 TF 链为：

```text
odom_combined -> base_footprint -> base_link -> laser_link
              -> origincar/laser_link/lidar_sensor
```

仿真 RViz 中，`Actual Nav2 Global Path (/plan)` 是 Nav2 实际发布的全局路径，
`Actual Nav2 Local Path (/local_plan)` 是控制器实际使用的局部路径。`Waypoint
Reference Constraints (not a Nav2 path)` 仅连接 YAML 中的任务航点，不能用于判断
Nav2 实际规划或可达性。`Official Field Reference` 的黄色 C 区双环是场地参考层，
不是任一 Nav2 路径；判断时以 RViz Display 的名称和 Topic 为准，不以颜色为准。

### 自动导航

```bash
rm -f /tmp/auto_train_results.json
ros2 launch smartcar_sim sim.launch.py \
  headless:=false use_rviz:=true run_route:=true
cat /tmp/auto_train_results.json
```

默认的 `nav_only.yaml` 是当前保存的仿真路线。需要试验路线时，通过
`waypoints_file:=/absolute/path/to/trial.yaml` 显式传入，避免覆盖该文件。自动路线退出
后，Gazebo 与 RViz 默认继续运行，便于检查最终 `/plan`、`/local_plan`、代价地图和
车位；CI 如需原来的一次性退出行为，追加 `shutdown_on_route_exit:=true`。

仓库保留两份未验证的 B→C1 入口候选，均通过航点与分段结构校验，但尚无
`overall_outcome=completed` 运行证据：

- `src/smartcar_nav2/config/waypoints/nav_only.b-yaw-trial.yaml`：仅将
  `b_corridor_enter` 朝向改为 `-15 deg`。
- `src/smartcar_nav2/config/waypoints/nav_only.reverse-c-entry-split-trial.yaml`：保持
  当前五阶段分段，将 B 点朝向改为 `-30 deg`、C1 朝向改为 `-155 deg`。

它们只能以显式 `waypoints_file` 用于隔离仿真，不得替代正式 `nav_only.yaml` 或作为
路线通过证据。

当前路线包含五段：`p_to_qr` 正向，`reverse_corridor`、`reverse_c_entry`、`c_exit`
和 `return_to_p` 均倒车。验收要求：JSON 的 `overall_outcome` 为 `completed`，包含
全部 9 个非起点航点，每个 `outcome` 为 `succeeded`。`behavior_tree` 必须与航点方向
一致；两层速度命令必须符合航向符号。

截至 2026-07-30，当前保存路线的 `b_corridor_enter -> c_corner_1` 在 C 区中央禁区
西南角仍没有运行时安全余量：最近一轮仿真先完成前两段，随后被局部代价地图的碰撞预测
中止。该路线不能标注为全程仿真通过；调整该入口前，必须继续在隔离试验 YAML 上验证。

### 仿真调参

```bash
python3 /root/ros2_ws/src/smartcar_sim/scripts/tune_params.py
python3 /root/ros2_ws/src/smartcar_sim/scripts/tune_params.py \
  --sweep precise_yaw_goal_tolerance
bash /root/ros2_ws/src/smartcar_sim/scripts/sim_tune.sh --headless --loop 1
```

调参工具和 `sim_tune.sh` 默认都只操作 `${SMARTCAR_WS:-/root/ros2_ws}/src`；
它们不会自动用 Windows 挂载目录覆盖 WSL 中已保存的航点。若确实要导入 Windows
源码，必须显式执行 `sim_tune.sh --sync-from-windows`（可用
`--windows-src <path>` 或 `SMARTCAR_WINDOWS_SRC` 指定来源）。同步前脚本会比较
两个 `nav_only.yaml` 的 SHA-256；内容不同时，先备份 WSL 版本到
`$SMARTCAR_TUNE_LOG_DIR/manual_backups/nav_only.before-windows-sync-*.yaml`。
同一显式同步也会对不同的 `nav2_params.yaml` 创建备份，避免调参参数被静默覆盖。
显式同步不会使用 `rsync --delete`。每次运行仍会构建 `smartcar_nav2` 的 fixed
参数文件；`--no-build` 仅保留为兼容参数但会被忽略。扫参只生成备份快照，不会把最后一个
候选值留在源文件中。QR 相关参数对应 `precise_goal_checker`。倒车曲率约束来自
`ReverseHandoff.AckermannConstraints.min_turning_r`、controller wrapper 输出门和
velocity smoother 的同比例缩放，不是 reverse BT 发布的普通 YAML 参数。

```bash
# 默认：使用并保留 WSL 中已保存的点位。
bash /root/ros2_ws/src/smartcar_sim/scripts/sim_tune.sh --headless --loop 1

# 只有明确需要用 Windows 源码替换 WSL 源码时才执行。
bash /root/ros2_ws/src/smartcar_sim/scripts/sim_tune.sh \
  --sync-from-windows --headless --loop 1
```

航点几何、C 区中央禁区、最小转弯半径和仿真 KeepoutFilter 包络统一由
`src/smartcar_tools/config/routes/route_planning.yaml` 管理。修改后使用
`sim_tune.sh` 重生 PGM、同步 Nav2 参数并运行仿真；它不是 SLAM 或定位调参。编辑
流程、参数含义和 WSL 用户路径示例见 [`waypoint-editor.md`](waypoint-editor.md)。

2026-07-30 在 WSL 2.6.3、Ubuntu-22.04、Gazebo 6.18 上的当前证据：

- mirrored：Fast DDS daemon 查询在 5 秒限制内超时。
- NAT：Fast DDS 双 participant discovery 约 0.9 秒。
- `ros2 node list`：约 1.2-1.35 秒。
- Gazebo world：10 秒内完成初始化。
- Nav2：20 秒内全部 active。
- 首段 P→A 和 A→B 的倒车段可以启动并完成；当前 all-reverse 配置随后在
  `b_corridor_enter -> c_corner_1` 入口处触发局部碰撞预测。
- C1-C4 坐标、P/A/终点位置不得为了绕开该问题直接移动。后续只在隔离试验 YAML 中
  调整允许的 B 区点位或航向，并以 `overall_outcome=completed` 作为全程通过证据。
- RViz/TF：窗口正常，激光帧可从 `odom_combined` 查询，日志无 transform 错误。

## 7. 症状速查

| 症状 | 原因 | 处理 |
|---|---|---|
| `sim_start.sh` 报 `loopback0 exists` | WSL 使用 mirrored 网络 | 改为 NAT，执行 `wsl.exe --shutdown` |
| `ros2 node list` 为空或卡死 | shell 环境不一致或残留 DDS profile | source `sim_env.sh`，清理后重启 |
| lifecycle manager 一直等 `get_state` | participant discovery 分裂或旧节点残留 | 确认 NAT，删除 DDS profile 环境变量，运行 `sim_cleanup.sh --kill-processes` |
| RViz 不弹窗 | WSLg socket 尚未创建 | 使用 `sim_start.sh`；检查 `/mnt/wslg/runtime-dir/wayland-0` |
| RViz `/plan` 不显示 | RViz 使用 `TRANSIENT_LOCAL`，而 Nav2 publisher 是 `VOLATILE` | 使用仓库内最新 `sim_nav.rviz`，确认两个 Path display 都是 `Volatile` |
| RViz 报激光 TF 缺失 | 安装空间未更新或静态 TF 方向错误 | 增量构建 `smartcar_sim`，确认 `laser_link` 是 Gazebo sensor frame 的父帧 |
| Gazebo 有进程但无 `/odom` | bridge 或旧 Gazebo 实例冲突 | 完整清理后重启，检查 `/tmp` 和 DDS SHM 残留 |
