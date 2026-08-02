# Ubuntu 本机 Gazebo 仿真运行手册

本文档说明如何在本仓库所在的 Ubuntu 22.04 开发机运行 `smartcar_sim`。仿真使用
ROS 2 Humble、Ignition Gazebo Fortress 6.18 和本机 RTX 图形驱动；不需要 WSL、
Windows 挂载目录或额外网络配置。

GPU 加速仅覆盖 Ignition 的 Ogre2 渲染和模型中的 `gpu_lidar` 传感器；本机为 RTX 4060。
Nav2 规划、行为树、costmap、控制器以及 Gazebo 物理仍主要使用 CPU，不是 CUDA 计算链。

`--headless` 仍会启用 Ignition 的 off-screen rendering。Fortress 6.18 的 GPU lidar
需要该渲染场景；不要把 server-only 模式改回只带 `-s` 的启动命令，否则扫描会退化为
最小量程，Nav2 会把它滤为车体自回波。

## 1. 安全边界

仿真复用真实 Nav2 planner、controller、BT 和航点，但使用 Gazebo 里程计，并将
`/cmd_vel_candidate` 桥接到 Gazebo 模型。它不会启动实体相机、串口底盘、RDK 驱动、
`direction_guard` 或 `smartcar_safety`。

默认只启动 Gazebo、TF、Nav2、场地参考和可选 RViz。只有显式传入
`run_route:=true` 时才会启动 `auto_train.py` 并向 Gazebo 模型发送非零速度。该参数
绝不能用于实车启动。

## 2. 首次初始化

在仓库根目录运行：

```bash
cd /home/zyh/SmartCar_V2026
bash scripts/setup_local_sim.sh
```

脚本安装 Gazebo Fortress、ROS-Gazebo bridge、`robot_localization`、Qt5、Noto CJK
字体和仿真依赖，并构建到 `smartcar_sim`。它会清理 Conda/Isaac 的库路径，避免错误
加载图形库。

## 3. 日常启动

推荐使用本机入口：

```bash
cd /home/zyh/SmartCar_V2026
bash scripts/local_sim.sh --headless --rviz
```

可用选项：

| 选项 | 作用 |
|---|---|
| `--headless` | 不打开 Gazebo GUI，但保留 off-screen 渲染以驱动 GPU lidar，适合日常 Nav2 调试。 |
| `--gui` | 同时打开 Gazebo GUI。 |
| `--rviz` | 打开 RViz。 |
| `--no-rviz` | 不启动 RViz。 |
| `--no-clean` | 跳过启动前清理，仅用于确认环境干净的调试。 |

入口会加载 `rmw_fastrtps_cpp` 和 `ROS_LOCALHOST_ONLY=1`、清理残留 Gazebo/Nav2/DDS
资源，并设置模型资源路径。不要额外设置 `FASTRTPS_DEFAULT_PROFILES_FILE`、
`FASTDDS_DEFAULT_PROFILES_FILE` 或 `CYCLONEDDS_URI`。

## 4. 验证

在另一个终端执行：

```bash
cd /home/zyh/SmartCar_V2026
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/smartcar_sim/share/smartcar_sim/scripts/sim_env.sh

ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother
timeout 4s ros2 run tf2_ros tf2_echo \
  odom_combined origincar/laser_link/lidar_sensor
ros2 topic echo /smartcar/sim_perception_ready --once \
  --qos-durability transient_local
```

四个 Nav2 lifecycle 节点应为 `active [3]`。RViz 固定坐标系为 `odom_combined`；
`Accepted Global Path (/smartcar/accepted_global_plan)` 是控制器确认接收的完整原始路径，
`Controller Tracking Window (/transformed_global_plan)` 是它当前的跟踪窗口。`/plan`
包含 free-heading 搜索的候选（包括被拒绝的候选），默认隐藏，不能作为车辆会执行的路线。
`Waypoint Reference Constraints (not a Nav2 path)` 仅是 YAML 航点参考。
`/smartcar/sim_perception_ready` 是 transient-local 的 `std_msgs/String` JSON。只有
`ready=true`，且 `checks.scan`、`odom`、`tf`、`local`、`global` 和 `a_zone_probe` 都为
`true` 时，才证明新鲜 `/scan` 能在其时间戳查询到 TF，且默认 A 区锥桶回波已同时标入
local/global `costmap_raw`。消息还记录有效波束数和 scan、odom、两张 costmap 的时间戳。
当前 Nav2 版本可能将 `metadata.update_time` 保持为零，诊断以较新的 header/update 时间
为准。

启用 `run_route:=true` 时，`auto_train` 必须在发送首个 Nav2 目标前取得这一完整状态，并
将快照写入结果 JSON 的 `perception` 字段。未就绪会直接失败，不会发送导航目标。它不参与
实车安全门或运动命令。

## 5. 航点编辑

本机点位编辑默认只打开 Matplotlib 分段编辑器，不拉起 Gazebo、Nav2、RViz 或安全节点：

```bash
cd /home/zyh/SmartCar_V2026
bash scripts/local_waypoint_editor.sh
```

需要 RViz 对照时追加 `use_rviz:=true`；需要与已运行仿真共享 `/clock` 时追加
`use_sim_time:=true`。完整操作说明见
[`waypoint-editor.md`](waypoint-editor.md)。

## 6. 自动路线和调参

当前 `nav_only.yaml` 只保留三个语义阶段：P→A 前进、A→C1 倒车、C1→P 前进，未使用
中间经过点。最新 P→A 复验的全局路径长 `4.270 m`、弦长 `3.276 m`、倍率 `1.303`，不含
大绕圈；但执行在高位紧右弯的横向反馈过冲到右侧保护包络，约行驶 `1.587 m` 后停在
`(0.533, 1.255)`，距 A `2.609 m`。因此路线仍未通过，不能通过增加经过点、放宽保护包络
或把 Gazebo 结果写成实体车辆验收来规避该问题。

运行路线会让 Gazebo 模型运动：

```bash
bash scripts/local_sim.sh --headless --rviz -- run_route:=true
```

调参脚本会重生成 keepout PGM、同步 Nav2 参数、构建并运行完整 Gazebo 路线：

```bash
python3 src/smartcar_sim/scripts/tune_params.py
bash src/smartcar_sim/scripts/sim_tune.sh --headless --loop 1
```

Gazebo 最小转弯半径由
`src/smartcar_tools/config/routes/route_planning.yaml` 的
`simulation_minimum_turning_radius_m` 单独管理，当前为 `0.22 m`；执行
`sim_tune.sh` 会将它同步到仿真 overlay。`tune_params.py` 不会修改实体
`nav2_params.yaml` 中同样为 `0.22 m`、但独立维护的运行基线。

两者默认只操作当前仓库；如从其他工作区调用，设置 `SMARTCAR_WS=/absolute/path/to/workspace`。
调参运行的结果 JSON 才能作为路线验收依据，离线几何预检不能替代运行时证据。

## 7. 清理与故障排查

异常退出或重建前执行：

```bash
cd /home/zyh/SmartCar_V2026
bash src/smartcar_sim/scripts/sim_cleanup.sh --kill-processes
```

| 现象 | 处理 |
|---|---|
| Gazebo/RViz 加载系统库失败 | 使用 `scripts/local_sim.sh` 或 `scripts/local_waypoint_editor.sh`，不要直接继承 Conda/Isaac 环境。 |
| `ros2 node list` 为空或 lifecycle 卡住 | 清理残留进程和 DDS 资源后重启，并确认所有终端加载相同的 `sim_env.sh`。 |
| RViz 不显示执行路径 | 使用仓库内 `sim_nav.rviz`；`/smartcar/accepted_global_plan` 的 Path QoS 必须为 `Reliable + Transient Local`，`/transformed_global_plan` 必须为 `Reliable + Volatile`。 |
| Gazebo 有进程但无 `/odom` | 运行清理脚本后重启，检查 bridge 日志和 `/tmp` 中残留资源。 |
