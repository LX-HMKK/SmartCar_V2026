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

安装后的 `smartcar_sim` 只包含启动必需的 launch、模型、配置、地图与
`sim_start.sh`/`sim_env.sh`/`sim_cleanup.sh`。地图生成、调参和结果校验保持为
源码工具，请按本手册后续命令从 `src/smartcar_sim/scripts/` 调用，不作为已安装
运行时入口。

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

本机 Gazebo 仿真固定使用 `0.30 m/s` 线速度上限；当前全正向路线和
`velocity_smoother` 使用同一档位。为保持 `0.22 m` 最小转弯半径，角速度上限为
`0.30 / 0.22 = 1.363636 rad/s`。`sim_speed_profile` 以及旧的 `0.15`、`0.20`、
`0.25 m/s` 速度档位已移除。

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

当前 `nav_only.yaml` 有四个显式动作，且全部为正向：P→A、A 经
`a_departure_exit`/`via_2_entry`/`via_2_corridor`→`via_2`、`via_2`→C1、C1 经
`c_north_1`/`c_north_2`/`via_1`/`via_3`/`return_corridor_exit`/`p_return_approach`→P。
`via` 是普通无朝向经过约束；A 与 C1 是锁定航向的 `precise` 单目标，`p_finish` 是
`standard` 终点。两条含经过点的动作使用常规正向
`navigate_through_poses_w_replanning_and_recovery.xml`；不使用 `reverse_handoff`、反向
ThroughPoses 树或短退恢复。反向候选、足迹扫掠和 `AckermannReverseRetreat` 仍是通用反向
基础设施，但当前活动路线不会调用它们。

P→A 与 `via_2`→C1 使用精确前进树。RPP 碰撞或 controller patience 超时时，`FollowPath` 会直接进入外层
recovery：清 local/global costmap、重新 `ComputePathToPose`，再发送新的控制器路径；不能先清
local costmap 后重发旧 `{path}`。当前全路线已由 `run_20260807_170954_1` 完成校验：四个动作均
`succeeded`，A/C1/P 的终态误差分别为 `0.053 m / 0.141 rad`、`0.039 m / 0.113 rad`、
`0.312 m / 0.350 rad`。该证据覆盖本机 Gazebo、感知门禁和正向控制命令；仿真 PGM 仅作 RViz
参考，不参与 Nav2 costmap 的 KeepoutFilter 约束。任何
Gazebo 结果均不构成实体车辆验收。

运行路线会让 Gazebo 模型运动：

```bash
bash scripts/local_sim.sh --headless --rviz -- run_route:=true
```

调参脚本会重生成 keepout PGM、同步 Nav2 参数、构建并运行完整 Gazebo 路线：

```bash
python3 src/smartcar_sim/scripts/tune_params.py
bash src/smartcar_sim/scripts/sim_tune.sh --headless --loop 1
```

要同时覆盖深度 PointCloud2 障碍层，显式传入
`--use-depth-obstacles`：

```bash
bash src/smartcar_sim/scripts/sim_tune.sh --headless --use-depth-obstacles --loop 1
```

该选项只在 Gazebo 中将模拟 `/scan` 转换为
`/smartcar/depth/points` 并替换 Nav2 的障碍物观测源。它不启用实体深度相机，也不改变实体
导航参数。

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
