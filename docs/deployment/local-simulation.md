# Ubuntu 本机 Gazebo 仿真运行手册

本文档说明如何在本仓库所在的 Ubuntu 22.04 开发机运行 `smartcar_sim`。仿真使用
ROS 2 Humble、Ignition Gazebo Fortress 6.18 和本机 RTX 图形驱动；不需要 WSL、
Windows 挂载目录或额外网络配置。

GPU 加速仅覆盖 Ignition 的 Ogre 渲染和模型中的 `gpu_lidar` 传感器；本机为 RTX 4060。
Nav2 规划、行为树、costmap、控制器以及 Gazebo 物理仍主要使用 CPU，不是 CUDA 计算链。

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
| `--headless` | Gazebo server-only，适合日常 Nav2 调试。 |
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
```

四个 Nav2 lifecycle 节点应为 `active [3]`。RViz 固定坐标系为 `odom_combined`；
`Actual Nav2 Global Path (/plan)` 和 `Actual Nav2 Local Path (/local_plan)` 是 Nav2
真实路径，`Waypoint Reference Constraints (not a Nav2 path)` 仅是 YAML 航点参考。

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

当前 `nav_only.yaml` 是五段路线：P→A 前进，其余四段倒车。当前证据只覆盖
Gazebo/RViz/TF/Nav2 启动、P→A 和 A→B；B→C1 在 C 区中央禁区西南角仍会触发局部
碰撞预测，不能标记为全程通过。

运行路线会让 Gazebo 模型运动：

```bash
bash scripts/local_sim.sh --headless --rviz -- run_route:=true
```

调参脚本会重生成 keepout PGM、同步 Nav2 参数、构建并运行完整 Gazebo 路线：

```bash
python3 src/smartcar_sim/scripts/tune_params.py
bash src/smartcar_sim/scripts/sim_tune.sh --headless --loop 1
```

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
| RViz 不显示 `/plan` | 使用仓库内 `sim_nav.rviz`，两个 Path display 的 QoS 必须是 `Volatile`。 |
| Gazebo 有进程但无 `/odom` | 运行清理脚本后重启，检查 bridge 日志和 `/tmp` 中残留资源。 |
