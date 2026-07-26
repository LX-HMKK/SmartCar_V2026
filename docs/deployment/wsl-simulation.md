# WSL2 Gazebo 仿真运行手册

本文档说明如何在 Windows 11 + WSL2 Ubuntu-22.04 中运行
`smartcar_sim`。仿真环境为 ROS2 Humble、Ignition Gazebo 6.18 和 WSLg。

## 1. 适用范围与安全边界

仿真复用真实 Nav2 planner、controller、BT 和航点，但使用 Gazebo 里程计，
并把 `/cmd_vel_candidate` 直接桥接到 Gazebo `/cmd_vel`。仿真不会启动实体
相机、串口底盘或 RDK 驱动，也不会经过实车 `direction_guard` 和
`smartcar_safety`。

`sim.launch.py` 会在启动约 20 秒后运行 `auto_train.py`，向 Gazebo 模型发送
非零速度，依次测试 P->QR 正向导航和 QR->VLM 导航。该入口只能在 WSL
仿真工作区中使用，不得部署为实车启动入口。

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

### 自动导航

```bash
cat /tmp/auto_train_results.json
```

验收要求：P->QR 和 QR->VLM 都为 `outcome: done`，单段耗时小于 60 秒。

2026-07-26 在 WSL 2.6.3、Ubuntu-22.04、Gazebo 6.18 上的验证结果：

- mirrored：Fast DDS daemon 查询在 5 秒限制内超时。
- NAT：Fast DDS 双 participant discovery 约 0.9 秒。
- `ros2 node list`：约 1.2-1.35 秒。
- Gazebo world：10 秒内完成初始化。
- Nav2：20 秒内全部 active。
- P->QR：约 22 秒完成。
- QR->VLM：约 48 秒完成。
- RViz/TF：窗口正常，激光帧可从 `odom_combined` 查询，日志无 transform 错误。

## 7. 症状速查

| 症状 | 原因 | 处理 |
|---|---|---|
| `sim_start.sh` 报 `loopback0 exists` | WSL 使用 mirrored 网络 | 改为 NAT，执行 `wsl.exe --shutdown` |
| `ros2 node list` 为空或卡死 | shell 环境不一致或残留 DDS profile | source `sim_env.sh`，清理后重启 |
| lifecycle manager 一直等 `get_state` | participant discovery 分裂或旧节点残留 | 确认 NAT，删除 DDS profile 环境变量，运行 `sim_cleanup.sh --kill-processes` |
| RViz 不弹窗 | WSLg socket 尚未创建 | 使用 `sim_start.sh`；检查 `/mnt/wslg/runtime-dir/wayland-0` |
| RViz 报激光 TF 缺失 | 安装空间未更新或静态 TF 方向错误 | 增量构建 `smartcar_sim`，确认 `laser_link` 是 Gazebo sensor frame 的父帧 |
| Gazebo 有进程但无 `/odom` | bridge 或旧 Gazebo 实例冲突 | 完整清理后重启，检查 `/tmp` 和 DDS SHM 残留 |
