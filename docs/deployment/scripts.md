# `scripts/` 使用手册

本页只说明仓库根目录 `scripts/` 的入口。默认在本机仓库根目录
`/home/zyh/SmartCar_V2026` 执行；RDK 命令会单独标出。

## 本机仿真与编辑

### `setup_local_sim.sh`

首次配置 Ubuntu 22.04 本机仿真环境。它会执行 `sudo apt-get`、`rosdep` 和仿真依赖的
`colcon build`，不连接 RDK 或实体设备。

```bash
bash scripts/setup_local_sim.sh
```

### `local_sim.sh`

启动本机 Gazebo、Nav2 和可选 RViz。默认是 headless 加 RViz，且默认不运行路线。

```bash
bash scripts/local_sim.sh --headless --rviz
bash scripts/local_sim.sh --gui --rviz
bash scripts/local_sim.sh --headless --no-rviz
```

`--no-clean` 保留已有本机 ROS/Gazebo 会话，仅用于已自行隔离 DDS/Gazebo 的调试。
其余 ROS launch 参数会原样转发；只有显式传入 `run_route:=true` 才会让 Gazebo 模型运动：

```bash
bash scripts/local_sim.sh --headless --rviz run_route:=true
```

这只影响仿真模型，不能连接 RDK 或实体底盘。

### `local_waypoint_editor.sh`

启动本机、无底盘的 `nav_only.yaml` 航点编辑器；默认不打开 RViz：

```bash
bash scripts/local_waypoint_editor.sh
bash scripts/local_waypoint_editor.sh use_rviz:=true
```

任何实际航点修改仍必须先在 RViz 展示并取得该次明确同意，然后同步两份路线 YAML；详见
[航点编辑与授权](waypoint-editor.md)。该脚本不启动 Nav2、相机或实体运动链。

## RDK 同步与构建

以下本机命令的目标由 `SMARTCAR_RDK_HOST` 决定。当前无线地址：

```bash
export SMARTCAR_RDK_HOST=root@172.16.24.170
```

### `sync_to_rdk.py`

底层同步工具。`push` 会镜像本机 `src/`、`config/` 和运行脚本到 RDK，默认带
`--delete`；先运行 dry-run。它本身不构建、不启动相机或车辆。

```bash
python3 scripts/sync_to_rdk.py push --dry-run
python3 scripts/sync_to_rdk.py push
python3 scripts/sync_to_rdk.py setup
```

`setup` 将 `scripts/source_env.sh` 写到 RDK 的 `~/source_env.sh`。其他低频子命令：

```bash
python3 scripts/sync_to_rdk.py pull --dry-run
python3 scripts/sync_to_rdk.py pull
python3 scripts/sync_to_rdk.py pull-waypoints --dry-run
python3 scripts/sync_to_rdk.py pull-waypoints
python3 scripts/sync_to_rdk.py init-vendor
```

`pull` 默认不删除本机文件；`pull --delete` 会删除本机不在 RDK 的文件，除非明确需要对齐
RDK 否则不要使用。`pull-waypoints` 只用于获准的现场航点回传，回传后必须审查并按双 YAML
合同同步。`init-vendor` 只用于首次初始化或重新获取官方 `origincar` 源码。

### `nav_deploy.sh`

日常导航代码或配置更新的本机入口。它会先在 RDK 的 `/root/nav_backups/` 备份 `src/config`，
再同步，并远程执行 `nav_prepare.sh`。不启动实体相机或车辆。

```bash
bash scripts/nav_deploy.sh --dry-run
bash scripts/nav_deploy.sh
```

`--dry-run` 仅预览同步，不备份、不清理、不构建。正常运行会修改 RDK，因此先确认
`SMARTCAR_RDK_HOST` 正确且 RDK 本地修改已回传或备份。

### `nav_prepare.sh`

RDK 上的准备入口，部署后路径为 `/root/nav_prepare.sh`。它会停止当前导航栈、增量构建到
`smartcar_bringup` 和 `smartcar_vision`，并写入构建指纹：

```bash
ssh "$SMARTCAR_RDK_HOST"
bash /root/nav_prepare.sh
```

通常通过 `nav_deploy.sh` 间接调用；只有在 RDK 已同步但需要重新准备时才直接执行。它不启动
相机或车辆，但会终止正在运行的导航栈。

### `source_env.sh`

RDK 环境定义文件，不直接执行。`sync_to_rdk.py setup` 将它部署为 `~/source_env.sh`；在 RDK
终端加载环境时使用：

```bash
source ~/source_env.sh
cd /home/sunrise/ros2_ws
```

## RDK 启动、状态与停止

### `nav_test.sh`

部署后位于 `/home/sunrise/ros2_ws/scripts/nav_test.sh`；`/root/nav_test.sh` 是同一版本的兼容入口。
固定启动 Aurora 深度、`pointcloud_to_laserscan` 和双 costmap。Nav2 节点先启动，生命周期激活会等待
Aurora 初始化稳定；核心状态正常后才启动 RViz。普通启动保持软件急停锁存：

```bash
bash /home/sunrise/ros2_ws/scripts/nav_test.sh
```

它会启动实体 Aurora 相机，因此需要本次实体相机授权。以下两个命令还会在状态检查后复位、解除
软件急停并发车，必须先取得本次非零运动授权，确认物理急停可用，并将车人工放在 P 原点、车头朝
`+X`：

```bash
bash /home/sunrise/ros2_ws/scripts/nav_test.sh --go
bash /home/sunrise/ros2_ws/scripts/nav_test.sh --go --wheel-only
bash /home/sunrise/ros2_ws/scripts/nav_test.sh --go --c-zone-direction=clockwise
```

`--wheel-only` 仅将 EKF 切到轮式里程计对照配置；默认始终是 `wheel_imu`。
`--c-zone-direction` 在启动时一次性选择 C 区方向：默认 `counterclockwise` 使用已确认
路线；`clockwise` 仅在内存中按 C 区 `x=2.0 m` 中线镜像已确认约束，YAML、航点 ID、顺序和
规划分段不变。RViz 与任务节点使用同一选择；运行中不会切换。

### `nav_status.py`

`nav_test.sh` 自动调用的核心启动健康检查器。它检查 Nav2 lifecycle、任务状态、急停、点云、深度
scan 和双 costmap；RViz 在该检查成功后启动。通常无需单独运行。需要人工诊断已启动栈时：

```bash
python3 /root/nav_status.py --depth-camera --timeout 60
```

可额外传入 `--launch-pid <pid>` 和 `--rviz-pid <pid>`，使检查同时验证对应进程仍在运行。

### `ros_cleanup.sh`

RDK 清理入口：

```bash
bash /home/sunrise/ros2_ws/scripts/ros_cleanup.sh
```

它先结束 `nav_test.sh` 记录的导航/RViz PID，再清理已知的导航残留节点、该栈的静态 TF 与可视化
节点；随后只删除无 DDS 进程占用的 Fast DDS 端口文件。不会使用宽泛 `pkill -f`，也不会清理 ROS
daemon 或航点编辑器。异常时先使用物理急停，再运行该命令。
