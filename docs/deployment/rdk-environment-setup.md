# RDK 部署与现场流程

本机 `src/` 与 `config/` 是权威源。本页只记录当前 RDK 操作；没有用户的明确授权，不得同步
RDK、启动实体相机、解除急停、发布非零速度或进行实车运动。

## 连接信息

| 网络 | SSH 目标 | 说明 |
| --- | --- | --- |
| 有线 | `root@192.168.128.10` | 同步工具的默认目标。 |
| 无线 | `root@172.16.24.170` | 当前 DHCP 地址，无固定 IP；变更后更新环境变量。 |

```bash
# 有线
ssh root@192.168.128.10

# 无线：当前 DHCP 地址；变更后替换该 IP
export SMARTCAR_RDK_HOST=root@172.16.24.170
ssh "$SMARTCAR_RDK_HOST"
```

RDK 工作空间为 `/home/sunrise/ros2_ws`，环境入口为 `~/source_env.sh`。迁移前的
`/root/ros2_ws` 仅保留作回退副本，不再作为部署或运行目标。进入 RDK 后先执行：

```bash
source ~/source_env.sh
```

## 本机同步

`push` 会将本机的 `src/` 与 `config/` 镜像到 RDK，默认包含 `--delete`。RDK 有本地修改时，
先回传或备份，再预览并同步：

```bash
cd /home/zyh/SmartCar_V2026
python3 scripts/sync_to_rdk.py pull --dry-run       # 有必要时先检查 RDK 修改
python3 scripts/sync_to_rdk.py push --dry-run
python3 scripts/sync_to_rdk.py push
python3 scripts/sync_to_rdk.py setup                # 更新 ~/source_env.sh
```

现场获准编辑航点后，只能用下列命令回传实车的语义路线；回传后仍须在本机审查，并按
[航点编辑与授权](waypoint-editor.md) 同步仿真路线。不得手工编辑或单独替换其中一份 YAML。

```bash
python3 scripts/sync_to_rdk.py pull-waypoints --dry-run
python3 scripts/sync_to_rdk.py pull-waypoints
```

## RDK 构建

```bash
source ~/source_env.sh
cd /home/sunrise/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

`nav2_params_fixed.yaml` 是构建产物。需要修改 Nav2 参数时，修改源文件
`nav2_params.yaml` 后重新构建，禁止手工修改构建产物。

## 安全启动规范

源代码或配置变更后的独立准备入口是：

```bash
cd /home/zyh/SmartCar_V2026
bash scripts/nav_deploy.sh
ssh root@192.168.128.10
bash /home/sunrise/ros2_ws/scripts/nav_test.sh
```

无线时将上例的 SSH 目标替换为 `$SMARTCAR_RDK_HOST`。`nav_deploy.sh` 会在本机推送
`src/config` 和运行脚本、先备份 RDK 的 `src/config`，再远程执行 `nav_prepare.sh` 的定向清理和
依赖闭包增量构建。它不启动相机或车辆。`nav_test.sh` 只启动系统和 RViz，并固定使用
`nav_only.yaml`，同时显式设置：

```text
autostart_mission:=false
safety_emergency_stop_on_start:=true
use_camera:=false
camera_driver:=aurora
use_depth_camera:=true
use_vision:=false
```

`nav_test.sh` 不会同步、清理或构建。它固定启动 Aurora 深度避障和 RViz，并用单个状态进程检查 Nav2 lifecycle、task、急停、depth points/scan 与两张 costmap。运行入口只有：

```bash
bash /home/sunrise/ros2_ws/scripts/nav_test.sh
bash /home/sunrise/ros2_ws/scripts/nav_test.sh --go
bash /home/sunrise/ros2_ws/scripts/nav_test.sh --go --wheel-only
bash /home/sunrise/ros2_ws/scripts/nav_test.sh --go --c-zone-direction=clockwise
bash /home/sunrise/ros2_ws/scripts/ros_cleanup.sh
```

不带 `--go` 时只进入急停锁存的就绪状态。`--go` 在本次明确运动授权、物理急停可用且车辆已在 P 点朝 `+X` 摆位后，健康检查通过即按“急停锁存复位 -> 解除软件急停 -> `/smartcar/task/start`”顺序发车。`--wheel-only` 只用于定位对照。`--c-zone-direction=clockwise` 在启动时仅内存镜像已确认的 C 区约束；省略时默认 `counterclockwise`，不写入 YAML，也不在运行中切换。`ros_cleanup.sh` 先结束 `nav_test.sh` 记录的导航和 RViz PID，再清理已知导航残留；它不会使用宽泛进程匹配或清理航点编辑器。任何运动仍须另行取得该次明确授权。

发车前必须人工将车辆放在 P 原点、车头朝 `+X`；软件 `reset` 不能替代物理复位。路线所有段均由
Nav2 基于实时 obstacle/inflation costmap 规划，不得添加人工连接路线、绕障规则或专用导航阈值。

## 当前实车配置

| 项目 | 当前值 |
| --- | --- |
| 路线 | 全正向 `P -> A -> via_1 -> via_2 -> via_3 -> via_6 -> C1 -> via_4 -> via_5 -> via_7 -> P` |
| 路线文件 | `default_waypoints.yaml` 与 `nav_only.yaml` 完全共用航点 ID、顺序、坐标、姿态、方向、profile 和 `planning_segments`；仅 A/C1 任务类型不同。 |
| 路线状态 | 两份路线均为 `calibrated: false`。当前全正向纯导航路线已通过 LiDAR 无障碍现场基础通行性验证；深度相机的动态障碍感知仍需单独验证。 |
| 任务 profile | A、C1 为 `precise`；P 为 `standard`。 |
| 运动门禁 | 默认均为 `false`，用于防止无人自动发车。对收到本次明确运动授权的官方受看护短测，脚本会为指定前缀一次性传入运行确认；不得将默认值反复作为 LiDAR 已验证路线的技术阻塞。固定深度相机外参不是门禁。 |
| 底盘串口 | `/dev/ttyACM0` |
| LiDAR 串口 | `/dev/ttyUSB0` |
| 默认相机驱动 | `aurora`；纯导航准备时明确关闭相机和视觉。 |
| Aurora 深度模式 | `10 Hz` IR/depth，relay 上限 `12 Hz`；校正 Aurora 固件采集时间戳后发布 `/smartcar/depth/points`。 |
| 深度有效距离 | 转换后的前向 `/smartcar/depth/scan`、障碍标记与 clearing 均限制在 `3.0 m` 内。 |
| 深度观测时窗 | relay 将相机同高切片转换为前向 `/smartcar/depth/scan`；local/global costmap `observation_persistence=0.0 s`、`expected_update_rate=1.0 s`、`inf_is_valid=true`，使空束清除已移走障碍；safety 原始点云心跳 `1.0 s`。 |
| 轴距/最大转角 | `0.144 m` / `0.70 rad` |
| 最大线速度 | `0.30 m/s` |
| 最低电压 | `10.0 V` |
| 速度标定 | `longitudinal_velocity_scale: 1.03` |
| IMU Z 轴偏置 | `gyro_z_bias: -0.00036369` |
| 转向标定 | scale `1.0`，offset `0.0 rad`，最大已标定命令 `0.70 rad` |

普通 launch 的 `emergency_stop_on_start` 默认是 `false`，不能作为现场启动依据；受保护的
`/home/sunrise/ros2_ws/scripts/nav_test.sh` 会覆盖为 `true`。全部运动命令必须经过：

```text
velocity_smoother -> direction_guard -> smartcar_safety -> /ackermann_cmd
```

不得绕过方向门或 safety，禁止直接向底盘发布 Twist 或 Ackermann 命令。LiDAR 用于连续扫描匹配
里程计与 Nav2 costmap，不用于 SLAM 或静态地图定位；EKF 是 `odom_combined -> base_footprint` 的唯一
TF owner。

## 深度静态验收

2026-08-10 在 RDK `172.16.24.170`、急停锁存且未发布非零速度的条件下完成深度链路静态检查：

- Aurora 已按 `10 Hz` 启动；50 帧 `/smartcar/depth/points` 的中位接收间隔为 `0.106 s`，最大间隔为 `0.783 s`。
- 修正后的采集时间戳最大年龄为 `0.215 s`，低于 relay 的 `0.35 s` 拒绝阈值。
- 当时 local/global costmap 的原始点云静态接入与 safety 的 `1.0 s` 深度心跳均已核对。当前版本改用前向 `/smartcar/depth/scan` 的 `+Inf` clearing；部署后须在急停锁存、无运动状态下复核其标记和移障清除。

该记录只证明点云时间戳与 costmap 静态接入。LiDAR 无障碍实测已证明路线的基础通行性；深度模式仍须在每次明确运动授权后验证动态障碍物的标记/清障与 Nav2 实时重规划，不要求重复航点或固定外参标定。

## 检查与异常处理

- 启动前确认急停可用、串口设备存在、车辆已物理复位，且路线和各项门禁仍处于预期状态。
- 出现异常时先执行物理急停，再使用 RDK 已部署的 `ros_cleanup` 或仓库的
  `scripts/ros_cleanup.sh` 清理残留 ROS 进程；不要使用宽泛的 `pkill -f`。
- 软件、构建或仿真检查不能构成 RDK 或实体车辆验收，也不能解除任何运动门禁。
- 需要修改航点时，先在 RViz 航点编辑器展示拟议点位并取得用户对该次修改的明确同意；详见
  [航点编辑与授权](waypoint-editor.md)。
