# RDK 部署与现场流程

本机 `src/` 与 `config/` 是权威源。本页只记录当前 RDK 操作；没有用户的明确授权，不得同步
RDK、启动实体相机、解除急停、发布非零速度或进行实车运动。

## 连接信息

| 网络 | SSH 目标 | 说明 |
| --- | --- | --- |
<!-- | 有线 | `root@192.168.128.10` | 同步工具的默认目标。 | -->
| 无线 | `root@10.15.243.92` | 当前 DHCP 地址，无固定 IP；变更后更新环境变量。 |

```bash
# 有线
# ssh root@192.168.128.10

# 无线：当前 DHCP 地址；变更后替换该 IP
export SMARTCAR_RDK_HOST=root@10.15.243.92
ssh "$SMARTCAR_RDK_HOST"
```

RDK 工作空间为 `/home/sunrise/ros2_ws`，环境入口为
`/home/sunrise/ros2_ws/scripts/source_env.sh`。迁移前的
`/root/ros2_ws` 仅保留作回退副本，不再作为部署或运行目标。进入 RDK 后先执行：

```bash
source /home/sunrise/ros2_ws/scripts/source_env.sh
```

## 本机同步

`push` 会将本机的 `src/` 与 `config/` 镜像到 RDK，默认包含 `--delete`。RDK 有本地修改时，
先回传或备份，再预览并同步：

```bash
cd /home/zyh/SmartCar_V2026
python3 scripts/sync_to_rdk.py pull --dry-run       # 有必要时先检查 RDK 修改
python3 scripts/sync_to_rdk.py push --dry-run
python3 scripts/sync_to_rdk.py push
python3 scripts/sync_to_rdk.py setup                # 更新工作空间 scripts/source_env.sh
```

现场获准编辑航点后，只能用下列命令回传实车的语义路线；回传后仍须在本机审查，并按
[航点编辑与授权](waypoint-editor.md) 同步仿真路线。不得手工编辑或单独替换其中一份 YAML。

```bash
python3 scripts/sync_to_rdk.py pull-waypoints --dry-run
python3 scripts/sync_to_rdk.py pull-waypoints
```

## RDK 构建

```bash
source /home/sunrise/ros2_ws/scripts/source_env.sh
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
# ssh root@192.168.128.10
ssh root@10.15.243.92
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

`nav_test.sh` 不会同步、清理或构建。它固定启动 Aurora 深度避障和 RViz，并用单个状态进程检查 Nav2 lifecycle、task、急停、depth points/scan 与两张 costmap。纯导航测试入口为：

```bash
bash /home/sunrise/ros2_ws/scripts/nav_test.sh
bash /home/sunrise/ros2_ws/scripts/nav_test.sh --go
bash /home/sunrise/ros2_ws/scripts/nav_test.sh --go --wheel-only
bash /home/sunrise/ros2_ws/scripts/nav_test.sh --go --c-zone-direction=clockwise
bash /home/sunrise/ros2_ws/scripts/ros_cleanup.sh
```

`nav_test.sh` 还会校验当前 `src/` 指纹是否与 `nav_prepare.sh` 写入的准备指纹一致。源码同步后未重新准备时，它会拒绝启动，防止旧 `install/` 产物掩盖当前代码。

不带 `--go` 时只进入急停锁存的就绪状态。`--go` 在本次明确运动授权、物理急停可用且车辆已在 P 点朝 `+X` 摆位后，健康检查通过即按“急停锁存复位 -> 解除软件急停 -> `/smartcar/task/start`”顺序发车。`--wheel-only` 只用于定位对照。`--c-zone-direction=clockwise` 在启动时仅内存镜像已确认的 C 区约束；省略时默认 `counterclockwise`，不写入 YAML，也不在运行中切换。`ros_cleanup.sh` 先结束 `nav_test.sh` 记录的导航和 RViz PID，再清理已知导航残留；它不会使用宽泛进程匹配或清理航点编辑器。任何运动仍须另行取得该次明确授权。

### 比赛模式

比赛使用语义路线和独立的两阶段入口，而不是 `nav_test.sh --go`：

```bash
# 同步、构建完成后；需要本次实体相机授权。先将车人工放在 P 原点、车头朝 +X。
bash /home/sunrise/ros2_ws/scripts/competition_mode.sh prepare

# 裁判发令时；需要本次非零运动授权、物理急停确认和 P 点人工摆位：
# 在“比赛输出”UI按“发车”，并在确认框中选择“是”

# 仅检查，或在异常时先物理急停后停止
bash /home/sunrise/ros2_ws/scripts/competition_mode.sh status
bash /home/sunrise/ros2_ws/scripts/competition_mode.sh stop
```

`prepare` 启动同一个 Aurora 驱动的 RGB 与深度链路、QR/VLM 服务、深度 costmap、Nav2 和比赛任务树，
但保持软件急停锁存且不执行任务。它在健康检查通过后、仍锁存急停时完成 P 点定位复位，并把预置结果绑定到
当前比赛栈 PID；因此裁判按钮不再等待第二次健康检查或定位复位。若赛前重新摆车或健康状态变化，先保持急停，
将车辆放回 P 点、车头朝 `+X`，再执行 `bash /home/sunrise/ros2_ws/scripts/competition_mode.sh arm` 重新预置。
它不启动 RViz、OpenCV `imshow` 或独立媒体短测；唯一图形界面是同屏
显示识别出的 QR 数字、已选 C 区方向、VLM 文本和任务状态的比赛输出 UI。不要在比赛栈运行时启动 `media_test.sh`，否则会重复
打开 Aurora。准备检查会确认 QR/VLM 服务可用；默认预热还会确认 ZBar reader 已启动。Volcengine
凭据优先读取 RDK 进程环境中的 `ARK_API_KEY`；兼容回退可使用工作区的
`config/volcengine_ark.local.yaml`。同步脚本不会传输或删除该本地文件，比赛脚本只做存在性检查，
绝不输出密钥内容。

比赛输出 UI 的“发车”按钮是裁判口令后的标准入口。比赛脚本显式授权该按钮后，它会显示 P 点摆位和物理
急停确认框，并在 RDK 本地异步执行同一 `competition_mode.sh start --confirm` 流程；远程桌面连接可以点击
这个按钮，但不能替代本次实体相机/非零运动的明确授权。`start --confirm` 保留为 UI 不可用时的受看护恢复
入口，不能与 UI 发车并行执行。无论入口为何，都会验证当前 PID 的预置标记、解除软件急停和任务 Trigger。比赛任务在
A 点读取 QR 后一次性选择已获准的 C 区运行时镜像：`奇数 -> clockwise（顺时针）`，
`偶数 -> counterclockwise（逆时针）`；`未识别`、歧义结果或 QR 读取失败则回退
`counterclockwise（逆时针）`，仍继续完整路线返回 P。识别出的 QR 数字和已选方向都会显示在比赛输出 UI。
该选择只替换内存中的后续 Nav2 输入变体，不修改两份 waypoint YAML、航点 ID/顺序或 planning segments。
VLM 无结果会明确失败，不显示通用描述或继续伪造语义任务完成；导航、定位、方向门、costmap 或 safety
异常同样按失败处理，不能为了完赛绕过安全链。

`prepare` 会先拒绝已有的底盘、Nav2、任务、Aurora 或视觉节点，防止旧 ROS 栈用同名全局服务或话题
误通过新栈健康检查。此时先确认物理急停，再停止或按现场流程清理旧栈。

发车前必须人工将车辆放在 P 原点、车头朝 `+X`；软件 `reset` 不能替代物理复位。路线所有段均由
Nav2 基于实时 obstacle/inflation costmap 规划，不得添加人工连接路线、绕障规则或专用导航阈值。

## 当前实车配置

| 项目 | 当前值 |
| --- | --- |
| 路线 | 全正向 `P -> A -> via_1 -> via_2 -> via_3 -> via_6 -> C1 -> via_4 -> via_5 -> via_7 -> P` |
| 路线文件 | `default_waypoints.yaml` 与 `nav_only.yaml` 完全共用航点 ID、顺序、坐标、姿态、方向、profile 和 `planning_segments`；仅 A/C1 任务类型不同。 |
| 路线状态 | 两份路线均为 `calibrated: false`。当前全正向纯导航路线已完成本机 Gazebo 全链软件校验；深度相机的动态障碍感知仍需现场验证。 |
| 任务 profile | A、C1 为 `precise`；P 为 `standard`。 |
| 运动门禁 | 默认均为 `false`，用于防止无人自动发车。对收到本次明确运动授权的官方受看护短测，脚本会为指定前缀一次性传入运行确认；不得将默认值反复作为已验证路线的技术阻塞。固定深度相机外参不是门禁。 |
| 底盘串口 | `/dev/ttyACM0` |
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

### 操作员现场声明

2026-08-14，操作员声明已在现场验证以下速度模式可行：大厅区域 `0.30 m/s`，通过通道后 `0.70 m/s`，从诊疗室返回并再次经过通道时降回 `0.30 m/s`。

该声明未附带制动距离、急停响应、障碍物工况、底盘控制日志或可复现的原始记录。因此它仅作为后续速度方案评审的现场线索；当前软件仍保持 Nav2 的统一 `0.30 m/s` 速度上限，未启用分区速度切换，也不构成实车运动授权。

普通 launch 的 `emergency_stop_on_start` 默认是 `false`，不能作为现场启动依据；受保护的
`/home/sunrise/ros2_ws/scripts/nav_test.sh` 会覆盖为 `true`。全部运动命令必须经过：

```text
velocity_smoother -> direction_guard -> smartcar_safety -> /ackermann_cmd
```

不得绕过方向门或 safety，禁止直接向底盘发布 Twist 或 Ackermann 命令。深度相机是唯一障碍与视觉
感知来源，不用于 SLAM 或静态地图定位；EKF 是 `odom_combined -> base_footprint` 的唯一 TF owner。
EKF 的 yaw 输出完全来自 IMU 的 z 轴角速度：轮式合成的 yaw 不可靠且被 EKF 忽略，标定后的
IMU yaw 可靠。

## 深度静态验收

2026-08-10 在 RDK `10.15.243.92`、急停锁存且未发布非零速度的条件下完成深度链路静态检查：

- Aurora 已按 `10 Hz` 启动；50 帧 `/smartcar/depth/points` 的中位接收间隔为 `0.106 s`，最大间隔为 `0.783 s`。
- 修正后的采集时间戳最大年龄为 `0.215 s`，低于 relay 的 `0.35 s` 拒绝阈值。
- 当时 local/global costmap 的原始点云静态接入与 safety 的 `1.0 s` 深度心跳均已核对。当前版本改用前向 `/smartcar/depth/scan` 的 `+Inf` clearing；部署后须在急停锁存、无运动状态下复核其标记和移障清除。

该记录只证明点云时间戳与 costmap 静态接入。深度模式仍须在每次明确运动授权后验证动态障碍物的标记/清障与 Nav2 实时重规划，不要求重复航点或固定外参标定。

## 检查与异常处理

- 启动前确认急停可用、串口设备存在、车辆已物理复位，且路线和各项门禁仍处于预期状态。
- 出现异常时先执行物理急停，再使用 RDK 已部署的 `ros_cleanup` 或仓库的
  `scripts/ros_cleanup.sh` 清理残留 ROS 进程；不要使用宽泛的 `pkill -f`。
- 软件、构建或仿真检查不能构成 RDK 或实体车辆验收，也不能解除任何运动门禁。
- 需要修改航点时，先在 RViz 航点编辑器展示拟议点位并取得用户对该次修改的明确同意；详见
  [航点编辑与授权](waypoint-editor.md)。
