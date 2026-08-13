# 智慧医疗赛智能车

第二十一届全国大学生智能汽车竞赛地瓜机器人智慧医疗赛工程。

## 软件里程碑

当前软件基线包含可选 RF2O、一条三个显式 planning segment、11 个航点约束的全正向路线、官方规则图参考层、可拖拽分段航点编辑器，以及语音/二维码/图生文三个独立媒体入口。实车 `default_waypoints.yaml` 与纯导航 `nav_only.yaml` 共享完全相同的点位和分段：P→A、A→`via_1`→`via_2`→`via_3`→`via_6`→C1、C1→`via_4`→`via_5`→`via_7`→P；仅 A/C1 的任务类型分别为 `qr`/`vlm` 与 `nav`。A 与 C1 使用 `precise` profile，P 终点使用 `standard` profile，且两份 YAML 均保持 `calibrated: false`。运行路径一律由 Nav2 基于实时 costmap 规划，不存在强制路线、先验墙体或点位专用阈值。当前全正向纯导航路线已完成 LiDAR 无障碍现场基础通行性验证；深度相机动态障碍物感知、QR/VLM 语义任务和 Gazebo 验证仍需分别验证。文档入口见 [docs/README.md](docs/README.md)。

所有根目录 `scripts/` 的用途、命令和副作用见 [脚本使用手册](docs/deployment/scripts.md)。

## 当前边界

软件代码已经覆盖底盘安全门、后置方向门、轮速与 IMU 标定、EKF、Nav2 阿克曼导航、二维码/VLM 服务、五子任务状态机和一键系统启动。RDK 合成传感器 smoke 已验证 BT 插件可加载、四个 Nav2 lifecycle 节点可激活、未授权的非零候选速度无法越过方向门，并确认无物理底盘输出消费者。

以下项目仍是部署或实车门槛，不应被代码测试结果替代：

- `nav_only.yaml` 当前保持 `calibrated: false`；本机 Gazebo 已使用它完成全路线软件校验，现场 LiDAR 无障碍实测已确认当前全正向纯导航路线的基础通行性。该结论不替代深度相机动态避障或语义任务验收。
- 正式语义路线 `default_waypoints.yaml` 也保持 `calibrated: false`；其 QR/A 点须单独按现场坐标复测，QR/VLM、语音和完整五子任务尚未完成实体验收。
- `base_footprint -> base_link`、`base_link -> laser`、`base_link -> camera` 外参已测量并接入；更换或重装传感器后必须重新确认。
- 轮速、陀螺仪和转向命令比例/偏置已完成标定；LiDAR 无障碍实测已覆盖当前纯导航路线的基础通行性。新增深度模式只验证其障碍感知与重规划效果，不重复要求基础路线或固定外参标定。
- 先前 QR→VLM 倒车软件链仅保留为未激活的通用能力；当前路线每一段均为正向。规划与安全链采用保守 `0.22 m` 最小转弯半径，离线预检不能视为实体路线验收。
- RF2O 无地图连续扫描匹配已作为可选激光里程计接入，默认关闭，且已通过 RDK Humble/aarch64 构建；真实雷达时间同步、外参、协方差、漂移、异常观测拒绝和轮速/IMU 退化回退仍未实测。
- 火山 Ark VLM 与火山 TTS 已提供可选适配器，但凭据、现场网络、模型可用性和扬声器尚未验收；默认均禁用。
- 使用火山云端后端前必须确认比赛规则允许公网且赛场网络稳定；否则需准备并实测端侧 VLM 备用方案。
- 当前 TTS 为异步 fire-and-forget；若比赛流程要求播报完成后才能继续导航，仍需增加 action/ack 协议。
- VFH/YOLO 是架构候选，不是当前 release 依赖。
- 任何新增传感器模式的动态地面测试仍须确认人工物理急停、现场摆位和本次运动授权；不得将这些操作性确认重复表述为路线或固定外参未标定。

五个运动门禁默认全部为 `false`：`waypoints_calibrated`、`extrinsics_calibrated`、`steering_calibrated`、`emergency_stop_ready`、`operator_approved`。它们防止无人自动发车；官方受看护短测在每次明确实体运动授权后一次性传入运行确认，不能作为反复拒绝已验证 LiDAR 路线的理由。启用 RF2O 时还会条件性要求 `laser_odometry_calibrated=true`。

## 平台与结构

- RDK X5 8G，TROS ROS 2 Humble，环境入口由 `~/source_env.sh` 统一提供。
- OriginCar 阿克曼底盘，`/odom`、`/imu/data_raw` 和可选 `/odom_laser` 经 EKF 输出 `/odom_combined`。
- YDLIDAR Tmini Plus 发布 `/scan`，同时供 obstacle/inflation costmap 和可选 RF2O 连续扫描匹配使用；实体运行不使用静态地图、AMCL、SLAM 或先验场地墙体约束。Nav2 仅依据实时 costmap 观测规划。
- 导航固定使用单一 Smac Hybrid `DUBIN` planner；活动路线使用正向行为树。保留的反向 BT 仅接受全程严格反向、无 cusp、曲率和端点均合规的路径，未被当前路线使用。
- 速度链为 `/cmd_vel_nav -> /cmd_vel_candidate -> direction_guard -> /cmd_vel -> smartcar_safety -> /ackermann_cmd`。方向门默认 STOP，错误方向或过期租约只能得到完整零输出。
- Madgwick `/imu/data` 和 URDF 车型发布默认关闭；它们不在 Nav2 costmap、EKF 或安全门的必需数据链上。
- 本机 Ubuntu 22.04 是唯一开发与 Gazebo 仿真环境；RDK 通过原生 `rsync` 单独同步。

主要包：

```text
src/origincar/                 vendor 底盘、IMU、EKF、YDLIDAR 与消息包
src/third_party/rf2o_laser_odometry/  可选无地图 scan-to-scan 激光里程计
src/smartcar_interfaces/       视觉服务与方向租约接口
src/smartcar_safety/           后置方向门和 fail-closed 速度安全门
src/smartcar_nav2/             Nav2、正向/反向 BT、RPP、Smac Hybrid、航点
src/smartcar_vision/           zbar 与有界 VLM 服务
src/smartcar_speech/           可选火山 TTS 与本地音频播放
src/smartcar_task/             五子任务状态机
src/smartcar_bringup/          分层与一键系统 launch
src/smartcar_tools/            场地参考、航点编辑、转向标定、诊断和媒体测试入口
src/smartcar_sim/              Ubuntu Gazebo 阿克曼仿真、RViz 与自动导航验证
```

## Ubuntu 本机仿真

原生 Ubuntu 22.04 开发机可直接使用当前仓库作为 colcon 工作空间。首次初始化会安装
Ignition Gazebo Fortress、ROS-Gazebo bridge 和仿真所需
ROS 依赖；命令会请求本机 `sudo` 认证：

```bash
cd /home/zyh/SmartCar_V2026
bash scripts/setup_local_sim.sh
```

日常以以下入口启动。它会隔离 Conda/Isaac 的库路径、清理残留仿真进程，并只启动
Gazebo、Nav2 与 RViz；默认不会启动自动路线或实体硬件：

```bash
bash scripts/local_sim.sh --headless --rviz
```

显式传入 `run_route:=true` 才会向 Gazebo 模型发送非零速度。它不会连接 RDK、底盘、
相机或安全节点，不能用作实车启动入口。

完整构建、验证、调参和故障排查见
[`docs/deployment/local-simulation.md`](docs/deployment/local-simulation.md)。

## 本地开发与同步

在本机 Ubuntu：

```bash
python3 -m unittest discover -s tests -v
python3 scripts/sync_to_rdk.py push --dry-run
```

`init-vendor` 只用于首次 bootstrap 或重新拉取官方包。日常以本地仓库为权威，使用 `push` 镜像 `src/` 和 `config/`。

RDK 使用 DHCP 地址时，在同步前设置目标即可；始终先审查 `--dry-run`，因为 `push`
默认包含 `--delete`：

```bash
export SMARTCAR_RDK_HOST=root@<RDK_IP>
python scripts/sync_to_rdk.py push --dry-run
python scripts/sync_to_rdk.py push
python scripts/sync_to_rdk.py setup
```

## RDK 构建与测试

```bash
ssh root@<RDK_IP>
source ~/source_env.sh
cd /home/sunrise/ros2_ws
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test-result --delete-yes
colcon test --return-code-on-test-failure
colcon test-result --all --verbose
```

2026-07-24 有线 RDK 验证中，`smartcar_safety smartcar_nav2 smartcar_task smartcar_bringup` 构建通过，清除历史 xunit 后为 `108 tests, 0 errors, 0 failures, 0 skipped`；其中含方向门 C++ 单测、反向路径合同、ROS action adapter 和两组无底盘 launch smoke。该历史结果不替代全工作区、当前全正向路线或实车验收。`ydlidar_ros2_driver`、`origincar_bringup`、`origincar_description` 的继承源码全量 lint 为显式 opt-in；需要清理 vendor 风格债务时使用 `colcon build --cmake-args -DSMARTCAR_ENABLE_VENDOR_LINT=ON`。

## 无硬件 smoke

`smartcar_bringup` 的 launch test 使用合成 `/scan`、`/odom`、`/odom_combined` 和静态 TF，关闭底盘、LiDAR、实体相机和语音节点，并在 Nav2 激活前锁存急停。它验证 `controller_server`、`planner_server`、`bt_navigator`、`velocity_smoother` 四个 lifecycle 节点、vision/task 服务、任务 `IDLE`、硬件节点隔离和 `/cmd_vel_safe` 精确全零。`smartcar_nav2` 的独立 launch test 还验证未授权非零 `/cmd_vel_candidate` 在 `/cmd_vel` 与 `/cmd_vel_safe` 上始终为零。

```bash
source ~/source_env.sh
cd /home/sunrise/ros2_ws
colcon test --packages-select smartcar_bringup
colcon test-result --test-result-base build/smartcar_bringup --verbose
```

这个 smoke 不打开 `/dev/ttyACM*`、`/dev/ttyUSB*` 或任何相机设备，也不发送非零速度。

## 一键系统启动

默认启动完整软件栈，但不会自动开始任务：

```bash
source ~/source_env.sh
ros2 launch smartcar_bringup smartcar_system.launch.py \
  autostart_mission:=false
```

常用开关：`use_base`、`use_lidar`、`use_laser_odometry`、`use_imu_filter`、`use_robot_description`、`use_safety`、`use_nav`、`use_camera`、`use_vision`、`use_depth_camera`、`use_task`、`use_visualization`、`use_speech`、`nav_autostart`、`safety_emergency_stop_on_start`、`use_sim_time`。`use_laser_odometry`、`use_imu_filter`、`use_robot_description`、`use_visualization`、`use_speech` 和 `use_depth_camera` 默认均为 `false`；常规模式的 YDLIDAR `/scan` 进入 costmap，深度模式将 `/smartcar/depth/points` 的相机同高切片转换为前向 `/smartcar/depth/scan` 进入 costmap，原始点云仍供 safety 与 RViz 诊断。RF2O 启用后要求底盘、LiDAR 和额外的 `laser_odometry_calibrated` 门禁。关闭 `use_base` 时仍会保留 safety 节点，适合无硬件 bench；物理底盘开启时系统强制要求 `use_safety=true` 且使用 safety 的 Ackermann 输出。

## 场地路线与独立测试入口

`smartcar_tools` 提供官方规则图参考层、中文路径分段航点编辑器、转向标定工具，以及互相隔离的语音、二维码和图生文入口。编辑器不启动 Nav2、LiDAR 或底盘；本机离线编辑默认不启动 safety，RDK 标定时必须显式传入 `start_safety:=true` 锁存软件急停。转向静态量角默认由 safety 拒绝，圆周标定必须走完整的方向租约和安全链；命令与前置条件见 [`docs/reference/field-tools.md`](docs/reference/field-tools.md)。

规则图参考层只用于 RViz 看图量点，不发布 `/map`、不参与定位或 costmap。当前路线不能直接视为实测路线，也不能凭软件测试解除运动门禁。完整的分段、途经点、无朝向、预检、保存和仿真同步流程见 [`docs/deployment/waypoint-editor.md`](docs/deployment/waypoint-editor.md)。

无视觉导航测试使用已部署的 `/home/sunrise/ros2_ws/scripts/nav_test.sh`。普通调用只启动栈和 RViz，保持急停锁存；`--go` 在健康检查通过后按“复位 -> 解除软件急停 -> 启动任务”顺序发车。源代码或配置变更后，先在本机运行独立部署入口；它同步、备份并执行 RDK 的定向清理和构建，不启动实体设备：

```bash
cd /home/zyh/SmartCar_V2026
bash scripts/nav_deploy.sh
ssh root@<RDK_IP>
bash /home/sunrise/ros2_ws/scripts/nav_test.sh
```

LiDAR 无障碍实测已确认当前全正向纯导航路线的基础通行性，不必为深度模式重复该验收。深度模式待验证的是动态障碍物在 costmap 的标记/清障及 Nav2 实时重规划，不是重复航点或固定外参标定。

已取得当次实体运动授权且车辆已在 P 点、车头朝 `+X` 摆正时，完整受看护路线直接执行：

```bash
bash /home/sunrise/ros2_ws/scripts/nav_test.sh --go
```

该选项只会在状态检查通过后自动复位并发车；不带该选项的普通调用始终保持急停锁存。

纯轮式里程计为临时诊断对照，不是默认定位配置。确认同一轮运动授权、物理急停和 P 点摆位后，可用下面的完整路线命令比较 IMU 融合与纯轮式的回零误差：

```bash
bash /home/sunrise/ros2_ws/scripts/nav_test.sh --go --wheel-only
```

`--wheel-only` 只将定位 profile 切换为 `wheel_only`；不改变实时 costmap、Nav2 路径规划或速度安全链。省略它始终使用默认 `wheel_imu`。

真实相机默认使用 USB 驱动和 `/image`；Aurora 和 MIPI 保留为备选。系统会把解析后的
`image_topic` 同时交给 vision 服务和任务按需启动的 zbar，因此切换驱动或显式覆盖话题时
无需另配二维码读取器。关闭相机但保留服务时需要提供合成或外部图像话题：

```bash
ros2 launch smartcar_bringup smartcar_system.launch.py \
  use_camera:=false image_topic:=/smartcar/vision/image \
  autostart_mission:=false
```

Aurora 930 深度避障使用独立的显式入口：

```bash
bash scripts/nav_deploy.sh
bash /home/sunrise/ros2_ws/scripts/nav_test.sh
```

该模式关闭 YDLIDAR 与 RF2O，仅保留轮式里程计和 IMU EKF 导航。中继会校验 source frame、
点云布局、有限的浮点 XYZ 样本和采集时间；Aurora 930 1.7.2 的时间戳会乘以 `1000` 后再供 TF
查询。两个 Nav2 costmap 订阅 `/smartcar/depth/scan`；转换器仅生成相机前向视场内的扫描，并对空束发布 `+Inf`，使障碍移走后产生明确 clearing 射线。safety 看门狗继续只订阅原始 `/smartcar/depth/points`。
脚本在保持急停锁存的状态下验证点云与两个实时 costmap 参数。
深度相机固定在前轮正上方、离地 `0.15 m`、水平朝前，其外参是已确认的结构约束，不设外参标定门禁。
深度模式以 `10 Hz` 请求 Aurora IR/depth，relay 上限为 `12 Hz`；relay 保留校正后的采集时间戳，
并拒绝超龄云。两个 costmap 的扫描 `observation_persistence=0.0 s`、`expected_update_rate=1.0 s`，深度 safety 心跳为 `1.0 s`；这些时限不能使超龄云变为有效云。点云 frame、清障和动态障碍物避让
仍须完成现场验证；这些验证用于确认障碍感知效果，而不是重新标定固定外参。

## 任务控制与急停

完整任务开始前必须完成五个运动门禁和现场校准；已获本次明确授权的官方受看护短测由启动入口一次性传入运行确认。服务调用：

```bash
ros2 service call /smartcar/task/start std_srvs/srv/Trigger "{}"
ros2 service call /smartcar/task/stop std_srvs/srv/Trigger "{}"
ros2 service call /smartcar/task/reset std_srvs/srv/Trigger "{}"
```

`stop` 只停止任务，不等于急停。急停和解除急停：

```bash
ros2 service call /smartcar/safety/emergency_stop \
  std_srvs/srv/SetBool "{data: true}"
ros2 service call /smartcar/safety/emergency_stop \
  std_srvs/srv/SetBool "{data: false}"
```

`reset` 只接受已停止导航的状态，内部顺序为撤销方向租约、确认 action 终态与原始里程计六轴停稳、调用 `/set_pose`，再等待新的有限 `/odom_combined` 接近原点。车辆必须物理位于 P 区原点且车头朝 `+X`；reset 不能替代物理复位。

## 录包与诊断

录包只读取话题，不会产生运动命令：

```bash
ros2 bag record -o /home/sunrise/ros2_ws/bags/$(date +%Y%m%d-%H%M%S) \
  /scan /odom /odom_laser /odom_combined /imu/data_raw /imu/data \
  /cmd_vel_nav /cmd_vel_candidate /cmd_vel /cmd_vel_safe \
  /ackermann_cmd /PowerVoltage \
  /tf /tf_static /smartcar/direction_guard/status \
  /smartcar/safety/status /smartcar/task/state \
  /barcode /smartcar/output/text /smartcar/output/speech \
  /smartcar/speech/status
```

关键输出：`/cmd_vel_candidate`、`/cmd_vel`、`/cmd_vel_safe`、`/ackermann_cmd`、`/smartcar/direction_guard/status`、`/smartcar/safety/status`、`/smartcar/task/state`、`/smartcar/vision/read_qr`、`/smartcar/vision/describe_scene`、`/smartcar/speech/status`。

当前操作文档见 [`docs/README.md`](docs/README.md)。
