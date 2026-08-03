# 智慧医疗赛智能车

第二十一届全国大学生智能汽车竞赛地瓜机器人智慧医疗赛工程。

## 软件里程碑

当前软件基线包含可选 RF2O、一条三阶段语义路线、官方规则图参考层、RViz 可拖拽航点编辑器，以及语音/二维码/图生文三个独立媒体入口。当前仿真路线为 P→A 前进、A→`via_2`→C1 倒车、C1→`via_1`→`via_3`→P 倒车。`via` 是无朝向的普通经过约束；C1 保持锁定航向的 `reverse_handoff`，只能作为倒车多目标 `NavigateThroughPoses` 的最后一个目标。专用 BT 用虚拟航向调用唯一的 DUBIN planner，后置方向门将动作 UUID 与速度方向租约绑定。2026-07-24 的 RDK 核心包验证为 `108 tests, 0 errors, 0 failures, 0 skipped`；当前本机根合同已通过。它们不包含实体倒车、真实相机、云端 API、音频或完整赛道运动验收。

## 当前边界

软件代码已经覆盖底盘安全门、后置方向门、轮速与 IMU 标定、EKF、Nav2 阿克曼导航、二维码/VLM 服务、五子任务状态机和一键系统启动。RDK 合成传感器 smoke 已验证 BT 插件可加载、四个 Nav2 lifecycle 节点可激活、未授权的非零候选速度无法越过方向门，并确认无物理底盘输出消费者。

以下项目仍是部署或实车门槛，不应被代码测试结果替代：

- 默认仿真路线为 P→A 前进、A→`via_2`→C1 倒车、C1→`via_1`→`via_3`→P 倒车，且保持 `calibrated: false`；必须现场实测后才能授权运动。
- `base_footprint -> base_link`、`base_link -> laser`、`base_link -> camera` 外参已测量并接入；更换或重装传感器后必须重新确认。
- 轮速、陀螺仪和转向命令比例/偏置已完成标定；运行链仍须通过车轮离地、低速地面和完整赛道复验。
- QR→VLM 倒车仅完成软件与无底盘验证；规划与安全链采用保守 `0.22 m` 最小转弯半径，不能视为实体倒车验收。
- P→A 的全局路径已消除大绕圈，但最新 Gazebo 执行仍在高位紧右弯横向过冲至右侧保护包络后停止；不得通过增加经过点或放宽保护包络掩盖该问题。
- RF2O 无地图连续扫描匹配已作为可选激光里程计接入，默认关闭，且已通过 RDK Humble/aarch64 构建；真实雷达时间同步、外参、协方差、漂移、异常观测拒绝和轮速/IMU 退化回退仍未实测。
- 火山 Ark VLM 与火山 TTS 已提供可选适配器，但凭据、现场网络、模型可用性和扬声器尚未验收；默认均禁用。
- 使用火山云端后端前必须确认比赛规则允许公网且赛场网络稳定；否则需准备并实测端侧 VLM 备用方案。
- 当前 TTS 为异步 fire-and-forget；若比赛流程要求播报完成后才能继续导航，仍需增加 action/ack 协议。
- VFH/YOLO 是架构候选，不是当前 release 依赖。
- 人工物理急停、车轮离地、低速地面和完整赛道测试尚未完成。

五个运动门禁默认全部为 `false`：`waypoints_calibrated`、`extrinsics_calibrated`、`steering_calibrated`、`emergency_stop_ready`、`operator_approved`。启用 RF2O 时还会条件性要求 `laser_odometry_calibrated=true`。未完成对应实测前不得把门禁改为 `true`。

## 平台与结构

- RDK X5 8G，TROS ROS 2 Humble，环境入口由 `~/source_env.sh` 统一提供。
- OriginCar 阿克曼底盘，`/odom`、`/imu/data_raw` 和可选 `/odom_laser` 经 EKF 输出 `/odom_combined`。
- YDLIDAR Tmini Plus 发布 `/scan`，同时供 obstacle/inflation costmap 和可选 RF2O 连续扫描匹配使用；实体运行不使用静态地图、AMCL 或 SLAM。仿真专用 keepout PGM 由独立 `map_server` 提供给 KeepoutFilter。
- 导航固定使用单一 Smac Hybrid `DUBIN` planner；倒车 BT 只接受全程严格反向、无 cusp、曲率和端点均合规的路径。
- 速度链为 `/cmd_vel_nav -> /cmd_vel_candidate -> direction_guard -> /cmd_vel -> smartcar_safety -> /ackermann_cmd`。方向门默认 STOP，错误方向或过期租约只能得到完整零输出。
- `obstacle_detector_2`、Madgwick `/imu/data` 和 URDF 车型发布默认关闭；它们不在 Nav2 costmap、EKF 或安全门的必需数据链上。
- 本机 Ubuntu 22.04 是唯一开发与 Gazebo 仿真环境；RDK 通过原生 `rsync` 单独同步。

主要包：

```text
src/origincar/                 vendor 底盘、IMU、EKF、YDLIDAR 与消息包
src/third_party/obstacle_detector_2/  LiDAR 障碍物提取
src/third_party/rf2o_laser_odometry/  可选无地图 scan-to-scan 激光里程计
src/smartcar_interfaces/       视觉服务与方向租约接口
src/smartcar_safety/           后置方向门和 fail-closed 速度安全门
src/smartcar_nav2/             Nav2、反向 BT、RPP、Smac Hybrid、航点
src/smartcar_vision/           zbar 与有界 VLM 服务
src/smartcar_speech/           可选火山 TTS 与本地音频播放
src/smartcar_task/             五子任务状态机
src/smartcar_bringup/          分层与一键系统 launch
src/smartcar_tools/            场地参考、航点编辑、诊断和三个媒体测试入口
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
ssh root@172.16.25.27
source ~/source_env.sh
cd /root/ros2_ws
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test-result --delete-yes
colcon test --return-code-on-test-failure
colcon test-result --all --verbose
```

2026-07-24 有线 RDK 验证中，`smartcar_safety smartcar_nav2 smartcar_task smartcar_bringup` 构建通过，清除历史 xunit 后为 `108 tests, 0 errors, 0 failures, 0 skipped`；其中含方向门 C++ 单测、反向路径合同、ROS action adapter 和两组无底盘 launch smoke。该结果只覆盖本次倒车改动的核心包，不替代全工作区或实车验收。`ydlidar_ros2_driver`、`origincar_bringup`、`origincar_description` 的继承源码全量 lint 为显式 opt-in；需要清理 vendor 风格债务时使用 `colcon build --cmake-args -DSMARTCAR_ENABLE_VENDOR_LINT=ON`。

## 无硬件 smoke

`smartcar_bringup` 的 launch test 使用合成 `/scan`、`/odom`、`/odom_combined` 和静态 TF，关闭底盘、LiDAR、障碍物驱动、实体相机和语音节点，并在 Nav2 激活前锁存急停。它验证 `controller_server`、`planner_server`、`bt_navigator`、`velocity_smoother` 四个 lifecycle 节点、vision/task 服务、任务 `IDLE`、硬件节点隔离和 `/cmd_vel_safe` 精确全零。`smartcar_nav2` 的独立 launch test 还验证未授权非零 `/cmd_vel_candidate` 在 `/cmd_vel` 与 `/cmd_vel_safe` 上始终为零。

```bash
source ~/source_env.sh
cd /root/ros2_ws
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

常用开关：`use_base`、`use_lidar`、`use_obstacle`、`use_laser_odometry`、`use_imu_filter`、`use_robot_description`、`use_safety`、`use_nav`、`use_camera`、`use_vision`、`use_task`、`use_speech`、`nav_autostart`、`safety_emergency_stop_on_start`、`use_sim_time`。`use_obstacle`、`use_laser_odometry`、`use_imu_filter`、`use_robot_description` 和 `use_speech` 默认均为 `false`；关闭 `use_obstacle` 不会关闭 costmap 的 `/scan` 避障。RF2O 启用后要求底盘、LiDAR 和额外的 `laser_odometry_calibrated` 门禁。关闭 `use_base` 时仍会保留 safety 节点，适合无硬件 bench；物理底盘开启时系统强制要求 `use_safety=true`。

## 场地路线与独立测试入口

`smartcar_tools` 提供官方规则图参考层、中文路径分段航点编辑器，以及互相隔离的语音、二维码和图生文入口。编辑器不启动 Nav2、LiDAR 或底盘；本机离线编辑默认不启动 safety，RDK 标定时必须显式传入 `start_safety:=true` 锁存软件急停。

规则图参考层只用于 RViz 看图量点，不发布 `/map`、不参与定位或 costmap。当前路线不能直接视为实测路线，也不能凭软件测试解除运动门禁。完整的分段、途经点、无朝向、预检、保存和仿真同步流程见 [`docs/deployment/waypoint-editor.md`](docs/deployment/waypoint-editor.md)。

无视觉导航测试使用已部署的 `/root/nav_test.sh`。脚本完成清理、增量构建、启动和 RViz 准备，但明确设置 `autostart_mission:=false` 与 `safety_emergency_stop_on_start:=true`，不会自动发车：

```bash
ssh root@192.168.128.10
bash /root/nav_test.sh
```

首次测试必须车轮离地并由现场人员持续观察；确认 RViz 后才可人工 reset、解除急停并 start。当前仅允许在 P→A 紧右弯控制问题修正并复验后，继续进行完整路线测试。

真实相机默认使用 USB 驱动和 `/image`；Aurora 和 MIPI 保留为备选。系统会把解析后的
`image_topic` 同时交给 vision 服务和任务按需启动的 zbar，因此切换驱动或显式覆盖话题时
无需另配二维码读取器。关闭相机但保留服务时需要提供合成或外部图像话题：

```bash
ros2 launch smartcar_bringup smartcar_system.launch.py \
  use_camera:=false image_topic:=/smartcar/vision/image \
  autostart_mission:=false
```

## 任务控制与急停

任务开始前必须完成五个运动门禁和现场校准。服务调用：

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
ros2 bag record -o /root/ros2_ws/bags/$(date +%Y%m%d-%H%M%S) \
  /scan /odom /odom_laser /odom_combined /imu/data_raw /imu/data \
  /cmd_vel_nav /cmd_vel_candidate /cmd_vel /cmd_vel_safe \
  /ackermann_cmd /PowerVoltage \
  /tf /tf_static /smartcar/direction_guard/status \
  /smartcar/safety/status /smartcar/task/state \
  /barcode /smartcar/output/text /smartcar/output/speech \
  /smartcar/speech/status
```

关键输出：`/cmd_vel_candidate`、`/cmd_vel`、`/cmd_vel_safe`、`/ackermann_cmd`、`/smartcar/direction_guard/status`、`/smartcar/safety/status`、`/smartcar/task/state`、`/smartcar/vision/read_qr`、`/smartcar/vision/describe_scene`、`/smartcar/speech/status`。

更多设计与部署细节见 [`docs/deployment/rdk-environment-setup.md`](docs/deployment/rdk-environment-setup.md)、[`docs/deployment/waypoint-editor.md`](docs/deployment/waypoint-editor.md) 和 [`docs/智慧医疗挑战赛_技术方案_本地部署版.md`](docs/智慧医疗挑战赛_技术方案_本地部署版.md)。
