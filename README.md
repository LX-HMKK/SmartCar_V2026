# 智慧医疗赛智能车

第二十一届全国大学生智能汽车竞赛地瓜机器人智慧医疗赛工程。

## 软件里程碑

当前软件基线包含可选 RF2O、一份 9 点语义任务路线、官方规则图参考层、RViz 可拖拽航点编辑器，以及语音/二维码/图生文三个独立媒体入口。2026-07-19 记录的 RDK 全量结果 `578 tests, 0 errors, 0 failures, 90 skipped` 属于删除旧纯导航链之前的历史基线；当前版本需重新完成 RDK 构建与 smoke。版本内容见 [`CHANGELOG.md`](CHANGELOG.md)。这些结果不包含真实传感器、云端 API、音频或车辆运动验收。

## 当前边界

软件代码已经覆盖底盘安全门、轮速与 IMU 标定、EKF、Nav2 阿克曼导航、二维码/VLM 服务、五子任务状态机和一键系统启动。删除旧纯导航链之前的自动化测试和 RDK 合成传感器 smoke 曾通过；当前 9 点路线版本仍需在 RDK 上重新构建并运行 smoke。

以下项目仍是部署或实车门槛，不应被代码测试结果替代：

- 唯一的 9 点语义路线来自官方规则图推算且保持 `calibrated: false`，必须现场实测后才能授权运动。
- `base_footprint -> base_link`、`base_link -> laser`、`base_link -> camera` 外参尚未实测。
- 转向、轮速、陀螺仪标定尚未完成。
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
- YDLIDAR Tmini Plus 发布 `/scan`，同时供 obstacle/inflation costmap 和可选 RF2O 连续扫描匹配使用；没有静态地图、`map_server`、AMCL 或 SLAM。
- `obstacle_detector_2`、Madgwick `/imu/data` 和 URDF 车型发布默认关闭；它们不在 Nav2 costmap、EKF 或安全门的必需数据链上。
- 本地 Windows 通过 WSL rsync 同步到 RDK 的单一工作空间 `/root/ros2_ws`。

主要包：

```text
src/origincar/                 vendor 底盘、IMU、EKF、YDLIDAR 与消息包
src/third_party/obstacle_detector_2/  LiDAR 障碍物提取
src/third_party/rf2o_laser_odometry/  可选无地图 scan-to-scan 激光里程计
src/smartcar_interfaces/       ReadQr、DescribeScene 服务接口
src/smartcar_safety/           fail-closed /cmd_vel 安全门
src/smartcar_nav2/             Nav2 配置、RPP、Smac Hybrid、航点
src/smartcar_vision/           zbar 与有界 VLM 服务
src/smartcar_speech/           可选火山 TTS 与本地音频播放
src/smartcar_task/             五子任务状态机
src/smartcar_bringup/          分层与一键系统 launch
src/smartcar_tools/            场地参考、航点编辑、诊断和三个媒体测试入口
```

## 本地开发与同步

在 Windows PowerShell：

```powershell
python -m unittest discover -s tests -v
python -m unittest discover -s src/smartcar_safety/test -v
python -m unittest discover -s src/smartcar_vision/test -v
python -m unittest discover -s src/smartcar_speech/test -v
python -m unittest discover -s src/smartcar_task/test -v
python -m unittest discover -s src/smartcar_tools/test -v
python scripts/sync_to_rdk.py push
```

`init-vendor` 只用于首次 bootstrap 或重新拉取官方包。日常以本地仓库为权威，使用 `push` 镜像 `src/` 和 `config/`。

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

2026-07-19 最近一次已记录的全工作区结果为 18 个包构建通过，`578 tests, 0 errors, 0 failures, 90 skipped`。该结果来自删除旧纯导航链之前，不能作为当前 9 点路线版本的验证证据；当前版本须重新执行上述构建和测试。`ydlidar_ros2_driver`、`origincar_bringup`、`origincar_description` 的继承源码全量 lint 为显式 opt-in；需要清理 vendor 风格债务时使用 `colcon build --cmake-args -DSMARTCAR_ENABLE_VENDOR_LINT=ON`，不影响默认功能测试。

## 无硬件 smoke

`smartcar_bringup` 的 launch test 使用合成 `/scan`、`/odom`、`/odom_combined` 和静态 TF，关闭底盘、LiDAR、障碍物驱动、实体相机和语音节点，并在 Nav2 激活前锁存急停。它验证 Nav2 lifecycle、vision/task 服务、任务 `IDLE`、硬件节点隔离和 `/cmd_vel_safe` 精确全零：

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

`smartcar_tools` 提供官方规则图参考层、直接拖拽 `default_waypoints.yaml` 的 RViz 编辑器，以及互相隔离的语音、二维码和图生文入口。编辑器不启动 Nav2、LiDAR 或底盘，并默认锁存软件急停。

规则图参考层只用于 RViz 看图量点，不发布 `/map`、不参与定位或 costmap。当前路线不能直接视为实测路线，也不能凭软件测试解除运动门禁。完整编辑命令、HDMI 的 `DISPLAY`/`XAUTHORITY` 设置和单文件 `pull-waypoints` 同步方式见 [`docs/deployment/field-test-entrypoints.md`](docs/deployment/field-test-entrypoints.md)。

真实相机默认使用 Aurora RGB；关闭相机但保留服务时需要提供合成或外部图像话题：

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

`reset` 只接受已停止导航的状态，内部顺序为停止导航、调用 `/set_pose`、等待新的有限 `/odom_combined`、清除定位故障。车辆必须物理位于 P 区原点且车头朝 `+X`。`bringup_coord.yaml` 目前是校准记录，不会被 launch 自动读取。

## 录包与诊断

录包只读取话题，不会产生运动命令：

```bash
ros2 bag record -o /root/ros2_ws/bags/$(date +%Y%m%d-%H%M%S) \
  /scan /odom /odom_laser /odom_combined /imu/data_raw /imu/data \
  /cmd_vel /cmd_vel_safe /ackermann_cmd /PowerVoltage \
  /tf /tf_static /smartcar/safety/status /smartcar/task/state \
  /barcode /smartcar/output/text /smartcar/output/speech \
  /smartcar/speech/status
```

关键输出：`/cmd_vel_safe`、`/smartcar/safety/status`、`/smartcar/task/state`、`/smartcar/vision/read_qr`、`/smartcar/vision/describe_scene`、`/smartcar/speech/status`。

更多设计与部署细节见 [`docs/deployment/rdk-environment-setup.md`](docs/deployment/rdk-environment-setup.md)、[`docs/deployment/field-test-entrypoints.md`](docs/deployment/field-test-entrypoints.md) 和 [`docs/智慧医疗挑战赛_技术方案_本地部署版.md`](docs/智慧医疗挑战赛_技术方案_本地部署版.md)。
