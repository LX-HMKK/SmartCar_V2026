# 智慧医疗赛智能车

第二十一届全国大学生智能汽车竞赛地瓜机器人智慧医疗赛工程。

## 软件里程碑

2026-07-18 完整软件里程碑已合并到 `main`，随后加入可选火山视觉与语音适配。本地根合同 114/114 通过；RDK X5 上 16 个包构建通过，完整测试结果为 `533 tests, 0 errors, 0 failures, 90 skipped`；严格无硬件 smoke 连续 3 轮通过。版本内容见 [`CHANGELOG.md`](CHANGELOG.md)。

## 当前边界

软件代码已经覆盖底盘安全门、轮速与 IMU 标定、EKF、Nav2 阿克曼导航、二维码/VLM 服务、五子任务状态机和一键系统启动。自动化测试和 RDK 合成传感器 smoke 已通过。

以下项目仍是部署或实车门槛，不应被代码测试结果替代：

- 默认航点仍是占位坐标，必须用赛场实测值替换。
- `base_footprint -> base_link`、`base_link -> laser`、`base_link -> camera` 外参尚未实测。
- 转向、轮速、陀螺仪标定尚未完成。
- 火山 Ark VLM 与火山 TTS 已提供可选适配器，但凭据、现场网络、模型可用性和扬声器尚未验收；默认均禁用。
- 使用火山云端后端前必须确认比赛规则允许公网且赛场网络稳定；否则需准备并实测端侧 VLM 备用方案。
- 当前 TTS 为异步 fire-and-forget；若比赛流程要求播报完成后才能继续导航，仍需增加 action/ack 协议。
- VFH/YOLO 是架构候选，不是当前 release 依赖。
- 人工物理急停、车轮离地、低速地面和完整赛道测试尚未完成。

五个运动门禁默认全部为 `false`：`waypoints_calibrated`、`extrinsics_calibrated`、`steering_calibrated`、`emergency_stop_ready`、`operator_approved`。未完成实测前不得把它们改为 `true`。

## 平台与结构

- RDK X5 8G，TROS ROS 2 Humble，环境入口由 `~/source_env.sh` 统一提供。
- OriginCar 阿克曼底盘，原始 `/odom` 与 `/imu/data_raw` 经 EKF 输出 `/odom_combined`。
- YDLIDAR Tmini Plus 发布 `/scan`，只用于避障，不参与定位。
- 本地 Windows 通过 WSL rsync 同步到 RDK 的单一工作空间 `/root/ros2_ws`。

主要包：

```text
src/origincar/                 vendor 底盘、IMU、EKF、YDLIDAR 与消息包
src/third_party/obstacle_detector_2/  LiDAR 障碍物提取
src/smartcar_interfaces/       ReadQr、DescribeScene 服务接口
src/smartcar_safety/           fail-closed /cmd_vel 安全门
src/smartcar_nav2/             Nav2 配置、RPP、Smac Hybrid、航点
src/smartcar_vision/           zbar 与有界 VLM 服务
src/smartcar_speech/           可选火山 TTS 与本地音频播放
src/smartcar_task/             五子任务状态机
src/smartcar_bringup/          分层与一键系统 launch
```

## 本地开发与同步

在 Windows PowerShell：

```powershell
python -m unittest discover -s tests -v
python -m unittest discover -s src/smartcar_safety/test -v
python -m unittest discover -s src/smartcar_vision/test -v
python -m unittest discover -s src/smartcar_speech/test -v
python -m unittest discover -s src/smartcar_task/test -v
python scripts/sync_to_rdk.py push
```

`init-vendor` 只用于首次 bootstrap 或重新拉取官方包。日常以本地仓库为权威，使用 `push` 镜像 `src/` 和 `config/`。

## RDK 构建与测试

```bash
ssh root@192.168.128.10
source ~/source_env.sh
cd /root/ros2_ws
colcon build --symlink-install
colcon test-result --delete-yes
colcon test --return-code-on-test-failure
colcon test-result --all --verbose
```

当前全工作区结果为 16 个包构建通过，`533 tests, 0 errors, 0 failures, 90 skipped`。`ydlidar_ros2_driver`、`origincar_bringup`、`origincar_description` 的继承源码全量 lint 为显式 opt-in；需要清理 vendor 风格债务时使用 `colcon build --cmake-args -DSMARTCAR_ENABLE_VENDOR_LINT=ON`，不影响默认功能测试。

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

常用开关：`use_base`、`use_lidar`、`use_obstacle`、`use_safety`、`use_nav`、`use_camera`、`use_vision`、`use_task`、`use_speech`、`nav_autostart`、`safety_emergency_stop_on_start`、`use_sim_time`。`use_speech` 默认 `false`，不属于运动门禁。关闭 `use_base` 时仍会保留 safety 节点，适合无硬件 bench；物理底盘开启时系统强制要求 `use_safety=true`。

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
  /scan /odom /odom_combined /imu/data_raw /imu/data \
  /cmd_vel /cmd_vel_safe /ackermann_cmd /PowerVoltage \
  /tf /tf_static /smartcar/safety/status /smartcar/task/state \
  /barcode /smartcar/output/text /smartcar/output/speech \
  /smartcar/speech/status
```

关键输出：`/cmd_vel_safe`、`/smartcar/safety/status`、`/smartcar/task/state`、`/smartcar/vision/read_qr`、`/smartcar/vision/describe_scene`、`/smartcar/speech/status`。

更多设计与部署细节见 [`docs/deployment/rdk-environment-setup.md`](docs/deployment/rdk-environment-setup.md) 和 [`docs/智慧医疗挑战赛_技术方案_本地部署版.md`](docs/智慧医疗挑战赛_技术方案_本地部署版.md)。
