# RDK X5 部署与运行手册

更新时间：2026-07-24

目标板：无线地址由现场 DHCP 决定；2026-07-24 有线验证地址为 `root@192.168.128.10`

工作空间：`/root/ros2_ws`

## 1. 环境与连接

RDK 的 ROS 2 环境入口是 `/opt/tros/humble/setup.bash`。仓库提供的 `~/source_env.sh` 会加载 TROS 和 `/root/ros2_ws/install`，日常只使用这一入口：

```bash
ssh root@192.168.128.10
source ~/source_env.sh
```

不要再叠加 source `/userdata/dev_ws/install/setup.bash`。官方源码已经 vendor 化到本仓库，并统一在 `/root/ros2_ws` 构建，重复 overlay 可能加载旧包。

已确认硬件：

| 设备 | 接口 | 已验证状态 |
|---|---|---|
| OriginCar 底盘 | `/dev/ttyACM0`，115200 | 原始 `/odom`、`/imu/data_raw` 可用 |
| YDLIDAR Tmini Plus | `/dev/ttyUSB0`，230400 | `/scan` 约 10 Hz |
| Aurora/USB/MIPI 相机 | 驱动已安装 | 实体相机不得在无硬件 smoke 中启动 |

## 2. 同步、构建与测试

Windows PowerShell：

```powershell
python -m unittest discover -s tests -v
python scripts/sync_to_rdk.py push --dry-run
python scripts/sync_to_rdk.py push
```

RDK：

```bash
source ~/source_env.sh
cd /root/ros2_ws
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

清除历史 xunit 后运行完整工作区测试：

```bash
colcon test-result --delete-yes
colcon test --return-code-on-test-failure
colcon test-result --all --verbose
```

2026-07-24 当前证据：本地根合同 `134/134`；有线 RDK 上 `smartcar_safety smartcar_nav2 smartcar_task smartcar_bringup` 构建通过，清除历史 xunit 后为 `108 tests, 0 errors, 0 failures, 0 skipped`。该结果覆盖方向门、反向路径插件、任务 action adapter 和无底盘 launch smoke，不包含实体倒车或完整工作区验收。三个 vendor-only 包的继承源码全量 lint 默认为关闭，避免旧版权和格式问题污染功能测试；显式检查使用：

```bash
colcon build --cmake-args -DSMARTCAR_ENABLE_VENDOR_LINT=ON
colcon test --packages-select \
  origincar_bringup origincar_description ydlidar_ros2_driver
```

## 3. 无硬件完整 smoke

首选方式是运行 `smartcar_bringup` 的 launch test。测试自动执行以下安全顺序：

1. 使用独立 ROS 域和 localhost-only 通信。
2. 关闭底盘、LiDAR、障碍物、RF2O、实体相机和语音节点。
3. 启动合成 `/scan`、`/odom`、`/odom_combined` 与静态 TF。
4. 在 Nav2 lifecycle 激活前锁存急停。
5. 验证 `controller_server`、`planner_server`、`bt_navigator`、`velocity_smoother` 四个 Nav2 节点 active，vision/task 服务存在且任务为 `IDLE`。
6. 连续验证 `/cmd_vel_safe` 六个字段精确为零，`/ackermann_cmd` 没有底盘消费者。
7. 验证底盘、EKF、IMU、YDLIDAR、robot_state_publisher 和相机节点未启动。

```bash
source ~/source_env.sh
cd /root/ros2_ws
colcon test --packages-select smartcar_bringup
colcon test-result --test-result-base build/smartcar_bringup --verbose
```

该 bringup 测试不发布非零速度。`smartcar_nav2` 的独立隔离 launch test 会向 `/cmd_vel_nav` 注入软件测试命令，确认它可到达 `/cmd_vel_candidate`，但在没有方向租约时 `/cmd_vel` 和 `/cmd_vel_safe` 始终保持完整零；测试域中没有底盘节点或 `/ackermann_cmd` 消费者。

仅查看视觉和任务服务时可手工启动最小 bench：

```bash
export ROS_DOMAIN_ID=221
export ROS_LOCALHOST_ONLY=1

ros2 launch smartcar_bringup smartcar_system.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_obstacle:=false \
  use_safety:=true \
  safety_emergency_stop_on_start:=true \
  use_nav:=false \
  nav_autostart:=false \
  use_camera:=false \
  image_topic:=/smartcar/vision/image \
  use_vision:=true \
  use_task:=true \
  autostart_mission:=false
```

## 4. 系统启动

完整栈入口：

```bash
source ~/source_env.sh
ros2 launch smartcar_bringup smartcar_system.launch.py \
  autostart_mission:=false
```

默认开关：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `use_base` | `true` | 底盘、原始 IMU、EKF 和导航必需静态 TF |
| `use_lidar` | `true` | YDLIDAR `/scan` |
| `use_obstacle` | `false` | 可选 obstacle detector；costmap 避障不依赖它 |
| `use_laser_odometry` | `false` | 可选 RF2O `/scan` -> `/odom_laser` |
| `use_imu_filter` | `false` | 可选 Madgwick `/imu/data`；EKF 使用 `/imu/data_raw` |
| `use_robot_description` | `false` | 可选 URDF、robot/joint state publisher，用于 RViz 车型显示 |
| `use_safety` | `true` | 启动后置方向门与 `/cmd_vel` 到 `/cmd_vel_safe` 安全门 |
| `safety_emergency_stop_on_start` | `false` | 节点创建时立即锁存急停 |
| `use_nav` | `true` | Nav2 |
| `nav_autostart` | `true` | 自动激活 Nav2 lifecycle |
| `use_camera` | `true` | 实体相机驱动 |
| `use_vision` | `true` | QR/VLM 服务与 zbar |
| `use_task` | `true` | 任务状态机 |
| `use_speech` | `false` | 可选火山 TTS 与本地播放器 |
| `autostart_mission` | `false` | 不自动开始任务 |

`smartcar_system.launch.py` 禁止 `use_base=true,use_safety=false`。`use_laser_odometry=true` 时必须同时启用底盘和 LiDAR；任务自动启动还会要求 `laser_odometry_calibrated=true`。默认精简配置不启动无消费者的 obstacle extractor、Madgwick、URDF 车型发布、Nav2 path smoother、`behavior_server` 或 `waypoint_follower`。底层 `smartcar_bringup.launch.py` 的 `use_safety=false` 只保留给受控调试，不得用于竞赛运行。

当前 Nav2 启动封装仍声明并转发 `use_waypoint_follower`，但该参数不再控制任何运行时节点；`params_file`、BT XML 和 `waypoints_file` 等旧入口也有部分被 `nav2_params_fixed.yaml` 固定链路旁路。部署时以构建生成的固定参数文件、当前 BT 和任务节点加载的航点为准，不得通过这些遗留参数推断实际配置已经生效。

## 5. 校准数据与运动门禁

当前校准状态：

- 已测量 `base_footprint -> base_link`：`(0.0841, 0, 0.03)`。
- 已测量 `base_link -> laser`：`(-0.05, 0, 0.23)`，`laser_yaw=1.5708`。
- 已测量 `base_link -> camera`：`(0.1205, 0, 0.11)`。
- 已验证 `gyro_z_bias=0.000853` 和 `longitudinal_velocity_scale=1.03`。
- 尚未完成 `steering_command_scale` / `steering_command_offset` 和有效 `minimum_turning_radius` 标定。
- 激光里程计：RF2O `/odom_laser` 时间戳/协方差、`base_link -> laser` 外参、异常观测拒绝和退化回退阈值。
- 五子任务 11 点语义航点：`src/smartcar_nav2/config/waypoints/default_waypoints.yaml`
- 官方规则图参考尺寸：`src/smartcar_tools/config/routes/field_geometry.yaml`
- 转向参数和实车转弯半径：`origincar_base` 与 Nav2 参数

`src/smartcar_bringup/config/bringup_coord.yaml` 当前只是协调和审计记录，不会自动写入 launch。运行时必须通过 launch 参数或后续实测参数文件提供真实值。

五个门禁默认均为 `false`：

```text
waypoints_calibrated
extrinsics_calibrated
steering_calibrated
emergency_stop_ready
operator_approved
```

只有所有实测完成后，才可显式传入 `true`。启用激光里程计时还必须单独传入 `laser_odometry_calibrated=true`；未启用时不要求该条件门禁。`autostart_mission=true` 还要求 `use_base/use_safety/use_nav/use_vision/use_task/nav_autostart=true`。

### 5.1 定位需求变更（2026-07-19）

项目不引入 SLAM 或静态地图定位。RF2O 连续扫描匹配已作为默认关闭的可选观测接入，链路为：

```text
/odom + /imu/data_raw + /odom_laser -> robot_localization EKF -> /odom_combined
```

`/odom_laser` 由 RF2O 2D scan-to-scan 算法提供，只作为 EKF 观测；RF2O 配置为 `publish_tf: false`，`odom_combined -> base_footprint` 只能由 EKF 发布。同一 `/scan` 仍继续供 Nav2 rolling costmap 避障。没有 `map_server`、AMCL、SLAM 或静态栅格地图；RF2O 不提供绝对定位，轮速/IMU 仍是其关闭或退化时的回退来源。

代码接入和 RDK 构建通过不等于完成标定。`use_laser_odometry` 默认 `false`，首次启用前必须用实测数据验证时间戳、外参、协方差、漂移、异常观测拒绝和回退；完成后才可设置 `laser_odometry_calibrated=true`。复位服务为 `/smartcar/localization/reset_laser_odometry`。

## 6. 航点、倒车与三个媒体分项测试

仓库只保留 `default_waypoints.yaml` 一份 11 点语义路线：P -> QR 留距位 -> 两个出站通道点 -> C 区角 1/VLM -> 角 2 -> 角 3 -> 角 4 -> 两个回程通道点 -> P。它明确标记 `calibrated: false`，规则图推算不能替代现场实测。

路线方向合同是固定的：start/QR 为 `forward`；QR 后两个出站 corridor 和 VLM 为 `reverse`；VLM 后至 return 全部为 `forward`。任务为每个非起点航点独立发送 `NavigateToPose`，不使用 `FollowWaypoints`。`validate_waypoints()` 会拒绝全正向或方向越界的 YAML。

倒车使用单一 DUBIN planner 和专用 BT：插件从 TF 获取实际起点，把起点/目标 yaw 临时加 π 后规划，再恢复路径 yaw；路径必须 frame、端点、四元数、反向投影、无 cusp 和曲率全部合规。随后动作 UUID 绑定的方向租约只允许负 `linear.x` 通过后置方向门。这里的保证是“严格倒车，否则完整零输出”，不是“实车路线已经通过”。

速度链和唯一所有者：

```text
controller_server -> /cmd_vel_nav
velocity_smoother -> /cmd_vel_candidate
direction_guard -> /cmd_vel
smartcar_safety -> /cmd_vel_safe + /ackermann_cmd
```

首次无视觉测试使用：

```bash
bash /root/nav_test.sh
```

脚本会清理、增量构建、启动无相机/无视觉系统并打开 RViz，但设置 `autostart_mission:=false` 和 `safety_emergency_stop_on_start:=true`，不会自动发车。必须先确认物理急停、车辆周围、RViz、P 点朝向和车轮离地条件，再人工 reset、解除急停、start。当前只测试到 VLM 后立即急停；`minimum_turning_radius: 0.55` 未标定，且 `c_corner_1 -> c_corner_2`、`c_corner_4 -> b_corridor_return_enter` 两个正向段存在解析兜圈风险，未修正前不得无人看守跑完整圈。

只允许把仓库根目录的 `scripts/nav_test.sh` 部署为 `/root/nav_test.sh`。`scripts/deploy/nav_test.sh` 是仍会解除急停并自动 start 的历史副本，在清理前不得复制或执行。当前 `scripts/monitor_mission.py` 也不能作为安全判据：其消息类型和格式化逻辑尚未与当前话题合同同步；安全确认必须直接观察下述 ROS 话题和物理状态。

`smartcar_tools` 提供以下隔离入口：

| 入口 | 启动内容 | 明确不启动 |
|---|---|---|
| `waypoint_editor.launch.py` | 官方场地参考层、语义航点交互编辑器；`use_rviz` 默认开启，RDK 标定须显式 `start_safety:=true` 锁存急停 | 底盘、LiDAR、Nav2、相机、视觉与语音 |
| `speech_test.launch.py` | 火山 TTS consumer | 底盘、Nav2、视觉与任务 |
| `qr_test.launch.py` | 指定相机或单图回放、zbar、QR 服务 | 底盘、Nav2、任务与语音 |
| `vlm_test.launch.py` | 指定相机或单图回放、VLM 服务、PyQt5 HDMI UI | 底盘、Nav2、任务与语音 |

参考层只发布 `odom_combined` 下的 Marker，不发布 `/map`、TF，也不加入 Nav2 costmap。分段编辑、途经点、无朝向、HDMI 环境变量、预检和保存见 [`waypoint-editor.md`](waypoint-editor.md)。入口隔离和合同测试只证明软件结构，不证明相机、云端 API、音响、RF2O、路线或实车运动已经通过现场验收。

## 7. 任务、停止和复位

```bash
ros2 service call /smartcar/task/start std_srvs/srv/Trigger "{}"
ros2 service call /smartcar/task/stop std_srvs/srv/Trigger "{}"
ros2 service call /smartcar/task/reset std_srvs/srv/Trigger "{}"
```

任务状态：`IDLE`、`WAITING_FOR_SERVERS`、`NAVIGATING`、`RUNNING_QR`、`RUNNING_VLM`、`COMPLETED`、`STOPPED`、`FAILED`。

`reset` 的前置条件和顺序：

1. 方向租约已撤销，`NavigateToPose` action 已确认终态，原始 `/odom` 六轴连续停稳。
2. 车辆物理位于 P 区原点，车头朝 `+X`。
3. 调用 robot_localization `/set_pose`。
4. 等待更新且有限的 `/odom_combined` 接近原点后返回 `IDLE`。

## 8. 急停与故障恢复

锁存急停：

```bash
ros2 service call /smartcar/safety/emergency_stop \
  std_srvs/srv/SetBool "{data: true}"
```

解除急停前必须同时确认：物理急停可触达；任务没有未确认 action；`/smartcar/direction_guard/status` 为 `stopped`；`/smartcar/safety/status` 只报告预期的 `emergency_stop` 而非传感器故障；`/cmd_vel`、`/cmd_vel_safe` 和最终 `/ackermann_cmd.drive.speed` 连续为零；速度话题没有竞争发布者。仅看 `/cmd_vel_safe` 或第三方监控界面的“0”不充分。

```bash
ros2 topic echo /smartcar/direction_guard/status --once
ros2 topic echo /smartcar/safety/status --once
ros2 topic echo /ackermann_cmd
ros2 topic info /cmd_vel --verbose
ros2 topic info /ackermann_cmd --verbose
```

确认完成后才可解除：

```bash
ros2 service call /smartcar/safety/emergency_stop \
  std_srvs/srv/SetBool "{data: false}"
```

`/smartcar/task/stop` 不等于急停。串口断线、无新鲜 `/scan`、无新鲜原始/融合里程计或非法速度都会让安全门 fail-closed。底盘进程关闭时会复用当前串口发送一次零命令；仍必须准备人工可触达的物理急停。

## 9. 视觉、火山 VLM 与语音合成

相机选择：`camera_driver:=usb|aurora|mipi`。当前默认 USB；Aurora 和 MIPI 仅作备选。

`vision.yaml` 默认 `vlm_backend_mode: disabled`，因此未选择后端时 DescribeScene 返回兜底文案。`vision_volcengine.yaml` 是显式启用的火山 Ark 配置：它使用 Python 3 标准库调用 OpenAI-compatible HTTPS 接口，不需要安装 Ark SDK，且仍由外层无 shell 命令后端强制终止。整个请求包含图像等待、JPEG 编码和后端推理，共享硬上限 8 秒；公网响应过慢时按既有合同返回“检测到人物立牌”。

凭据只从环境变量读取，不得写入 YAML、launch、源码或 rosbag。建议放入权限 `0600` 的现场环境文件，再由 `~/source_env.sh` 加载：

```bash
export ARK_API_KEY='<火山 Ark API key>'
export VOLC_ARK_MODEL='doubao-1-5-vision-pro-32k-250115'
export VOLCENGINE_TTS_APP_ID='<火山语音应用 ID>'
export VOLCENGINE_TTS_ACCESS_TOKEN='<火山语音 access token>'
```

VLM 兼容参考工程使用的 `DOUBAO_KEY`，但新部署优先使用 `ARK_API_KEY`。模型可通过 `VOLC_ARK_MODEL` 覆盖，基础地址仅在确有区域差异时通过 `VOLC_ARK_BASE_URL` 覆盖；客户端拒绝非 HTTPS、URL 内凭据和 HTTP 重定向，避免认证头跨主机泄露。

云端方案投入比赛前必须确认比赛规则允许访问公网，并在实际赛场网络下统计成功率和端到端耗时。公网不可用或 8 秒期限内稳定性不足时，只会得到既定兜底文案，不能替代有效的人物描述；这种情况下必须切换到经过实测的端侧 VLM 后端。

首次 API 和音频 bench 必须关闭底盘、LiDAR、障碍物驱动、Nav2 和实体相机，并锁存急停：

```bash
VISION_CONFIG="$(ros2 pkg prefix smartcar_vision)/share/smartcar_vision/config/vision_volcengine.yaml"

ros2 launch smartcar_bringup smartcar_system.launch.py \
  vision_config_file:="$VISION_CONFIG" \
  use_base:=false \
  use_lidar:=false \
  use_obstacle:=false \
  use_safety:=true \
  safety_emergency_stop_on_start:=true \
  use_nav:=false \
  nav_autostart:=false \
  use_camera:=false \
  image_topic:=/smartcar/vision/image \
  use_task:=false \
  use_speech:=true \
  autostart_mission:=false
```

这个 bench 只等待外部或合成图像和手工发布的语音文本，不会启动实体相机或任务运动。完成 API、扬声器、现场网络和全部运动门禁验收后，才可把同一 `vision_config_file` 与 `use_speech:=true` 带入完整系统启动。

`smartcar_speech` 订阅 `/smartcar/output/speech`，用火山 V1 同步 TTS 合成 MP3，再通过 RDK 已安装的 `ffplay` 播放。启动前检查：

```bash
command -v ffplay
ros2 topic echo /smartcar/speech/status
```

状态为 JSON，包含 `state`、`request_id`、`detail`；正常顺序为 `queued -> synthesizing -> playing -> completed`，配置缺失、API 错误、队列满或播放器错误分别报告 `unconfigured`、`failed` 或 `dropped`。任务状态机对语音采用 fire-and-forget，不会等待 `completed` 后再导航；当前比赛流程若要求严格“播完再走”，需要后续增加 action/ack 协议，不能把状态话题当同步确认。

本仓库自动化测试不会访问火山 API，也不会启动播放器。真实 API、现场网络和扬声器测试必须在保持任务不自动启动、车辆静止的人工 bench 中单独验收。

VFH 备用避障和 YOLO 自动触发未纳入当前 release；人物描述由语义航点 `task: vlm` 调用。

## 10. 录包与检查

```bash
mkdir -p /root/ros2_ws/bags
ros2 bag record -o /root/ros2_ws/bags/$(date +%Y%m%d-%H%M%S) \
  /scan /odom /odom_laser /odom_combined /imu/data_raw /imu/data \
  /cmd_vel_nav /cmd_vel_candidate /cmd_vel /cmd_vel_safe \
  /ackermann_cmd /PowerVoltage \
  /tf /tf_static /smartcar/direction_guard/status \
  /smartcar/safety/status /smartcar/task/state \
  /barcode /smartcar/output/text /smartcar/output/speech \
  /smartcar/speech/status
```

常用检查：

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /odom_laser
ros2 topic hz /odom_combined
ros2 topic echo /smartcar/safety/status
ros2 topic echo /smartcar/direction_guard/status
ros2 topic echo /smartcar/task/state
ros2 topic echo /ackermann_cmd
ros2 topic info /cmd_vel --verbose
ros2 topic info /ackermann_cmd --verbose
ros2 service list | grep /smartcar
ros2 action list | grep navigate_to_pose
```

## 11. 物理测试顺序

1. 自动化测试和无硬件 smoke 全部通过。
2. 复核现有外参、轮速和陀螺仪标定，完成转向比例、偏置和有效转弯半径标定；如启用 RF2O，单独完成激光里程计数据质量和回退验证。
3. 用 `waypoint_editor.launch.py` 对照官方参考层标定唯一语义路线，现场逐点复核期间保持 `calibrated: false`。
4. 人工物理急停可用，车轮离地测试。
5. 低速直线、低速转向、急停、串口断线分别验证。
6. 先在车轮离地条件下验证正向租约只出正速度、QR→VLM 租约只出负速度以及 STOP 立即归零。
7. 低速地面只跑 P→QR→VLM，抵达 VLM 立即急停；完成两处正向航向修正后，再按 `0.15 m/s` 逐段验证完整路线。
8. 最后进行视觉、语音和完整五子任务联调。

在上述步骤完成前，软件状态只能称为“代码合同完成、待物理标定与实车验证”。
