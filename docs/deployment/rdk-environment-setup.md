# RDK X5 部署与运行手册

更新时间：2026-07-22

目标板：无线 `root@172.16.25.27`；有线备用 `root@192.168.128.10`

工作空间：`/root/ros2_ws`

## 1. 环境与连接

RDK 的 ROS 2 环境入口是 `/opt/tros/humble/setup.bash`。仓库提供的 `~/source_env.sh` 会加载 TROS 和 `/root/ros2_ws/install`，日常只使用这一入口：

```bash
ssh root@172.16.25.27
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

2026-07-19 历史验证结果：18 个包构建通过，清除旧 xunit 后的全量结果为 `578 tests, 0 errors, 0 failures, 90 skipped`；其中仍包含后来删除的 68 点路线和独立纯导航测试链，不能作为当前 9 点路线版本的验证证据。当前版本须重新执行上述构建、测试和 smoke。三个 vendor-only 包的继承源码全量 lint 默认为关闭，避免旧版权和格式问题污染功能测试；显式检查使用：

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
5. 验证七个 Nav2 节点 active、vision/task 服务存在、任务为 `IDLE`。
6. 连续验证 `/cmd_vel_safe` 六个字段精确为零。
7. 验证底盘、EKF、IMU、YDLIDAR、robot_state_publisher 和相机节点未启动。

```bash
source ~/source_env.sh
cd /root/ros2_ws
colcon test --packages-select smartcar_bringup
colcon test-result --test-result-base build/smartcar_bringup --verbose
```

该测试不发布非零速度。安全门压制非零命令的逻辑由 `smartcar_safety` 单元测试覆盖。

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
| `use_safety` | `true` | `/cmd_vel` 到 `/cmd_vel_safe` 安全门 |
| `safety_emergency_stop_on_start` | `false` | 节点创建时立即锁存急停 |
| `use_nav` | `true` | Nav2 |
| `nav_autostart` | `true` | 自动激活 Nav2 lifecycle |
| `use_camera` | `true` | 实体相机驱动 |
| `use_vision` | `true` | QR/VLM 服务与 zbar |
| `use_task` | `true` | 任务状态机 |
| `use_speech` | `false` | 可选火山 TTS 与本地播放器 |
| `autostart_mission` | `false` | 不自动开始任务 |

`smartcar_system.launch.py` 禁止 `use_base=true,use_safety=false`。`use_laser_odometry=true` 时必须同时启用底盘和 LiDAR；任务自动启动还会要求 `laser_odometry_calibrated=true`。默认精简配置不启动无消费者的 obstacle extractor、Madgwick、URDF 车型发布和 Nav2 path smoother；完整任务仍按需启动 `waypoint_follower`。底层 `smartcar_bringup.launch.py` 的 `use_safety=false` 只保留给受控调试，不得用于竞赛运行。

## 5. 校准数据与运动门禁

以下值均需实测：

- `base_footprint -> base_link`：`base_x/base_y/base_z/base_roll/base_pitch/base_yaw`
- `base_link -> laser`：`laser_x/laser_y/laser_z/laser_roll/laser_pitch/laser_yaw`
- `base_link -> camera`：`camera_x/camera_y/camera_z/camera_roll/camera_pitch/camera_yaw`
- 激光里程计：RF2O `/odom_laser` 时间戳/协方差、`base_link -> laser` 外参、异常观测拒绝和退化回退阈值。
- 五子任务语义航点：`src/smartcar_nav2/config/waypoints/default_waypoints.yaml`
- 官方规则图参考尺寸：`src/smartcar_tools/config/routes/field_geometry.yaml`
- 转向、轮速、陀螺仪参数：`origincar_base` launch 参数

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

## 6. 航点编辑与三个媒体分项测试

仓库只保留 `default_waypoints.yaml` 一份 9 点语义路线：P -> QR 留距位 -> 出站通道 -> C 区角 1/VLM -> 角 2 -> 角 3 -> 角 4 -> 回程通道 -> P。它明确标记 `calibrated: false`，规则图推算不能替代现场实测。

`smartcar_tools` 提供以下隔离入口：

| 入口 | 启动内容 | 明确不启动 |
|---|---|---|
| `waypoint_editor.launch.py` | 官方场地参考层、语义航点交互编辑器、RViz、锁存急停 | 底盘、LiDAR、Nav2、相机、视觉与语音 |
| `speech_test.launch.py` | 火山 TTS consumer | 底盘、Nav2、视觉与任务 |
| `qr_test.launch.py` | 指定相机或单图回放、zbar、QR 服务 | 底盘、Nav2、任务与语音 |
| `vlm_test.launch.py` | 指定相机或单图回放、VLM 服务、PyQt5 HDMI UI | 底盘、Nav2、任务与语音 |

参考层只发布 `odom_combined` 下的 Marker，不发布 `/map`、TF，也不加入 Nav2 costmap。拖拽编辑、右键保存、HDMI 环境变量和 `pull-waypoints` 见 [`field-test-entrypoints.md`](field-test-entrypoints.md)。入口隔离和合同测试只证明软件结构，不证明相机、云端 API、音响、RF2O、路线或实车运动已经通过现场验收。

## 7. 任务、停止和复位

```bash
ros2 service call /smartcar/task/start std_srvs/srv/Trigger "{}"
ros2 service call /smartcar/task/stop std_srvs/srv/Trigger "{}"
ros2 service call /smartcar/task/reset std_srvs/srv/Trigger "{}"
```

任务状态：`IDLE`、`WAITING_FOR_SERVERS`、`NAVIGATING`、`RUNNING_QR`、`RUNNING_VLM`、`COMPLETED`、`STOPPED`、`FAILED`。

`reset` 的前置条件和顺序：

1. 导航 worker 与 FollowWaypoints action 已确认停止。
2. 车辆物理位于 P 区原点，车头朝 `+X`。
3. 调用 robot_localization `/set_pose`。
4. 等待更新且有限的 `/odom_combined` 接近原点。
5. 调用 `/smartcar/safety/clear_localization_fault`。

不要直接调用 clear service 绕过 task reset。

## 8. 急停与故障恢复

锁存急停：

```bash
ros2 service call /smartcar/safety/emergency_stop \
  std_srvs/srv/SetBool "{data: true}"
```

解除急停前必须确认车辆周围安全、导航目标已取消且 `/cmd_vel_safe` 为零：

```bash
ros2 service call /smartcar/safety/emergency_stop \
  std_srvs/srv/SetBool "{data: false}"
```

`/smartcar/task/stop` 不等于急停。串口断线、无新鲜 `/scan`、无新鲜原始/融合里程计、非法速度或定位故障都会让安全门 fail-closed。底盘进程关闭时会复用当前串口发送一次零命令；仍必须准备人工可触达的物理急停。

## 9. 视觉、火山 VLM 与语音合成

相机选择：`camera_driver:=aurora|usb|mipi`。默认 Aurora 只启用 15 FPS RGB，关闭 depth/IR/point cloud。

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
  /cmd_vel /cmd_vel_safe /ackermann_cmd /PowerVoltage \
  /tf /tf_static /smartcar/safety/status /smartcar/task/state \
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
ros2 topic echo /smartcar/task/state
ros2 service list | grep /smartcar
ros2 action list | grep follow_waypoints
```

## 11. 物理测试顺序

1. 自动化测试和无硬件 smoke 全部通过。
2. 测量外参，完成转向、轮速和 IMU 标定；如启用 RF2O，单独完成激光里程计数据质量和回退验证。
3. 用 `waypoint_editor.launch.py` 对照官方参考层标定唯一语义路线，现场逐点复核期间保持 `calibrated: false`。
4. 人工物理急停可用，车轮离地测试。
5. 低速直线、低速转向、急停、串口断线分别验证。
6. 通过正式任务链按 QR 段、VLM 段、返程段和 `0.15 m/s` 全程逐级验证。
7. 最后进行视觉、语音和完整五子任务联调。

在上述步骤完成前，软件状态只能称为“代码合同完成、待物理标定与实车验证”。
