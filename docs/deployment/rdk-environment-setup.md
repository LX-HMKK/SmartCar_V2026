# RDK X5 部署与运行手册

更新时间：2026-08-03

目标板：无线地址由现场 DHCP 决定；2026-07-24 有线验证地址为 `root@192.168.128.10`

工作空间：`/root/ros2_ws`

## 1. 环境与连接

RDK 的 ROS 2 环境入口是 `/opt/tros/humble/setup.bash`。仓库提供的 `~/source_env.sh` 会加载 TROS 和 `/root/ros2_ws/install`，日常只使用这一入口：

```bash
ssh root@<RDK_IP>
source ~/source_env.sh
```

不要再叠加 source `/userdata/dev_ws/install/setup.bash`。官方源码已经 vendor 化到本仓库，并统一在 `/root/ros2_ws` 构建，重复 overlay 可能加载旧包。

### USB 有线直连（RNDIS）

RDK 的 USB Device 网口使用静态地址，不提供 DHCP。本机通过 USB 连接后，RDK 侧
`usb0` 为 `192.168.128.10/24`；本机应使用同网段的不同地址。当前开发机已保存以下
NetworkManager 配置，正常插线后等待数秒即可：

```bash
ping 192.168.128.10
ssh root@192.168.128.10
```

若未自动连接，激活已保存的连接：

```bash
nmcli connection up '有线连接 5'
```

若更换主机或该连接被删除，先用 `nmcli device status` 找到 RNDIS 对应的有线连接名，再将
该连接改为静态地址（本机当前使用 `192.168.128.100`，不得填写 `192.168.128.10`）：

```bash
nmcli connection modify '有线连接 5' \
  ipv4.method manual \
  ipv4.addresses 192.168.128.100/24 \
  ipv4.gateway '' \
  ipv4.never-default yes \
  ipv6.method disabled
nmcli connection up '有线连接 5'
```

`ipv4.never-default yes` 很重要：USB 直连只用于 RDK 网段，不能抢占本机 Wi-Fi 的默认路由。

诊断时，以下特征表示 USB 网卡驱动正常，而无 IPv4 通常只是误设为 DHCP：

```bash
lsusb -t                 # 应显示 rndis_host
ip -brief addr           # 应显示 RNDIS 网卡的 192.168.128.100/24
nmcli device status      # 不应长期停在“正在获取 IP 配置”
```

在本机 2026-08-03 的已验证设备中，USB 设备为 `0525:a4a2 Linux-USB Ethernet/RNDIS Gadget`，
内核驱动为 `rndis_host`。驱动已绑定但 NetworkManager 长期等待 DHCP 时，不是 RNDIS 驱动故障；
按上述静态地址配置后，再测试 `ping 192.168.128.10`。

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

2026-08-03 本机根合同为 `212` 项通过。2026-07-24 的有线 RDK 历史证据为 `smartcar_safety smartcar_nav2 smartcar_task smartcar_bringup` 构建通过，清除历史 xunit 后 `108 tests, 0 errors, 0 failures, 0 skipped`。该记录覆盖方向门、反向路径插件、任务 action adapter 和无底盘 launch smoke，不包含实体倒车或完整工作区验收。三个 vendor-only 包的继承源码全量 lint 默认为关闭，避免旧版权和格式问题污染功能测试；显式检查使用：

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
| `use_laser_odometry` | `false` | 可选 RF2O `/scan` -> `/odom_laser` |
| `use_imu_filter` | `false` | 可选 Madgwick `/imu/data`；EKF 使用 `/imu/data_raw` |
| `use_robot_description` | `false` | 可选 URDF、robot/joint state publisher，用于 RViz 车型显示 |
| `use_safety` | `true` | 启动后置方向门与 `/cmd_vel` 到 `/cmd_vel_safe` 安全门 |
| `safety_emergency_stop_on_start` | `false` | 节点创建时立即锁存急停 |
| `use_nav` | `true` | Nav2 |
| `nav_autostart` | `true` | 自动激活 Nav2 lifecycle |
| `use_camera` | `true` | 实体相机驱动 |
| `use_vision` | `true` | QR/VLM 服务；zbar 由任务在 QR 点按需启动 |
| `use_task` | `true` | 任务状态机 |
| `use_visualization` | `false` | 显式启动场地参考和航点 Marker |
| `use_speech` | `false` | 可选火山 TTS 与本地播放器 |
| `autostart_mission` | `false` | 不自动开始任务 |

`smartcar_system.launch.py` 与底层 bringup 都禁止 `use_base=true` 时关闭 safety 或 safety Ackermann 输出。`use_laser_odometry=true` 时必须同时启用底盘和 LiDAR；任务自动启动还会要求 `laser_odometry_calibrated=true`。默认精简配置不启动 Madgwick、URDF 车型发布、场地 Marker、Nav2 path smoother、`behavior_server` 或 `waypoint_follower`。

`navigation_launch.py` 是唯一 Nav2 节点启动入口。它接收已解析的 `params_file` 和可选 `params_overlay_file`，并由构建生成的 `nav2_params_fixed.yaml` 固定默认 BT 路径；航点始终由任务节点加载。`use_waypoint_follower`、BT XML 覆盖和 Nav2 航点文件入口均已删除，不得依赖这些旧接口。

## 5. 校准数据与运动门禁

当前校准状态：

- 已测量 `base_footprint -> base_link`：`(0.0841, 0, 0.03)`。
- 已测量 `base_link -> laser`：`(-0.05, 0, 0.23)`，`laser_yaw=1.5708`。
- 已测量 `base_link -> camera`：`(0.1205, 0, 0.11)`。
- 2026-08-05 车辆静止 60 秒、1200 样本复标后，使用
  `gyro_z_bias=-0.00036369`；`longitudinal_velocity_scale=1.03` 保持不变。
- 已验证转向直通配置：`steering_command_scale=1.0`、`steering_command_offset_rad=0.0`，
  `0.70 rad` 指令可到达下位机；0.7 rad 地面画圆卷尺测得 `R_min ≈ 0.20 m`。运行时
  Nav2 与路线预检使用保守的 `minimum_turning_radius=0.22 m`，不等同于完整路线验收。
- 激光里程计：RF2O `/odom_laser` 时间戳/协方差、`base_link -> laser` 外参、异常观测拒绝和退化回退阈值。
- 五子任务 7 点语义航点：`src/smartcar_nav2/config/waypoints/default_waypoints.yaml`
- 官方规则图参考尺寸：`src/smartcar_tools/config/routes/field_geometry.yaml`
- 转向参数和实车转弯半径：`origincar_base` 与 Nav2 参数

`src/smartcar_bringup/config/bringup_coord.yaml` 是协调和审计记录；顶层 launch、底盘默认值和
安全节点均已同步上述转向配置。运行时仍可用 launch 参数覆盖，但不得据此绕开运动门禁。

五个门禁默认均为 `false`：

```text
waypoints_calibrated
extrinsics_calibrated
steering_calibrated
emergency_stop_ready
operator_approved
```

只有所有实测完成后，才可显式传入 `true`。启用激光里程计时还必须单独传入 `laser_odometry_calibrated=true`；未启用时不要求该条件门禁。系统和任务配置均默认 `autostart_mission=false`，不得将自动发车作为实车测试入口。

### 5.1 定位需求变更（2026-07-19）

项目不引入 SLAM 或静态地图定位。RF2O 连续扫描匹配已作为默认关闭的可选观测接入，链路为：

```text
/odom + /imu/data_raw + /odom_laser -> robot_localization EKF -> /odom_combined
```

`/odom_laser` 由 RF2O 2D scan-to-scan 算法提供，只作为 EKF 观测；RF2O 配置为 `publish_tf: false`，`odom_combined -> base_footprint` 只能由 EKF 发布。同一 `/scan` 仍继续供 Nav2 rolling costmap 避障。没有 `map_server`、AMCL、SLAM 或静态栅格地图；RF2O 不提供绝对定位，轮速/IMU 仍是其关闭或退化时的回退来源。

代码接入和 RDK 构建通过不等于完成标定。`use_laser_odometry` 默认 `false`，首次启用前必须用实测数据验证时间戳、外参、协方差、漂移、异常观测拒绝和回退；完成后才可设置 `laser_odometry_calibrated=true`。复位服务为 `/smartcar/localization/reset_laser_odometry`。

## 6. 航点、全正向路线与三个媒体分项测试

`default_waypoints.yaml` 使用四个显式 planning segment、14 个航点约束的语义路线：P -> QR；QR
经 `a_departure_exit`、`via_2_entry`、`via_2_corridor` 到 `via_2`；`via_2` -> C1/VLM；C1 经
`c_north_1`、`c_north_2`、`via_1`、`via_3`、`return_corridor_exit`、`p_return_approach` 回 P。
它明确标记 `calibrated: false`，仿真通过和规则图坐标都不能替代现场实测。

实车语义路线与受看护纯导航 `nav_only.yaml` 共享 `planning_segments`、航点 ID、全正向方向、goal
profile 和除 A 外的姿态；前者的 A 为 QR 语义点，后者的 A 为 P→A 停点，二者需要分别复测。
两份 YAML 均标记 `calibrated: false`。前者在 A/C1 恢复 `qr`/`vlm` 任务，后者以 `nav` 替代
它们以跳过媒体服务。四段均为正向：P→A、A 经三个 `via` 到 `via_2`、`via_2`→C1、C1 经六个
`via` 回 P；A/C1 使用 `precise` profile，P 终点使用 `standard` profile。任务按
`planning_segments` 执行：单目标段发送 `NavigateToPose`，同向多目标段发送
`NavigateThroughPoses`，不使用 `FollowWaypoints`。两份当前 YAML 的 `planning_segments` 和经其
物化的航点均为 `forward`，并由合同测试锁定。路线已通过离线几何预检和本机 Gazebo 全路线校验
`run_20260807_170954_1`，但仍未完成 RDK 或实体车辆验收。

### 6.1 历史受看护纯导航现场记录（2026-08-04）

RDK `172.16.24.164` 上的受看护记录中，前一版含倒车段路线的三个纯导航动作均报告成功：P→A
`12.10 s`、A→`via_2`→C1 `26.90 s`、C1→`via_1`→`via_3`→P `32.17 s`，终态为
`mission_completed`。但操作员观察到 P→A 在完成判定前越过物理 QR/A 点；这表明定位或
场地坐标仍需复测，不能通过放宽 goal checker 掩盖。该记录不能验证当前全正向 YAML。完整证据见
[`rdk-field-navigation-test-2026-08-04.md`](../review/rdk-field-navigation-test-2026-08-04.md)。

该记录只覆盖无相机、无二维码、无 VLM 的旧 `task: nav` 路线。测试后 `nav_only.yaml` 已恢复为
`calibrated: false`；语义路线 `default_waypoints.yaml` 同样保持 `calibrated: false`。五项默认
运动门禁和完整语义任务验收状态均未因这次受看护测试改变；当前全正向路线没有继承该现场验收。

当前路线使用单一 DUBIN planner 和正向行为树；A/C1 单目标使用精确终点树，含经过点的动作使用
正向 ThroughPoses 树。动作 UUID 绑定的方向租约只允许正 `linear.x` 通过后置方向门。保留的反向
专用 BT 仍会把起点/目标 yaw 临时加 π 后规划并验证 frame、端点、四元数、反向投影、无 cusp 与曲率，但
当前路线不会调用它。两种软件约束均不等同于实车路线已经通过。

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

脚本会清理、增量构建、启动无相机/无视觉系统并打开 RViz，但固定设置 `autostart_mission:=false` 和 `safety_emergency_stop_on_start:=true`，没有自动发车选项。它还会验证两个 costmap 的 KeepoutFilter、规则掩膜和 filter-info 服务；任一项失败都会保持急停锁存并退出。默认 `nav_only.yaml` 未标定，因而普通 `start` 会被拒绝。只有显式 `--p-to-a --supervised-p-to-a` 的受看护单段复验才会临时满足该固定前缀的门禁；它仍不解除急停、不自动发车，也不授权 C1 或全路线。运行配置的 `minimum_turning_radius` 使用保守的 `0.22 m`，线速度为 `0.30 m/s`。当前全正向路线、媒体服务和完整五子任务仍未验证，任何路线测试均不得无人看守。

只允许把仓库根目录的 `scripts/nav_test.sh` 部署为 `/root/nav_test.sh`。任何旧 RDK 副本中会自动解除急停或调用 start 的脚本都不得恢复或执行。历史的 `scripts/safe_start.sh`、`scripts/deploy/safe_start.sh`、`scripts/deploy/ros_cleanup.sh`、`scripts/verify_autostart.sh` 和 `scripts/monitor_mission.py` 已移除，不得从旧 RDK 副本恢复使用。安全确认必须直接观察下述 ROS 话题和物理状态。

`smartcar_tools` 提供以下隔离入口：

| 入口 | 启动内容 | 明确不启动 |
|---|---|---|
| `waypoint_editor.launch.py` | 官方场地参考层、语义航点交互编辑器；`use_rviz` 默认开启，RDK 标定须显式 `start_safety:=true` 锁存急停 | 底盘、LiDAR、Nav2、相机、视觉与语音 |
| `speech_test.launch.py` | 火山 TTS consumer | 底盘、Nav2、视觉与任务 |
| `qr_test.launch.py` | 指定相机或单图回放、zbar、QR 服务 | 底盘、Nav2、任务与语音 |
| `vlm_test.launch.py` | 指定相机或单图回放、VLM 服务、PyQt5 HDMI UI | 底盘、Nav2、任务与语音 |

参考层只发布 `odom_combined` 下的 Marker，不发布 `/map`、TF，也不加入 Nav2 costmap。分段编辑、途经点、无朝向、HDMI 环境变量、预检和保存见 [`waypoint-editor.md`](waypoint-editor.md)。入口隔离和合同测试只证明软件结构，不证明相机、云端 API、音响、RF2O、路线或实车运动已经通过现场验收。

转向标定命令不属于隔离 launch：`steering_circle_drive` 需要完整的
`velocity_smoother -> direction_guard -> smartcar_safety` 运动链和前进方向租约；
`steering_hold` 只请求安全节点默认关闭的、零速度且限时的静态量角服务。两者都要求
人工确认，且不修改任何运动门禁。命令、上限和现场前置条件见
[`../reference/field-tools.md`](../reference/field-tools.md)。

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

`/smartcar/task/stop` 不等于急停。串口断线、当前避障源（普通模式 `/scan`、深度模式
`/smartcar/depth/points`）不新鲜、无新鲜原始/融合里程计或非法速度都会让安全门 fail-closed。
底盘进程关闭时会复用当前串口发送一次零命令；仍必须准备人工可触达的物理急停。

## 9. 视觉、火山 VLM 与语音合成

相机选择：`camera_driver:=usb|aurora|mipi`。当前默认 USB，默认话题为 `/image`；Aurora
和 MIPI 仅作备选。顶层启动会解析 `image_topic` 并将同一话题交给 vision 服务和任务按需
启动的 zbar，切换驱动或显式覆盖话题时不需要维护第二份二维码 remap。

Aurora 930 可作为唯一的 Nav2 动态障碍物来源，但必须显式开启：

```bash
bash /root/nav_test.sh --depth-camera
```

该入口关闭 LiDAR/RF2O，使 Aurora 发布深度点云，并通过 `depth_pointcloud_relay` 验证 frame、
点云布局和有限浮点 XYZ 样本。Aurora 930 1.7.2 的源时间戳乘以 `1000` 后与 ROS 时钟对齐，再发布
`/smartcar/depth/points`。深度模式选择
`depth_camera_obstacle_overlay.yaml`，两个 costmap 的 `observation_sources` 均为
`depth_points`，不包含 `/scan`；safety 同样以该点云作为唯一避障心跳。深度相机安装在前轴正上方、
离地 `0.15 m`，其相对 `base_link` 的位置为 `(0.1049, 0, 0.12)`；点云朝向和障碍覆盖范围尚未
实测。脚本在急停保持锁存时检查点云、两张 costmap 和 KeepoutFilter；
`depth_camera_calibrated` 与全部运动门禁仍默认 `false`，所以该步骤不允许发车。完成 Aurora 外参、
frame、量程、清障和实车避障验收后，才可按既有门禁流程进行受看护运动测试。

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
3. 用 `waypoint_editor.launch.py` 对照官方参考层标定语义路线 `default_waypoints.yaml`，现场逐点复核期间保持 `calibrated: false`。
4. 人工物理急停可用，车轮离地测试。
5. 低速直线、低速转向、急停、串口断线分别验证。
6. 先在车轮离地条件下验证四个正向动作的租约只出正速度，以及 STOP 立即归零。
7. 历史 RDK 受看护纯导航记录中的三段均报告成功，但它对应旧倒车路线且 P→A 已观察到越过物理 QR/A 点；先完成当前全正向 P→A 单段测量并修正坐标或里程计，再分别验证 A→`via_2`、`via_2`→C1、C1→P 和媒体任务。
8. 最后进行视觉、语音和完整五子任务联调。

在当前全正向路线的实体复验、媒体分项和完整五子任务完成前，软件状态只能称为“本机 Gazebo 全路线通过，实体路线与完整任务待验证”。
