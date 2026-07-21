# 场地路线与四项独立测试入口

更新时间：2026-07-21

本文用于 RDK X5 上的分项 bench 和后续实车分阶段验收。当前代码提供了可编辑的规则图基线路线、可选 RF2O 激光里程计，以及导航、语音、二维码、图生文四个互相隔离的入口。

这些能力尚未完成 RDK 全量构建和物理验收。除单图回放等无硬件 bench 外，未经现场负责人明确授权，不得启动实体相机、音频播放器、底盘或任何实车运动。

## 1. 架构与安全边界

定位和导航不使用静态栅格地图，不启动 `map_server`、AMCL 或 SLAM。Nav2 使用 rolling costmap，LiDAR `/scan` 用于动态避障，并可选地提供 RF2O 连续扫描匹配：

```text
/odom + /imu/data_raw + optional /odom_laser
                    -> robot_localization EKF -> /odom_combined
```

RF2O 只发布 `/odom_laser`，配置为 `publish_tf: false`；`odom_combined -> base_footprint` 的唯一 TF owner 是 EKF。`use_laser_odometry` 默认 `false`，启用后才条件性要求 `laser_odometry_calibrated=true`。该布尔参数只是验收记录，不会自动完成标定。

所有运动命令必须经过 `smartcar_safety`。纯导航启动时急停默认锁存，必须依次成功调用 `prepare`、`arm`、`start`；完成、取消、超时或异常后会重新锁停。`arm` 后 30 秒内没有调用 `start` 也会自动重新锁停。

## 2. 构建与公共路径

先完成同步和构建：

```powershell
python scripts/sync_to_rdk.py push --dry-run
python scripts/sync_to_rdk.py push
```

```bash
ssh root@172.16.25.27
source ~/source_env.sh
cd /root/ros2_ws
colcon build --symlink-install
```

RDK 上后续命令使用以下路径：

```bash
export ROUTE=/root/ros2_ws/src/smartcar_tools/config/routes/full_course_route.yaml
export GEOMETRY=/root/ros2_ws/src/smartcar_tools/config/routes/field_geometry.yaml
```

## 3. 68 点规则图基线路线

`full_course_route.yaml` 的坐标约定为 P 区中心 `(0, 0)`、车头朝 `+X`，`+Y` 指向 B/C 区。路线顺序为 P -> 任务发布点 -> 中央通道 -> C 环顺时针一圈 -> 原通道返回 -> P。

场地尺寸来自规则图：整体 `5 m x 5 m`，A/B/C 高度分别为 `2/0.5/2.5 m`，中央通道宽 `1 m`，C 环外轮廓 `4 x 1.65 m`、内轮廓 `3 x 0.65 m`。C 环中线的相邻采样点不超过 `0.25 m`。P 区和任务发布点缺少完整定位尺寸，当前坐标含估算值，所以文件保持：

```yaml
calibrated: false
```

检查和列出点位：

```bash
ros2 run smartcar_tools route_tool validate "$ROUTE"
ros2 run smartcar_tools route_tool list "$ROUTE"
```

按规则尺寸重新生成基线会覆盖所有现场修改，只有明确需要重建时才使用：

```bash
ros2 run smartcar_tools route_tool generate "$ROUTE" \
  --geometry "$GEOMETRY" --force
```

### 3.1 命令行微调

```bash
# 直接设置一个点
ros2 run smartcar_tools route_tool set "$ROUTE" a_task \
  --x 4.10 --y 1.32 --yaw-deg 0

# 增量微调
ros2 run smartcar_tools route_tool nudge "$ROUTE" a_task \
  --dx -0.02 --dy 0.01 --dyaw-deg 2

# 在已有点后插入
ros2 run smartcar_tools route_tool insert "$ROUTE" \
  --after a_task --id a_task_exit --zone A \
  --x 4.20 --y 1.20 --yaw-deg -45

# 删除点
ros2 run smartcar_tools route_tool delete "$ROUTE" a_task_exit
```

每次坐标或结构修改都会完整校验并原子替换文件，同时自动恢复为 `calibrated: false`。逐点测量、短段验证和路线复核全部完成后，才可由现场负责人显式记录：

```bash
ros2 run smartcar_tools route_tool mark-calibrated "$ROUTE"
```

需要撤销标记时：

```bash
ros2 run smartcar_tools route_tool mark-calibrated "$ROUTE" --uncalibrated
```

### 3.2 RViz 点位编辑

路线编辑入口不启动底盘、LiDAR 或 Nav2；它启动一个默认锁存急停的 safety 节点、编辑节点和 RViz：

```bash
ros2 launch smartcar_tools route_editor.launch.py route_file:="$ROUTE"
```

替换已有点时，先在另一终端选择 ID，再在 RViz 使用 `2D Goal Pose` 给出位置和朝向：

```bash
ros2 topic pub --once /smartcar/route_editor/selected_id \
  std_msgs/msg/String "{data: a_task}"
```

没有选择 ID 时，下一次 `2D Goal Pose` 会在终点 P 之前插入一个临时命名点。编辑服务为：

```bash
ros2 service call /smartcar/route_editor/load std_srvs/srv/Trigger "{}"
ros2 service call /smartcar/route_editor/undo std_srvs/srv/Trigger "{}"
ros2 service call /smartcar/route_editor/clear std_srvs/srv/Trigger "{}"
ros2 service call /smartcar/route_editor/save std_srvs/srv/Trigger "{}"
```

`save` 会重新校验、原子写入，并把路线标为未标定。状态和选择话题分别为 `/smartcar/route_editor/status`、`/smartcar/route_editor/selected_id`。

### 3.3 只回传路线

在 RDK 完成点位修改后，从 Windows PowerShell 只回传单个路线文件，避免全量 `pull` 覆盖本地源码：

```powershell
python scripts/sync_to_rdk.py pull-route --dry-run
python scripts/sync_to_rdk.py pull-route
python -m unittest discover -s src/smartcar_tools/test \
  -p test_route_model.py -v
```

回传目标固定为 `src/smartcar_tools/config/routes/full_course_route.yaml`，不会删除或覆盖其他源码。

## 4. 独立完整导航

`navigation_test.launch.py` 只启动底盘、EKF、LiDAR、costmap 避障、安全门、精简 Nav2 和路线执行器；相机、QR/VLM、五子任务状态机、语音、无消费者的 obstacle extractor、未接入 EKF 的 Madgwick、URDF 车型发布、path smoother 和 waypoint follower 均不启动。执行器使用 `/navigate_through_poses` 和专用行为树，现场参数的期望线速度为 `0.15 m/s`。如需诊断提取后的障碍物或在 RViz 显示完整车型，可分别显式传入 `use_obstacle:=true`、`use_robot_description:=true`；二者都不是运动依赖。

当前基线路线未标定。只有获得底盘和 LiDAR 启动授权、车辆架空或场地已清空时，才可用以下启动检查节点和状态；`arm` 必须拒绝：

```bash
ros2 launch smartcar_tools navigation_test.launch.py route_file:="$ROUTE"
```

只有完成外参、转向、物理急停、路线和操作员授权后，才可在车辆物理放回 P 原点、车头朝 `+X` 的情况下显式打开五个门禁：

```bash
ros2 launch smartcar_tools navigation_test.launch.py \
  route_file:="$ROUTE" \
  waypoints_calibrated:=true \
  extrinsics_calibrated:=true \
  steering_calibrated:=true \
  emergency_stop_ready:=true \
  operator_approved:=true \
  use_laser_odometry:=false
```

首次地面验证必须限制为短段，不要直接提交 68 点全程。`route_end_id` 是包含式终点，例如只验证 P 到首个前进点：

```bash
ros2 launch smartcar_tools navigation_test.launch.py \
  route_file:="$ROUTE" \
  route_end_id:=a_depart \
  waypoints_calibrated:=true \
  extrinsics_calibrated:=true \
  steering_calibrated:=true \
  emergency_stop_ready:=true \
  operator_approved:=true \
  use_laser_odometry:=false
```

如已额外完成 RF2O 标定和退化回退验证，可将最后一行替换为：

```bash
  use_laser_odometry:=true laser_odometry_calibrated:=true
```

RF2O 启用后，`prepare` 会先调用 `/smartcar/localization/reset_laser_odometry`，再调用 EKF `/set_pose` 并等待新鲜 `/odom_combined` 验证 P 原点；`arm` 还会检查新鲜 `/odom_laser`。不要在未完成激光里程计实测时传入 `laser_odometry_calibrated=true`。

另开终端观察 JSON 状态：

```bash
source ~/source_env.sh
ros2 topic echo /smartcar/test/navigation/status
```

### 4.1 快速启动（推荐）

启动栈后直接运行以下命令。它在一个 ROS 进程内等待 `/navigate_through_poses` 可用，再连续执行 `prepare -> arm -> start`，避免三次独立 `ros2 service call` 重复启动 Python 和等待 ROS 图发现：

```bash
source ~/source_env.sh
ros2 run smartcar_tools navigation_probe start
```

`run` 是 `start` 的别名。默认最多等待 Nav2 60 秒；任一步失败、超时或异常都会 best-effort 调用 `stop` 重新锁停。该命令仍是操作员显式发车动作，不是自动发车，也不会绕过路线标定、传感器新鲜度或五个运动门禁。

正常或紧急停止统一使用：

```bash
ros2 run smartcar_tools navigation_probe stop
```

### 4.2 分步诊断（仅排障）

以下命令保留用于定位具体门禁或服务问题。每条 `ros2 service call` 都会创建新进程并重新发现 ROS 图，在 RDK 上可能耗时数秒；不要依靠三条独立命令赶 30 秒 `arm` 窗口。

```bash
# 1. 锁停、确认无活动目标、复位并验证 P 原点
ros2 service call /smartcar/test/navigation/prepare \
  std_srvs/srv/Trigger "{}"

# 2. 检查全部门禁、TF、scan/odom 新鲜度，再解除软件急停
ros2 service call /smartcar/test/navigation/arm \
  std_srvs/srv/Trigger "{}"

# 3. 必须在 arm 成功后的 30 秒内显式开始
ros2 service call /smartcar/test/navigation/start \
  std_srvs/srv/Trigger "{}"

# 4. 随时停止：先锁存急停，再取消 Nav2 目标并等待终态
ros2 service call /smartcar/test/navigation/stop \
  std_srvs/srv/Trigger "{}"
```

若快速停止入口不可用，先直接锁存安全门：

```bash
ros2 service call /smartcar/safety/emergency_stop \
  std_srvs/srv/SetBool "{data: true}"
```

通过 transient systemd unit 启动测试栈时，确认急停已锁存后可结束整个进程组：

```bash
systemctl kill --kill-who=all --signal=SIGINT \
  smartcar-navigation-test.service
```

结束进程组不能替代软件急停或现场物理急停。不要先等待 ROS launch 的逐节点优雅退出，再处理仍在运行的车辆。

状态包含当前点、剩余点数、剩余距离、耗时、急停状态和失败原因。首次实车不得直接跑全程，应依次完成车轮离地、单点、短段、低速全程；全程目标为经过任务发布点和中央通道，完成 C 环顺时针一圈并返回 P，终点误差目标不超过 `0.20 m`。

### 4.1 2026-07-19 现场记录

- 首次发车发生后退。日志确认是现场 BT 的 `RemovePassedGoals` 使用默认 `map/base_link` 导致规划失败，随后触发 `BackUp` recovery；不是底盘速度符号反转。现场树现已固定为 `odom_combined/base_footprint`，并从行为树、控制器和速度平滑器三处禁止倒车。
- P 到 `a_depart (0.80, 0.05)` 的两点短段已成功，耗时 `6.53 s`；终态为 `succeeded`，急停重新锁存，`/cmd_vel_safe` 回到精确零。
- P 到 `a_sweep (2.10, 0.25)` 的三点测试未出发。Smac Hybrid 连续报告到 `a_depart` 无有效路径，操作端调用 `stop` 后确认急停锁存。
- 锁停后的诊断快照中，`/odom_combined` 位置为 `(0, 0)`，但航向为 `20.9 deg`；全局成本图中 P 和 `a_depart` 分别为膨胀代价 `101/79`，P 到 `a_sweep` 的直线采样穿过 `15` 个致命栅格。避障链已工作，当前问题是路线、定位/外参或现场障碍与成本图不一致。
- 本轮没有完成全程导航验收。下次测试前必须核实车头物理朝向、IMU/轮式航向复位、`base_link -> laser` 实测外参，并在 RViz 中对照 `/scan`、全局成本图和路线微调点位；不得仅通过缩小膨胀半径或恢复倒车来绕过问题。
- RDK 测试结束后导航服务已停止，运行时路线重新标记为 `calibrated: false`。

## 5. 独立语音测试

该入口只启动火山 TTS consumer，不启动底盘、Nav2、视觉或任务。先设置凭据并确认播放器：

```bash
export VOLCENGINE_TTS_APP_ID='<应用 ID>'
export VOLCENGINE_TTS_ACCESS_TOKEN='<access token>'
command -v ffplay
ros2 launch smartcar_tools speech_test.launch.py
```

另开终端发送一次请求，并跟踪同一 `request_id` 的 `queued -> synthesizing -> playing -> completed`：

```bash
source ~/source_env.sh
ros2 run smartcar_tools speech_probe --text '语音分项测试'
```

`speech_probe` 成功退出码为 `0`；请求失败为 `1`，超时为 `2`，consumer 不可用为 `3`。该测试会访问网络并实际播放音频，不能纳入无硬件自动化 smoke。

## 6. 独立二维码测试

### 6.1 单张图片回放

该模式不打开实体相机：

```bash
ros2 launch smartcar_tools qr_test.launch.py \
  input_source:=file \
  image_file:=/root/test-data/qr.png
```

另开终端调用一次或连续调用：

```bash
source ~/source_env.sh
ros2 run smartcar_tools qr_probe --timeout-sec 3
ros2 run smartcar_tools qr_probe --continuous --count 10 --interval-sec 0.25
```

输出为包含 `success`、`content`、`status` 的 JSON。退出码 `0` 表示识别成功，`1` 表示未识别，`2` 表示服务不可用，`3` 表示传输错误。

### 6.2 实体相机

获得实体相机启动授权后，按驱动选择：

```bash
ros2 launch smartcar_tools qr_test.launch.py \
  input_source:=camera camera_driver:=aurora

# 或 USB
ros2 launch smartcar_tools qr_test.launch.py \
  input_source:=camera camera_driver:=usb usb_video_device:=/dev/video0
```

该入口只启动所选相机、zbar 和 QR 服务，不启动底盘或 Nav2。

## 7. 独立图生文与 HDMI UI

该入口启动相机或单图回放、VLM 服务和 PyQt5 UI，不启动语音、底盘、Nav2 或任务。UI 显示实时画面、触发按钮、处理状态、耗时和大字号描述文本，同时订阅 `/smartcar/output/text`。

RDK 的 LightDM HDMI 默认设置为：

```bash
test -r /var/run/lightdm/root/:0
export DISPLAY=:0
export XAUTHORITY=/var/run/lightdm/root/:0
```

火山 Ark 模式还需在启动前设置：

```bash
export ARK_API_KEY='<火山 Ark API key>'
export VOLC_ARK_MODEL='doubao-1-5-vision-pro-32k-250115'
```

先用单张图片回放验证服务和界面：

```bash
ros2 launch smartcar_tools vlm_test.launch.py \
  input_source:=file \
  image_file:=/root/test-data/person.png \
  display:=:0 \
  xauthority:=/var/run/lightdm/root/:0
```

获得实体相机授权后再切换输入：

```bash
ros2 launch smartcar_tools vlm_test.launch.py \
  input_source:=camera camera_driver:=aurora \
  display:=:0 \
  xauthority:=/var/run/lightdm/root/:0
```

点击 UI 的触发按钮会调用 `/smartcar/vision/describe_scene`。请求包含等待新鲜图像、JPEG 编码和后端推理，硬期限为 8 秒；界面会区分处理中、成功、兜底和失败。可单独验证比赛文字输出通道：

```bash
ros2 topic pub --once /smartcar/output/text \
  std_msgs/msg/String "{data: 'HDMI 文字显示测试'}"
```

若 HDMI 用户、显示号或 LightDM 配置变化，先用 `ps` 和 `/var/run/lightdm` 实际状态确认 X authority 文件，不要把凭据写入 launch 或仓库。云端 VLM 还必须确认比赛规则允许公网，并在赛场网络下完成成功率和 8 秒时限验收；自动化回放或兜底文案不能证明真实后端可用。

## 8. 验证与证据边界

新增包的无硬件检查：

```bash
source ~/source_env.sh
cd /root/ros2_ws
colcon test-result --delete-yes
colcon test --packages-select \
  rf2o_laser_odometry smartcar_tools smartcar_bringup \
  --return-code-on-test-failure
colcon test-result --all --verbose
```

自动化测试不得启动底盘、LiDAR 驱动、实体相机、音频播放器或发布非零速度。它只能证明构建、接口、门禁和回放合同；以下项目仍需分别记录现场证据：

2026-07-19 RDK X5 记录：18 个包构建通过，干净全量结果为 `578 tests, 0 errors, 0 failures, 90 skipped`；严格无硬件 smoke 连续 3 轮通过，四个测试 launch 和路线编辑 launch 均通过 `--show-args` 解析。RF2O 和 `smartcar_tools` 已完成目标板软件构建，但没有启动任何硬件。

- RF2O 时间戳、外参、协方差、漂移、拒绝异常观测和轮速/IMU 回退。
- 路线逐点坐标、C 环完整性、通过性和回到 P 的误差。
- 相机、二维码、VLM 后端、HDMI、网络、TTS 和扬声器。
- 物理急停、车轮离地、单点、短段和 `0.15 m/s` 全程实车测试。

上述证据完成前，项目状态只能表述为“软件入口、安全合同和 RDK 无硬件验证已完成，待物理标定和实车验证”，不能表述为“可直接上场”或“完整赛道已跑通”。
