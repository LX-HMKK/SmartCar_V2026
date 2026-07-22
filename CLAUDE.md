# CLAUDE.md

本文件为 Claude Code（claude.ai/code）提供本仓库的操作指导。

## 项目背景与当前状态

本仓库面向**第二十一届全国大学生智能汽车竞赛-地瓜机器人智慧医疗赛**，硬件平台为 **OriginCar + RDK X5 8G + ROS2 Humble**。

截至 2026-07-22，`main` 已包含完整软件里程碑，以及 RF2O 激光里程计（已标定朝向）、68 点场地基线路线和导航/语音/二维码/图生文四个独立测试入口。代码合同、RDK 构建与无硬件 smoke 已通过。实车标定与 P→任务发布点导航验证已完成，但航点实测、VLM 后端部署和完整赛道测试尚未完成，不得表述为“已具备竞赛现场运行条件”。

## 高层架构

- **任务决策层**：`smartcar_task` 管理五子任务、语义航点、`FollowWaypoints`、停止与复位。
- **视觉层**：`smartcar_vision` 使用 `zbar_ros` 识别二维码；图生文请求包含图像等待、JPEG 编码和后端推理，统一受 8 秒硬期限约束并提供兜底文案。
- **导航层**：Nav2 Waypoint Follower + Smac Hybrid（DUBIN）+ Regulated Pure Pursuit；禁止 Spin recovery 和原地旋转。
- **避障感知层**：YDLIDAR `/scan` 直接进入 obstacle/inflation costmap；无消费者的 `obstacle_detector_2` 默认关闭，仅保留诊断开关。
- **定位层**：STM32 轮式里程计 + IMU + 无地图连续扫描匹配激光里程计，经 `robot_localization` EKF 输出 `/odom_combined`；无 SLAM。
- **控制层**：`/cmd_vel` 经 fail-closed `smartcar_safety` 输出 `/cmd_vel_safe`，再转换为 OriginCar 阿克曼命令。

核心约束：

- LiDAR 不做 SLAM 或静态地图定位；必须允许通过连续扫描匹配生成激光里程计并融合进 EKF，同时继续进入 obstacle/inflation costmap。激光里程计不得直接发布 `odom_combined -> base_footprint` TF，EKF 是唯一 TF owner。
- 运动链必须经过 `smartcar_safety`；系统禁止 `use_base=true,use_safety=false`。
- 人物描述由语义航点 `task: vlm` 触发。VFH、YOLO 自动触发不是当前 release 依赖；TTS consumer 已提供但默认关闭，不属于任务或运动门禁依赖。
- 发车时车辆手动置于 P 区原点，车头朝 `+X`；任务 reset 不能替代物理复位。
- 未经用户明确授权，不得启动实体相机、发布非零速度或进行实车运动测试。

## 仓库结构

```text
src/origincar/                       vendor 底盘、IMU、EKF、YDLIDAR
src/third_party/obstacle_detector_2/ LiDAR 障碍物提取
src/third_party/rf2o_laser_odometry/ 可选无地图激光里程计
src/smartcar_interfaces/             ReadQr、DescribeScene 接口
src/smartcar_safety/                 速度安全门
src/smartcar_nav2/                   Nav2 配置、行为树和航点
src/smartcar_vision/                 QR 与 VLM 服务
src/smartcar_speech/                 可选火山 TTS consumer
src/smartcar_task/                   五子任务状态机
src/smartcar_bringup/                分层和完整系统 launch
src/smartcar_tools/                  场地路线、独立测试入口、航点可视化与里程计诊断
scripts/                             RDK 同步与环境脚本
tests/                               仓库级合同测试
```

权威运行手册为 `docs/deployment/rdk-environment-setup.md`；软件完工证据见 `docs/superpowers/plans/2026-07-17-smartcar-completion.md`。早期设计/spec 保留决策过程，若与当前实现冲突，以代码、README 和部署手册为准。

## 本地开发与同步

Windows PowerShell：

```powershell
python -m unittest discover -s tests -v
python -m unittest discover -s src/smartcar_safety/test -v
python -m unittest discover -s src/smartcar_vision/test -v
python -m unittest discover -s src/smartcar_speech/test -v
python -m unittest discover -s src/smartcar_task/test -v
python -m unittest discover -s src/smartcar_tools/test -v
python scripts/sync_to_rdk.py push --dry-run
python scripts/sync_to_rdk.py push
```

本地仓库的 `src/` 和 `config/` 是权威源。日常使用 `push` 镜像到 RDK `/root/ros2_ws`；`init-vendor` 仅用于首次 bootstrap 或明确要求的 vendor 重建。

## RDK 构建与验证

RDK 通过无线网络免密连接（IP 由现场 DHCP 分配，以 `! ssh root@<ip>` 实际地址为准）；有线备用地址为 `root@192.168.128.10`。ROS2 环境入口是 `/opt/tros/humble/setup.bash`，日常统一使用 `~/source_env.sh`，不要叠加 `/userdata/dev_ws` overlay。

```bash
source ~/source_env.sh
cd /root/ros2_ws
colcon build --symlink-install
colcon test-result --delete-yes
colcon test --return-code-on-test-failure
colcon test-result --all --verbose
```

2026-07-19 最新证据：本地根合同 130/130；RDK 18 个包构建通过，`578 tests, 0 errors, 0 failures, 90 skipped`；严格无硬件 smoke 连续 3 轮通过。smoke 必须关闭底盘、LiDAR 驱动、障碍物驱动、RF2O、实体相机和语音节点，使用合成传感器，并在 Nav2 激活前锁存急停。

vendor-only 全量 lint 默认 opt-in：需要时使用 `-DSMARTCAR_ENABLE_VENDOR_LINT=ON`，不要把继承源码的历史格式债务混入默认功能测试。

## 部署与实车门禁

截至 2026-07-21，标定与导航验证进展：

### 已完成

- ✅ 陀螺仪零偏标定：`gyro_z_bias = 0.000853`（参数链路已打通至 `smartcar_system.launch.py` 默认值）。
- ✅ 外参测量（URDF + 微调）：`base_footprint→base_link (0.0841,0,0.03)`、`base_link→laser (-0.05,0,0.23)`、`base_link→camera (0.1205,0,0.11)`。
- ✅ 轮速标定验证：`longitudinal_velocity_scale = 1.03` 实测通过。
- ✅ P → 任务发布点导航：0.15 m/s，5 航点，约 35s 完成，自动锁停。2D LiDAR 避障通过锥桶膨胀补偿（local inflation 0.55 / global 0.65）+ footprint 自过滤可用。
- ✅ 标定参数链路：8 个传感器参数从 `smartcar_system` → `smartcar_bringup` → `origincar_bringup` → `base_serial` 完整传递。
- ✅ RF2O 激光里程计：朝向修复（laser_yaw=1.5708），X 跟踪与轮式一致，Y 漂移率 ~14%。建议在特征丰富的竞赛场地启用，当前训练场地可保持关闭。

### 待完成

- 实测并替换 `default_waypoints.yaml` 占位航点（当前使用规则图推算值）。
- 完成转向标定（`steering_command_scale` / `offset`）。
- 配置并实测 VLM 后端（火山 Ark 或端侧模型）；如需语音，验证 TTS + 网络 + 扬声器。
- 验证人工物理急停 → 车轮离地 → 低速地面 → 完整赛道测试。
- 0.30 m/s 提速触发 EKF 更新超限 + BT 过载，需性能优化后才能提速。

五个运动门禁默认为 `false`，仅对应项目实测完成后显式开启。

### 已知问题与避坑指南

- **LiDAR 物理朝向**：2D LiDAR 物理安装转了 90°（Y 朝后/X 朝左），`laser_yaw=1.5708` 已修复。若更换/重新安装 LiDAR 必须重新确认朝向。
- **`navigation_test.launch.py` 外参默认全零**：启动时必须显式传入 `base_x/base_z/laser_x/laser_z/laser_yaw`，否则 TF 全零导致 costmap 充满致命障碍。
- **`cmd_vel_to_ackermann_drive` 竞争**：直接往 `/ackermann_cmd` pub 时，必须先 kill 该节点，否则 safety_node 20Hz 零值覆盖你的指令（车不动或抖动）。
- **`velocity_smoother` 生命周期卡死**：激活后偶发 `get_state` 服务无响应。解决：`ros2 daemon stop` → `kill -9` 所有 ROS 进程 → `ros2 daemon start` → 重启 launch。
- **2D LiDAR 无法区分高度**：锥桶上窄下宽——雷达看到的是上半窄截面，车体下半蹭宽底。通过非对称 footprint（后轴原点）+ `obstacle_min_range=0.25` 滤车身 + 膨胀半径补偿，不能根本解决。物理上需 3D 感知或更谨慎的锥桶摆放。
- **Ackermann 终点兜圈**：无法原地旋转。`xy_goal_tolerance=0.25, yaw_goal_tolerance=0.35` 已修复。
- **0.30 m/s 提速失控**：EKF `Failed to meet update rate` + BT tick rate 超限。根因分析见 `docs/review/odometry-speed-analysis.md`。关键发现：`origincar_base` 的 `IntegrationClock` 在串口帧间隔 >250ms 时静默跳过位置积分（`max_integration_dt_sec=0.25`），与 safety `raw_odom_timeout_sec=0.25` 同窗口触发。高速下 CPU 争抢 → Control() 阻塞 → 串口间隙 → 积分跳帧 → EKF 观测矛盾。安全速度 0.15 m/s。

### 导航快捷命令

RDK 上启动纯导航测试（到任务发布点）：

```bash
# 车放在 P 原点，车头朝 +X
source ~/source_env.sh
nohup /tmp/start_nav.sh > /tmp/nav_test.log 2>&1 &   # 启动导航栈 + 路线执行器
# 等待约 20s 后：
ros2 run smartcar_tools navigation_probe start        # prepare → arm → start
# 紧急停止：
ros2 run smartcar_tools navigation_probe stop
# 或在另一终端查看状态：
ros2 topic echo /smartcar/test/navigation/status --once --field data
```

RViz 可视化（HDMI 屏幕）：

```bash
export DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0
# 导航监控（costmap + plan + 航点）：
rviz2 -d /root/ros2_ws/src/smartcar_tools/rviz/navigation.rviz
# 航点编辑器（编辑 route YAML）：
rviz2 -d /root/ros2_ws/src/smartcar_tools/rviz/route_editor.rviz
```

航点可视化（独立工具，不依赖导航栈）：

```bash
ros2 run smartcar_tools waypoint_viz --ros-args -p waypoints_file:=<path_to_yaml>
# 发布 MarkerArray 到 /smartcar/waypoints/markers（latched），RViz 订阅即可查看
```

里程计诊断（排查 EKF/串口丢帧）：

```bash
ros2 run smartcar_tools odom_diag --ros-args -p duration_sec:=30.0
# 输出 /odom、/odom_combined、/scan 速率和间隔统计，以及 EKF 诊断警告
```

## 提交规范

提交信息采用 Angular 规范，主题使用中文。使用 PowerShell 时可通过 UTF-8 临时文件提交多行信息：

```powershell
@'
fix(rdkx5): 修复导出脚本路径问题

- 修复 ONNX 路径使用 POSIX 斜杠
- 校准数据 pad 值对齐 Ultralytics
'@ | Out-File -Encoding utf8 .git-msg

git commit -F .git-msg
Remove-Item .git-msg
```

禁止添加 `Co-Authored-By` 或其他协作者元数据。提交只记录实际执行者。
