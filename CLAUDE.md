# CLAUDE.md

本文件为 Claude Code（claude.ai/code）提供本仓库的操作指导。

## 项目背景与当前状态

本仓库面向**第二十一届全国大学生智能汽车竞赛-地瓜机器人智慧医疗赛**，硬件平台为 **OriginCar + RDK X5 8G + ROS2 Humble**。

截至 2026-07-24，仓库包含 RF2O 激光里程计（已标定朝向）、唯一 9 点语义任务路线、官方场地参考层、RViz 航点编辑器和语音/二维码/图生文三个独立媒体入口。旧 68 点路线和独立纯导航测试链已删除。实车标定与 P→任务发布点导航验证已完成（含 Nav2 参数链路修复、行为树加固和代价地图调优）。新增 `”nav”` 任务类型支持跳过视觉的纯导航全路线测试，`nav_only.yaml` + `/root/nav_test.sh` 一键启动。QR/VLM 后端部署和完整任务测试尚未完成，不得表述为”已具备竞赛现场运行条件”。

已完成八轮 CPU 优化（详见 CHANGELOG），系统总 CPU 从 ~120% 降至 ~10%（idle 状态），其中 safety_node 从 Python 43.9% 降至 C++ 6.4%，barcode_reader 改为任务按需启动（idle 0%），aurora930 切换为 USB 摄像头。

## 高层架构

- **任务决策层**：`smartcar_task` 管理五子任务、语义航点、`FollowWaypoints`、停止与复位。
- **视觉层**：`smartcar_vision` 使用 `zbar_ros` 识别二维码；图生文请求包含图像等待、JPEG 编码和后端推理，统一受 8 秒硬期限约束并提供兜底文案。
- **导航层**：Nav2 Waypoint Follower + Smac Hybrid（REEDS_SHEPP，`allow_reversing: true`）+ Regulated Pure Pursuit；BT RateController 0.5 Hz 避免 RDK X5 规划竞态；禁止 Spin recovery 和原地旋转（但允许 QR→VLM 段直线倒车）。
- **避障感知层**：YDLIDAR `/scan` 直接进入 obstacle/inflation costmap；无消费者的 `obstacle_detector_2` 默认关闭，仅保留诊断开关。
- **定位层**：STM32 轮式里程计 + IMU + 无地图连续扫描匹配激光里程计，经 `robot_localization` EKF 输出 `/odom_combined`；无 SLAM。
- **控制层**：`/cmd_vel` 经 fail-closed `smartcar_safety`（C++ 默认，Python 备选）直接输出 `/ackermann_cmd`（内部 Twist→Ackermann 转换，`use_safety_ackermann:=true` 时跳过独立转换器节点）。安全节点只做三件事：指令消毒、急停锁存、传感器心跳超时。

核心约束：

- LiDAR 不做 SLAM 或静态地图定位；必须允许通过连续扫描匹配生成激光里程计并融合进 EKF，同时继续进入 obstacle/inflation costmap。激光里程计不得直接发布 `odom_combined -> base_footprint` TF，EKF 是唯一 TF owner。
- 运动链必须经过 `smartcar_safety`；系统禁止 `use_base=true,use_safety=false`。安全节点直接发布 `/ackermann_cmd`，`use_safety_ackermann:=true` 时跳过独立的 `cmd_vel_to_ackermann_drive` 节点。
- 安全节点 odom 回调有 50ms 节流（`odom_throttle_interval_sec`），有效 odom 看门狗窗口 = `odom_timeout + throttle_interval + timer_period`（最坏 ~450ms，0.15 m/s 下 ~6.8 cm）。`raw_odom_timeout_sec` 已参数化。
- 人物描述由语义航点 `task: vlm` 触发。VFH、YOLO 自动触发不是当前 release 依赖；TTS consumer 已提供但默认关闭，不属于任务或运动门禁依赖。
- 发车时车辆手动置于 P 区原点，车头朝 `+X`；任务 reset 不能替代物理复位。
- 系统默认使用 USB 摄像头 (camera_driver:=usb) 和按需 QR 扫描 (barcode_reader 由 task_node 子进程拉起)，Aurora 930 和持续 zbar 为备选。
- 未经用户明确授权，不得启动实体相机、发布非零速度或进行实车运动测试。

## 仓库结构

```text
src/origincar/                       vendor 底盘、IMU、EKF、YDLIDAR
src/third_party/obstacle_detector_2/ LiDAR 障碍物提取
src/third_party/rf2o_laser_odometry/ 可选无地图激光里程计
src/smartcar_interfaces/             ReadQr、DescribeScene 接口
src/smartcar_safety/                 速度安全门（C++ 默认，Python 备选）
src/smartcar_nav2/                   Nav2 配置、行为树和航点
src/smartcar_vision/                 QR 与 VLM 服务
src/smartcar_speech/                 可选火山 TTS consumer
src/smartcar_task/                   五子任务状态机
src/smartcar_bringup/                分层和完整系统 launch
src/smartcar_tools/                  场地参考、航点编辑、媒体入口与里程计诊断
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

本地仓库的 `src/` 和 `config/` 是权威源。日常使用 `push` 镜像到 RDK `/root/ros2_ws`；`init-vendor` 仅用于首次 bootstrap 或明确要求的 vendor 重建。**⚠ `push` 默认含 `--delete`，会删除 RDK 端本地源不存在的文件——在 RDK 上直接改过的文件（如航点 YAML）会被静默覆盖，务必先 `pull` 或手动备份。**

## RDK 构建与验证

RDK 通过无线网络免密连接（IP 由现场 DHCP 分配，以 `! ssh root@<ip>` 实际地址为准）；有线备用地址为 `root@192.168.128.10`。ROS2 环境入口是 `/opt/tros/humble/setup.bash`，日常统一使用 `~/source_env.sh`，不要叠加 `/userdata/dev_ws` overlay。

```bash
source ~/source_env.sh
cd /root/ros2_ws
colcon build --symlink-install \
  --allow-overriding smartcar_nav2 smartcar_task \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test-result --delete-yes
colcon test --return-code-on-test-failure
colcon test-result --all --verbose
```

2026-07-24 最新证据：本地根合同 131/131；待实车验证倒车段。

vendor-only 全量 lint 默认 opt-in：需要时使用 `-DSMARTCAR_ENABLE_VENDOR_LINT=ON`，不要把继承源码的历史格式债务混入默认功能测试。

## 部署与实车门禁

截至 2026-07-23，标定与导航验证进展：

### 已完成

- ✅ 陀螺仪零偏标定：`gyro_z_bias = 0.000853`（参数链路已打通至 `smartcar_system.launch.py` 默认值）。
- ✅ 外参测量（URDF + 微调）：`base_footprint→base_link (0.0841,0,0.03)`、`base_link→laser (-0.05,0,0.23)`、`base_link→camera (0.1205,0,0.11)`。
- ✅ 轮速标定验证：`longitudinal_velocity_scale = 1.03` 实测通过。
- ✅ P → 任务发布点导航：0.15 m/s，5 航点，约 35s 完成，自动锁停。2D LiDAR 避障通过锥桶膨胀补偿（local inflation 0.55 / global 0.65）+ footprint 自过滤可用。
- ✅ 标定参数链路：8 个传感器参数从 `smartcar_system` → `smartcar_bringup` → `origincar_bringup` → `base_serial` 完整传递。
- ✅ RF2O 激光里程计：朝向修复（laser_yaw=1.5708），X 跟踪与轮式一致，Y 漂移率 ~14%。建议在特征丰富的竞赛场地启用，当前训练场地可保持关闭。

### 待完成

- 实测并替换 `default_waypoints.yaml` 占位航点（`nav_only.yaml` 已支持纯导航实测，语义航点坐标仍需场地验证后标定）。
- 完成转向标定（`steering_command_scale` / `offset`）。
- 配置并实测 VLM 后端（火山 Ark 或端侧模型）；如需语音，验证 TTS + 网络 + 扬声器。
- 验证人工物理急停 → 车轮离地 → 低速地面 → 完整赛道测试。
- 0.30 m/s 运动异常尚未完成单变量复测；历史 EKF 超期发生在发车前，不能归因于车速。已实施首批调度修复，仍须现场分级验证。

五个运动门禁默认为 `false`，仅对应项目实测完成后显式开启。

### 已知问题与避坑指南

- **LiDAR 物理朝向**：2D LiDAR 物理安装转了 90°（Y 朝后/X 朝左），`laser_yaw=1.5708` 已修复。若更换/重新安装 LiDAR 必须重新确认朝向。
- **`cmd_vel_to_ackermann_drive` 竞争**：直接往 `/ackermann_cmd` pub 时，必须先 kill 该节点，否则 safety_node 20Hz 零值覆盖你的指令（车不动或抖动）。
- **`velocity_smoother` 生命周期卡死**：激活后偶发 `get_state` 服务无响应。解决：`ros2 daemon stop` → `kill -9` 所有 ROS 进程 → `ros2 daemon start` → 重启 launch。
- **2D LiDAR 无法区分高度**：锥桶上窄下宽——雷达看到的是上半窄截面，车体下半蹭宽底。通过非对称 footprint（后轴原点）+ `obstacle_min_range=0.25` 滤车身 + 膨胀半径补偿，不能根本解决。物理上需 3D 感知或更谨慎的锥桶摆放。
- **Ackermann 终点兜圈**：无法原地旋转。`xy_goal_tolerance=0.25, yaw_goal_tolerance=0.50` 已修复。
- **0.30 m/s 运动异常未定因**：原始日志表明 EKF 最严重的更新超期发生在路线启动前；`odom0_config` 也不融合原始 pose，因此旧版”IntegrationClock 导致 EKF pose/速度冲突”的推断已撤回。现已消除 EKF 非零 TF 等待、降低 BT tick 负载并关闭 RF2O 逐帧 INFO；复测前仍按 0.15 m/s 已验证上限管理，详见 `docs/review/odometry-speed-analysis.md`。
- **Nav2 参数链路 (2026-07-23 修复)**：Nav2 1.1.20 (TROS Humble) 的 `RewrittenYaml` chain 存在 bug——`ParameterFile` 输出被 YDLIDAR 驱动参数覆盖，导致 `controller_server` 加载默认 DWB 而非 RPP。修复方案：`CMakeLists.txt` 自动生成 `nav2_params_fixed.yaml`（BT 路径硬编码），`navigation_launch.py` 直接使用该文件。任何对 `nav2_params.yaml` 的修改会在 `colcon build` 时自动同步到 fixed 文件。不可手动创建/编辑 `nav2_params_fixed.yaml`。
- **重启必须彻底杀旧进程**：多次 kill/restart 循环会残留旧 launch 子进程（YDLIDAR、obstacle_extractor、task_node 等），占用 `/dev/ttyUSB0` 串口并消耗 CPU。**推荐使用 `bash scripts/ros_cleanup.sh` 一键清理**；手动方式：`pkill -9` 全部 ROS 节点可执行文件 → `ros2 daemon stop` → `sleep 1` → `ros2 daemon start` → `ros2 launch`。仅靠 `ctrl-c` 或部分 pkill 不可靠。
- **任务起点航点 (2026-07-23 修复)**：`mission.py` 在构建导航段时跳过 `task=start` 的航点，避免 planner 对零长度路径 (0,0)→(0,0) 规划失败。发车前必须将车辆手动置于 P 点原点，车头朝 +X。
- **行为树已移除 backup/wait recovery (2026-07-23)**：两个 BT XML 文件不再包含 `BackUp` 和 `Wait` 恢复动作，仅保留 `ClearEntireCostmap` + 重规划。阿克曼底盘不可引入原地旋转或后退恢复。
- **纯导航任务类型 `nav` (2026-07-23)**：`waypoints.py` 新增 `"nav"` 任务类型，可用于替代 `"qr"` 和 `"vlm"` 进行无视觉纯导航测试。`"nav"` 在状态机中等效于 qr/vlm 的位置约束，但不触发任何视觉服务调用，导航段不拆分直通下一航点。见 `nav_only.yaml`。
- **一键导航测试脚本 (2026-07-23)**：`/root/nav_test.sh` 全自动执行清理→构建→启动→等就绪→RViz→发车，使用 `nav_only.yaml` 和 `use_vision:=false`。RDK 上 `setsid bash /root/nav_test.sh > /tmp/nav_test_output.log 2>&1 &` 即可启动。配套监控 `/root/monitor_mission.py`（`python3 /root/monitor_mission.py`）实时显示任务状态、位姿和速度。
- **倒车导航设计 (2026-07-24)**：QR→VLM 段（通道）必须倒车。方案：`Waypoint.direction: "reverse"` 标记航点 → `_navigation_segments()` 按方向边界拆分 → 每段独立 `FollowWaypoints` 调用。不做朝向翻转——靠 `reverse_penalty: 0.5`（REEDS_SHEPP 倒车原语比正向便宜 50%）让规划器在目标位于后方时自然选择倒车路径。到 VLM 点时 heading ≈ 60°，下一正向段无缝衔接（x'=+1.27 > 0）。`allow_reversing: true` + `min_velocity: [-0.15, 0.0, -0.75]` + `use_rotate_to_heading: false`（阿克曼不原地旋转）。STM32 负速度已由 `int16_t` 编码验证兼容，实际倒车运动待实车验证。
- **电压监控 (2026-07-24)**：`voltage_monitor` 工具订阅 `/PowerVoltage`（STM32 串口 byte 20-21，mV→V），记录到 `/tmp/voltage_history.log`（含时间戳，上限 10 万行自动轮转）。安全节点 `minimum_voltage: 10.0`（`safety.yaml`）——低于 10.0V 锁止运动。当前电池 3S 18650 LiPo，满电 12.6V，充电监控：`ros2 topic echo /PowerVoltage --once`。
- **ROS2 CLI 卡死备用方案**：`ros2 service call` 在 lifecycle 异常后可能无限等待。紧急停车可用 `pkill -9 -f "ros2 launch"` 直接杀 launch 进程，STM32 超时自动发送停止指令。日常清理推荐 `bash scripts/ros_cleanup.sh`。

### 航点编辑与可视化

RDK 上启动无运动航点编辑器：

```bash
export DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0
ros2 launch smartcar_tools waypoint_editor.launch.py
```

编辑器直接拖拽唯一的 `default_waypoints.yaml`，同时显示官方尺寸参考层；右键航点可保存、撤销或重载。它不启动 Nav2 或底盘，并默认锁存软件急停。

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

电压监控（记录 `/PowerVoltage` 到日志，支持离线分析）：

```bash
ros2 run smartcar_tools voltage_monitor
# 日志: /tmp/voltage_history.log（时间戳 + 电压值，10 万行自动轮转）
# 实时查看: tail -f /tmp/voltage_history.log
# 当前值:   ros2 topic echo /PowerVoltage --once
```

### 纯导航全路线测试（无视觉）

RDK 上一键启动：

```bash
setsid bash /root/nav_test.sh > /tmp/nav_test_output.log 2>&1 &
# 监控: python3 /root/monitor_mission.py
# 日志: tail -f /tmp/bringup.log
```

脚本自动完成：全杀旧进程 → 构建 → 启动系统（无相机/无视觉）→ 等待 Nav2 生命周期就绪 → 开 RViz → 发车。使用 `nav_only.yaml`（qr/vlm → nav 直通），不依赖视觉服务。紧急停车：`pkill -9 -f "ros2 launch"`（STM32 超时自动刹停）。

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
