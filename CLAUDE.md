# CLAUDE.md

本文件为 Claude Code（claude.ai/code）提供本仓库的操作指导。

## 项目背景与当前状态

本仓库面向**第二十一届全国大学生智能汽车竞赛-地瓜机器人智慧医疗赛**，硬件平台为 **OriginCar + RDK X5 8G + ROS2 Humble**。

截至 2026-07-25：11 点语义路线、Smac DUBIN + RPP 导航、QR→VLM 倒车段（转弯修复已验证，绕圈已做软件修正待复测）。纯导航测试经 `nav_only.yaml` + `/root/nav_test.sh` 一键启动，急停锁存。VLM 后端和完整赛道未完成，**不具备竞赛现场运行条件**。CPU 优化八轮，idle ~10%（详见 CHANGELOG）。

## 高层架构

- **任务决策层**：`smartcar_task` 管理五子任务和语义航点；每个非起点航点独立发送 `NavigateToPose`，并把动作 UUID、方向、generation 与方向租约绑定。
- **视觉层**：`smartcar_vision` 使用 `zbar_ros` 识别二维码；图生文请求包含图像等待、JPEG 编码和后端推理，统一受 8 秒硬期限约束并提供兜底文案。
- **导航层**：单一 Smac Hybrid `DUBIN` planner + Regulated Pure Pursuit；正向使用普通 BT，倒车使用 `ComputeReversePathToPose` 自定义 BT 节点做虚拟航向规划、路径恢复和严格反向校验。BT 以 0.5 Hz 重规划；不启动 `behavior_server` 或 `waypoint_follower`，禁止 Spin recovery 和原地旋转。
- **避障感知层**：YDLIDAR `/scan` 直接进入 obstacle/inflation costmap；无消费者的 `obstacle_detector_2` 默认关闭，仅保留诊断开关。
- **定位层**：STM32 轮式里程计 + IMU + 无地图连续扫描匹配激光里程计，经 `robot_localization` EKF 输出 `/odom_combined`；无 SLAM。
- **控制层**：`controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_candidate -> direction_guard -> /cmd_vel -> smartcar_safety -> /ackermann_cmd`。方向门默认 STOP，只允许与动作租约一致的速度符号；安全节点（C++ 默认，Python 备选）继续负责指令消毒、急停锁存、传感器心跳超时和 Twist→Ackermann 转换。

核心约束：

- LiDAR 不做 SLAM 或静态地图定位；必须允许通过连续扫描匹配生成激光里程计并融合进 EKF，同时继续进入 obstacle/inflation costmap。激光里程计不得直接发布 `odom_combined -> base_footprint` TF，EKF 是唯一 TF owner。
- 运动链必须经过 `smartcar_safety`；系统禁止 `use_base=true,use_safety=false`。安全节点直接发布 `/ackermann_cmd`，`use_safety_ackermann:=true` 时跳过独立的 `cmd_vel_to_ackermann_drive` 节点。
- 方向门必须位于 velocity smoother 之后。`Prepare/Activate/Renew/Stop` 租约同时绑定 boot epoch、lease ID、generation 和 `NavigateToPose` UUID；错方向、过期许可、非法 Twist 或重放必须完整输出零并锁存故障。
- 唯一路线方向模式是：start/QR 为 `forward`，QR 后的出站 corridor 与 VLM 为 `reverse`，VLM 后至 return 全部为 `forward`。`validate_waypoints()` 必须拒绝全正向或越界方向配置。
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
src/smartcar_interfaces/             ReadQr、DescribeScene 与方向租约接口
src/smartcar_safety/                 后置方向门和速度安全门（C++ 默认，Python 备选）
src/smartcar_nav2/                   Nav2 配置、行为树和航点
src/smartcar_vision/                 QR 与 VLM 服务
src/smartcar_speech/                 可选火山 TTS consumer
src/smartcar_task/                   五子任务状态机
src/smartcar_bringup/                分层和完整系统 launch
src/smartcar_tools/                  场地参考、航点编辑、媒体入口与里程计诊断
scripts/                             RDK 同步与环境脚本
tests/                               仓库级合同测试
```

权威运行手册为 `docs/deployment/rdk-environment-setup.md`。若文档与当前实现冲突，以代码、README 和部署手册为准。

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

RDK 免密连接：有线 `root@192.168.128.10`，无线 `root@172.16.24.193`。ROS2 环境入口是 `/opt/tros/humble/setup.bash`，日常统一使用 `~/source_env.sh`，不要叠加 `/userdata/dev_ws` overlay。

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

vendor-only 全量 lint 默认 opt-in：需要时使用 `-DSMARTCAR_ENABLE_VENDOR_LINT=ON`，不要把继承源码的历史格式债务混入默认功能测试。

## 部署与实车门禁

截至 2026-07-25，标定与导航验证进展：

### 已完成

- ✅ 陀螺仪零偏标定：`gyro_z_bias = 0.000614`（30 分钟稳态复标，参数链路已打通至 `smartcar_system.launch.py` 默认值）。
- ✅ 外参测量（URDF + 微调）：`base_footprint→base_link (0.0841,0,0.03)`、`base_link→laser (-0.05,0,0.23)`、`base_link→camera (0.1205,0,0.11)`。
- ✅ 轮速标定验证：`longitudinal_velocity_scale = 1.03` 实测通过。
- ✅ P → 任务发布点导航：0.15 m/s，5 航点，约 35s 完成，自动锁停。2D LiDAR 避障通过锥桶膨胀补偿（local inflation 0.55 / global 0.65）+ footprint 自过滤可用。
- ✅ 标定参数链路：8 个传感器参数从 `smartcar_system` → `smartcar_bringup` → `origincar_bringup` → `base_serial` 完整传递。
- ✅ RF2O 激光里程计：朝向修复（laser_yaw=1.5708），X 跟踪与轮式一致，Y 漂移率 ~14%。建议在特征丰富的竞赛场地启用，当前训练场地可保持关闭。

### 待完成

- 实测并替换 `default_waypoints.yaml` 占位航点（`nav_only.yaml` 已支持纯导航实测，语义航点坐标仍需场地验证后标定）。
- 完成转向标定（`steering_command_scale` / `offset`）。
- 车轮离地验证 QR→VLM 实际倒车转向（已完成 angular.z 翻转修复）；倒车路径绕圈问题已做软件修正（QR 精确到位 + 规划半径恢复至 0.55），待实车复测。
- 修正并现场确认两处正向 DUBIN 兜圈风险：`c_corner_1 -> c_corner_2` 和 `c_corner_4 -> b_corridor_return_enter`。几何候选航向约为 `c_corner_2=38°`、`b_corridor_return_enter=-135°`，尚未写入路线。
- 配置并实测 VLM 后端（火山 Ark 或端侧模型）；如需语音，验证 TTS + 网络 + 扬声器。
- 验证人工物理急停 → 车轮离地 → 低速地面 → 完整赛道测试。
- 0.30 m/s 运动异常尚未完成单变量复测；历史 EKF 超期发生在发车前，不能归因于车速。已实施首批调度修复，仍须现场分级验证。

五个运动门禁默认为 `false`，仅对应项目实测完成后显式开启。

> **已知问题与避坑指南** → `docs/reference/known-issues.md`（20+ 条硬件/导航/运维/任务避坑记录）
> **场地工具与纯导航测试** → `docs/reference/field-tools.md`（航点编辑/可视化/里程计诊断/电压监控/纯导航一键测试）

## 脚本

| 脚本 | 用途 |
|------|------|
| `scripts/sync_to_rdk.py` | Windows→RDK 源码同步（`push`/`pull`/`setup`） |
| `scripts/nav_test.sh` | **纯导航一键测试**：清理→构建→启动→等就绪→RViz，急停锁存，不自动发车 |
| `scripts/monitor_mission.py` | **任务状态监控**：航点进度、当前动作、耗时，每秒刷新 |
| `scripts/safe_start.sh` | **安全启动**：急停锁存 + 完整 bringup，提示用户确认后手动发车 |
| `scripts/ros_cleanup.sh` | 全量进程清理 + SHM 端口释放；部署到 `/usr/local/bin/ros_cleanup` |
| `scripts/verify_autostart.sh` | CI 级 autostart 验证：构建→启动→监控→判过/不过 |
| `scripts/source_env.sh` | RDK ROS2 环境入口（TROS + 工作空间 overlay） |

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
