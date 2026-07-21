# 变更日志

## 2026-07-21 - 实车标定与导航验证

### 标定完成

- **陀螺仪**: `gyro_z_bias = 0.000853 rad/s`（静止采样 173 点，均值 0.049°/s）。
- **外参**（URDF 理论值 + 现场微调）:
  - `base_footprint → base_link`: (0.0841, 0, 0.03) — 后轴投影至底盘几何中心
  - `base_link → laser`: (-0.05, 0, 0.23) — 底盘中心上方 12cm，后移 5cm 减少车身自检
  - `base_link → camera`: (0.1205, 0, 0.11) — URDF camera_joint
- **轮速**: `longitudinal_velocity_scale = 1.03` 验证通过（直行 1m 里程计 ≈ 实测）。
- **转向**: 跳过（用户选择优先验证导航）。

### 参数链路打通

- `gyro_z_bias` 等 8 个传感器标定参数从 `smartcar_system.launch.py` 经 `smartcar_bringup.launch.py` → `origincar_bringup.launch.py` → `base_serial.launch.py` 完整传递。
- `bringup_coord.yaml` 新增 `calibration` 节记录标定值，`extrinsics` 全部标记 `measured: true`。

### 导航验证

- **P → a_task 任务发布点** 导航成功，0.15 m/s，约 35s 完成，自动锁停。
- 0.30 m/s 提速导致 EKF 更新超限 (`Failed to meet update rate`) 和 BT 过载，车失控乱跑，已恢复 0.15 m/s。

### 避障调优（2D LiDAR + 锥桶）

关键发现：2D LiDAR 无高度信息，锥桶上窄下宽——雷达只看到上半窄截面，车体下半蹭宽底。

| 参数 | 初始值 | 最终值 | 说明 |
|---|---|---|---|
| `footprint` | `[[±0.168, ±0.112]]` 对称 | `[[0.27,0.13],[-0.10,-0.13]]` 非对称 | 后轴原点，前 0.27m 后 0.10m |
| `footprint_padding` | 无 | 0.03 | 额外车身间隙 |
| `obstacle_min_range` | 0.0 | 0.25 | 过滤 25cm 内车身自检回波 |
| 局部 `inflation_radius` | 0.30 | 0.55 | 锥桶膨胀补偿 |
| 全局 `inflation_radius` | 0.45 | 0.65 | 路径规划提前绕行 |
| `xy_goal_tolerance` | 0.15 | 0.25 | Ackermann 无法原地旋转，放宽防兜圈 |
| `yaw_goal_tolerance` | 0.20 | 0.35 | 同上 |

### 已知问题

1. **RDK X5 性能瓶颈**: EKF 在高负载下无法维持更新率，BT 100Hz tick 偶发超限。速度 > 0.20 m/s 时风险增大。
2. **ROS2 lifecycle 卡死**: `velocity_smoother` 激活后偶发 `get_state` 服务无响应，需彻底清理进程 + 重启 daemon 恢复。
3. **`navigation_test.launch.py` 外参默认全零**: 必须显式传入 `base_x/laser_x/laser_z` 等参数，否则 TF 错误导致 costmap 充满致命障碍。
4. **转向标定未做**: `steering_command_scale=0.5` 保持默认，航向偏差可能累积。
5. **路线未实测标定**: 航点坐标来自规则图推算，`calibrated` 为测试性标记。

### 修改文件

- `src/origincar/origincar_base/launch/origincar_bringup.launch.py` — 校准参数声明与转发
- `src/smartcar_bringup/config/bringup_coord.yaml` — 外参实测值 + calibration 节
- `src/smartcar_bringup/launch/smartcar_bringup.launch.py` — 校准参数中转
- `src/smartcar_bringup/launch/smartcar_system.launch.py` — 校准参数入口默认值
- `src/smartcar_nav2/config/field_test_nav2_params.yaml` — 避障/终点容忍度调优

## 2026-07-19 - 场地路线与独立测试入口

### 新增

- 新增默认关闭的 RF2O 无地图 scan-to-scan 激光里程计，发布 `/odom_laser` 并作为可选观测融合进 EKF；EKF 仍是 `odom_combined -> base_footprint` 的唯一 TF owner。
- 新增按规则图生成的 68 点未标定基线路线、`route_tool` 原子编辑命令、RViz 点位编辑器和单文件 `pull-route` 同步。
- 新增纯导航、语音、二维码和图生文四个独立测试入口；纯导航采用 `/navigate_through_poses`、`0.15 m/s` 现场参数和显式 `prepare -> arm -> start` 安全流程。
- 新增 PyQt5 HDMI 图生文界面，支持实体相机或单图循环回放，并复用 `/smartcar/output/text`。

### 安全边界

- 不增加静态地图、`map_server`、AMCL 或 SLAM；RF2O 默认关闭，启用时条件性要求 `laser_odometry_calibrated=true`。
- 路线保持 `calibrated: false`，五项原有运动门禁保持默认 `false`；任何导航终态重新锁存急停。
- 自动化验证没有启动底盘、LiDAR 驱动、实体相机、音频、真实 API，也没有发布非零速度。

### 验证

- 本地根合同 130/130；`smartcar_tools` 共 44 项，其中 43 项通过，Windows 因缺少 PyQt5 跳过 1 项 offscreen UI 布局测试。
- RDK X5 上 18 个包构建通过，干净全量结果为 `578 tests, 0 errors, 0 failures, 90 skipped`；RDK 的 PyQt5 offscreen UI 测试通过。
- 严格无硬件 smoke 连续 3 轮通过；7 个 `smartcar_tools` CLI 已安装，五个 launch 入口通过 `--show-args` 解析。

### 待现场验收

- 仍需完成路线逐点、外参、转向、轮速、IMU、RF2O 数据质量和物理急停标定。
- 相机、二维码实景、VLM 后端、HDMI、网络、TTS、扬声器及完整赛道运动均未实测，不能据此宣称可直接上场。

## 2026-07-18 - 火山视觉与语音适配

### 新增

- 新增火山 Ark 图生文 Python 3 适配器和显式 opt-in 配置，复用现有 8 秒共享硬期限与固定兜底文案。
- 新增可选 `smartcar_speech` ROS 2 包，消费任务语音文本，调用火山 V1 TTS 并通过 RDK `ffplay` 播放。
- 新增 `/smartcar/speech/status` 请求状态、队列上限、响应大小、网络和播放超时控制。

### 安全边界

- VLM 与 TTS 默认禁用，凭据仅从环境变量读取，HTTP 客户端拒绝非 HTTPS 和重定向。
- 自动化测试不访问外部 API、不播放音频、不启动实体相机，也不发送运动命令。
- 真实 API、现场网络、模型、扬声器和完整任务播报仍待人工验收；不得据此宣称已具备竞赛现场运行条件。
- 云端 VLM 还需确认比赛公网规则和赛场稳定性；当前异步 TTS 不提供“播完再走”的 action/ack 保证。

### 验证

- 本地根合同 `114/114`；视觉单测 33 项、语音单测 14 项通过。
- RDK X5 上 16 个包构建通过，完整结果为 `533 tests, 0 errors, 0 failures, 90 skipped`。
- 关闭底盘、LiDAR、障碍物、实体相机和语音节点的严格无硬件 smoke 连续 3 轮通过。

## 2026-07-18 - 完整软件里程碑

本里程碑已合并到 `main`，完成从底盘控制到任务编排的竞赛软件主链路。

### 新增

- 新增视觉服务接口、二维码识别和端侧 VLM 服务，统一执行 8 秒请求期限与兜底返回。
- 新增 fail-closed 速度安全门、定位故障锁存、急停和复位接口。
- 新增 Smac Hybrid（DUBIN）+ Regulated Pure Pursuit 阿克曼导航配置，禁止 Spin 和原地旋转。
- 新增五子任务状态机、`FollowWaypoints` 调用、QR/VLM 语义航点、停止与复位流程。
- 新增 `smartcar_system.launch.py` 完整系统入口和合成传感器无硬件 smoke。

### 加固

- 增加底盘命令模式隔离、串口帧校验、命令 watchdog、串口故障停机和正常退出零命令。
- 增加轮速、转向和 IMU 标定参数，并统一 EKF `/odom_combined` 定位链路。
- 将继承 vendor 包的全量 lint 改为显式 opt-in，默认测试聚焦功能合同。

### 验证

- 本地仓库级合同测试：108/108 通过。
- RDK X5：15 个包构建通过，`508 tests, 0 errors, 0 failures, 90 skipped`。
- 严格无硬件系统 smoke 连续 3 轮通过；未启动硬件、实体相机或发布非零速度。

### 部署门禁

- 默认航点仍需赛场实测替换，底盘/LiDAR/相机外参仍需测量。
- 转向、轮速和陀螺仪仍需实车标定，人工物理急停仍需验收。
- 端侧 VLM 模型尚未部署；VFH 和 YOLO 自动触发不属于当前 release 依赖。
- 车轮离地、低速地面和完整赛道测试尚未进行。
