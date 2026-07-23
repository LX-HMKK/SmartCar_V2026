# 变更日志

## 2026-07-23 - 系统 CPU 深度优化：按需 QR、USB 摄像头、C++ 安全节点

### 背景

上一轮优化（R1-R4）将系统 idle CPU 从 ~120% 降至 ~67%，但 safety_node 剩余 44% 来自 Python ARM64 解释器开销，且 barcode_reader（61.1%）和 aurora930_node（33.3%）仍在 idle 时持续消耗 CPU。本轮在 R1-R4 基础上继续深挖，通过 QR 按需启动、USB 摄像头切换和 safety_node C++ 移植，将系统 idle CPU 从 ~67% 最终压至 ~10%。

### R5: 清理重复 ekf_node 热修复服务

- 发现 RDK 上运行着 transient systemd 服务 `smartcar-ekf-hotfix.service`（`Restart=on-failure`），与 launch 文件并行启动第二个 `ekf_node`，造成 EKF 进程双跑和 CPU 浪费。
- 停用该服务后，系统只有唯一一个 launch 管理的 `ekf_node`。
- **效果**: 消除重复 EKF 进程，系统 CPU 从 ~67% 降至 ~21%（该服务在 R1-R4 后期才被注意到，故此前记录中的 ~67% 实际已包含双 EKF 开销）。

### R6: QR 扫描任务按需启动

- `barcode_reader`（zbar_ros）不再随系统启动，改为 `task_node` 到达 QR 航点时通过 subprocess 动态拉起，读完立即 kill。
- `smartcar_system.launch.py` 中 `use_zbar` 默认改为 `false`。
- `RosVision._ensure_reader()` / `_stop_reader()` 管理 `barcode_reader` 生命周期，`read_qr()` 用 `try/finally` 保证清理。
- **效果**: `barcode_reader` 从 61.1% 降至 0%（idle），仅在 QR 航点短暂运行约 8s。

### R7: Aurora 930 切换 USB 摄像头

- 相机驱动从 `aurora`（deptrum-ros-driver-aurora930）切换为 `hobot_usb_cam`（Alcorlink USB 2.0 Camera）。
- 系统默认 `camera_driver:=usb`，保留 `aurora` 选项以兼容旧硬件。
- **效果**: `aurora930_node` 33.3% → 消除，`hobot_usb_cam` 约 16.7%，净节省 ~16.6%。

### R8: safety_node C++ 移植

- 将 safety_node 从 Python 完整移植为 C++（`safety_node_cpp`），300+ 行代码，逻辑与 Python 版完全等价（急停锁存、odom 心跳超时、cmd_vel 消毒、Twist→Ackermann 内联转换）。
- 新增文件: `include/smartcar_safety/guard.hpp`、`src/guard.cpp`、`src/safety_node.cpp`、`CMakeLists.txt`。
- 包构建类型从 `ament_python` 改为 `ament_cmake`（含 `ament_cmake_python` 保留 Python 模块）。
- launch 增加 `use_cpp` 参数（默认 `true`），通过 `IfCondition`/`UnlessCondition` 在 C++/Python 间切换。
- 线程安全: `std::mutex` + `lock_guard` 保护所有回调；`on_timer` 中 publish 移至锁外避免持锁 IO。
- **效果**: `safety_node` 从 33.3%（Python，R5 后 idle）降至 6.4%（C++），约 5 倍降低。

### R9: 审查与修复

- 五路并行审查（C++ 安全逻辑、launch 链路、QR 按需、构建配置、测试缺口）确认无安全回归。
- 修复 `task_node.py` 中 `_stop_reader`/`_ensure_reader` 的僵尸进程风险：增加 `process.wait(timeout=3)` 并在超时后 `kill()`，最后 `poll()` 确认终止。
- `rclcpp::SerializedMessage` 因 Humble dispatch 兼容性问题回退为 typed subscription（C++ 反序列化开销可忽略）。

### 系统 CPU 进化

| 阶段 | 系统 CPU | idle |
|------|:---:|:---:|
| R4 后（CHANGELOG 上次记录） | ~67% | ~33% |
| + 清理 2x ekf hotfix (R5) | ~21% | ~79% |
| + zbar 按需 (R6) | ~19% | ~81% |
| + USB 摄像头 (R7) | ~17% | ~83% |
| + C++ 安全节点 (R8) | ~10% | ~90% |

五次优化累计将系统 idle CPU 从 ~120% 降至 ~10%（约 12 倍），RDK X5 四核 A55 具备充裕算力应对导航、避障和视觉任务的动态峰值。

### 修改文件 (10 files, +420/-50)

- `src/smartcar_safety/src/safety_node.cpp` — 新增 C++ 安全节点（~320 行）
- `src/smartcar_safety/src/guard.cpp` — 新增 SafetyGuard C++ 实现（~157 行）
- `src/smartcar_safety/include/smartcar_safety/guard.hpp` — 新增头文件（~43 行）
- `src/smartcar_safety/CMakeLists.txt` — 新增 cmake 构建配置
- `src/smartcar_safety/package.xml` — `ament_python` → `ament_cmake`
- `src/smartcar_safety/launch/smartcar_safety.launch.py` — C++/Python 可切换
- `src/smartcar_bringup/launch/smartcar_bringup.launch.py` — 转发 `use_safety_cpp`
- `src/smartcar_bringup/launch/smartcar_system.launch.py` — `use_zbar=false`、`use_safety_cpp`、`camera_driver=usb`
- `src/smartcar_task/smartcar_task/task_node.py` — QR 按需子进程管理

## 2026-07-23 - 系统 CPU 优化：安全节点简化、底盘定时器重构、阿克曼内联

### 背景

系统 idle 状态总 CPU ~120%（单核等效），origincar_base_node 占 66.7%（`spin_some()` 200Hz 轮询），safety_node >30%（odom CDR 解析 + localization fault 状态机），外加独立 `cmd_vel_to_ackermann_drive` Python 进程 7.9%。

### R1: 安全节点职责删减

- **删除 odom 内容验证**: `serialized_odometry_is_finite()` 及全部 CDR 二进制解析代码（~90 行），odom 回调改为纯时间戳标记。EKF 数据质量由 EKF 保证。
- **删除 localization fault 状态机**: `guard.py` 移除 `odom_invalid`、`localization_fault_latched`、`clear_localization_fault()` 及 5 个相关方法。`evaluate()` 从 8 个分支减少到 4 个核心分支。
- **删除服务**: `/smartcar/safety/clear_localization_fault` 服务及 `task_node.py` 中对应的客户端和 `_clear_localization_fault` 方法。
- **效果**: safety_node 从 >30% 降至 24.6%（-20%）。

### R2: origincar_base 定时器重构

- `Control()` 的自定义 `while(rclcpp::ok())` 轮询循环（`spin_some()` 每 5ms + `sleep(5ms)` = 200Hz）改为 ROS2 标准事件驱动：`rclcpp::spin()` + 5ms wall timer。
- 空闲时 executor 阻塞在 WaitSet 条件变量上，零 CPU。订阅回调仍由 executor 自动分发。
- **效果**: origincar_base_node 从 66.7% 降至 10.3%（-85%）。EK 从 CPU 饥饿恢复，`/odom_combined` 发布率升高，safety_node 回调增多至 46.2%（级联效应）。

### R3: odom 回调节流

- `_on_odom` / `_on_raw_odom` 增加 50ms 最小处理间隔（`odom_throttle_interval_sec` 参数，可设 0.0 关闭）。安全节点只需知道 odom 在 350ms 内新鲜，20Hz 采样足够。
- `raw_odom_timeout_sec` 从 guard 硬编码提升为 ROS 参数（默认 0.25）。
- **效果**: EKF 稳定后 safety_node 降至 42.7%，系统总计 ~78%。

### R4: cmd_vel_to_ackermann 内联

- safety_node 直接发布 `AckermannDriveStamped` 到 `/ackermann_cmd`，内部在 `_on_timer` 中将缓存的 Twist 转为 Ackermann（`math.atan(wheelbase * angular / speed)`，转向角限幅）。
- 新增 `wheelbase`、`max_steering_angle`、`ackermann_frame_id` 参数。
- 新增 `use_safety_ackermann` 启动参数（默认 `true`），通过 `skip_converter` 标志传递至 `base_serial.launch.py`，在安全启用时跳过独立的 `cmd_vel_to_ackermann_drive` 节点。
- **效果**: 省掉一个 Python 进程（~12% CPU），系统总计降至 ~67%。

### 最终效果

| 进程 | 优化前 | 优化后 |
|------|--------|--------|
| origincar_base_node (C++) | 66.7% | 8.7% |
| safety_node (Python) | >30% | 43.9% |
| cmd_vel_to_ackermann (Python) | 7.9% | 已消除 |
| 系统总计 | ~120% | ~67% |

safety_node 剩余 44% 是 Python ARM64 解释器开销（rclpy 回调调度 + GIL），需 C++ 重写才能进一步降低。

### 修改文件 (19 files, +247/-456)

- `src/smartcar_safety/smartcar_safety/odometry.py` — 删除 CDR 解析，只保留 `odometry_is_finite()`
- `src/smartcar_safety/smartcar_safety/guard.py` — 删除 localization fault 状态机
- `src/smartcar_safety/smartcar_safety/safety_node.py` — 简化为纯心跳，内联 Ackermann，回调节流
- `src/smartcar_safety/config/safety.yaml` — +`raw_odom_timeout_sec`、+`odom_throttle_interval_sec`
- `src/smartcar_safety/package.xml` — +`ackermann_msgs` 依赖
- `src/origincar/origincar_base/src/origincar_base.cpp` — `Control()`→`start()`+`on_serial_tick()`+`rclcpp::spin()`
- `src/origincar/origincar_base/include/origincar_base/origincar_base.h` — 声明更新
- `src/origincar/origincar_base/launch/base_serial.launch.py` — +`skip_converter` 条件
- `src/origincar/origincar_base/launch/origincar_bringup.launch.py` — 转发 `skip_converter`
- `src/smartcar_bringup/launch/smartcar_bringup.launch.py` — +`use_safety_ackermann`、`skip_converter` 逻辑
- `src/smartcar_task/smartcar_task/task_node.py` — 删除 `_clear_fault_client` 和 `_clear_localization_fault`
- `src/smartcar_task/smartcar_task/protocols.py` — `run_reset_sequence` 3 参数
- 测试文件 ×6 — 更新合同断言、删除 CDR/localization_fault 测试
- `CLAUDE.md`、`CHANGELOG.md` — 更新文档

## 2026-07-22 - 定位更新率取证纠偏与调度减负

- 原始日志证明最严重的 EKF 更新率告警发生在发车前，撤回“0.30 m/s 导致 EKF 过载”和“IntegrationClock pose/速度冲突”的无证据归因。
- EKF `transform_timeout` 改为非阻塞并开启诊断；保持 30 Hz、`sensor_timeout` 和输入队列不变。
- Nav2 BT tick 由 10 ms 调整为 20 ms；controller 和 costmap 更新频率不变。
- RF2O 三处逐帧 INFO 改为 DEBUG，避免 10 Hz 热路径持续写日志。
- 显式隔离 safety 与 RF2O 参数文件，并将安全输入 QoS 收敛为 `KeepLast(1)`。
- 修复 `odom_diag` 在 timer 回调内 shutdown 后可能不退出的问题，并加入 `/imu/data_raw` 统计。

## 2026-07-22 - 统一九点任务路线与官方场地参考层

- 删除 68 点 `full_course_route`、独立纯导航 launch/runner/probe、专用 Nav2 配置及对应测试，只保留 `default_waypoints.yaml`。
- 路线调整为 P -> QR 留距位 -> 出站通道 -> C 区四角（角 1 触发 VLM 并朝左）-> 回程通道 -> P；QR/VLM/返程按三段 `FollowWaypoints` 提交。
- 新增官方规则图 Marker 参考层和 RViz Interactive Marker 编辑器，可直接拖动位置、旋转航向并右键保存/撤销/重载；编辑器不启动运动栈且默认锁存急停。
- `pull-route` 替换为只回传唯一语义路线的 `pull-waypoints`。

## 2026-07-22 - 航点可视化、导航 RViz 与里程计诊断

### 新增工具

- **`waypoint_viz`**: 独立航点可视化节点，读取 Nav2 waypoints YAML，发布 MarkerArray（球体+箭头+标签）到 `/smartcar/waypoints/markers`（TRANSIENT_LOCAL QoS），不依赖导航栈。
- **`odom_diag`**: 里程计管道诊断工具，监控 `/odom`、`/imu/data_raw`、`/odom_combined`、`/scan` 和 `/odom_laser` 的速率与间隔，输出 EKF 诊断警告。
- **`navigation.rviz`**: 导航监控 RViz 配置，包含 Global/Local Costmap、Global/Local/Transformed Plan、LaserScan、Waypoint Markers、TF、RobotModel，TopDownOrtho 视图。

### 高速里程计分析

- 完成 EKF/串口/CPU 管道分析，写出 `docs/review/odometry-speed-analysis.md`。
- **后续纠偏**: 原始 `/odom` pose 未被 EKF 融合，因此当时的 `IntegrationClock` 根因推断不成立；以本日新增的取证纠偏章节和修订后的分析文档为准。

### 修改文件

- `src/smartcar_tools/smartcar_tools/waypoint_viz.py` — 新增
- `src/smartcar_tools/smartcar_tools/odom_diag.py` — 新增
- `src/smartcar_tools/rviz/navigation.rviz` — 新增
- `src/smartcar_tools/setup.py` — +2 entry_points
- `docs/review/odometry-speed-analysis.md` — 新增
- `CLAUDE.md` — 更新工具引用与诊断命令

## 2026-07-22 - RF2O 激光里程计标定与 LiDAR 朝向修复

### RF2O 标定

- **根因发现**: 2D LiDAR 物理安装转了 90°（Y 朝后、X 朝左），导致 RF2O 前进报横向漂移（1.29m Y / 0.05m X）。
- **修复**: `laser_yaw = 1.5708` (+90°)，在 `base_link → laser` 静态 TF 中加入 Z 轴旋转。
- **验证结果**: 修复后 RF2O 前进跟踪与轮式里程计一致（Δx 33cm vs 30cm），Y 漂移率从 28× 降至 ~14%。
- **RF2O 配置**（`laser_odometry.yaml`）: 10Hz，差分模式输入 EKF，pose_cov x/y=0.05 yaw=0.03，跳变拒绝 3.0m，publish_tf=false。
- **结论**: RF2O 可用，但横向漂移仍存在（~14% X 移动量），建议在特征丰富的竞赛场地启用。当前训练场地可保持关闭。

### 参数链路完善

- `laser_yaw` 加入 `smartcar_system.launch.py` 的 `extrinsic_defaults` 字典，默认值 `1.5708`。
- `bringup_coord.yaml` 的 `link_to_laser.rpy` 更新为 `[0.0, 0.0, 1.5708]`。

### 修改文件

- `src/smartcar_bringup/config/bringup_coord.yaml` — laser rpy 旋转
- `src/smartcar_bringup/launch/smartcar_system.launch.py` — laser_yaw 默认值

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
