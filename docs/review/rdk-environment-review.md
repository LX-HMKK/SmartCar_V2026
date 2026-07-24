# 智慧医疗赛技术方案与 RDK X5 实际环境审查报告

> 审查时间：2026-07-15  
> 目标平台：OriginCar + RDK X5 8G + ROS2 Humble  
> 参考文档：`docs/智慧医疗挑战赛_技术方案_本地部署版.md`  
> 最新环境配置：`docs/deployment/rdk-environment-setup.md`

> **后续状态（2026-07-24）**：本文是 7 月 15 日的环境快照。外参、陀螺仪、轮速和 P→任务发布点低速导航已经完成；QR→VLM 确定性倒车软件链已部署并通过无底盘测试。转向/有效转弯半径、实体倒车、两处正向端姿、完整路线和 VLM 后端仍待验证。正文中的“尚未运动”与旧架构描述只代表 7 月 15 日，当前结论与命令以部署手册为准。

## 1. 审查方法

- 读取本地技术方案，提取关键决策、依赖、硬件假设与风险。
- 通过 `ssh root@192.168.128.10` 登录 RDK，检查操作系统、ROS2 环境、已安装包、硬件接口、系统资源与模型文件。
- 将方案假设与实际环境逐项对比，评估匹配度与风险。

## 2. 已确认的环境基础

| 项目 | 状态 | 说明 |
|---|---|---|
| 操作系统 | ✅ | Ubuntu 22.04.5 LTS，kernel 6.1.83 aarch64 |
| ROS2 Humble 基础 | ✅ | `ros-humble-ros-base`、`rclcpp`、`rclpy`、`tf2`、`nav-msgs` 等已安装 |
| `robot_localization` | ✅ | 已安装，可支撑 IMU + 轮式里程计 EKF 融合 |
| TogetheROS 生态 | ✅ | `tros-humble-base`、`mipi-cam`、`imu-sensor`、`dnn-node`、`hobot-yolo-world`、`hobot-cv`、`hobot-codec`、`websocket` |
| BPU 运行时 | ✅ | `hobot-dnn` 已安装，可支撑端侧模型推理 |
| MIPI 相机驱动 | ✅ | `tros-humble-mipi-cam` 已安装 |
| IMU 驱动 | ✅ | `tros-humble-imu-sensor` 已安装 |
| YOLO 检测 | ✅ | `tros-humble-hobot-yolo-world` 及 `/app/pydev_demo` 下多个示例 |
| 基础模型包 | ✅ | `hobot-models-basic` 已安装 |
| 串口/相机设备 | ✅ | `ttyACM0`、`ttyUSB0`、`video0`、`video1` 存在 |
| **Nav2 导航栈** | ✅ | 已通过 apt 安装完整栈 |
| **zbar_ros 二维码识别** | ✅ | 已通过 apt 安装 `ros-humble-zbar-ros` |
| **`obstacle_detector_2`** | ✅ | 已克隆到 `/root/ros2_ws/src` 并编译为 `obstacle_detector`；rosdep 已通过 |
| **YDLIDAR 驱动** | ✅ | OriginCar 工作空间 `/userdata/dev_ws` 已含 `ydlidar_ros2_driver`，已验证 `/scan` 约 10 Hz |
| **OriginCar 底盘/IMU/EKF** | ✅ | `/userdata/dev_ws/src/origincar` 包含完整底盘、Madgwick、EKF、TF launch |

## 3. 关键缺口

| 缺口 | 风险 | 说明 |
|---|---|---|
| **VFH 备用避障** | 中 | `obstacle-avoider-ros2` 因 RDK 无法访问 GitHub 未获取；属于备用方案，可后续补入 |
| **端侧 VLM 模型文件** | 高 | 未见 Gemma-3-4B-IT-QAT / Qwen2-VL 等模型，需下载/量化/转换为 BPU 格式；内存 6.9 GiB，4B 模型存在 OOM 风险 |
| **Nav2 与底盘/雷达联调** | 中 | 各组件单独可用，尚未做端到端导航/避障联调 |
| **实车运动测试** | 中 | 已验证静态 `/odom_combined`/`/imu/data`/`/scan`，尚未发布 `/cmd_vel` 让车实际移动 |
| **rosdep 已初始化** | ✅ 已解决 | 2026-07-15 已执行 `rosdep init && rosdep update` |

## 4. 硬件与资源评估

- **相机**：满足。检测到 USB Camera（Alcorlink）与 Aurora 930（深度/双目），MIPI 驱动已装。
- **IMU**：驱动已装，`origincar_bringup.launch.py` 已配置 Madgwick + EKF，`/imu/data` 已验证。
- **底盘**：串口 `/dev/ttyACM0`、波特率 115200，已启动并输出 `/odom`、`/odom_combined`。
- **LiDAR**：**YDLIDAR Tmini Plus**，`/dev/ttyUSB0`、波特率 230400，已验证 `/scan` 约 10 Hz。
- **内存**：总量 **6.9 GiB**（空闲约 5.8 GiB），并非方案假设的 8 GiB；无 Swap。4B 量化 VLM 叠加 ROS2/Nav2/YOLO/相机/雷达后，**存在 OOM 风险**。
- **存储**：19 GiB 可用，足够。

## 5. 建议下一步

1. 解决 RDK 外网访问问题，或从本地 PC 下载 `obstacle-avoider-ros2` 后通过 `scp` 上传到 `/root/ros2_ws/src`。
2. 获取/转换 VLM 模型到 BPU 格式，实测推理延迟与峰值内存；若 4B 模型 OOM，降级到 2B。
3. 启动 `origincar_bringup.launch.py` 实车验证底盘里程计、IMU、EKF `/odom_combined` 输出（**已完成静态验证，实车运动待做**）。
4. 运行 `obstacle_extractor_node` + LiDAR，验证 `/raw_obstacles` 与 Nav2 costmap 对接。
5. 配置 Nav2 waypoints，进行端到端导航/避障联调。

## 6. 原始检查数据

### 6.1 操作系统

```text
Linux ubuntu 6.1.83 #2 SMP PREEMPT Fri Nov 22 11:51:26 CST 2024 aarch64 aarch64 aarch64 GNU/Linux
PRETTY_NAME="Ubuntu 22.04.5 LTS"
```

### 6.2 ROS2 环境

```text
ros2: command not found          # 非交互式会话未 source
source /opt/tros/humble/setup.bash  # 实际路径
```

### 6.3 已安装 ROS/TogetheROS 包（节选）

```text
ros-humble-robot-localization
ros-humble-nav2-bringup
ros-humble-nav2-waypoint-follower
ros-humble-nav2-regulated-pure-pursuit-controller
ros-humble-nav2-dwb-controller
ros-humble-nav2-costmap-2d
ros-humble-nav2-rviz-plugins
ros-humble-zbar-ros
tros-humble-base
tros-humble-mipi-cam
tros-humble-imu-sensor
tros-humble-dnn-node
tros-humble-hobot-yolo-world
tros-humble-hobot-cv
tros-humble-hobot-codec
tros-humble-websocket
hobot-dnn
hobot-models-basic
```

### 6.4 硬件接口

```text
/dev: ttyACM0 (CH340), ttyUSB0 (CP2102), video0, video1
lsusb: Aurora 930, USB 2.0 Camera (Alcorlink), CP210x UART Bridge, QinHeng USB Serial
YDLIDAR: /dev/ttyUSB0 @ 230400, Tmini Plus
Chassis: /dev/ttyACM0 @ 115200
```

### 6.5 资源

```text
Mem: 6.9Gi total, 5.8Gi available, Swap: 0B
/dev/root: 29G total, 19G available
```

### 6.6 工作空间

```text
/userdata/dev_ws      # OriginCar 官方工作空间（build/install/log/src 齐全）
/root/ros2_ws         # 自建工作空间，当前含 obstacle_detector_2
```
