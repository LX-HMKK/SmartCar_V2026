# CLAUDE.md

本文件为 Claude Code（claude.ai/code）提供本仓库的操作指导。

## 项目背景

本仓库面向**第二十一届全国大学生智能汽车竞赛-地瓜机器人智慧医疗赛**，硬件平台为 **OriginCar + RDK X5 8G + ROS2 Humble**。

## 高层架构

系统架构详见 `docs/智慧医疗挑战赛_技术方案_本地部署版.md`，为分层 ROS2 栈：

- **任务决策层**：五子任务状态机、二维码识别（`zbar_ros`）、人形立牌图生文（相机 → 端侧 VLM → 语音/屏幕输出）。
- **导航层**：Nav2 Waypoint Follower + Regulated Pure Pursuit；航点序列为 P → 任务点 → 通道口 → C 环道 → 返回 P。
- **避障感知层**：2D LiDAR 障碍物提取（`obstacle_detector_2`）、VFH 反应式紧急避障备用、YOLO 人形检测触发 VLM 图生文。
- **定位层**：纯惯导，轮式里程计（STM32 编码器）+ IMU，经 `robot_localization` EKF 融合；无 SLAM。
- **控制层**：OriginCar 官方阿克曼底盘驱动（舵机转角 + 电机速度）。

核心约束：

- LiDAR **仅用于避障**，不参与定位。
- 锥桶仅依靠 LiDAR 避障，无需视觉。
- 人形立牌由相机 + 端侧 VLM 完成图生文，设置 8 秒超时与兜底文案。
- 发车时车模手动置于 P 区中心，代码假设该点为坐标原点。

## 仓库现状

```
src/
  origincar/                 # 官方包 vendor 副本（.git 已剥离，含 Humble 修复）
    origincar_base/          # 底盘驱动 + bringup（已修复 static_transform_publisher）
    origincar_bringup/       # USB/WebSocket 显示（空壳包，保留不动）
    origincar_description/   # URDF + 网格
    origincar_msg/           # 自定义消息
    utils/                   # 图像传输工具
    ydlidar_ros2_driver/     # 激光驱动（Humble LifecycleNode→Node 修复）
    3rdparty/                # ackermann_msgs、serial_ros2、aurora930
  third_party/
    obstacle_detector_2/     # 锥桶 LiDAR 检测（包名 obstacle_detector）
  smartcar_bringup/          # 顶层 launch 组合器（完成）
config/                      # 跨包共享配置（待填充）
scripts/
  sync_to_rdk.py             # 本机 <-> RDK 同步（push/pull/init-vendor/setup）
  source_env.sh              # RDK 工作空间环境入口
tests/                       # sync_to_rdk.py 单元测试（38 用例，stdlib unittest）
docs/                        # 技术方案、部署、评审文档、superpowers spec/plan
```

CI/构建/测试命令详情见下文 §命令速查；更多文档见 `README.md`。

## 部署连接

- RDK X5 当前为有线连接，`ssh root@192.168.128.10` 免密登录。
- 单一工作空间策略：**`/root/ros2_ws` 为统一构建/运行空间**，官方源 `/userdata/dev_ws` 退为 vendor 备份。
- 本机 `src/` 为权威源，经 **WSL rsync** 同步到 RDK `/root/ros2_ws/src/`（choco 的 rsync 不可用，盘符冒号冲突；WSL distro 默认 Ubuntu-22.04，`WSL_DISTRO` 环境变量可覆盖）。
- `source_env.sh` 已 scp 部署到 RDK `~/`，封装 source 顺序：`/opt/tros/humble/setup.bash` → `/userdata/dev_ws/install/setup.bash` → `/root/ros2_ws/install/setup.bash`。
- 详细连接方式、环境入口、已配置包与启动命令见 `docs/deployment/rdk-environment-setup.md`。

## 重要环境事实

- RDK 上 ROS2 环境入口为 `/opt/tros/humble/setup.bash`（不是 `/opt/tros/setup.bash`）。
- 硬件：**YDLIDAR Tmini Plus**（`/dev/ttyUSB0`，230400）、**OriginCar 底盘**（`/dev/ttyACM0`，115200）。
- **odom 话题与帧统一为 `/odom_combined` + `odom_combined`**（非技术方案的 `/odometry/filtered`+`odom`；EKF `publish_tf: true`，TF `odom_combined→base_footprint→base_link→laser` 完整）。
- `/scan` 不在 origincar_bringup 内，需通过 ydlidar 独立 launch 发布；smartcar_bringup 已用 `use_lidar` 开关统一进来。
- ⚠️ **base_to_link 静态 TF 偏移 x=0.41, y=0.12 存疑**（base_footprint→base_link 通常应为 0,0,0），赛前需实车标定复核（`memory/base-to-link-offset-todo.md`）。
- **端侧 VLM 模型**与**VFH 备用避障**尚未部署；**实车运动测试**尚未进行。
- RDK 无法访问 GitHub（443 超时），第三方包需本机下载后 scp 上传。
- 已修复点（vendor 化后固化）：
  - `static_transform_publisher` 的 Humble 兼容性（`--x/--y/--z/--roll/--pitch/--yaw/--frame-id/--child-frame-id` 命名参数）。
  - ydlidar launch 的 Humble 兼容性（LifecycleNode→Node、`node_executable`→`executable`）。
  - obstacle_detector 原 launch 为 ROS1 式（nodelet/rosparam）Humble 不可用，已在 smartcar_bringup 内重写。

## 命令速查

**本机**：
```powershell
# 同步（Windows 经 WSL rsync，colcon build 产物已内置排除）
python scripts/sync_to_rdk.py push            # 本机 -> RDK（--delete 镜像）
python scripts/sync_to_rdk.py push --dry-run  # 预览
python scripts/sync_to_rdk.py pull            # RDK -> 本机（--delete 显式开启）
python scripts/sync_to_rdk.py setup           # 部署 source_env.sh 到 RDK ~/
python scripts/sync_to_rdk.py init-vendor     # 一次性：回传官方包入 VCS

# 测试
python -m unittest tests.test_sync_to_rdk     # 38 用例
```

**RDK 上**（`ssh root@192.168.128.10`）：
```bash
source ~/source_env.sh                              # 加载环境
cd /root/ros2_ws && colcon build --symlink-install  # 全量构建
# 仅构建指定包
cd /root/ros2_ws && colcon build --packages-select smartcar_bringup --symlink-install

# 启动
ros2 launch smartcar_bringup smartcar_bringup.launch.py                  # 全量（底盘+LiDAR+障碍物检测）
ros2 launch smartcar_bringup smartcar_bringup.launch.py use_lidar:=false use_obstacle:=false  # 仅底盘
```

## 模块状态

| 模块 | 状态 | 分支 | 说明 |
|---|---|---|---|
| 工作空间基础设施 | ✅ done | merged to main | sync_to_rdk.py, vendor 化, .gitattributes, README |
| smartcar_bringup | ✅ done | merged to main | 顶层 launch 组合器，use_lidar/use_obstacle 开关，obstacle_detector launch 重写 |
| smartcar_nav2 | 🔄 实现中 | feat/smartcar-nav2 | Nav2 参数 + RPP + Waypoint Follower + launch 已完成，RDK 构建通过，待实车启动验证 |
| smartcar_vision | ⏳ 未开始 | — | zbar_ros QR + VLM 图生文 |
| smartcar_task | ⏳ 未开始 | — | 五子任务状态机 |

各模块 spec 见 `docs/superpowers/specs/`；每模块开始前须先调研 OSS 参考（`memory/oss-reference-repos.md`）。

## 提交规范

提交信息采用 Angular 规范，**主题使用中文**。

在 PowerShell 中，为避免 `@` 被解析为数组运算符，先用 here-string 写入临时文件再提交：

```powershell
@'
fix(rdkx5): 修复导出脚本路径问题

- 修复 ONNX 路径使用 POSIX 斜杠
- 校准数据 pad 值对齐 Ultralytics
'@ | Out-File -Encoding utf8 .git-msg

git commit -F .git-msg
Remove-Item .git-msg
```

禁止在提交信息中添加 `Co-Authored-By` 尾注或任何协作者元数据。提交仅记录实际执行者。
