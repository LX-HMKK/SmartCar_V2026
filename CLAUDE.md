# CLAUDE.md

本文件为 Claude Code（claude.ai/code）提供本仓库的操作指导。

## 项目背景

本仓库面向**第二十一届全国大学生智能汽车竞赛-地瓜机器人智慧医疗赛**，硬件平台为 **OriginCar + RDK X5 8G + ROS2 Humble**。

当前仓库处于早期搭建阶段。仅 `docs/` 下有内容，`src/`、`config/`、`scripts/`、`tools/`、`assets/` 为占位目录。构建、测试、lint 命令尚未定义。

## 高层架构

系统架构详见 `docs/智慧医疗挑战赛_技术方案_本地部署版.md`，为分层 ROS2 栈：

- **任务决策层**：五子任务状态机、二维码识别（`zbar_ros`）、人形立牌图生文（相机 → 端侧 VLM → 语音/屏幕输出）。
- **导航层**：Nav2 Waypoint Follower + Regulated Pure Pursuit；航点序列为 P → 任务点 → 通道口 → C 环道 → 返回 P。
- **避障感知层**：2D LiDAR 障碍物提取（如 `obstacle_detector_2`）、VFH 反应式紧急避障备用、YOLO 人形检测触发 VLM 图生文。
- **定位层**：纯惯导，轮式里程计（STM32 编码器）+ IMU，经 `robot_localization` EKF 融合；无 SLAM。
- **控制层**：OriginCar 官方阿克曼底盘驱动（舵机转角 + 电机速度）。

核心约束：

- LiDAR **仅用于避障**，不参与定位。
- 锥桶仅依靠 LiDAR 避障，无需视觉。
- 人形立牌由相机 + 端侧 VLM 完成图生文，设置 8 秒超时与兜底文案。
- 发车时车模手动置于 P 区中心，代码假设该点为坐标原点。

## 仓库现状

- `README.md`、`CHANGLOG.md` 当前为空。
- 尚无构建文件（如 `CMakeLists.txt`、`package.xml`、`pyproject.toml`、`Makefile`）。
- 无 Cursor 规则（`.cursorrules`、`.cursor/rules/`）或 Copilot 指令（`.github/copilot-instructions.md`）。
- 已有一份环境审查报告，记录 RDK 实际环境与方案的匹配度：见 `docs/review/rdk-environment-review.md`。

待源码包加入后，应补充具体构建、测试、运行命令。

## 部署连接

RDK X5 当前为有线连接，可通过 `ssh root@192.168.128.10` 免密登录。详细连接方式、环境入口、已配置包与启动命令见 `docs/deployment/rdk-environment-setup.md`。

## 重要环境事实

- RDK 上 ROS2 环境入口为 `/opt/tros/humble/setup.bash`，不是 `/opt/tros/setup.bash`。
- OriginCar 官方工作空间位于 `/userdata/dev_ws`，自研/第三方包建议放在 `/root/ros2_ws`。
- 硬件已确认：**YDLIDAR Tmini Plus**（`/dev/ttyUSB0`，230400）、**OriginCar 底盘**（`/dev/ttyACM0`，115200）。
- 已修复 `origincar_bringup.launch.py` 中 `static_transform_publisher` 的 Humble 兼容性问题，启动后 TF 树完整。
- 已验证：`/odom_combined`（~20 Hz）、`/imu/data`（~20 Hz）、`/scan`（~10 Hz）均正常输出。
- Nav2、zbar-ros、`obstacle_detector_2`、YDLIDAR 驱动已可用；**端侧 VLM 模型**与**VFH 备用避障**尚未部署；**实车运动测试**尚未进行。

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
