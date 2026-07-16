# 智慧医疗赛智能车

第二十一届全国大学生智能汽车竞赛 · 地瓜机器人智慧医疗赛参赛工程。

## 平台

- **车载计算**：RDK X5 8G（地瓜机器人），TROS = ROS2 Humble，环境入口 `/opt/tros/humble/setup.bash`
- **底盘**：OriginCar 阿克曼（舵机转角 + 电机速度），STM32 编码器里程计 + IMU，`robot_localization` EKF 融合
- **激光**：YDLIDAR TMini Plus（`/dev/ttyUSB0`，230400）—— **仅避障，不参与定位**
- **开发机**：Windows，经 WSL rsync 同步工作空间至 RDK

> 任务分解与系统架构详见 [`docs/智慧医疗挑战赛_技术方案_本地部署版.md`](docs/智慧医疗挑战赛_技术方案_本地部署版.md)。

## 仓库结构

```
src/
  origincar/                 # 官方包（vendor 化，.git 已剥离）
    origincar_base/          # 底盘驱动 + bringup（已修复 Humble 兼容）
    origincar_bringup/       # USB/WebSocket 显示
    origincar_description/   # URDF + 网格
    origincar_msg/           # 自定义消息
    utils/                   # 图像传输工具
    ydlidar_ros2_driver/     # 激光驱动
    3rdparty/                # ackermann_msgs、serial_ros2、aurora930
  third_party/
    obstacle_detector_2/     # 锥桶 LiDAR 检测
config/                      # 运行参数（随模块实现填充）
scripts/
  sync_to_rdk.py             # 本机 <-> RDK 同步（push/pull/init-vendor/setup）
  source_env.sh              # RDK 工作空间环境入口
tests/                       # sync_to_rdk.py 单元测试（stdlib unittest）
docs/                        # 技术方案、部署、评审文档
```

## 开发机准备（Windows）

1. 启用 WSL Ubuntu-22.04，并配置到 RDK 的免密 ssh key（与 Windows 侧同密钥即可）。
   choco 的 rsync 无法处理盘符路径（`D:` 被当作 `host:path`），故同步经 WSL 运行。
2. 免密 ssh 到 `root@192.168.128.10`（RDK 有线 IP）。
3. 单元测试：
   ```powershell
   python -m unittest tests.test_sync_to_rdk
   ```

## 首次部署

```powershell
python scripts/sync_to_rdk.py init-vendor   # 回传官方/第三方包入本机 VCS
python scripts/sync_to_rdk.py setup         # 部署 source_env.sh 到 RDK ~/
python scripts/sync_to_rdk.py push          # 镜像 src/+config/ 到 RDK（--delete）
```

## RDK 上构建与运行

```bash
ssh root@192.168.128.10
source ~/source_env.sh          # /opt/tros/humble + /root/ros2_ws/install
cd /root/ros2_ws
colcon build --symlink-install
```

## 日常同步

```powershell
python scripts/sync_to_rdk.py push           # 本机 -> RDK（镜像，--delete）
python scripts/sync_to_rdk.py push --dry-run # 预览
python scripts/sync_to_rdk.py pull           # RDK -> 本机（默认不删本地，--delete 显式开启）
```

## 状态

- ✅ 工作空间基础设施：单一 `/root/ros2_ws`，vendor 化官方包，本机↔RDK 双向同步
- ✅ 单一工作空间构建：9 包 `colcon build --symlink-install` 从零通过
- ✅ origincar bringup 回归：TF 树完整（`odom_combined→base_footprint→base_link→laser`），`/odom_combined` ~20Hz、`/imu/data` ~20Hz、`/scan` ~10Hz（LiDAR 需单独 `ros2 launch ydlidar_ros2_driver ydlidar_launch.py`）
- ⏳ 待实现模块：`smartcar_bringup`、`smartcar_nav2`、`smartcar_vision`、`smartcar_task`
- ⏳ 待部署：端侧 VLM 图生文、VFH 备用避障
- ⏳ 实车运动测试尚未进行

更多见 [`docs/`](docs/) 与 [`CLAUDE.md`](CLAUDE.md)。
