# RDK X5 部署与运行手册

更新时间：2026-07-18

目标板：`root@192.168.128.10`

工作空间：`/root/ros2_ws`

## 1. 环境与连接

RDK 的 ROS 2 环境入口是 `/opt/tros/humble/setup.bash`。仓库提供的 `~/source_env.sh` 会加载 TROS 和 `/root/ros2_ws/install`，日常只使用这一入口：

```bash
ssh root@192.168.128.10
source ~/source_env.sh
```

不要再叠加 source `/userdata/dev_ws/install/setup.bash`。官方源码已经 vendor 化到本仓库，并统一在 `/root/ros2_ws` 构建，重复 overlay 可能加载旧包。

已确认硬件：

| 设备 | 接口 | 已验证状态 |
|---|---|---|
| OriginCar 底盘 | `/dev/ttyACM0`，115200 | 原始 `/odom`、`/imu/data_raw` 可用 |
| YDLIDAR Tmini Plus | `/dev/ttyUSB0`，230400 | `/scan` 约 10 Hz |
| Aurora/USB/MIPI 相机 | 驱动已安装 | 实体相机不得在无硬件 smoke 中启动 |

## 2. 同步、构建与测试

Windows PowerShell：

```powershell
python -m unittest discover -s tests -v
python scripts/sync_to_rdk.py push --dry-run
python scripts/sync_to_rdk.py push
```

RDK：

```bash
source ~/source_env.sh
cd /root/ros2_ws
colcon build --symlink-install
```

清除历史 xunit 后运行完整工作区测试：

```bash
colcon test-result --delete-yes
colcon test
colcon test-result --all --verbose
```

2026-07-18 验证结果：15 个包构建通过，`506 tests, 0 errors, 0 failures, 90 skipped`。三个 vendor-only 包的继承源码全量 lint 默认为关闭，避免旧版权和格式问题污染功能测试；显式检查使用：

```bash
colcon build --cmake-args -DSMARTCAR_ENABLE_VENDOR_LINT=ON
colcon test --packages-select \
  origincar_bringup origincar_description ydlidar_ros2_driver
```

## 3. 无硬件完整 smoke

首选方式是运行 `smartcar_bringup` 的 launch test。测试自动执行以下安全顺序：

1. 使用独立 ROS 域和 localhost-only 通信。
2. 关闭底盘、LiDAR、障碍物驱动和实体相机。
3. 启动合成 `/scan`、`/odom`、`/odom_combined` 与静态 TF。
4. 在 Nav2 lifecycle 激活前锁存急停。
5. 验证七个 Nav2 节点 active、vision/task 服务存在、任务为 `IDLE`。
6. 连续验证 `/cmd_vel_safe` 六个字段精确为零。
7. 验证底盘、EKF、IMU、YDLIDAR、robot_state_publisher 和相机节点未启动。

```bash
source ~/source_env.sh
cd /root/ros2_ws
colcon test --packages-select smartcar_bringup
colcon test-result --test-result-base build/smartcar_bringup --verbose
```

该测试不发布非零速度。安全门压制非零命令的逻辑由 `smartcar_safety` 单元测试覆盖。

仅查看视觉和任务服务时可手工启动最小 bench：

```bash
export ROS_DOMAIN_ID=221
export ROS_LOCALHOST_ONLY=1

ros2 launch smartcar_bringup smartcar_system.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_obstacle:=false \
  use_safety:=true \
  use_nav:=false \
  nav_autostart:=false \
  use_camera:=false \
  image_topic:=/smartcar/vision/image \
  use_vision:=true \
  use_task:=true \
  autostart_mission:=false
```

## 4. 系统启动

完整栈入口：

```bash
source ~/source_env.sh
ros2 launch smartcar_bringup smartcar_system.launch.py \
  autostart_mission:=false
```

默认开关：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `use_base` | `true` | 底盘、IMU、EKF、URDF/TF |
| `use_lidar` | `true` | YDLIDAR `/scan` |
| `use_obstacle` | `true` | obstacle detector |
| `use_safety` | `true` | `/cmd_vel` 到 `/cmd_vel_safe` 安全门 |
| `use_nav` | `true` | Nav2 |
| `nav_autostart` | `true` | 自动激活 Nav2 lifecycle |
| `use_camera` | `true` | 实体相机驱动 |
| `use_vision` | `true` | QR/VLM 服务与 zbar |
| `use_task` | `true` | 任务状态机 |
| `autostart_mission` | `false` | 不自动开始任务 |

`smartcar_system.launch.py` 禁止 `use_base=true,use_safety=false`。底层 `smartcar_bringup.launch.py` 的 `use_safety=false` 只保留给受控调试，不得用于竞赛运行。

## 5. 校准数据与运动门禁

以下值均需实测：

- `base_footprint -> base_link`：`base_x/base_y/base_z/base_roll/base_pitch/base_yaw`
- `base_link -> laser`：`laser_x/laser_y/laser_z/laser_roll/laser_pitch/laser_yaw`
- `base_link -> camera`：`camera_x/camera_y/camera_z/camera_roll/camera_pitch/camera_yaw`
- 航点文件：`src/smartcar_nav2/config/waypoints/default_waypoints.yaml`
- 转向、轮速、陀螺仪参数：`origincar_base` launch 参数

`src/smartcar_bringup/config/bringup_coord.yaml` 当前只是协调和审计记录，不会自动写入 launch。运行时必须通过 launch 参数或后续实测参数文件提供真实值。

五个门禁默认均为 `false`：

```text
waypoints_calibrated
extrinsics_calibrated
steering_calibrated
emergency_stop_ready
operator_approved
```

只有所有实测完成后，才可显式传入 `true`。`autostart_mission=true` 还要求 `use_base/use_safety/use_nav/use_vision/use_task/nav_autostart=true`。

## 6. 任务、停止和复位

```bash
ros2 service call /smartcar/task/start std_srvs/srv/Trigger "{}"
ros2 service call /smartcar/task/stop std_srvs/srv/Trigger "{}"
ros2 service call /smartcar/task/reset std_srvs/srv/Trigger "{}"
```

任务状态：`IDLE`、`WAITING_FOR_SERVERS`、`NAVIGATING`、`RUNNING_QR`、`RUNNING_VLM`、`COMPLETED`、`STOPPED`、`FAILED`。

`reset` 的前置条件和顺序：

1. 导航 worker 与 FollowWaypoints action 已确认停止。
2. 车辆物理位于 P 区原点，车头朝 `+X`。
3. 调用 robot_localization `/set_pose`。
4. 等待更新且有限的 `/odom_combined` 接近原点。
5. 调用 `/smartcar/safety/clear_localization_fault`。

不要直接调用 clear service 绕过 task reset。

## 7. 急停与故障恢复

锁存急停：

```bash
ros2 service call /smartcar/safety/emergency_stop \
  std_srvs/srv/SetBool "{data: true}"
```

解除急停前必须确认车辆周围安全、导航目标已取消且 `/cmd_vel_safe` 为零：

```bash
ros2 service call /smartcar/safety/emergency_stop \
  std_srvs/srv/SetBool "{data: false}"
```

`/smartcar/task/stop` 不等于急停。串口断线、无新鲜 `/scan`、无新鲜原始/融合里程计、非法速度或定位故障都会让安全门 fail-closed。底盘进程关闭时会复用当前串口发送一次零命令；仍必须准备人工可触达的物理急停。

## 8. 视觉与端侧 VLM

相机选择：`camera_driver:=aurora|usb|mipi`。默认 Aurora 只启用 15 FPS RGB，关闭 depth/IR/point cloud。

`vision.yaml` 默认 `vlm_backend_mode: disabled`，因此未部署模型时 DescribeScene 返回兜底文案。部署本地模型后使用 `command` 模式和 argv 列表，不经过 shell；整个请求包含图像等待、JPEG 编码和后端推理，硬上限为 8 秒。

VFH 备用避障和 YOLO 自动触发未纳入当前 release；人物描述由语义航点 `task: vlm` 调用。

## 9. 录包与检查

```bash
mkdir -p /root/ros2_ws/bags
ros2 bag record -o /root/ros2_ws/bags/$(date +%Y%m%d-%H%M%S) \
  /scan /odom /odom_combined /imu/data_raw /imu/data \
  /cmd_vel /cmd_vel_safe /ackermann_cmd /PowerVoltage \
  /tf /tf_static /smartcar/safety/status /smartcar/task/state \
  /barcode /smartcar/output/text /smartcar/output/speech
```

常用检查：

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /odom_combined
ros2 topic echo /smartcar/safety/status
ros2 topic echo /smartcar/task/state
ros2 service list | grep /smartcar
ros2 action list | grep follow_waypoints
```

## 10. 物理测试顺序

1. 自动化测试和无硬件 smoke 全部通过。
2. 测量航点、外参和转向参数。
3. 人工急停可用，车轮离地测试。
4. 低速直线、低速转向、急停、串口断线分别验证。
5. 最后才进行地面导航和任务联调。

在上述步骤完成前，软件状态只能称为“代码合同完成、待物理标定与实车验证”。
