# RDK X5 环境配置状态

> 更新时间：2026-07-15  
> 目标板：`root@192.168.128.10`（有线免密 SSH）

## 环境入口

RDK 上实际 ROS2 环境路径为 `/opt/tros/humble/setup.bash`，不是 `/opt/tros/setup.bash`。官方 OriginCar 工作空间位于 `/userdata/dev_ws`。

常用 source 顺序：

```bash
source /opt/tros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
source /root/ros2_ws/install/setup.bash   # obstacle_detector 等第三方包
```

## 已确认可用的硬件

| 设备 | 接口/参数 | 状态 |
|---|---|---|
| YDLIDAR Tmini Plus | `/dev/ttyUSB0`，波特率 230400 | ✅ 已验证，`/scan` 约 10 Hz |
| OriginCar 底盘 | `/dev/ttyACM0`，波特率 115200 | ✅ 已启动，`/odom`、`/odom_combined` 正常 |
| MIPI 相机 | `/dev/video*` | ✅ 驱动已装 |
| IMU | `gyro_link` | ✅ 已验证，`/imu/data` 约 20 Hz |

## 已安装/编译的软件

### apt 安装

- `ros-humble-nav2-bringup`
- `ros-humble-nav2-waypoint-follower`
- `ros-humble-nav2-regulated-pure-pursuit-controller`
- `ros-humble-nav2-dwb-controller`
- `ros-humble-nav2-costmap-2d`
- `ros-humble-nav2-rviz-plugins`
- `ros-humble-zbar-ros`
- `python3-colcon-common-extensions`
- `python3-rosdep`
- `ros-humble-robot-localization`（此前已装）

### 源码编译

- `/root/ros2_ws/src/obstacle_detector_2` → 编译为 `obstacle_detector`
- rosdep 已通过（已修正 `nodelet`/`roslaunch` → 删除，`rviz` → `rviz2`）

### OriginCar 官方工作空间

- `/userdata/dev_ws/src/origincar/` 包含：
  - `origincar_base`：底盘节点、launch、EKF/Madgwick 配置
  - `origincar_bringup`：整车启动 launch
  - `origincar_description`：URDF 与显示
  - `origincar_msg`：自定义消息
  - `ydlidar_ros2_driver`：YDLIDAR 驱动
  - `YDLidar-SDK-master`

## 关键启动命令

```bash
# 1. 启动整车基础链路（底盘 + IMU + EKF + TF + LiDAR）
source /opt/tros/humble/setup.bash
source /userdata/dev_ws/install/setup.bash
ros2 launch origincar_base origincar_bringup.launch.py

# 2. 仅启动 LiDAR
ros2 launch ydlidar_ros2_driver ydlidar_launch.py

# 3. 仅启动底盘
ros2 launch origincar_base base_serial.launch.py

# 4. 启动 obstacle_detector（订阅 /scan 发布 /raw_obstacles）
source /opt/tros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 run obstacle_detector obstacle_extractor_node
```

## 已验证的 TF 树

启动 `origincar_bringup.launch.py` 后，TF 树为：

```
odom_combined → base_footprint
base_footprint → base_link
base_footprint → gyro_link
base_link → laser
base_link → board_link
base_link → camera
base_link → up_left_Link / up_right_Link / down_left_Link / down_right_Link
```

> 修复记录：原 launch 中 `static_transform_publisher` 使用旧式空格参数，在 Humble 下解析失败。已改为显式 `--x/--y/--z/--roll/--pitch/--yaw/--frame-id/--child-frame-id` 命名参数，并重新编译 `origincar_base`。

## 已验证的话题

| 话题 | 类型 | 频率 | 状态 |
|---|---|---|---|
| `/odom_combined` | `nav_msgs/Odometry` | ~20 Hz | ✅ 正常 |
| `/imu/data` | `sensor_msgs/Imu` | ~20 Hz | ✅ 正常 |
| `/scan` | `sensor_msgs/LaserScan` | ~10 Hz | ✅ 正常 |
| `/odom` | `nav_msgs/Odometry` | 由底盘发布 | ✅ 正常 |
| `/cmd_vel` | `geometry_msgs/Twist` | 外部输入 | ✅ 话题存在 |

## 仍待补齐

| 项目 | 状态 | 说明 |
|---|---|---|
| VFH 备用避障 | ⏸️ | `obstacle-avoider-ros2` 因 RDK 无法访问 GitHub 未获取；非关键路径，可后续补入 |
| 端侧 VLM 模型文件 | ⏸️ | 未下载/转换 Gemma-3-4B-IT-QAT / Qwen2-VL；需单独部署并实测内存 |
| Nav2 与底盘/雷达联调 | ⏸️ | 各组件单独可用，尚未做端到端导航/避障联调 |
| 实车运动测试 | ⏸️ | 已验证 `/odom_combined` 静态输出，尚未发布 `/cmd_vel` 让车实际移动 |

## 注意事项

- RDK 实际内存为 6.9 GiB（非 8 GiB），无 Swap，运行 4B VLM 需实测内存余量。
- RDK 当前无法访问 GitHub（443 超时），后续第三方包建议通过本地 PC 下载后 `scp` 上传。
- `obstacle_detector_2` 原 `package.xml` 含 ROS1 依赖 `nodelet`/`roslaunch`，已清理；`rviz` 已改为 `rviz2`。
- `origincar_bringup.launch.py` 中的 `static_transform_publisher` 已修复为 Humble 兼容写法。
- EKF 配置：`odom_frame`/`world_frame` = `odom_combined`，`base_link_frame` = `base_footprint`，`two_d_mode: true`。
- IMU Madgwick：`world_frame: enu`，`use_mag: false`，`publish_tf: false`。
