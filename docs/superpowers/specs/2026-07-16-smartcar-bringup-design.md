# smartcar_bringup 模块设计 spec

> 日期：2026-07-16 ｜ 分支：feat/smartcar-bringup ｜ 状态：待评审
>
> 依据：技术方案 §3 架构、§6 RViz；RDK 实测（bringup 回归 2026-07-16）；OSS 调研（OriginBot/HorizonRDK/DoveSmart/HUAT）；记忆 [[rdk-bringup-verified]]、[[odom-topic-discrepancy]]、[[reuse-oss-per-module]]。

## 0. 决策摘要

| 决策项 | 取值 | 理由 |
|---|---|---|
| odom 话题/帧 | `/odom_combined` + `odom_combined` 帧 | 零改动复用已验证 ~20Hz 官方 EKF yaml；方案文档侧统一到实际值 |
| ydlidar | 纳入统一启动，`use_lidar` 开关 | 保证 `/scan` 与 `base_link->laser` TF 启动一致性 |
| obstacle_detector | 纳入统一启动，重写 launch，`use_obstacle` 开关 | 现有 launch 为 ROS1 式不可用且参数全错，须重写 |
| config 层级 | per-package share | OSS 主流，包隔离，`install(DIRECTORY config)` |
| 仿真 launch | 不做 | 实车竞赛，仿真不在本期范围 |
| 开发位置 | 本地 `src/` -> WSL rsync -> RDK `/root/ros2_ws` | 复用已就绪同步基础设施 |
| base_to_link 偏移 | 暂保留官方值 x=0.41,y=0.12，**赛前实车标定复核 TODO** | 疑为 LiDAR/前轴位置错挂，影响定位/避障坐标 |
| 官方 origincar_bringup 空壳包 | 保留不动 | smartcar_bringup 为新建独立包，避免改动 vendor |

## 1. 模块定位与职责边界

`smartcar_bringup` 是**顶层 launch 组合器包**（学 OriginBot `originbot.launch.py` 模式）：通过 `IncludeLaunchDescription` 复用已 vendor 化的 `origincar_base`（底盘+IMU+EKF+静态TF+URDF）、`ydlidar_ros2_driver`（LiDAR）、`obstacle_detector`（锥桶检测），统一启动"定位层（IMU+轮式里程计+EKF 纯惯导无 SLAM）+ 控制层（阿克曼底盘）+ 感知硬件驱动（LiDAR 仅避障）"。自身**不启任何 Node，只做组合与功能开关**。

**应做**：
- 提供顶层 `smartcar_bringup.launch.py` 统一入口。
- 功能开关 `use_lidar`/`use_obstacle`/`use_nav`（`DeclareLaunchArgument` + `IfCondition`），支持单模块调试。
- 统一命名到 RDK 实际帧/话题（`/odom_combined`、`odom_combined` 帧、`base_footprint` 为 EKF base 帧）。
- 保证 TF 树完整连续，LiDAR 不进定位链（无 `map->odom`，无 SLAM）。
- 输出基础话题 `/odom_combined`/`/imu/data`/`/scan`/`/cmd_vel`/`/robot_description`。
- 参数化端口/帧/话题，默认值用 `get_package_share_directory` 拼接，杜绝硬编码绝对路径。
- 重写 `obstacle_detector` launch 为 Humble 兼容版本。
- 提供 `source_env.sh` 便捷脚本（已部署，spec 记录 source 顺序）。

**不应做**：
- 不重造底盘/EKF/驱动 launch（直接 include vendor 包）。
- 不做 Nav2 配置（waypoints/costmap/Regulated Pure Pursuit 归 `smartcar_nav2`）。
- 不做任务决策逻辑（状态机/QR/VLM 归 `smartcar_task`）。
- 不做避障算法调优（obstacle_detector 参数调优/VFH 归避障感知层；bringup 仅启动节点）。
- 不做 IMU/里程计/转向角标定（赛前离线工具，bringup 只消费标定结果）。
- 不做航点预设与区域限速（归导航/任务层）。
- 不引入 `map->odom` TF 或任何 SLAM。

## 2. 包结构与文件布局

```
src/smartcar_bringup/
├── launch/
│   ├── smartcar_bringup.launch.py     # 顶层组合器
│   └── obstacle_detector.launch.py     # 重写的 Humble 兼容 obstacle_detector launch
├── config/
│   └── bringup_coord.yaml              # 协调层 config（帧名约定/开关默认值/端口默认值）
├── CMakeLists.txt                      # install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
└── package.xml                         # exec_depend 见 §3
```

## 3. 复用清单与 include 关系图

| 子系统 | 复用包 | include 方式 | 条件 |
|---|---|---|---|
| 底盘+IMU+EKF+URDF+静态TF | `origincar_base` (`origincar_bringup.launch.py`) | 直接 `IncludeLaunchDescription` | 无条件（底盘必需） |
| LiDAR 驱动 | `ydlidar_ros2_driver` (`ydlidar_launch.py`, params `ydlidar.yaml` frame_id=laser) | `IncludeLaunchDescription` | `IfCondition(use_lidar)` |
| 锥桶检测 | `obstacle_detector`（包名，目录 `obstacle_detector_2`） | include 本包内重写的 `obstacle_detector.launch.py` | `IfCondition(use_obstacle)` |
| Nav2 | `smartcar_nav2`（未实现） | 预留 `use_nav` 开关，本期不 include | `IfCondition(use_nav)` |
| QR/VLM | `smartcar_task`（未实现） | 不在 bringup | — |

`package.xml` exec_depend：`origincar_base`、`ydlidar_ros2_driver`、`obstacle_detector`、`robot_state_publisher`、`tf2_ros`、`robot_localization`、`imu_filter_madgwick`。

依赖拓扑：
```
smartcar_bringup.launch.py
├── origincar_base/origincar_bringup.launch.py
│   ├── origincar_base_node        (底盘串口 /dev/ttyACM0)
│   ├── imu_filter_madgwick_node   (/imu/data)
│   ├── ekf_node                   (/odom_combined, TF odom_combined->base_footprint)
│   ├── robot_state_publisher      (URDF TF)
│   ├── joint_state_publisher
│   └── 3× static_transform_publisher (base_footprint->base_link, ->gyro_link, base_link->laser)
├── [use_lidar] ydlidar_ros2_driver/ydlidar_launch.py
│   └── ydlidar_ros2_driver_node   (/scan, frame laser)
└── [use_obstacle] smartcar_bringup/obstacle_detector.launch.py
    └── obstacle_extractor_node    (消费 /scan, frame odom_combined)
```

## 4. 顶层 launch 组合设计

`smartcar_bringup.launch.py` 自身不启 Node，仅声明参数 + 条件 include：

`DeclareLaunchArgument` 清单：
| 参数 | 默认值 | 说明 |
|---|---|---|
| `use_lidar` | `true` | 启动 LiDAR 驱动 |
| `use_obstacle` | `true` | 启动 obstacle_detector |
| `use_nav` | `false` | 预留：启动 Nav2（本期不实现） |
| `use_sim_time` | `false` | 实车不用仿真时间 |
| `serial_port` | `/dev/ttyACM0` | 底盘串口 |
| `lidar_port` | `/dev/ttyUSB0` | LiDAR 串口 |
| `lidar_baudrate` | `230400` | LiDAR 波特率 |
| `odom_frame` | `odom_combined` | 全局里程计帧 |
| `base_frame` | `base_footprint` | EKF base 帧 |
| `laser_frame` | `laser` | LiDAR 帧（对齐 bringup 静态 TF） |

- 用 `IfCondition(LaunchConfiguration('use_lidar'))` 等条件启动。
- `IncludeLaunchDescription` 透传所需 `LaunchConfiguration`。
- 默认值用 `get_package_share_directory('smartcar_bringup')` 拼接 config 路径，**不写死绝对路径**（反 DoveSmart 模式）。
- 返回 `LaunchDescription([...])`，保证声明与返回一致（反 DoveSmart return 列表不符模式）。

## 5. 话题规范与命名统一

| 话题 | 类型 | 频率 | 来源 | 用途 |
|---|---|---|---|---|
| `/odom_combined` | `nav_msgs/Odometry` | ~20Hz | EKF | Nav2 定位输入 |
| `/imu/data` | `sensor_msgs/Imu` | ~20Hz | imu_filter_madgwick | EKF 输入 |
| `/scan` | `sensor_msgs/LaserScan` | ~10Hz | ydlidar | 避障/Nav2 costmap |
| `/cmd_vel` | `geometry_msgs/Twist` | — | Nav2 发布，底盘订阅 | 运动控制 |
| `/robot_description` | `std_msgs/String` | latched | robot_state_publisher | URDF |

**命名统一对照**（三套历史命名 -> 统一到 RDK 实际）：
| 来源 | 旧话题 | 旧帧 | 统一后 |
|---|---|---|---|
| 技术方案/RViz | `/odometry/filtered` | `odom` | `/odom_combined` / `odom_combined` |
| RDK EKF 实际 | `/odom_combined` | `odom_combined` | 保持 |
| obstacle_detector 原 launch | `/odometry/filtered` | `map` | `/odom_combined` / `odom_combined`（重写 launch 时改） |

obstacle_detector 侧 remap：消费 `/scan`（非 `/merged_scan`），输出障碍物帧用 `odom_combined`。

## 6. TF 树规范

完整链路：
```
odom_combined -> base_footprint -> base_link -> {laser, camera, board_link, 四轮}
                              └-> gyro_link
```

| TF 边 | 发布者 | 类型 |
|---|---|---|
| `odom_combined -> base_footprint` | EKF (`robot_localization`) | 动态 |
| `base_footprint -> base_link` | `static_transform_publisher` (x=0.41,y=0.12,**待标定**) | 静态 |
| `base_footprint -> gyro_link` | `static_transform_publisher` | 静态 |
| `base_link -> laser` | `static_transform_publisher` | 静态 |
| `base_link -> {camera,board_link,四轮}` | `robot_state_publisher` (URDF) | 静态/joint |

约束：
- LiDAR 仅作避障输入，**绝不进入定位 TF**；无 `map->odom`，无 SLAM。
- 帧名统一 `laser`（非 `laser_frame`）；不使用 `ydlidar_launch_view.py`。
- **TF 去重**：实现时核对 `origincar.urdf` 是否已定义 `base_footprint->base_link` 或 `base_link->laser`；若已定义，则与 `origincar_bringup` 内 3 个 `static_transform_publisher` 重复，会产生 TF 警告。处理路径：在 smartcar_bringup 实现阶段验证，若重复则在 vendor 的 `origincar_bringup.launch.py` 中注释冗余 static TF（作为 vendor 补丁，记录于 spec §11），目标 TF 重复发布零警告。

## 7. 配置文件管理

- per-package share：各包 `CMakeLists.txt` `install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})`。
- `smartcar_bringup/config/bringup_coord.yaml`：仅放**协调层** config —— 全局帧名约定、话题重映射表、功能开关默认值、设备端口默认值。
- 各功能包自管自身 config：`ekf.yaml`/`imu.yaml` 在 `origincar_base`，`ydlidar.yaml` 在 `ydlidar_ros2_driver`。smartcar_bringup 不接管这些。
- smartcar_bringup 不修改 vendor 包的 config（除 §6/§11 明确的 vendor 补丁）。

## 8. 参数化设计

见 §4 参数清单。所有可配项经 `DeclareLaunchArgument` 暴露，默认值来源：
- 硬件事实：`/dev/ttyACM0`（底盘）、`/dev/ttyUSB0`（LiDAR）、`230400`（LiDAR 波特率）—— 见 CLAUDE.md。
- 帧名：`odom_combined`/`base_footprint`/`laser` —— 见 §5/§6。
- config 路径：`get_package_share_directory` 拼接。
- 杜绝硬编码 `/root/...` 绝对路径（反 DoveSmart 模式）。

## 9. 启动模式与便捷脚本

```bash
# 全量（底盘+LiDAR+obstacle）
ros2 launch smartcar_bringup smartcar_bringup.launch.py

# 仅底盘（调试定位/控制）
ros2 launch smartcar_bringup smartcar_bringup.launch.py use_lidar:=false use_obstacle:=false

# 底盘+LiDAR（调试避障，无 Nav2）
ros2 launch smartcar_bringup smartcar_bringup.launch.py use_obstacle:=false

# 调试模式（预留 use_nav，本期不实现 Nav2）
ros2 launch smartcar_bringup smartcar_bringup.launch.py use_nav:=false
```

`source_env.sh`（已部署到 RDK `~/`）封装 source 顺序：`/opt/tros/humble/setup.bash` -> `/userdata/dev_ws/install/setup.bash` -> `/root/ros2_ws/install/setup.bash`。

## 10. obstacle_detector 集成方案

重写 `smartcar_bringup/launch/obstacle_detector.launch.py`（Humble 兼容），启动 `obstacle_extractor_node`（包名 `obstacle_detector`，executable `obstacle_extractor_node`），参数：
- `frame_id` = `odom_combined`（原 `map`，系统无 map 帧）
- `use_scan` = `true`，`use_pcl` = `false`（系统只有 `/scan`，无 PCL）
- `use_sim_time` = `false`（实车）
- 去除 `/scan -> /merged_scan` remap（系统无 `scans_merger` 产生该话题）
- `IfCondition(use_obstacle)` 条件启动
- ROS1 式 `nodes.launch`/`demo.launch`（`type=`/`nodelet`/`rosparam`）不纳入，标记删除。

包名与目录名不一致：包名 `obstacle_detector`，目录 `obstacle_detector_2`；launch 引用包名 `obstacle_detector`。

## 11. 已知缺口与遗留处理

| 项 | 处理 |
|---|---|
| `origincar_base/launch/ekf.launch.py` 孤儿 | 不引用（bringup 内联自己的 EKF），保留不动 |
| `base_to_link` x=0.41,y=0.12 偏移 | **赛前实车标定复核 TODO**；暂保留官方值 |
| `ydlidar_launch_view.py` laser_frame | 不使用；用 `ydlidar_launch.py` + `ydlidar.yaml`(frame_id=laser) |
| `origincar_base` CMakeLists CATKIN 残留 | 标注；build 已通过，本期不改 |
| TF 重复（URDF vs static_transform_publisher） | 实现阶段验证，若重复则 vendor 补丁注释冗余 static TF |
| 官方 `origincar_bringup` 空壳包 | 保留不动；smartcar_bringup 新建独立包 |

## 12. 部署与构建

- smartcar_bringup 纳入 `/root/ros2_ws` 单一工作空间（与官方 `/userdata/dev_ws` 隔离）。
- 本地 `src/smartcar_bringup/` 开发 -> `python scripts/sync_to_rdk.py push`（WSL rsync） -> RDK `colcon build --symlink-install`。
- `source /root/ros2_ws/install/setup.bash` 后 `ros2 launch smartcar_bringup smartcar_bringup.launch.py`。
- RDK 无法访问 GitHub，新第三方包需 scp 上传（本期无新第三方包）。

## 13. 验收标准

- `/odom_combined` ~20Hz、`/imu/data` ~20Hz、`/scan` ~10Hz。
- `ros2 run tf2_tools view_frames` 生成的 TF 树无断链：`odom_combined -> base_footprint -> base_link -> {laser,...}`。
- `rviz2` RobotModel + Odometry + LaserScan 正常显示。
- `ros2 topic echo /cmd_vel` 可达底盘（手动发 cmd_vel 车动）。
- TF 重复发布零警告（去重后）。
- `use_lidar:=false` 时无 `/scan`；`use_obstacle:=false` 时无 obstacle 节点。
- 单一工作空间 `colcon build --symlink-install` 通过（含 smartcar_bringup）。
