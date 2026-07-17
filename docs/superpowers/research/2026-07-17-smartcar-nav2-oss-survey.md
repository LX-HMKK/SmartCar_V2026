# smartcar_nav2 OSS 调研报告

**日期**：2026-07-17  
**调研范围**：OriginCar / RDK X5 / ROS2 Humble 平台下可复用的 Nav2 配置  
**结论**：官方与同届参考仓库均未提供直接可用的 OriginCar+RDK X5 Nav2 参数，需以 Nav2 官方 bringup + OriginBot 社区配置为模板，针对纯惯导、LiDAR--only 避障、阿克曼底盘自行设计。

---

## 1. 调研目标

为 `smartcar_nav2` 模块的设计 spec 收集可复用的开源参考，重点回答：

1. 全局/局部规划器在 OriginCar 阿克曼底盘上的适用性。
2. 无 SLAM、纯惯导条件下代价地图层的合理配置。
3. Waypoint Follower 的任务执行器与行为树选择。
4. 速度、频率、odom/TF 对接等关键参数的行业参考值。

---

## 2. 参考源与结论

### 2.1 Nav2 官方 bringup（ros-navigation/navigation2 humble）

- **Nav2 版本**：Humble
- **全局规划器**：`nav2_navfn_planner/NavfnPlanner`
- **局部控制器**：`dwb_core::DWBLocalPlanner`（默认，但阿克曼底盘不适用）
- **代价地图层**：
  - global：`static_layer + obstacle_layer + inflation_layer`
  - local：`voxel_layer + inflation_layer`
- **Waypoint Follower**：`nav2_waypoint_follower::WaitAtWaypoint`，loop_rate=20 Hz，stop_on_failure=false
- **行为树**：`navigate_to_pose_w_replanning_and_recovery.xml`（RoundRobin：ClearCostmap → Spin → Wait → BackUp）
- **控制器频率**：20 Hz
- **速度限制**：max_vel_x=0.26 m/s，max_vel_theta=1.0 rad/s
- **odom 话题**：`/odom`

**对当前项目的意义**：Nav2 默认参数是最佳起点，但需将局部控制器从 DWB 改为 Regulated Pure Pursuit 以适配阿克曼底盘；代价地图去掉 static_layer（无 SLAM）。

### 2.2 OriginBot 官方（guyuehome/originbot）

- **是否用 Nav2**：✅
- **全局规划器**：`nav2_navfn_planner/NavfnPlanner`
- **局部控制器**：`dwb_core::DWBLocalPlanner`
- **代价地图层**：
  - global：`StaticLayer + ObstacleLayer + InflationLayer`
  - local：`StaticLayer + ObstacleLayer + VoxelLayer + InflationLayer`
- **Waypoint Follower**：`WaitAtWaypoint`
- **定位**：Cartographer + AMCL（依赖 SLAM，与本项目约束冲突）
- **控制器频率**：10 Hz
- **速度限制**：max_vel_x=0.22 m/s，max_vel_theta=0.8 rad/s
- **odom 话题**：`/odom`

**对当前项目的意义**：OriginBot 的 Nav2 包装方式（`nav_bringup.launch.py` 包含 `nav2_bringup/bringup_launch.py`）可直接复用；但需去掉 AMCL/SLAM，改用纯惯导。

### 2.3 DoveSmart（Konieg，同济大学第 19 届智慧医疗赛）

- **是否用 Nav2**：❌
- **导航方式**：深度/传统视觉巡线 + YOLO 避障 + QR 触发模式切换
- **底盘接口**：`geometry_msgs/Twist` 与 `ackermann_msgs/AckermannDriveStamped` 均支持
- **EKF**：`robot_localization` 20 Hz，融合 `/odom` + `/imu/data_raw`，输出 `/odom_combined`
- **静态 TF**：`base_footprint → base_link` x=0.41, y=0.12
- **速度参考**：巡线 0.25 m/s，YOLO 避障 0.30 m/s
- **DDS**：CycloneDDS，ROS_DOMAIN_ID=16

**对当前项目的意义**：确认了本仓库 `base_to_link` 偏移与 `/odom_combined` 的来源；提供了速度上限参考（0.25–0.30 m/s）。

### 2.4 HUAT-IAE-OriginCar/OriginCar_X5

- **是否用 Nav2**：❌
- **导航方式**：自定义 Python 状态机 + 视觉巡线 + 硬编码避障动作
- **速度参考**：stage 1 巡航 1.15 m/s，stage 3 0.8 m/s
- **odom/TF**：`/odom_combined`，`base_footprint → base_link` x=0.41, y=0.12
- **底盘参数**：wheel_base=0.15 m，max_steering_angle=0.6 rad，robot_width=0.25 m，robot_length=0.35 m

**对当前项目的意义**：同平台底盘几何参数参考；再次确认 `/odom_combined` 与 base_to_link 偏移。

### 2.5 HorizonRDK 官方

- **是否用 Nav2**：❌（无 OriginCar 专用包）
- **导航相关包**：`line_follower`、`parking_search`、`parking_perception`、`hobot_vio`、`orb_slam3`
- **D-Robotics 文档**：仅演示标准 `ros-humble-navigation2` + `nav2_bringup` 安装与默认 launch
- **hobot_nav2**：未在 GitHub 开源，可能以闭源 apt/NodeHub 形式存在

**对当前项目的意义**：无法直接复用，必须以 Nav2 官方+OriginBot 社区为主自行设计。

---

## 3. 关键决策维度与推荐

| 维度 | 可选方案 | 默认推荐 | 理由 |
|---|---|---|---|
| 全局规划器 | Navfn / SmacPlanner2D / SmacPlannerHybrid | **NavfnPlanner** | 赛道为固定粗粒度航点序列，Navfn 足够且成熟；SmacHybrid 阿克曼-aware 但调参负担重 |
| 局部控制器 | RPP / DWB / RPP+RotationShim | **RegulatedPurePursuitController** | DWB 不适合阿克曼（会发 y/theta 速度）；RPP 输出曲率指令，支持最小转弯半径与轴距 |
| 代价地图层 | static/obstacle/inflation 组合 | **global: obstacle+inflation；local: obstacle+inflation** | 无 SLAM 不用 static_layer；LiDAR-only 用 obstacle_layer 订阅 /scan |
| 控制器频率/速度 | 10–20 Hz / 0.22–0.30 m/s | **20 Hz / desired 0.25 / max 0.30 m/s** | 与 EKF 20 Hz 匹配； competitors 速度参考 0.25–0.30 m/s |
| Waypoint Follower | WaitAtWaypoint / PhotoAtWaypoint / 自定义 | **WaitAtWaypoint** | 默认+OriginBot 均使用；可在任务点停车执行 QR/VLM |
| 行为树 | 默认 / through_poses / 自定义 | **navigate_to_pose_w_replanning_and_recovery.xml** | 已验证；缩短恢复超时避免赛道耗时 |
| odom 话题/坐标系 | /odom / /odometry/filtered / /odom_combined | **/odom_combined + odom_combined** | 与当前 bringup/EKF 实际一致 |

---

## 4. 主要风险

1. **纯惯导漂移**：C 环道与返回段累积误差大，需较大的航点容差或频繁重规划。
2. **阿克曼转弯限制**：最小转弯半径与舵机最大角可能导致急弯或终点姿态对齐失败。
3. **无持久地图**：global costmap 只能记住当前视野内的障碍物，返回路径风险更高。
4. **RPP 大航向偏差**：必要时叠加 `RotationShimController`，但原地旋转会加剧轮式里程计漂移。
5. **RDK X5 算力**：costmap 分辨率/频率过高会抢占控制器 CPU，导致指令抖动。
6. **LiDAR 感知盲区**：低于扫描平面、玻璃、细杆等障碍物需 VFH 反应式避障作为安全备份。

---

## 5. 待用户确认的 7 个问题

1. **全局规划器**：是否采用 `nav2_navfn_planner/NavfnPlanner`？
2. **局部控制器**：是否采用 `RegulatedPurePursuitController`，大航向偏差时叠加 `RotationShimController`？
3. **代价地图层**：是否采用 global/local 均为 `obstacle_layer + inflation_layer`？
4. **控制器频率/速度**：是否采用 20 Hz、desired 0.25 m/s、max 0.30 m/s？
5. **Waypoint Follower**：是否采用 `WaitAtWaypoint`，pause 3–5 s？
6. **行为树**：是否采用默认行为树并缩短恢复超时？
7. **odom 对接**：是否统一使用 `/odom_combined` + `odom_combined`？

**默认决策**：全部接受上述默认推荐，进入 spec 撰写。
