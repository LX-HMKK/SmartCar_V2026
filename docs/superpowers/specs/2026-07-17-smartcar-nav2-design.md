# smartcar_nav2 设计 spec

**日期**：2026-07-17  
**状态**：待用户确认关键决策后定稿  
**对应模块**：`smartcar_nav2`（纯配置型 ROS2 包）  
**前置调研**：[OSS 调研报告](./../research/2026-07-17-smartcar-nav2-oss-survey.md)

---

## 1. 目标

为第二十一届全国大学生智能汽车竞赛-地瓜机器人智慧医疗赛提供**基于 Nav2 的航点导航能力**，使 OriginCar 在纯惯导、LiDAR-only 避障、阿克曼底盘约束下，能够按顺序访问预设航点：

```
P 区 → 任务点 1 → 任务点 2 → 通道口 → C 环道 → 返回 P 区
```

## 2. 约束

- **定位**：纯惯导，无 SLAM/AMCL。`robot_localization` EKF 融合轮式里程计 + IMU，输出 `/odom_combined` + TF `odom_combined→base_footprint`。
- **避障**：LiDAR 仅用于避障；锥桶、人形立牌等不进入 costmap（立牌由视觉+VLM 处理）。
- **底盘**：阿克曼转向，舵机最大角度、最小转弯半径有限。
- **算力**：RDK X5 8G，costmap 分辨率与频率需控制。
- **ROS2 版本**：Humble。

## 3. 高层架构

```text
┌─────────────────────────────────────────────────────────────┐
│                     smartcar_task（状态机）                  │
│         生成航点序列 → 调用 /follow_waypoints action         │
└─────────────────────────┬───────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────┐
│                 nav2_waypoint_follower                       │
│    逐点下发 navigate_to_pose 目标到 bt_navigator             │
└─────────────────────────┬───────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────┐
│  bt_navigator  +  planner_server  +  controller_server       │
│  Navfn(global) + RPP(local) + BehaviorTree(recovery)        │
└─────────────────────────┬───────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────┐
│  global_costmap / local_costmap (obstacle_layer+inflation)  │
│                       订阅 /scan                            │
└─────────────────────────┬───────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────┐
│              smartcar_bringup（已提供）                       │
│   base/ydlidar/obstacle_detector → /scan + /odom_combined   │
└─────────────────────────────────────────────────────────────┘
```

## 4. 关键决策（基于 OSS 调研默认推荐）

| # | 决策项 | 选择 | 备注 |
|---|---|---|---|
| 1 | 全局规划器 | `nav2_navfn_planner/NavfnPlanner` | 固定粗粒度航点序列，A*/Dijkstra 足够 |
| 2 | 局部控制器 | `nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController` | 阿克曼适配，输出曲率指令 |
| 3 | 航向修正 | 预留 `nav2_rotation_shim_controller` 包装位 | 若大航向偏差收敛困难再启用 |
| 4 | 代价地图层 | global/local 均为 `obstacle_layer + inflation_layer` | 无 SLAM，LiDAR-only |
| 5 | 控制器频率 | 20 Hz | 与 EKF 20 Hz 对齐 |
| 6 | 速度上限 | desired 0.25 m/s，max 0.30 m/s | 参考 competitors |
| 7 | Waypoint Follower | `nav2_waypoint_follower::WaitAtWaypoint` | 任务点停车执行 QR/VLM |
| 8 | 行为树 | `navigate_to_pose_w_replanning_and_recovery.xml` | 缩短恢复超时 |
| 9 | odom/TF | `/odom_combined` + `odom_combined` | 与 bringup 一致 |

## 5. 包结构

```text
src/smartcar_nav2/
├── package.xml
├── CMakeLists.txt
├── config/
│   ├── nav2_params.yaml          # Nav2 核心参数
│   ├── waypoints/
│   │   └── default_waypoints.yaml # 默认航点序列（P → 任务点 → C → P）
│   └── behavior_trees/
│       └── navigate_to_pose_w_replanning_and_recovery.xml  # 恢复超时缩短版
├── launch/
│   ├── nav2_bringup.launch.py    # 启动 Nav2 + costmap + controller + planner
│   ├── waypoint_follower.launch.py  # 启动 waypoint_follower（可选独立）
│   └── smartcar_nav2.launch.py   # 顶层：包含 nav2_bringup + 加载航点
└── maps/
    └── .gitkeep                  # 无 SLAM，保留空目录以备调试
```

**包类型**：纯配置型（`ament_cmake` 或 `ament_python`）。推荐 `ament_cmake` 以便通过 `CMake install` 部署 YAML/XML 到 `share/smartcar_nav2`。

## 6. 参数设计（初稿）

### 6.1 全局代价地图

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 1.0
      global_frame: odom_combined      # 无 map frame
      robot_base_frame: base_footprint
      rolling_window: true             # 随机器人滚动，替代 static_layer
      width: 10
      height: 10
      resolution: 0.05
      robot_radius: 0.25
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          data_type: LaserScan
          clearing: true
          marking: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 0.45
        cost_scaling_factor: 5.0
```

### 6.2 局部代价地图

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 2.0
      global_frame: odom_combined
      robot_base_frame: base_footprint
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.03
      robot_radius: 0.25
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          data_type: LaserScan
          clearing: true
          marking: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 0.30
        cost_scaling_factor: 5.0
```

### 6.3 控制器（Regulated Pure Pursuit）

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.25
      max_linear_vel: 0.30
      min_linear_vel: 0.05
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.0
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: true
      use_approach_linear_velocity_scaling: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_collision_detection: true
      use_cost_regulated_linear_velocity_scaling: true
      cost_scaling_dist: 0.6
      cost_scaling_gain: 1.0
      inflation_cost_scaling_factor: 3.0
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.05
      use_fixed_curvature_lookahead: false
      curvature_lookahead_dist: 0.25
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      max_angular_vel: 1.0
      max_lateral_accel: 1.0
```

**阿克曼关键参数**：
- `desired_linear_vel` / `max_linear_vel`：根据实车标定可调。
- `min_turning_radius` 由底盘决定；若 RPP 频繁失败，需测量最小转弯半径并填入。
- `use_rotate_to_heading: true` 允许 RPP 在起点做原地转向对齐；若漂移严重，后续可改用 `RotationShimController` 包装。

### 6.4 规划器

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 5.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

### 6.5 Waypoint Follower

```yaml
waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 5000   # 5 s，单位：ms
```

**注意**：任务点的 QR/VLM 动作不应依赖 Nav2 的 pause 时长（ Nav2 的 wait 只是固定延时）。真正的任务触发由 `smartcar_task` 在 waypoint 到达后通过 action feedback 或独立状态机控制。

### 6.6 恢复与行为树

使用 Nav2 默认 `navigate_to_pose_w_replanning_and_recovery.xml`，但将恢复超时缩短：

- `Wait`：5 s → 1 s
- `BackUp`：0.30 m → 0.15 m
- `Spin`：1.57 rad → 1.0 rad

## 7. Launch 设计

### 7.1 `nav2_bringup.launch.py`

功能：启动 Nav2 核心节点。

参数：
- `use_sim_time`: false（实车）
- `params_file`: `config/nav2_params.yaml`
- `autostart`: true
- `use_composition`: false（RDK 调试方便，后续可改为 true）
- `map`: 空（无 SLAM）

实现方式：直接 `IncludeLaunchDescription` 调用 `nav2_bringup/launch/bringup_launch.py`，传入自定义参数文件。

### 7.2 `smartcar_nav2.launch.py`

功能：组合 Nav2 bringup + 航点加载 + RViz（可选）。

参数：
- `use_rviz`: false
- `waypoints_file`: `config/waypoints/default_waypoints.yaml`

## 8. 与周边模块集成

### 8.1 与 smartcar_bringup

- `smartcar_bringup` 已提供 `/scan`、`/odom_combined`、TF `odom_combined→base_footprint→base_link→laser`。
- `smartcar_nav2` 不重复启动底盘/LiDAR，仅依赖上述话题与 TF。

### 8.2 与 smartcar_vision

- 人形立牌识别由相机 + VLM 完成，不进入 costmap。
- QR 码识别结果由 `smartcar_task` 消费，不直接传给 Nav2。

### 8.3 与 smartcar_task

- `smartcar_task` 生成航点序列，通过 action `/follow_waypoints` 调用 `nav2_waypoint_follower`。
- 到达任务点后，`smartcar_task` 执行子任务（QR/VLM/语音/屏幕）。
- 若 Nav2 报告 waypoint 失败，`smartcar_task` 根据策略继续下一航点或进入降级状态。

## 9. 测试计划

1. **单元测试**：
   - 参数文件 YAML 语法检查。
   - launch 文件 `launch_test`（ament_cmake 可添加）。
2. **仿真/离线测试**：
   - 使用 `ros2 launch nav2_bringup` + 录制 bag 回放 `/scan` 与 `/odom_combined`，验证 Nav2 启动无报错。
3. **实车测试**：
   - 阶段 1：原地启动 Nav2，确认 costmap 能正确显示 `/scan` 障碍物。
   - 阶段 2：发布单点 goal（P 区附近），验证 RPP 能驶向目标。
   - 阶段 3：加载完整航点序列，验证 Waypoint Follower 逐点执行。
   - 阶段 4：加入锥桶障碍，验证避障与重规划。
4. **调参记录**：
   - 记录 `lookahead_dist`、`robot_radius`、`inflation_radius`、`xy_goal_tolerance` 等关键参数的实际效果。

## 10. 未决问题

1. **base_to_link 偏移**：x=0.41, y=0.12 是否准确影响 costmap 中心与底盘中心对齐，需实车标定复核（见 [[base-to-link-offset-todo]]）。
2. **最小转弯半径**：RPP 参数中的最小转弯半径需实车测量。
3. **航点坐标**：default_waypoints.yaml 中的坐标需根据赛场实际测量。
4. **RotationShimController**：是否启用取决于 RPP 大航向偏差时的现场表现。
5. **用户确认**：第 4 节中的 9 项决策需用户确认或调整后方可定稿。

## 11. 参考

- [OSS 调研报告](./../research/2026-07-17-smartcar-nav2-oss-survey.md)
- `docs/智慧医疗挑战赛_技术方案_本地部署版.md` §8
- Nav2 Humble 官方文档：https://navigation.ros.org/
- OriginBot 导航教程：https://www.originbot.org/en/application/navigation.html
