# smartcar_nav2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

> **实现状态（2026-07-18）**：本计划的初版 Navfn 方案已被后续阿克曼安全复审替换。当前实现为 SmacPlannerHybrid（DUBIN）+ Regulated Pure Pursuit，禁止 Spin 和原地旋转；权威参数见 `src/smartcar_nav2/config/nav2_params.yaml`，完整里程碑见 `docs/superpowers/plans/2026-07-17-smartcar-completion.md`。

**Goal:** 创建纯配置型 ROS2 Humble 包 `smartcar_nav2`，为 OriginCar + RDK X5 提供基于 Nav2 的航点导航能力，适配纯惯导、LiDAR-only 避障、阿克曼底盘约束。

**Architecture:** 以 `nav2_bringup` 为模板，通过 `IncludeLaunchDescription` 引入官方 bringup launch，替换为自定义 `nav2_params.yaml`、行为树 XML 与航点 YAML；局部控制器采用 `RegulatedPurePursuitController`，全局规划器采用 `NavfnPlanner`，代价地图仅订阅 `/scan`。

**Tech Stack:** ROS2 Humble, Nav2, ament_cmake, launch_ros, Python launch, YAML, XML behavior tree.

---

## 文件结构

```text
src/smartcar_nav2/
├── package.xml
├── CMakeLists.txt
├── config/
│   ├── nav2_params.yaml
│   ├── waypoints/
│   │   └── default_waypoints.yaml
│   └── behavior_trees/
│       └── navigate_to_pose_w_replanning_and_recovery.xml
├── launch/
│   ├── nav2_bringup.launch.py
│   └── smartcar_nav2.launch.py
└── test/
    └── test_smartcar_nav2_launch.py
```

---

## Task 1: 创建 smartcar_nav2 包骨架

**Files:**
- Create: `src/smartcar_nav2/package.xml`
- Create: `src/smartcar_nav2/CMakeLists.txt`
- Create: `src/smartcar_nav2/launch/.gitkeep`
- Create: `src/smartcar_nav2/config/.gitkeep`
- Create: `src/smartcar_nav2/config/waypoints/.gitkeep`
- Create: `src/smartcar_nav2/config/behavior_trees/.gitkeep`
- Create: `src/smartcar_nav2/test/.gitkeep`
- Create: `src/smartcar_nav2/maps/.gitkeep`

- [ ] **Step 1: 创建目录结构**

Run:

```powershell
$dirs = @(
  "src/smartcar_nav2/launch",
  "src/smartcar_nav2/config/waypoints",
  "src/smartcar_nav2/config/behavior_trees",
  "src/smartcar_nav2/test",
  "src/smartcar_nav2/maps"
)
foreach ($d in $dirs) {
  New-Item -ItemType Directory -Force -Path $d | Out-Null
}
```

- [ ] **Step 2: 写入 package.xml**

Create: `src/smartcar_nav2/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>smartcar_nav2</name>
  <version>0.1.0</version>
  <description>smartcar Nav2 航点导航配置包：纯惯导、LiDAR-only 避障、阿克曼底盘适配</description>
  <maintainer email="lx_hmkk@qq.com">LX-HMKK</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_waypoint_follower</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>launch</exec_depend>

  <test_depend>ament_cmake</test_depend>
  <test_depend>launch_testing</test_depend>
  <test_depend>launch_testing_ros</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: 写入 CMakeLists.txt**

Create: `src/smartcar_nav2/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(smartcar_nav2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_smartcar_nav2_launch.py)
endif()

ament_package()
```

- [ ] **Step 4: 提交骨架**

Run:

```powershell
@'
feat(nav2): 创建 smartcar_nav2 包骨架

- 新增 package.xml、CMakeLists.txt
- 新增 launch/config/maps/test 占位目录
'@ | Out-File -Encoding utf8 .git-msg
git add src/smartcar_nav2
git commit -F .git-msg
Remove-Item .git-msg
```

---

## Task 2: 编写 Nav2 核心参数文件

**Files:**
- Create: `src/smartcar_nav2/config/nav2_params.yaml`

- [ ] **Step 1: 写入 nav2_params.yaml**

Create: `src/smartcar_nav2/config/nav2_params.yaml`

```yaml
amcl:
  ros__parameters:
    use_sim_time: false

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: odom_combined
    robot_base_frame: base_footprint
    odom_topic: /odom_combined
    bt_loop_duration: 10
    default_server_timeout: 20
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator/NavigateToPose"
      goal_blackboard_id: "goal"
    navigate_through_poses:
      plugin: "nav2_bt_navigator/NavigateThroughPoses"
      goals_blackboard_id: "goals"
    # 行为树路径会在 launch 中通过 default_nav_to_pose_bt_xml 参数注入

bt_navigator_navigate_through_poses_rclcpp_node:
  ros__parameters:
    use_sim_time: false

bt_navigator_navigate_to_pose_rclcpp_node:
  ros__parameters:
    use_sim_time: false

controller_server:
  ros__parameters:
    use_sim_time: false
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
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 0.30
        cost_scaling_factor: 5.0
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 1.0
      global_frame: odom_combined
      robot_base_frame: base_footprint
      rolling_window: true
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
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 0.45
        cost_scaling_factor: 5.0
      always_send_full_costmap: true

planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 5.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

recoveries_server:
  ros__parameters:
    use_sim_time: false
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
    global_frame: odom_combined
    robot_base_frame: base_footprint
    transform_timeout: 0.1

waypoint_follower:
  ros__parameters:
    use_sim_time: false
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 5000

velocity_smoother:
  ros__parameters:
    use_sim_time: false
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    max_velocity: [0.30, 0.0, 1.0]
    min_velocity: [-0.05, 0.0, -1.0]
    max_accel: [0.5, 0.0, 1.0]
    max_decel: [-0.5, 0.0, -1.0]
    odom_topic: "odom_combined"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
```

- [ ] **Step 2: 本地 YAML 语法检查**

Run:

```powershell
python -c "import yaml; yaml.safe_load(open('src/smartcar_nav2/config/nav2_params.yaml', encoding='utf-8'))" && echo "YAML OK"
```

Expected: `YAML OK`

> Windows 注意：本机 Python 默认可能使用 `gbk` 编码，因此 `open()` 必须显式指定 `encoding='utf-8'`。

- [ ] **Step 3: 提交参数文件**

Run:

```powershell
@'
feat(nav2): 添加 Nav2 核心参数配置

- 采用 RegulatedPurePursuitController 适配阿克曼底盘
- 代价地图仅 obstacle_layer + inflation_layer，订阅 /scan
- 统一使用 /odom_combined 与 odom_combined 坐标系
- Waypoint Follower 使用 WaitAtWaypoint
'@ | Out-File -Encoding utf8 .git-msg
git add src/smartcar_nav2/config/nav2_params.yaml
git commit -F .git-msg
Remove-Item .git-msg
```

---

## Task 3: 编写缩短恢复超时的行为树

**Files:**
- Create: `src/smartcar_nav2/config/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`

- [ ] **Step 1: 写入行为树 XML**

Create: `src/smartcar_nav2/config/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`

```xml
<!--
  基于 Nav2 Humble 默认 navigate_to_pose_w_replanning_and_recovery.xml
  调整恢复行为超时，避免赛道中恢复动作耗时过长
-->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.0" name="SpinRecovery"/>
          <Wait wait_duration="1" name="WaitRecovery"/>
          <BackUp backup_dist="0.15" backup_speed="0.05" name="BackUpRecovery"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

- [ ] **Step 2: XML 语法检查**

Run:

```powershell
python -c "import xml.etree.ElementTree as ET; ET.parse('src/smartcar_nav2/config/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml')" && echo "XML OK"
```

Expected: `XML OK`

- [ ] **Step 3: 提交行为树**

Run:

```powershell
@'
feat(nav2): 添加恢复超时缩短的行为树

- Spin 1.0 rad / Wait 1 s / BackUp 0.15 m
- 保留默认恢复链结构，避免赛道中恢复耗时过长
'@ | Out-File -Encoding utf8 .git-msg
git add src/smartcar_nav2/config/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
git commit -F .git-msg
Remove-Item .git-msg
```

---

## Task 4: 编写默认航点序列

**Files:**
- Create: `src/smartcar_nav2/config/waypoints/default_waypoints.yaml`

- [ ] **Step 1: 写入默认航点**

Create: `src/smartcar_nav2/config/waypoints/default_waypoints.yaml`

```yaml
# 默认航点序列：P 区 → 任务点 → 通道口 → C 环道 → 返回 P 区
# 坐标单位为米，基于发车时 P 区中心为坐标原点
# 注意：以下数值为占位符，需根据赛场实际测量后替换

waypoints:
  - frame_id: "odom_combined"
    pose:
      position: { x: 0.0, y: 0.0, z: 0.0 }
      orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    task: "start"   # P 区发车点

  - frame_id: "odom_combined"
    pose:
      position: { x: 1.5, y: 0.0, z: 0.0 }
      orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    task: "qr"      # 任务点 1：二维码识别

  - frame_id: "odom_combined"
    pose:
      position: { x: 3.0, y: 0.5, z: 0.0 }
      orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    task: "vlm"     # 任务点 2：VLM 图生文（人形立牌）

  - frame_id: "odom_combined"
    pose:
      position: { x: 4.0, y: 1.0, z: 0.0 }
      orientation: { x: 0.0, y: 0.0, z: 0.707, w: 0.707 }
    task: "corridor" # 通道口

  - frame_id: "odom_combined"
    pose:
      position: { x: 4.0, y: 3.0, z: 0.0 }
      orientation: { x: 0.0, y: 0.0, z: 1.0, w: 0.0 }
    task: "loop"    # C 环道终点

  - frame_id: "odom_combined"
    pose:
      position: { x: 0.0, y: 0.0, z: 0.0 }
      orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    task: "return"  # 返回 P 区
```

- [ ] **Step 2: YAML 语法检查**

Run:

```powershell
python -c "import yaml; yaml.safe_load(open('src/smartcar_nav2/config/waypoints/default_waypoints.yaml', encoding='utf-8'))" && echo "YAML OK"
```

Expected: `YAML OK`

- [ ] **Step 3: 提交航点文件**

Run:

```powershell
@'
feat(nav2): 添加默认航点序列占位文件

- P → 任务点 1/2 → 通道口 → C 环道 → 返回 P
- 坐标为占位值，赛前需根据实际赛场测量替换
'@ | Out-File -Encoding utf8 .git-msg
git add src/smartcar_nav2/config/waypoints/default_waypoints.yaml
git commit -F .git-msg
Remove-Item .git-msg
```

---

## Task 5: 编写 nav2_bringup.launch.py

**Files:**
- Create: `src/smartcar_nav2/launch/nav2_bringup.launch.py`

- [ ] **Step 1: 写入 launch 文件**

Create: `src/smartcar_nav2/launch/nav2_bringup.launch.py`

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('smartcar_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file')

    declare_bt_xml_file = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(
            pkg_dir, 'config', 'behavior_trees',
            'navigate_to_pose_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree XML file')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'bt_xml_file': bt_xml_file,
            'autostart': 'true',
            'use_composition': 'false',
            'map': '',  # 无 SLAM，不加载地图
        }.items())

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_bt_xml_file,
        nav2_launch,
    ])
```

- [ ] **Step 2: Python 语法检查**

Run:

```powershell
python -m py_compile src/smartcar_nav2/launch/nav2_bringup.launch.py && echo "PY OK"
```

Expected: `PY OK`

- [ ] **Step 3: 提交 launch 文件**

Run:

```powershell
@'
feat(nav2): 添加 nav2_bringup.launch.py

- 通过 IncludeLaunchDescription 复用 nav2_bringup bringup_launch.py
- 注入自定义 nav2_params.yaml 与行为树 XML
- 无 SLAM：map 参数置空
'@ | Out-File -Encoding utf8 .git-msg
git add src/smartcar_nav2/launch/nav2_bringup.launch.py
git commit -F .git-msg
Remove-Item .git-msg
```

---

## Task 6: 编写 smartcar_nav2.launch.py（顶层 launch）

**Files:**
- Create: `src/smartcar_nav2/launch/smartcar_nav2.launch.py`

- [ ] **Step 1: 写入顶层 launch 文件**

Create: `src/smartcar_nav2/launch/smartcar_nav2.launch.py`

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('smartcar_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    waypoints_file = LaunchConfiguration('waypoints_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Nav2 parameters file')

    declare_bt_xml_file = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(
            pkg_dir, 'config', 'behavior_trees',
            'navigate_to_pose_w_replanning_and_recovery.xml'),
        description='Behavior tree XML file')

    declare_waypoints_file = DeclareLaunchArgument(
        'waypoints_file',
        default_value=os.path.join(
            pkg_dir, 'config', 'waypoints', 'default_waypoints.yaml'),
        description='Waypoint sequence YAML file')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'nav2_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'bt_xml_file': bt_xml_file,
        }.items())

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_bt_xml_file,
        declare_waypoints_file,
        nav2_bringup,
    ])
```

- [ ] **Step 2: Python 语法检查**

Run:

```powershell
python -m py_compile src/smartcar_nav2/launch/smartcar_nav2.launch.py && echo "PY OK"
```

Expected: `PY OK`

- [ ] **Step 3: 提交顶层 launch**

Run:

```powershell
@'
feat(nav2): 添加 smartcar_nav2 顶层 launch

- 组合 nav2_bringup.launch.py
- 暴露 waypoints_file 参数供后续任务状态机使用
'@ | Out-File -Encoding utf8 .git-msg
git add src/smartcar_nav2/launch/smartcar_nav2.launch.py
git commit -F .git-msg
Remove-Item .git-msg
```

---

## Task 7: 添加 launch 测试

**Files:**
- Create: `src/smartcar_nav2/test/test_smartcar_nav2_launch.py`
- Modify: `src/smartcar_nav2/CMakeLists.txt`

- [ ] **Step 1: 写入 launch 测试**

Create: `src/smartcar_nav2/test/test_smartcar_nav2_launch.py`

```python
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


class TestSmartcarNav2Launch(unittest.TestCase):

    def test_count_until_timeout(self, proc_output):
        # launch 启动后 5 秒内应能完成初始化并稳定运行
        proc_output.assertWaitFor(
            'Nav2 lifecycle manager completed', timeout=5, stream='stdout')


def generate_test_description():
    pkg_dir = get_package_share_directory('smartcar_nav2')
    launch_file = os.path.join(pkg_dir, 'launch', 'smartcar_nav2.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'use_sim_time': 'false'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])
```

- [ ] **Step 2: Python 语法检查**

Run:

```powershell
python -m py_compile src/smartcar_nav2/test/test_smartcar_nav2_launch.py && echo "PY OK"
```

Expected: `PY OK`

- [ ] **Step 3: 更新 CMakeLists.txt 启用测试**

Create: `src/smartcar_nav2/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(smartcar_nav2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_smartcar_nav2_launch.py)
endif()

ament_package()
```

- [ ] **Step 4: 提交测试**

Run:

```powershell
@'
test(nav2): 添加 launch 初始化测试

- 验证 smartcar_nav2.launch.py 可在 5 秒内完成 Nav2 生命周期初始化
'@ | Out-File -Encoding utf8 .git-msg
git add src/smartcar_nav2/test/test_smartcar_nav2_launch.py src/smartcar_nav2/CMakeLists.txt
git commit -F .git-msg
Remove-Item .git-msg
```

---

## Task 8: 本地构建与语法检查

- [ ] **Step 1: YAML/XML/PY 语法全量检查**

Run:

```powershell
python -c "import yaml; yaml.safe_load(open('src/smartcar_nav2/config/nav2_params.yaml', encoding='utf-8'))"
python -c "import yaml; yaml.safe_load(open('src/smartcar_nav2/config/waypoints/default_waypoints.yaml', encoding='utf-8'))"
python -c "import xml.etree.ElementTree as ET; ET.parse('src/smartcar_nav2/config/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml')"
Get-ChildItem src/smartcar_nav2/launch/*.launch.py | ForEach-Object { python -m py_compile $_.FullName }
Get-ChildItem src/smartcar_nav2/test/*.py | ForEach-Object { python -m py_compile $_.FullName }
Write-Host "All syntax checks passed"
```

Expected: `All syntax checks passed`

> Windows 注意：PowerShell 不会自动展开 `*.launch.py` glob，需用 `Get-ChildItem` 遍历。

- [ ] **Step 2: 运行现有 sync_to_rdk 单元测试**

Run:

```powershell
python -m unittest tests.test_sync_to_rdk
```

Expected: `OK` (38 tests)

---

## Task 9: 同步到 RDK 并构建

- [ ] **Step 1: 同步到 RDK**

Run:

```powershell
python scripts/sync_to_rdk.py push --dry-run
```

确认只新增/修改 `src/smartcar_nav2` 及相关文档后：

```powershell
python scripts/sync_to_rdk.py push
```

- [ ] **Step 2: RDK 上构建 smartcar_nav2**

SSH 到 RDK：

```bash
ssh root@192.168.128.10
source ~/source_env.sh
cd /root/ros2_ws
colcon build --packages-select smartcar_nav2 --symlink-install
```

Expected: build 成功，无 ERROR。

- [ ] **Step 3: 在 RDK 上启动 Nav2 并检查节点**

在 RDK 上先启动 bringup：

```bash
source ~/source_env.sh
ros2 launch smartcar_bringup smartcar_bringup.launch.py
```

另开终端：

```bash
source ~/source_env.sh
ros2 launch smartcar_nav2 smartcar_nav2.launch.py
```

检查节点：

```bash
ros2 node list | grep -E 'controller_server|planner_server|bt_navigator|waypoint_follower'
```

Expected: 列出 `controller_server`、`planner_server`、`bt_navigator`、`waypoint_follower`。

---

## Task 10: 更新 CLAUDE.md 与 README

**Files:**
- Modify: `CLAUDE.md` §模块状态表格
- Modify: `README.md`（若存在 Nav2 相关命令区）

- [ ] **Step 1: 更新 CLAUDE.md 模块状态**

Modify: `CLAUDE.md` 中模块状态表格

```markdown
| smartcar_nav2 | 🔄 实现中 | feat/smartcar-nav2 | 纯配置型 Nav2 包：nav2_params.yaml + RPP + Waypoint Follower |
```

- [ ] **Step 2: 在 README.md 添加启动命令**

Modify: `README.md`

```markdown
# 启动 Nav2 航点导航
ros2 launch smartcar_nav2 smartcar_nav2.launch.py
```

- [ ] **Step 3: 提交文档更新**

Run:

```powershell
@'
docs(nav2): 更新 CLAUDE.md 与 README 记录 smartcar_nav2 状态
'@ | Out-File -Encoding utf8 .git-msg
git add CLAUDE.md README.md
git commit -F .git-msg
Remove-Item .git-msg
```

---

## Task 11: 分支收尾

- [ ] **Step 1: 最终检查**

Run:

```powershell
git status --short
git log --oneline -12
```

Expected:
- 工作区 clean
- 最近 12 条提交包含 smartcar_nav2 的 6-7 个增量提交 + 文档更新

- [ ] **Step 2: 使用 finishing-a-development-branch skill**

根据 `superpowers:finishing-a-development-branch` 流程：

1. 确认测试通过
2. 检测环境
3. 选择合并/PR/保留/丢弃

运行：

```powershell
git status --short
git branch
```

然后按 skill 指引向用户呈现选项：

```
Implementation complete. What would you like to do?
1. Merge back to main locally
2. Push and create a Pull Request
3. Keep the branch as-is (I'll handle it later)
4. Discard this work
```

---

## Self-Review

### Spec coverage

- [x] 全局规划器 `NavfnPlanner` → Task 2 nav2_params.yaml §planner_server
- [x] 局部控制器 `RegulatedPurePursuitController` → Task 2 nav2_params.yaml §controller_server
- [x] 代价地图 obstacle+inflation → Task 2 nav2_params.yaml §local_costmap/global_costmap
- [x] 行为树缩短恢复超时 → Task 3
- [x] Waypoint Follower `WaitAtWaypoint` → Task 2 nav2_params.yaml §waypoint_follower
- [x] `/odom_combined` 统一 → Task 2 nav2_params.yaml
- [x] 航点序列 → Task 4
- [x] launch 文件 → Task 5, Task 6
- [x] 测试 → Task 7
- [x] RDK 构建验证 → Task 9

### Placeholder scan

- 无 "TBD/TODO"
- 所有代码片段完整
- 所有命令包含预期输出

### Type consistency

- `use_sim_time` 在 launch 与 params 中均为字符串 `'false'` 或 YAML boolean `false`
- `odom_combined` 作为 global_frame 在 costmap、bt_navigator、recoveries_server 中一致
- `base_footprint` 作为 robot_base_frame 一致
