# 逆向导航技术方案研究报告

> **日期**: 2026-07-24 | **作者**: LX-HMKK | **状态**: 方案设计阶段，尚未实施

## 1. 研究背景

### 1.1 问题陈述

竞赛场地中存在约 0.55m 长度直通道。当前系统采用 DUBIN 运动模型（仅前向），Ackermann 底盘无法原地旋转，若通道宽度不足以支持车辆前向转弯通过，则需要逆向行驶能力。`default_waypoints.yaml` 中已有 `b_corridor_return_enter` 和 `b_corridor_return` 两个回程航点指向该通道。

### 1.2 当前系统约束

- 底盘：Ackermann 转向，不可原地旋转
- 导航规划器：SmacPlannerHybrid，运动模型 DUBIN（仅前向）
- 控制器：Regulated Pure Pursuit（RPP），`allow_reversing=true` 已启用
- 速度平滑器：`velocity_smoother`，当前逆向速度上限 `min_velocity.x = -0.05 m/s`
- 安全节点：`smartcar_safety`（C++），Ackermann 转换公式 `steering = atan(L * ω / v)` 天然支持负 v；无速度限幅器
- STM32 固件对负速度的支持：**未经验证**

---

## 2. 方案调研

### 2.1 方案一：REEDS_SHEPP 运动模型切换

**原理**：将 Nav2 规划器的 `motion_model_for_search` 从 DUBIN 切换为 REEDS_SHEPP，规划器生成包含 cusp 点（方向反转标记）的路径。FeasiblePathHandler 在每个 cusp 点裁剪路径，强制控制器到达反转点后再继续；RPP 检测 cusp 后翻转 lookahead，输出负 `linear.x`。

**优势**：

- 全 Nav2 原生支持，无需任务层代码变更
- 规划器自动决定何时需要逆向，无人工标注航点方向
- 有完整的障碍物避让

**劣势与风险**：

| 风险项 | 严重度 | 说明 |
|--------|--------|------|
| 多组件耦合 | 高 | 需同时变更 planner、controller（FeasiblePathHandler）、velocity_smoother，任一环节失败即整体失效 |
| STM32 固件 | 高 | 底盘必须接受负速度，未经验证 |
| Forward 路径退化 | 中 | DUBIN→REEDS_SHEPP 改变搜索空间，可能影响现有前向路径质量 |
| 无速度限幅器 | 中 | safety_node 不做速度限幅，仅靠 RPP+smoother 控制逆向速度 |
| Cusp 处理鲁棒性 | 中 | `FeasiblePathHandler` 当前未配置，`enforce_path_inversion` 在真实走廊中未经验证 |
| 成本地图 footprint | 低 | 逆向行驶时 footprint 前后颠倒，需验证碰撞检测方向正确性 |

**配置变更量**：约 12-15 行（planner + controller + smoother + behavior server 清理）

**结论**：可行（verdict: YES），但改动面大、风险多。**作为 Plan B 保留**，当前不推荐作为首选方案。

---

### 2.2 方案二：开环倒车（Timed Command）

**原理**：在走廊回程时，直接发布固定时长（~3.7s）的负 `/cmd_vel`，绕过 Nav2 规划。纯开环控制，无位姿反馈。

**可行性评估：不可行**

致命安全隐患：
1. 安全节点 `command_timeout_sec=0.30`——单次发布 300ms 后被清零，需要持续 20Hz 发布的后台线程
2. 无滑移补偿：20% 滑移（光滑地面）= 55cm 距离上 11cm 位置误差
3. 无偏航反馈：2 度偏航漂移 = 1.9cm 横向误差
4. 无避障：LiDAR 数据完全忽略
5. 运动后位姿不确定：Nav2 信念与实际位置偏差，影响后续导航段

**结论：明确不推荐。**

---

### 2.3 方案三：双控制器动态切换（Dual Controller）

**原理**：配置两个 RPP 插件实例（`ForwardPath` / `ReversePath`），通过 Nav2 的 `ControllerSelector` BT 节点和 `controller_selector` 话题在运行时切换。

**优势**：
- 使用 Nav2 内置机制，无需动态修改参数
- 前向/逆向控制器参数独立调优（lookahead 方向不同）

**劣势**：
- 需要修改行为树 XML（添加 `ControllerSelector` 节点）
- 需要运行时 topic 发布切换控制器，增加任务层逻辑
- 存在竞态：任务层 topic 发布和 BT tick 之间可能有时序问题

**结论**：技术上可行，但比方案四复杂。不推荐作为首选，可作为方案四的后备。

---

### 2.4 方案四：方向字段 + 航点朝向翻转（Orientation Flip）【推荐】

**原理**：在航点数据结构中新增 `direction` 字段（`"forward"` / `"reverse"`）。任务层在构建导航段时，对 `direction="reverse"` 的航点，将其朝向（四元数）绕 Z 轴旋转 180 度。RPP 的 `allow_reversing=true` 在检测到目标位姿位于车辆"背后"时，**自动**输出负线速度。无需 planner 变更，无需行为树变更，无需动态参数。

**核心技术链路**：

```
航点 yaw 翻转 (180°)
  → RPP lookahead 检测目标在背后
  → RPP 输出负 linear.x (allow_reversing=true)
  → velocity_smoother 平滑透过 (min_velocity.x=-0.20)
  → safety_node Ackermann 转换 (atan(L*ω/v) 正确处理 v<0)
  → /ackermann_cmd speed < 0
  → STM32/底盘执行逆向运动
```

**为什么不需要 SetParameters 动态改 desired_linear_vel**：
RPP 的 `allow_reversing=true` 使其在检测到目标朝向与当前朝向不匹配（~180° 差异）时，自动翻转 lookahead 方向并产出负速度。速度值由 RPP 内部的速度缩放逻辑决定，其大小受 `desired_linear_vel` 参数约束——但符号由方向不匹配自动决定，无需外部干预。

**优势**：

| 对比维度 | Orientation Flip | REEDS_SHEPP | Dual Controller | 开环倒车 |
|---------|-----------------|-------------|-----------------|---------|
| 改动文件数 | 4（~130 行） | 1（~15 行） + 验证 | 3（~60 行） + BT XML | 2 |
| Planner 影响 | 无 | 有（搜索空间变更） | 无 | 绕过 |
| BT XML 变更 | 无 | 无 | 需要 | 无 |
| Forward 路径影响 | 零 | 可能退化 | 零 | N/A |
| 运行时复杂度 | 低（一次翻转） | 低（planner 自动） | 中（topic 切换） | 中（后台线程） |
| 障碍物避让 | 完整（Nav2） | 完整（Nav2） | 完整（Nav2） | 无 |
| STM32 依赖 | 需要负速度 | 需要负速度 | 需要负速度 | 需要负速度 |

**选择理由**：
- **最小改动量**：不改 planner，不改行为树，只改任务层航点处理
- **Forward 路径零影响**：非逆向航点 `direction` 默认为 `"forward"`，处理路径与当前完全一致
- **利用已有机制**：RPP 的 `allow_reversing=true` 已启用，只需让目标位姿的朝向"看起来在背后"
- **自然回正**：逆向段结束时，翻转后的航点朝向与车辆物理朝向一致（都朝 +Y），GoalChecker yaw 误差约等于 0，无需放宽容差

---

## 3. 推荐实施方案

### 3.1 方案概览

**Tier 1（首选，本次实施）**：Orientation Flip（方案四）——方向字段 + 航点朝向翻转 + velocity_smoother 修复

**Tier 2（Plan B，Tier 1 验证失败后切换）**：REEDS_SHEPP（方案一）——切换运动模型，利用 planner 自动生成逆向路径。配置变更见 [附录 A](#a-plan-b-reeds_shepp-配置变更)。

**回退路径**：若 Tier 1 和 Plan B 均失败，回退至纯参数调优方案（缩小 inflation_radius + 调整 footprint 使通道可正向通过），放弃逆向能力。

### 3.2 实施步骤

#### Step 1: velocity_smoother 逆向速度上限修复（前置基础设施）

**文件**：`src/smartcar_nav2/config/nav2_params.yaml`

```yaml
# velocity_smoother section (line ~183-196)
min_velocity: [-0.20, 0.0, -0.75]  # x 分量：-0.05 → -0.20
max_velocity: [0.30, 0.0, 0.75]    # 不变
```

同时清理 `behavior_server` 中未使用的 `backup`/`wait` 插件（`behavior_plugins` 列表），减少不必要的节点加载。

**预估行数**：3 行

#### Step 2: Waypoint 数据结构扩展——`direction` 字段

**文件**：`src/smartcar_task/smartcar_task/waypoints.py`

- 新增 `ALLOWED_DIRECTIONS = frozenset({"forward", "reverse"})`
- `Waypoint` dataclass 新增 `direction: str = "forward"` 字段（在 `task` 之后，`id` 之前）
- `_parse_waypoint()`：读取 `item.get("direction", "forward")`，校验后传入
- `_waypoint_mapping()`：输出包含 `direction` 字段
- `validate_waypoints()`：新增约束 `waypoints[0].direction == "forward"` 且 `waypoints[-1].direction == "forward"`（起点和终点永远正向）

**预估行数**：30 行

#### Step 3: 任务层——按方向拆段 + 朝向翻转

**文件**：
- `src/smartcar_task/smartcar_task/mission.py`
- `src/smartcar_task/smartcar_task/task_node.py`

**mission.py 变更**：
- 重写 `_navigation_segments()` 为双重条件拆分器——在 task 终点（qr/vlm/return）**和** direction 变更点都拆段。生成 `(segment, direction)` 元组
- `_run()` 解包为 `for segment, direction in self._navigation_segments(waypoints)`
- `_navigate()` 新增 `direction` 参数，传递给 navigator

**task_node.py 变更**：
- `RosNavigator` 新增 `_flip_waypoint_orientations(waypoints)` 静态方法——将每个航点的四元数绕 Z 轴旋转 180 度：`(qx, qy, qz, qw) → (0, 0, qw, -qz)`（适用于 2D 地面位姿，qx=qy≈0）
- `navigate(waypoints, direction="forward")` 方法：若 `direction="reverse"`，先翻转航点朝向再构建 `FollowWaypoints` goal
- 在 `finally` 块中恢复正向速度参数（确保 mission retry/异常退出后的安全性）

**关键设计决策**：不需要 `SetParameters` 动态改 `desired_linear_vel`。RPP 的 `allow_reversing=true` 会在检测到朝向不匹配时自动产出负速度，符号由 RPP 内部逻辑决定。

**预估行数**：80 行

#### Step 4: 默认航点文件添加 `direction` 标记

**文件**：`src/smartcar_nav2/config/waypoints/default_waypoints.yaml`

为回程通道航点 `b_corridor_return_enter` 和 `b_corridor_return` 添加 `direction: "reverse"`。

其余所有航点隐式默认为 `direction: "forward"`（YAML 中省略）。回程航点对应的朝向保持当前 -Y 朝向（`z=-0.707, w=0.707`），代码会在运行时自动翻转 180 度为 +Y。

`nav_only.yaml`（若存在）同样处理。

**预估行数**：2 行

#### Step 5: 单元测试更新

**文件**：
- `src/smartcar_task/test/test_waypoints.py`
- `src/smartcar_task/test/test_mission.py`

**test_waypoints.py**：
- `valid_document()` 中 `corridor_return` 航点添加 `direction: "reverse"`
- 新增 `test_direction_field_validation`——正向/逆向合法性、非法 direction 值拒绝、start/return 必须是 forward
- 新增 `test_orientation_flip_math`——验证 `flip((0,0,0.707,0.707)) → (0,0,0.707,-0.707)` 等价于 +90° → -90°（绕 Z 轴 180 度旋转）

**test_mission.py**：
- 新增 `test_navigation_segments_respects_direction`——验证方向变更点正确拆段
- 更新现有测试中的 `Waypoint` 构造（新增 `direction` 字段默认值不影响旧测试）

**预估行数**：35 行

**目标**：确保 130/130 仓库合同测试全部通过。

---

## 4. 风险与缓解措施

| 风险项 | 概率 | 影响 | 缓解措施 |
|--------|------|------|---------|
| RPP `allow_reversing=true` 在 Nav2 1.1.20 (TROS Humble) 上逆向速度产出不一致 | 中 | 高 | 阶段 3 纯导航测试中直接观察 `/cmd_vel`；若失败启用 Plan B (REEDS_SHEPP) |
| STM32 固件不接受负速度 | 中 | 极严重 | 阶段 3 首次逆向测试时速度上限 0.10 m/s，实时观察车轮；若拒绝，中止逆向测试，切换至回退方案 |
| velocity_smoother 内部死区逻辑不同于文档描述，仍剪裁负速度 | 低 | 中 | 阶段 3 直接对比 `/cmd_vel` 和 `/cmd_vel_smooth`（或下游 topic）验证 |
| 逆向段出现摆动或偏航 | 中 | 低 | RPP 的 `lookahead_dist` 可能需要为逆向段调大；阶段 4 中用 LiDAR 观察两侧距离对称性 |
| 逆向段结束后定位偏移 | 低 | 中 | 阶段 4 对比 odom_combined 与实际位置；若偏移超出可接受范围，增加逆向段容差或降速 |
| 朝向翻转假设 qx=qy≈0 不满足（3D 航点） | 极低 | 低 | 当前所有航点均为 2D 地面位姿；已实现通用翻转公式作为 fallback |
| Mission retry 时速度状态不一致 | 低 | 中 | `finally` 块确保每条退出路径都恢复正向速度 |

---

## 5. 测试方案

### 5.1 五阶段渐进测试

每阶段通过后才进入下一阶段。

#### 阶段 1: 离线单元测试（Windows，无需硬件）

```powershell
python -m unittest discover -s src/smartcar_task/test -v
```

**验证项**：
- direction 字段的序列化/反序列化
- direction 值校验（合法/非法值）
- 段拆分逻辑（direction 变更点 + task 终点的双重拆分）
- 朝向翻转数学正确性
- **目标**：130/130 测试全绿

#### 阶段 2: RDK 无运动 smoke 测试

```bash
source ~/source_env.sh
cd /root/ros2_ws
colcon build --symlink-install
# 启动最小 smoke launch（关闭底盘/LiDAR/相机/视觉）
```

**验证项**：
- task_node 正确解析 `direction` 字段
- mission 段拆分日志输出正确
- waypoint_viz 可视化 MarkerArray 正确显示翻转后的朝向

#### 阶段 3: RDK 纯导航逆向测试（有线连接，紧急停止就绪）

```bash
ros2 launch smartcar_bringup smartcar_system.launch.py
# 使用精简测试航点文件（仅 2 个走廊航点 + return）
```

**关键验证项**（逐项检查）：

| 检查点 | 命令/方法 | 预期结果 |
|--------|----------|---------|
| RPP 产出负 /cmd_vel | `ros2 topic echo /cmd_vel` | `linear.x < 0` |
| velocity_smoother 平滑透过 | `ros2 topic echo /cmd_vel_smooth` | `linear.x < 0`，未被截断 |
| safety_node 输出负 /ackermann_cmd | `ros2 topic echo /ackermann_cmd` | `speed < 0` |
| STM32/底盘执行逆向 | 观察车轮物理转动 | 车轮反转 |
| 逆向段后恢复正向 | `ros2 topic echo /cmd_vel` | `linear.x > 0` |

**安全措施**：
- 所有测试在有线连接下进行
- 首次逆向速度上限 0.10 m/s
- 紧急停止：`pkill -9 -f "ros2 launch"`（STM32 超时自动刹停）
- 若 STM32 拒绝负速度，**立即中止**并切换至 Plan B

#### 阶段 4: 实车逆向全路线测试（运动门禁逐项开启）

使用 `nav_only.yaml` 完整路线：`qr/nav → corridor → loop → corridor(reverse) → return`

参数：0.15 m/s 前向速度上限，逆向预期 ~0.10-0.15 m/s

**验证项**：
- 逆向通道 transit 时间（预估 0.55m / 0.10 m/s ≈ 5.5s）
- 逆向段结束后定位精度（odom_combined vs 实际位置）
- GoalChecker 在逆向段终点正确判定到达（yaw 容差 0.50 rad 内）
- 逆向段路径直线性（LiDAR 观察两侧障碍物距离对称）
- 若出现摆动或偏航，调整 RPP 的 `lookahead_dist`

#### 阶段 5: 完整赛道 + 视觉集成测试

使用完整 `default_waypoints.yaml`（含 QR + VLM + loop + 逆向通道 + return）。

**验证项**：
- 全路线自主跑圈，逆向通道 transit 不中断任务流
- 逆向段入口/出口与前后航点衔接平滑
- 全路线耗时统计，确保逆向 transit 在合理时间预算内

---

## 6. 工作量估算

| 任务 | 预估时间 |
|------|---------|
| Step 1: velocity_smoother 修复 | 0.5h |
| Step 2: Waypoint direction 字段 | 1.0h |
| Step 3: 任务层拆段 + 朝向翻转 | 2.0h |
| Step 4: 航点文件标记 | 0.25h |
| Step 5: 单元测试 | 1.25h |
| 阶段 1-2 测试（离线 + smoke） | 0.5h |
| 阶段 3 测试（纯导航逆向） | 0.5h |
| 阶段 4-5 测试（实车 + 集成） | 0.5h |
| **合计** | **~6.5h** |

---

## 附录

### A. Plan B: REEDS_SHEPP 配置变更

若 Tier 1（Orientation Flip）在 RDK 上验证失败，切换至 REEDS_SHEPP。以下配置变更已记录但暂不实施。

**nav2_params.yaml 变更（约 8 行）**：

```yaml
# 1. planner_server section
planner_server:
  ros__parameters:
    motion_model_for_search: "REEDS_SHEPP"  # was "DUBIN"
    reverse_penalty: 2.0                     # NEW
    change_penalty: 0.5                      # NEW

# 2. controller_server section
controller_server:
  ros__parameters:
    path_handler_plugin_id: "PathHandler"    # NEW
    FollowPath:
      desired_linear_vel: 0.15               # NEW (make explicit)
      max_linear_vel: 0.15                   # NEW
      min_linear_vel: -0.15                  # NEW

    PathHandler:                             # NEW block
      plugin: "nav2_controller::FeasiblePathHandler"
      enforce_path_inversion: true
      inversion_xy_tolerance: 0.25
      inversion_yaw_tolerance: 0.4

# 3. velocity_smoother (already fixed in Tier 1)
min_velocity: [-0.20, 0.0, -0.75]
```

**工作流**：REEDS_SHEPP 生成含 cusp 的全局路径 → FeasiblePathHandler 在 cusp 点裁剪 → RPP 执行到反转点后自动逆向 → 无需航点朝向翻转。

**优势**：无需任务层代码变更。**代价**：planner 行为变更（DUBIN→REEDS_SHEPP），可能影响现有前向导航路径质量，需全路线复测。

### B. 回退方案

若 Tier 1 和 Plan B 均失败，回退所有变更至当前状态（DUBIN + 原始 velocity_smoother），仅依靠 costmap/footprint 调优使通道可正向通过：

- 缩小 `inflation_radius`（当前 local 0.55 / global 0.65）
- 调整 footprint 为非对称（增大前后 margin 差）
- 接受无逆向能力的限制

这是纯参数调优方案，无需代码变更，但也不能解决真正的狭窄通道问题。

---

> **关联文档**：
> - 运动与定位分析：`docs/review/odometry-speed-analysis.md`
> - 部署手册：`docs/deployment/rdk-environment-setup.md`
> - Nav2 参数链路修复：`CLAUDE.md` 已知问题章节
