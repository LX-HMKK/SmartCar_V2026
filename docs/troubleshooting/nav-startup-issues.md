# 纯导航任务启动问题日志

> 记录日期: 2026-07-24 | 会话：无线连接→nav_test.sh 启动→发车

## 问题 1: TROS setup.bash `unbound variable` 导致脚本静默退出

**症状**: `nav_test.sh` 有 `set -euo pipefail`，启动后第 1 行就退出，日志仅 `=== [1/7] ===` 及之前内容。

**根因**: TROS Humble 的 `/opt/tros/humble/setup.bash` 内引用了 `AMENT_TRACE_SETUP_FILES` 变量，但该变量未定义。`nav_test.sh` 设置 `set -u`（nounset），`source` 时继承该模式，bash 遇到未绑定的变量即退出。

**修复** (已应用到 `scripts/nav_test.sh`):
```bash
# 在 source 前后包裹 set +u / set -u
set +u
source "$SOURCE_ENV"
set -u
```

**影响文件**: `scripts/nav_test.sh`, 任何 RDK 上带 `set -u` 的脚本 source TROS setup.bash 时均受影响。

---

## 问题 2: `ros_cleanup` pkill 杀死调用者自身

**症状**: `nav_test.sh` 第 1 步执行 `bash /usr/local/bin/ros_cleanup` 后脚本消失，无后续输出。

**根因**: `ros_cleanup` 的 pkill 列表包含 `nav_test`，而 `nav_test.sh` 作为调用者进程名匹配该模式，被自己调用的清理脚本杀死。

**修复** (已应用到 `scripts/ros_cleanup.sh`):
```diff
-PROCS=(... nav_test full_test go_test run_test)
+PROCS=(... full_test go_test run_test)
```

**规则**: 清理脚本的 kill 列表**不得**包含调用它的脚本名。如需清理旧实例，应在调用者脚本中先 pkill 同名旧进程，再 exec 自身。

---

## 问题 3: Forward BT XML 缺少 `goal_checker_id`

**症状**:
```
[controller_server] [ERROR]: FollowPath called with goal_checker name in
parameter 'current_goal_checker', which does not exist.
Available goal checkers are: goal_checker reverse_goal_checker
```
任务立即 FAILED。

**根因**: 正向 BT XML (`navigate_to_pose_w_replanning_and_recovery.xml`) 的 `FollowPath` 节点未指定 `goal_checker_id`，而 `current_goal_checker` 参数默认为空。反向 XML 正确指定了 `goal_checker_id="reverse_goal_checker"`。

**修复** (已应用到 `config/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`):
```xml
<!-- Before -->
<FollowPath path="{path}" controller_id="FollowPath"/>

<!-- After -->
<FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="goal_checker"/>
```

**教训**: Nav2 的 `goal_checker_selector` BT 节点动态设置 `current_goal_checker`，但如果 BT 树未包含该 selector 节点，则必须直接在 `FollowPath` 上指定 `goal_checker_id`。`nav2_params.yaml` 中的 `current_goal_checker` 参数是 selector 的目标，不是 controller_server 直接读取的静态配置。

**影响文件**:
- `src/smartcar_nav2/config/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`
- `src/smartcar_nav2/config/nav2_params.yaml`（同步添加 `current_goal_checker: "goal_checker"` 作为显式默认值）

---

## 问题 4: `ros2 service call` / `ros2 topic echo --once` 偶发超时

**症状**: 部分 `ros2 service` 和 `ros2 topic` 命令在 SSH 非交互式会话中挂起，超时后无输出。

**触发条件**: 
- `ros2 service list` 在非完整的 SSH 环境中无输出（需 source_env.sh）
- `ros2 topic echo --once` 有时无限等待（topic 可能未活跃发布）
- `ros2 lifecycle list` 偶尔超时

**缓解**:
- 用 `timeout` 包裹或 SSH timeout 参数（`ConnectTimeout` / `ServerAliveInterval`）
- 关键操作前先 `source /root/source_env.sh`
- `--once` 改用 `--once --timeout 5`（如支持）或后台运行
- 监控优先用 Monitor 工具的 `tail -f /tmp/bringup.log` 代替 `ros2 topic echo`

---

## 发车操作清单（已验证可用）

```bash
# 1. 释放急停
ros2 service call /smartcar/safety/emergency_stop std_srvs/srv/SetBool '{data: false}'
# 预期: success=True, message='emergency stop cleared'

# 2. 重置 (可选，非终态下会被拒绝，不影响)
ros2 service call /smartcar/task/reset std_srvs/srv/Trigger '{}'
# 预期: success=False, message='reset_not_terminal' ← 正常

# 3. 启动
ros2 service call /smartcar/task/start std_srvs/srv/Trigger '{}'
# 预期: success=True, message='mission started'
```

## 参数验证清单（启动后发车前）

```bash
# motion model 必须是 DUBIN
grep motion_model_for_search "$FIXED_YAML"

# allow_reversing 必须是 true
grep allow_reversing "$FIXED_YAML"

# goal_checker 必须存在
grep current_goal_checker "$FIXED_YAML"

# min_velocity 必须有负值
grep 'min_velocity:' "$FIXED_YAML"
```

---

## 问题 5: `direction_activate:raw_odom_not_stopped`

**症状**:
```
[task_node] navigation_failed:direction_activate:raw_odom_not_stopped
[bt_navigator] Goal canceled
```
任务启动后第一个导航段立即失败。

**根因**: `direction_guard` 在 `activate()` 时调用 `raw_odom_ready()` 检查：
1. 最近收到原始 odom 消息（`raw_odom_timeout_sec` 内）
2. 车辆已连续停止 ≥ `stop_settle_sec`

原始配置 `raw_odom_timeout_sec=0.25` 和 `stop_settle_sec=0.25` 在 RDK X5 的 20Hz odom + 偶发丢帧（0.66% drop）条件下过于激进。prepare 和 activate 两步服务调用间隙中，若恰好错过 odom 帧（50ms间隔），`last_odom_at_` 陈旧可能导致检查失败。

**修复** (已应用到 `config/direction_guard.yaml`):
```yaml
raw_odom_timeout_sec: 0.50  # 从 0.25 放宽
stop_settle_sec: 0.15       # 从 0.25 缩短（降低对"已停止时长"的要求）
```

**影响文件**: `src/smartcar_safety/config/direction_guard.yaml`

---

## 问题 6: 反向导航 `curvature_exceeded`

**症状**:
```
[bt_navigator] Rejected reverse path: curvature_exceeded at segment 1
```
连续 12+ 次拒绝后 `Goal failed`，任务 FAILED。

**上下文**: 正向第一段**实际行驶成功**（P→2.93, 0.87），倒车段（QR→VLM）被曲率检查阻挡。

**根因**: `minimum_turning_radius=0.55` + `curvature_tolerance=0.20` 计算出的 `maximum_curvature = 1/0.55 + 0.20 ≈ 2.018 rad/m`。反向路径是 DUBIN planner 规划 → 朝向翻转 π → 曲率验证的产物。翻转后的路径第一段（连接起点）角变化/段长比超限。

**临时诊断**（已应用但不是 release 修复）:
```yaml
# nav2_params.yaml
minimum_turning_radius: 0.30  # 从 0.55 临时降低

# navigate_to_pose_reverse_w_replanning_and_recovery.xml
minimum_turning_radius="0.30"   # 从 0.55 临时降低
curvature_tolerance="0.50"      # 从 0.20 临时放宽
```
新 `maximum_curvature = 1/0.30 + 0.50 ≈ 3.833 rad/m`，对应约 `0.261 m` 的等效曲率半径。

**复核（2026-07-25）**: 该参数组合破坏 `0.55 m` 合同；`R=0.30 m` 需要约 `0.562 rad` 转角，超过 safety 配置的 `0.45 rad` 上限，不能作为可驾驶配置。`curvature_exceeded` 在路径交给 RPP 和方向门之前产生，因此与 `angular.z` 补偿无关。应恢复一致的可执行半径，并保存首段路径以修正离散曲率验证，而不是继续放宽阈值。

**影响文件**: `src/smartcar_nav2/config/nav2_params.yaml`, `.../behavior_trees/navigate_to_pose_reverse_w_replanning_and_recovery.xml`

---

## 问题 7: planner_server 首次超时（RDK X5 性能）

**症状**:
```
[bt_navigator] Timed out while waiting for action server to acknowledge goal
request for compute_path_to_pose
[BehaviorTreeEngine]: Behavior Tree tick rate 50.00 was exceeded!
```
首次规划失败后自动重试，第二次成功到达目标。

**根因**: RDK X5 ARM64 算力有限，`planner_server` 首次规划 + 代价地图初始化时偶发超时。BT 的 6 次重试 + 清代价地图恢复机制使重试最终成功。

**缓解**: 当前 BT 已配置 `RecoveryNode number_of_retries="6"`；如需进一步降低超时概率可增大 `max_planning_time`（当前 2.0s）。

---

## 问题 8: 倒车时方向打反（现场补偿已验证）

**症状**: 倒车段 (2.94, 0.89) → (2.09, 1.83) 启动后车辆移动到 (3.06, 0.30)，即向目标**相反方向**运动。

**现场处理**: 用户直接观察到倒车转向符号与目标相反，在 `direction_guard` 的 reverse 租约路径翻转 `angular.z` 后重新实测，首个倒车 goal 转向方向正确并返回成功。该补偿是当前实体硬件链的确认结果，应保留。

**边界**: 该验证证明当前硬件链需要转向符号补偿，不证明绕圈来自转向，也不影响更早发生的路径曲率校验。首个 goal 的绕圈另由 DUBIN 起点端姿、QR 到达容差和真实可执行转弯半径共同排查；当前没有 Reeds-Shepp planner 或可通过的 cusp。

**后续必要步骤**:
1. 为现有补偿补齐正/倒车 x 左/右四象限 C++ 和命令链回归测试
2. 车轮离地记录 `/cmd_vel_candidate`、`/cmd_vel`、`/ackermann_cmd` 与实物舵向
3. 低速地面只跑首个 reverse goal，并录制实际起点 yaw、规划 Path、odom 和 action UUID
4. 完成转向比例、偏置与真实最小转弯半径标定后再开启对应运动门禁

**禁止的替代方案**: 不得通过删除已验证的 `angular.z` 补偿、切换 `REEDS_SHEPP` 或继续降低规划半径来掩盖绕圈。

---

## 问题 9: QR 成功端姿使首个倒车 DUBIN goal 绕行

**症状**: 车辆已能按正确转向完成首个 reverse goal，但从 QR 切换到倒车后会走一段明显的大圆弧，接近目标后才报告成功并进入下一点。

**根因**: QR 名义点为 `(3.13, 0.98, 30 deg)`，现场成功终态约 `(2.94, 0.89)`，位置误差约 `0.206 m`，仍在旧正向 checker 的 `0.25 m / 0.50 rad` 容差内。反向 BT 使用该实际 TF 作为虚拟 DUBIN 起点；在 `R=0.55 m` 下，成功端姿跨过短 CSC 路径的可行边界时，最短解会切换成 CCC 绕行。planner 仍是 DUBIN，严格反向校验也会拒绝 cusp，因此不能归因于 Reeds-Shepp。

**软件修正（2026-07-25）**:

1. 航点 schema 新增 `goal_profile`，QR 方向切换点标为 `precise`。
2. 新增正向精确 BT，显式选择 `precise_goal_checker`；使用 `xy=0.12 m`、`yaw=0.15 rad`、`stateful=false`。
3. `b_corridor_enter` 航向从 `-60 deg` 改为 `-70 deg`，使其与到 `b_corridor_out` 的下一段倒车切线一致。
4. Smac `tolerance` 设为 `0.0`，禁止用附近路径端点替代原始 QR goal；ControllerServer 的 checker 目标是 `path.poses.back()`，非零 planner 回退会绕过精确包络。
5. `minimum_turning_radius` 恢复为 `0.55 m`，反向 `curvature_tolerance` 恢复为 `0.20`；保留实车确认的倒车 `angular.z` 补偿。

**离线证据**: 名义首段倒车最短路径约 `1.57 m`。对新的 QR 成功位置圆盘和航向区间进行离线采样，未再出现 CCC 最短解，最长约 `1.80 m`；相关内公切线条件在采样包络边界仍保留约 `0.011 m` 裕量。

**验证边界**: 上述结果尚未经过新配置的实体复测。下一次只运行 QR 到首个 reverse goal，保存 QR 成功瞬间 `(x,y,yaw)`、planner Path、action UUID 和 `/odom`；不得直接无人看守跑完整路线。
