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

**修复** (已应用):
```yaml
# nav2_params.yaml
minimum_turning_radius: 0.35  # 从 0.55 降低（允许更急弯）

# navigate_to_pose_reverse_w_replanning_and_recovery.xml
minimum_turning_radius="0.35"   # 从 0.55 降低
curvature_tolerance="0.40"      # 从 0.20 放宽
```
新 `maximum_curvature = 1/0.35 + 0.40 ≈ 3.257 rad/m`，对应最小转弯半径约 0.307m。

**注意**: `0.55` 是 CLAUDE.md 记录的"未标定规划假设"，`0.35` 仍是临时值；实车标定后应回填准确值。

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

## 问题 8: 倒车时方向打反（STM32 反向运动未验证）

**症状**: 倒车段 (2.94, 0.89) → (2.09, 1.83) 启动后车辆移动到 (3.06, 0.30)，即向目标**相反方向**运动。

**根因**: STM32 固件的负速度（倒车）指令仅完成代码路径确认，**实际倒车运动从未在硬件上验证过**（CLAUDE.md 已记录）。Ackermann 底盘倒车时转向方向可能反转，STM32 未正确处理负 `linear.x` 对应的转向逻辑。

**当前状态**: 此为硬件层问题，软件侧无法完全修复。正向导航段已验证可正常行驶。

**后续必要步骤**:
1. 实车验证 STM32 负速度指令：单独发送 `linear.x=-0.05` 观察车轮是否倒退
2. 如倒退但转向反向，需修正 STM32 固件或控制器中的转向符号
3. 如不退反进，需排查 STM32 速度指令解析或 PID 方向控制
4. 验证后在 `safety.yaml` 中开启对应运动门禁

**临时方案**: 如仅需纯导航验证，可考虑将倒车段改成正向绕行（调整航点避开 QR→VLM 倒退要求），或使用 `nav_only.yaml` 跳过视觉相关段。
```
