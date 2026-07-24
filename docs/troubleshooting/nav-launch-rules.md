# 纯导航任务启动规则

> 提取自 2026-07-24 无线连接导航测试的实操经验，用于指导脚本维护和故障排查。

## 1. 启动脚本编写规则

### 1.1 TROS 环境 source 必须包裹 `set +u`

RDK 上任何带 `set -euo pipefail`（含 `set -u`）的脚本，在 `source /root/source_env.sh`（间接 source `/opt/tros/humble/setup.bash`）时必须临时关闭 nounset：

```bash
set +u
source "$SOURCE_ENV"
set -u
```

TROS Humble 的 `setup.bash` 引用了未绑定的 `AMENT_TRACE_SETUP_FILES`。不处理会导致脚本第一行 source 后就退出。

### 1.2 清理脚本不得 kill 自身调用者

`/usr/local/bin/ros_cleanup` 的 `pkill` 列表规则：
- **可以 kill**：ROS 节点可执行文件名（`controller_server`、`safety_node` 等）
- **不得 kill**：启动/测试脚本名（`nav_test`、`full_test` 等）——这些脚本会调用 ros_cleanup，pkill 会把调用者自身杀掉
- 如需清理旧脚本实例，在调用者脚本顶部自行 pkill，再 exec

### 1.3 构建后必须验证关键参数

`colcon build` 成功后，必须从 `nav2_params_fixed.yaml` 验证以下参数：
- `motion_model_for_search: DUBIN`
- `allow_reversing: true`
- `current_goal_checker: "goal_checker"`
- `min_velocity` 含负值（倒车）

### 1.4 发车前检查清单

```bash
# 系统状态
grep "Managed nodes are active" /tmp/bringup.log  # Nav2 生命周期全部 active

# 关键节点存在
ros2 node list | grep -E "safety_node|task_node|controller_server|direction_guard"

# 服务可用
ros2 service list | grep -E "smartcar/(safety/emergency_stop|task/start)"
```

## 2. BT XML 编写规则

### 2.1 `FollowPath` 必须显式指定 `goal_checker_id`

Nav2 的 `goal_checker_selector` BT 节点负责动态设置 `current_goal_checker` 参数。如果 BT 树**未包含**该 selector 节点，则必须直接在 `FollowPath` 动作上指定：

```xml
<!-- Forward -->
<FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="goal_checker"/>

<!-- Reverse -->
<FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="reverse_goal_checker"/>
```

`nav2_params.yaml` 中的 `current_goal_checker` 参数是 selector 的输出目标，不是 controller_server 直接读取的静态默认值。

### 2.2 反向 BT 必须指定 `goal_checker_id="reverse_goal_checker"`

反向导航使用更严格的容忍度（`xy_goal_tolerance: 0.12, yaw_goal_tolerance: 0.25`），通过 `reverse_goal_checker` 插件配置。

## 3. 远程操作规则

### 3.1 SSH 操作规范

- 优先无线连接（`172.16.24.x`），有线 `192.168.128.10` 为备用
- 所有 `ros2` CLI 命令前必须 `source /root/source_env.sh`
- 后台启动使用 `nohup` + 输出重定向，不要依赖 SSH 会话保持
- `ros2 service call` 可能阻塞——`reset_not_terminal` 是预期响应不是错误

### 3.2 监控最佳实践

- 日志追踪优先于 topic echo（`tail -f /tmp/bringup.log | grep ...` 比 `ros2 topic echo` 更可靠）
- Monitor 工具设置 600s timeout，任务完成后手动停止
- 关键 grep 关键词：`NavigateToPose`、`reached`、`ERROR`、`FAILED`、`reverse`、`navigation_status`

### 3.3 应急操作

**紧急停车（按优先级）**：
1. `ros2 service call /smartcar/safety/emergency_stop std_srvs/srv/SetBool '{data: true}'`
2. 若 CLI 卡死：`pkill -9 -f "ros2 launch"`（STM32 超时自动发停止指令）
3. 物理急停按钮

**完全清理重建**：
```bash
bash /usr/local/bin/ros_cleanup
rm -f /tmp/nav_test_output.log /tmp/bringup.log
nohup bash /root/nav_test.sh > /tmp/nav_test_output.log 2>&1 &
```

## 4. 已知陷阱速查

| 症状 | 根因 | 修复 |
|------|------|------|
| 脚本首行就退出 | TROS `unbound variable` | `set +u` 包裹 source |
| cleanup 后脚本消失 | ros_cleanup kill 自己 | 从 kill 列表移除脚本名 |
| `goal_checker ... does not exist` | BT FollowPath 缺 goal_checker_id | 添加 `goal_checker_id="goal_checker"` |
| `ros2 service call` 挂起 | daemon 状态不一致 | `ros2 daemon stop && ros2 daemon start` |
| `reset_not_terminal` | 任务未在终态 | **正常**，直接 start 即可 |
