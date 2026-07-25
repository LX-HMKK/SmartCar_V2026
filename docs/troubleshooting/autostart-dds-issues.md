# 自动发车与 DDS 发现故障排查 (2026-07-25)

## 背景

车已摆好（P 区原点，车头 +X），通过无线 SSH 尝试发车。系统使用 TROS Humble + FastDDS 共享内存传输 (`hobot_shm`)。

## 问题 1: DDS 共享内存端口冲突

**现象**: `ros2 service call` 和 `rclpy` 脚本无法发现已有 ROS 服务/action。
`ros2 service list`（通过 daemon）可以列出服务，但 `ros2 service call` 一直阻塞在 "waiting for service to become available..."。

**错误信息**: `RTPS_TRANSPORT_SHM Error: Failed init_port fastrtps_port7427: open_and_lock_file failed`

**根因**: TROS 的 `shm_fastdds.xml` 配置所有进程使用相同的共享内存端口。首个 launch 进程占用 7427 端口后，后续 `ros2 service call` 或 `rclpy` 节点无法通过 SHM 发现已有节点。

**影响范围**: 所有从外部 SSH 触发的服务调用均失败。系统内部进程间通信正常（同 launch 的节点共享 DDS participant）。

**解决方案**:
- 方案 A（已采用）: `autostart_mission:=true`，任务节点定时器自动发车，绕过外部服务调用
- 方案 B: 设置 `FASTRTPS_DEFAULT_PROFILES_FILE` 指向禁用 SHM 的配置后再调用服务
- 方案 C: 每次重启前 `rm -f /dev/shm/fastrtps_port* /dev/shm/fastdds_port*`

## 问题 2: autostart 时序竞争

**现象**: `autostart_mission:=true` 下任务节点报 `navigation_server_unavailable`，任务未启动。

**根因**: 两条时间线存在竞争：
1. `task_node` 的 autostart 定时器（3s 间隔，最多 60 次重试）→ `_start_worker()` 启动 mission worker → mission worker 调用 `navigator.wait_ready()` 等待 Nav2 action server
2. Nav2 lifecycle manager 激活顺序: controller → planner → bt_navigator → velocity_smoother，全程需 ~15s

`wait_ready()` 调用 `ActionClient.wait_for_server()`，在 SHM 模式下可能不等待直接返回 False。

`server_wait_timeout_sec` ROS 参数默认为 **5.0 秒**（`task_node.py:1130`），不足以覆盖 Nav2 就绪时间。

**修复**:
1. `task_node.py`: `declare_parameter("server_wait_timeout_sec", 5.0)` → `30.0`
2. `task_node.py`: autostart 重试从一次性改为周期重试（3s 间隔，60 次，共 3 分钟）
3. `task_node.py`: `RosNavigator.wait_ready()` 改为 0.5s 轮询等待（取代单次阻塞 `wait_for_server`）
4. `task_node.py`: `RosDirectionGuard.wait_ready()` 同样改为轮询（见问题 3）
5. `mission.py`: `MissionConfig.server_wait_timeout_sec` dataclass 默认 5.0→30.0（**注意**: 此值被 ROS 参数覆盖，实际生效的是参数声明，此处仅保持一致性）

**教训**: `mission.py` 的 dataclass 默认值会被 `task_node.py` 的 ROS 参数声明覆盖 → 改默认值必须改 ROS 参数声明。

## 问题 3: 错误修改 wait_ready

`task_node.py` 有两个 `wait_ready()`：
- `RosDirectionGuard.wait_ready` (line ~192): 等待 direction_guard 服务就绪
- `RosNavigator.wait_ready` (line ~370): 等待 NavigateToPose action server 就绪

首次修改只改了 `RosDirectionGuard.wait_ready`，实际阻塞在 `RosNavigator.wait_ready`。

## 问题 4: Python 缓存

symlink install 下修改源码后需清理 `__pycache__`，否则旧 `.pyc` 可能被加载。
```bash
find /root/ros2_ws -name '__pycache__' -path '*smartcar_task*' -exec rm -rf {} +
```

## 修复后的 autostart 完整链路

1. 启动: `autostart_mission:=true` + 全部 motion gate 为 true + `safety_emergency_stop_on_start:=false`
2. `task_node` 加载航点后，3s 定时器开始重试 `_on_autostart()`
3. 每次重试调用 `_start_worker()` 检查 motion gates
4. 成功后启动 mission worker 线程
5. mission worker 调用 `navigator.wait_ready(30.0)`——轮询等待 Nav2 action server 最多 30 秒
6. Nav2 就绪后开始导航

## 发车命令（未来参考）

由于 SHM 端口冲突，外部服务调用不可靠。推荐使用 autostart：

```bash
ros2 launch smartcar_bringup smartcar_system.launch.py \
  use_base:=true use_lidar:=true use_nav:=true \
  use_camera:=false use_vision:=true use_task:=true \
  use_safety:=true autostart_mission:=true \
  safety_emergency_stop_on_start:=false \
  waypoints_calibrated:=true extrinsics_calibrated:=true \
  steering_calibrated:=true emergency_stop_ready:=true \
  operator_approved:=true
```

## 相关提交

- `41c8e68`: 删除 superpowers 文档 + gyro_z_bias 更新
- 后续提交: autostart 重试 + server_wait_timeout 修正
