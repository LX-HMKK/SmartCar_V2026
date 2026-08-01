# 2026-08-01 阿克曼反向短退恢复验收

## 结论

`AckermannReverseRetreat` 的软件行为已通过定向验收：在 Gazebo 中，反向规划候选在
清图后仍不可达时，车辆完成一次受完整足迹检查保护的 `0.15 m` 直线后退，并重新规划出
后续反向路径。该结论不等同于整条路线或实体底盘验收通过。

## 已验证行为

| 项目 | 证据 |
| --- | --- |
| 编译 | `colcon build --packages-select smartcar_nav2 --symlink-install --cmake-args -DBUILD_TESTING=ON` 通过 |
| 单元测试 | `test_costmap_footprint_sweep`、`test_ackermann_reverse_retreat_path` 通过 |
| 合同测试 | `src/smartcar_nav2/test/test_reverse_navigation_contracts.py` 与 `tests/test_nav2_contracts.py` 共 36 项通过 |
| 仿真启动 | Ignition Gazebo 6.18、Nav2、TF 和 RViz 正常启动 |
| 触发条件 | 第二段首次规划失败，清 global costmap 后仍无完整 free-heading chain |
| 物理恢复 | 日志 `1785564636.052` 记录 `retreating 0.150 m before one replan`；严格 recovery goal 于 `1785564643.453` 到达 |
| 指令方向 | 第二段聚合的 controller/cmd 采样均为负线速度，没有正向样本 |

恢复只在四个反向 BT 中存在。它经由 `FollowPath`、`ReverseRecovery`、速度平滑、方向门和
安全节点链路执行，不直接发布 Twist，也不启用 Nav2 `BackUp`、`DriveOnHeading`、Spin 或
behavior server。

## 安全判定

恢复路径是当前车姿到物理 `-X` 方向 `0.15 m` 的同航向直线。下发前必须同时满足：

- 全局和局部 raw costmap 可用、frame 正确、完整且未过期；
- 两张图上的 padded vehicle footprint 全路径扫掠均无 lethal overlap；
- 完整足迹不离开滚动 costmap；
- 本 action 尚未执行过物理短退。

任一条件失败都会拒绝运动。日志可分别报告 missing、wrong frame、malformed、stale、
footprint leaves costmap bounds 或 lethal footprint overlap。

## 当前路线边界

本轮 `/tmp/reverse_retreat_diagnostics.json` 记录：P→A 成功；第二段的恢复本身成功，但
`b_corridor_enter` 最终距离 `0.533 m`，超过 `transit_goal_checker` 的 `0.50 m`，因此该段
为 `contract_failed`，全路线结果为 `failed`。不得标记为全程完成。

离线几何扫描建议将非任务点 `b_corridor_enter` 从 `(1.10, 2.85)` 调整为 `(1.10, 2.65)`，
保持 `b_corridor_gate` `(1.80, 2.85)`。该候选可通过 A→B 和
`c_entry_west -> C1 -> C2 -> C3` 的 padded-OBB/keepout 预检，且对到达误差比 `(1.10, 2.75)`
更稳。该候选尚未写入路线，也尚未完成 Gazebo 全链验证。

P、QR、VLM 任务点的坐标与朝向保持保护；其他点位可以按几何预检调整位置，但不应为其
添加强制到达朝向。
