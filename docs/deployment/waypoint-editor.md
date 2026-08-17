# 航点编辑与授权

两份路线文件必须保持一致：

- `src/smartcar_nav2/config/waypoints/default_waypoints.yaml`
- `src/smartcar_nav2/config/waypoints/nav_only.yaml`

允许的唯一差异是 A/C1 的任务类型：实车为 `qr`/`vlm`，仿真为 `nav`。ID、顺序、坐标、姿态、
方向、profile、`planning_segments` 和 `through_ids` 必须相同。

禁止自动化或助手新增、删除、移动、重命名航点，或为绕障添加连接点。需要改点时，先用 RViz
航点编辑器展示拟议路线并取得用户对本次改动的明确同意；获准后才同步两份 YAML。

`p_start` 始终固定在 P 原点并朝 `+X`。`p_finish` 是无朝向的位置约束终点，可由已获本次
授权的操作者在编辑器中调整，以补偿重复出现的返航终点偏差；调整后必须把同一坐标同步到两份
YAML 并运行航点同步合同测试。

RDK 无运动编辑器：

```bash
export DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0
ros2 launch smartcar_tools waypoint_editor.launch.py start_safety:=true
```

只查看航点：

```bash
ros2 run smartcar_tools waypoint_viz --ros-args -p waypoints_file:=<yaml>
```

编辑器不产生 Nav2 路径，也不授权实车运动。运行时路径只能由 Nav2 根据实时 costmap
规划；禁止连接器、手工弧线、离线或固定路径后处理、控制器补偿和航点/障碍专用阈值。本次
获准的唯一平滑阶段是原生 `ConstrainedSmoother`：它只处理当前 `ComputePathThroughPoses`
结果（先去除连续重合 pose），使用实时 costmap 和完整 footprint 碰撞检查，输出仅供该次
`FollowPath`，不保存、不复用，也不改动 YAML 或航点。

`through_ids` 中的 `via` 点只在规划段开始时传给 Nav2 的 `ComputePathThroughPoses` 作为有序路径
规划约束，随后在实时 costmap 上经上述原生平滑后，只用一次 `FollowPath` 跟踪该段当前结果。平滑可在
碰撞约束下自然切过 `via` 边缘，但不改变它们的规划约束或到达语义。它们不构成到达判定或停车点。每个
规划段只能在非 `via` 的 `end_id` 完成，`FollowPath` 的到达检测只针对这个段末端。

每个导航动作唯一的失败恢复例外是路径跟踪失败或不可达时最多 6 次原生 Nav2 直线 `BackUp`：每次距离 `0.20 m`、
速度上限 `0.25 m/s`，然后以实时 costmap 只对剩余规划约束重算当前动作。对有 `via` 约束的动作，
`RemovePassedGoals` 在跟踪期间及每次回退后持久清除已通过的队首规划约束；该节点不参与到达、停车、任务完成或
航点判定，且其半径只表示 Nav2 的队首裁剪规则，并不保证拓扑意义上的通过。不得反向规划或重投已清理的 `via` 点。方向门若收到超限或带转向的回退命令，只拒绝该命令并记录警告，保持前进
任务许可，不锁存停车。
