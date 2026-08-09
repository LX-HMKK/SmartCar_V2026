# 航点编辑与授权

两份路线文件必须保持一致：

- `src/smartcar_nav2/config/waypoints/default_waypoints.yaml`
- `src/smartcar_nav2/config/waypoints/nav_only.yaml`

允许的唯一差异是 A/C1 的任务类型：实车为 `qr`/`vlm`，仿真为 `nav`。ID、顺序、坐标、姿态、
方向、profile、`planning_segments` 和 `through_ids` 必须相同。

禁止自动化或助手新增、删除、移动、重命名航点，或为绕障添加连接点。需要改点时，先用 RViz
航点编辑器展示拟议路线并取得用户对本次改动的明确同意；获准后才同步两份 YAML。

RDK 无运动编辑器：

```bash
export DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0
ros2 launch smartcar_tools waypoint_editor.launch.py start_safety:=true
```

只查看航点：

```bash
ros2 run smartcar_tools waypoint_viz --ros-args -p waypoints_file:=<yaml>
```

编辑器和离线预检不产生 Nav2 路径，也不授权实车运动。运行时路径只能由 Nav2 根据实时 costmap
规划；禁止连接器、手工弧线、路径后处理、控制器补偿和航点/障碍专用阈值。
