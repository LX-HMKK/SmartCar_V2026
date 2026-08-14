# 场地与诊断工具

所有工具默认只读或静态操作，不授权车辆运动。

```bash
# 发布航点 Marker
ros2 run smartcar_tools waypoint_viz --ros-args -p waypoints_file:=<yaml>

# 统计 /odom、/odom_combined 里程计话题
ros2 run smartcar_tools odom_diag --ros-args -p duration_sec:=30.0

# 记录电压
ros2 run smartcar_tools voltage_monitor
ros2 topic echo /PowerVoltage --once
```

规则场地参考层只用于查看和量点，不发布定位地图，不参与 SLAM 或静态地图定位。深度相机是唯一
障碍与视觉感知来源，其深度模块经 `/smartcar/depth/scan` 供 Nav2 obstacle/inflation costmap 使用；
EKF 是 `odom_combined -> base_footprint` 的唯一 TF owner。
