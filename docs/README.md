# 文档索引

本目录只保留当前操作规则。代码、`README.md` 和 `AGENTS.md` 为权威来源。

- [航点编辑与授权](deployment/waypoint-editor.md)
- [本机 Gazebo 仿真](deployment/local-simulation.md)
- [RDK 部署与现场流程](deployment/rdk-environment-setup.md)
- [场地与诊断工具](reference/field-tools.md)

当前路线为全正向 P→A→`via_1`→`via_2`→`via_3`→C1→`via_4`→`via_5`→P。实车
`default_waypoints.yaml` 与仿真 `nav_only.yaml` 共享全部几何和分段，仅 A/C1 任务类型不同。
任何航点修改必须先在 RViz 展示并取得用户同意。

## 项目框架

```text
smartcar_task       任务状态机、语义航点和 Nav2 action
smartcar_vision     QR/VLM 服务
smartcar_nav2       Smac Hybrid、原生 Nav2 行为树、航点配置
smartcar_sim        Gazebo、RViz 和纯导航仿真
smartcar_tools      场地参考、航点编辑和诊断
smartcar_bringup    全系统 launch
origincar           底盘、IMU、EKF 和 YDLIDAR 驱动
smartcar_safety     方向门与安全节点
```

运行链为：`task -> Nav2 -> /cmd_vel_nav -> velocity_smoother -> direction_guard
-> smartcar_safety -> /ackermann_cmd`。Nav2 使用实时 obstacle/inflation costmap 规划；
LiDAR 可参与连续扫描匹配里程计，EKF 是 `odom_combined -> base_footprint` 的唯一 TF owner。
