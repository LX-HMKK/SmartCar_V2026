# 文档与架构

本目录记录当前操作规则和入口；实现与参数的权威源是本机 `src/`、`config/`、根目录
[`README.md`](../README.md) 与 [`AGENTS.md`](../AGENTS.md)。部署记录不替代当前代码或现场验收。

## 先读什么

- [航点编辑与授权](deployment/waypoint-editor.md)：航点展示、授权与双 YAML 同步规则。
- [本机 Gazebo 仿真](deployment/local-simulation.md)：纯导航仿真与结果验证。
- [RDK 部署与现场流程](deployment/rdk-environment-setup.md)：同步、构建、急停和实体设备检查。
- [场地与诊断工具](reference/field-tools.md)：场地参考、航点编辑和独立诊断入口。

当前路线始终为全正向 P→A→`via_1`→`via_2`→`via_3`→C1→`via_4`→`via_5`→P。
实车 `default_waypoints.yaml` 与仿真 `nav_only.yaml` 共享全部几何和分段，仅 A/C1 的任务类型不同。
任何航点修改都必须先在 RViz 展示并取得用户同意。

## 模块分层

```text
src/
  smartcar_common, smartcar_interfaces   公共 QoS、消息与服务契约
  smartcar_bringup                       全系统组合、TF 与运行开关
  smartcar_task                          航点任务状态机与 Nav2 action 编排
  smartcar_nav2                          Smac Hybrid、原生行为树、costmap 与路线
  smartcar_safety                        速度平滑后的方向门、安全看门狗与 Ackermann 输出
  smartcar_vision, smartcar_speech       Aurora/QR/VLM 与可选语音服务
  origincar                              底盘串口、轮式里程计、IMU、EKF 与可选 LiDAR/RF2O
  smartcar_tools                         航点编辑、场地参考、短测与独立诊断
  smartcar_sim                           Gazebo、仿真传感器适配与纯导航验证
  third_party/rf2o_laser_odometry        可选 scan-to-scan 里程计
```

`smartcar_bringup` 只负责组装；路线和规划参数由 `smartcar_task`、`smartcar_nav2` 所有；
`smartcar_safety` 是唯一的最终运动出口保护层。`smartcar_tools` 与 `smartcar_sim` 不属于实体
默认运行链，分别用于受控诊断和本机仿真。

## 运行数据流

### 运动与规划

```text
smartcar_task -> Nav2 -> /cmd_vel_nav -> velocity_smoother
-> direction_guard -> smartcar_safety -> /ackermann_cmd -> 底盘
```

每一段路线均由 Nav2 根据实时 obstacle/inflation costmap 规划。不得绕过方向门或 safety，
也不得以手工路径、点位专用阈值或控制器补偿替代规划。

### 定位与感知

```text
轮式 /odom + /imu/data_raw -> EKF -> /odom_combined -> base_footprint

Aurora /aurora/points2 -> 深度 relay -> /smartcar/depth/points
                                      -> local/global costmap + safety
```

EKF 是 `odom_combined -> base_footprint` 的唯一 TF owner。深度点云仅作为 Nav2 障碍物数据，
不参与 SLAM 或静态地图定位。LiDAR 模式可提供 costmap 观测与连续扫描匹配里程计，但不做 SLAM。

### 实车与仿真边界

```text
本机 Gazebo: smartcar_sim -> 仿真传感器/车辆 -> Nav2
RDK 实车:    smartcar_system -> 底盘、EKF、传感器、Nav2、safety
```

仿真和 RDK 共享路线几何，但仿真结果不能替代当前实体验收。`nav2_params_fixed.yaml` 是构建产物；
修改 Nav2 参数只能编辑源文件后通过构建生成。实体相机、RDK 同步与任何非零运动都需要该次明确授权。
