# 场地工具与纯导航测试

> 源文件：`CLAUDE.md` → 本文件（低频查阅，为 CLAUDE.md 减负）
> 最后更新：2026-08-03

## 路径分段航点编辑器

航点编辑器支持分段方向、起终点朝向、无朝向途经点、C 区禁区预检和仿真参数同步。
完整命令和操作流程见 [`../deployment/waypoint-editor.md`](../deployment/waypoint-editor.md)。

## 航点可视化

独立于导航栈的 MarkerArray 可视化：

```bash
ros2 run smartcar_tools waypoint_viz --ros-args -p waypoints_file:=/root/ros2_ws/install/smartcar_nav2/share/smartcar_nav2/config/waypoints/default_waypoints.yaml
# 发布 MarkerArray 到 /smartcar/waypoints/markers（latched），RViz 订阅即可查看
```

## 里程计诊断

排查 EKF/串口丢帧：

```bash
ros2 run smartcar_tools odom_diag --ros-args -p duration_sec:=30.0
# 输出 /odom、/odom_combined、/scan 速率和间隔统计，以及 EKF 诊断警告
```

## 电压监控

记录 `/PowerVoltage` 到日志，支持离线分析：

```bash
ros2 run smartcar_tools voltage_monitor
# 日志: /tmp/voltage_history.log（时间戳 + 电压值，10 万行自动轮转）
# 实时查看: tail -f /tmp/voltage_history.log
# 当前值:   ros2 topic echo /PowerVoltage --once
```

## 转向标定

这三项工具不会修改 `steering_calibrated` 或任何运动门禁。静态量角和地面圆周测试均为
实体执行器操作，必须由现场操作员完成急停、场地和车轮状态确认；运行期间不得同时执行任务
或导航动作。

静态量角只允许前轮离地。安全节点默认拒绝该接口，操作员需在确认车辆已停止、传感器健康且
软件急停已解除后，临时显式启用：

```bash
ros2 param set /safety_node allow_steering_calibration true
ros2 run smartcar_tools steering_hold --angle 0.30 --hold 12 --yes
ros2 param set /safety_node allow_steering_calibration false
```

`steering_hold` 不直接发布 `/ackermann_cmd`；它请求安全节点自身以零速度保持不超过
`0.70 rad` 的舵角。服务最长保持 15 秒，到期、急停、传感器门禁失败或收到非零运动指令时
都会回正。

圆周测试只在净空不小于 3 m 的地面区域进行，操作员全程手持物理急停。它申请前进方向
租约，并通过 `/cmd_vel_nav -> velocity_smoother -> direction_guard -> smartcar_safety`
发送命令，不会解除急停或绕过安全链：

```bash
ros2 run smartcar_tools steering_circle_drive \
  --angle 0.30 --speed 0.15 --duration 20 \
  --out /tmp/steering_circle_030.csv --yes
ros2 run smartcar_tools steering_circle_analyze \
  --csv /tmp/steering_circle_030.csv --wheelbase 0.189
```

圆周速度上限为 `0.15 m/s`，持续时间上限为 60 秒。中断、租约续期失败或服务不可用时，
工具先发布零速度并停止方向租约；离线分析器不依赖 ROS 或实体硬件。

## 纯导航全路线测试（无视觉）

RDK 上一键启动：

```bash
setsid bash /root/nav_test.sh > /tmp/nav_test_output.log 2>&1 &
# 日志: tail -f /tmp/bringup.log
```

脚本自动完成：清理旧进程 → 构建 → 启动系统（无相机/无视觉）→ 等待 Nav2 生命周期就绪 → 开 RViz。它保持软件急停锁存且任务不自动开始，也不会授予五项运动门禁；当前 YAML 为 `calibrated: false`，因此 `start` 会被拒绝。完成对应实测、现场标定并显式满足门禁后，才可 reset、解除急停并 start。任务运行上限与 Gazebo 同步为 `0.30 m/s`；实体首次复验仍须由现场人员分段监护 P→QR、QR→`via_2`→VLM、VLM→`via_1`→`via_3`→P。上文的 `0.15 m/s` 仅适用于转向圆周标定工具。

紧急时先使用物理急停；随后运行 `/usr/local/bin/ros_cleanup`（已部署时）或 `/root/ros2_ws/scripts/ros_cleanup.sh`。CLI 可用时再调用软件急停服务。
