# 场地工具与纯导航测试

> 源文件：`CLAUDE.md` → 本文件（低频查阅，为 CLAUDE.md 减负）
> 最后更新：2026-07-29

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

## 纯导航全路线测试（无视觉）

RDK 上一键启动：

```bash
setsid bash /root/nav_test.sh > /tmp/nav_test_output.log 2>&1 &
# 监控任务状态: python3 /root/monitor_mission.py
# 日志: tail -f /tmp/bringup.log
```

脚本自动完成：全杀旧进程 → 构建 → 启动系统（无相机/无视觉）→ 等待 Nav2 生命周期就绪 → 开 RViz。它保持软件急停锁存且任务不自动开始；检查 RViz、物理急停和车轮离地条件后，才可人工 reset、解除急停并 start。首次测试只验证到 VLM 并立即停车，不得在两处正向航向风险修正前无人看守跑完整圈。

紧急停车：优先调用急停服务，CLI 异常时使用 `pkill -9 -f "ros2 launch"`。
