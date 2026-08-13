# 本机 Gazebo 仿真

仅支持 Ubuntu 22.04 本机 Ignition Gazebo，不使用 WSL。默认入口不会运行路线：

```bash
cd /home/zyh/SmartCar_V2026
bash scripts/local_sim.sh --headless --rviz
```

只有显式 `run_route:=true` 才会向 Gazebo 模型发送非零速度。未经用户授权不要使用该选项。
仿真使用 `nav_only.yaml`；它和实车 YAML 是同一张路线，任务类型除外。

RViz 查看 `/plan` 和 `/transformed_global_plan`。它们是 Nav2 自己的规划和跟踪话题；有 `via` 的
段会在当前 `ComputePathThroughPoses` 结果上删除连续重合 pose，并由原生
`ConstrainedSmoother` 根据实时 costmap 和完整 footprint 碰撞检查后交给 `FollowPath`。结果不会
保存或作为固定路线复用。航点 Marker 仅是参考约束，不是 Nav2 路径。

完成后使用：

```bash
bash src/smartcar_sim/scripts/sim_cleanup.sh
```

仿真结果不能作为 RDK 或实车验收证据，也不能解除任何运动门禁。
