# AGENTS.md

本文件定义本仓库的执行约束。低频操作、历史证据和详细命令见
[`docs/README.md`](docs/README.md)。

## 硬规则

- 禁止未经用户明确授权启动实体相机、发布非零速度、进行实车运动、同步到 RDK 或修改运动门禁。
- 实车语义路线 `default_waypoints.yaml` 与仿真纯导航路线 `nav_only.yaml` 必须使用同一张路线：航点 ID、顺序、坐标、姿态、方向、profile 和 `planning_segments` 必须一致。两者仅可在 A/C1 的任务类型上不同：`qr`/`vlm` 与 `nav`。
- 禁止助手、脚本、预检、仿真结果或任何自动化逻辑擅自新增、删除、移动、重命名航点，或改动姿态、方向、profile、分段和经过约束。不得为绕过规划或测试失败添加连接点。
- 任何航点修改必须先在 RViz 航点编辑器中向用户展示拟议路线和点位，并取得用户对该次具体修改的明确同意。获准后才可同步两份 YAML，并运行航点同步合同测试。
- 仿真与实车的每一段都只能由 Nav2 基于实时 costmap 规划。禁止强制路径、连接器、手工弧线、路径后处理、点位专用绕障、控制器补偿或任何人为路径引导。
- 经过点（`task: via`、`through_ids`）只作为 Nav2 `ComputePathThroughPoses` 的有序路径规划约束，明确拒绝参与到达判定、停车、任务完成或分段终点。`via` 不得作为 `end_id`，`FollowPath` 的到达检测只能针对该段非 `via` 的最终端点。每段正常情况下只允许在开始时基于实时 costmap 计算一次完整路径，随后单次 `FollowPath` 跟踪该路径；跟踪期间 `RemovePassedGoals` 只持久清理已通过的队首规划约束，不参与到达判定。唯一例外是路径跟踪失败或不可达后的原生 Nav2 脱困：每段最多 3 次直线 `BackUp`（每次 `0.20 m`、速度不超过 `0.15 m/s`），每次随后只用剩余规划约束重算当前段；它不构成到达、停车或任务完成，也不重投已清理的 `via`。`RemovePassedGoals` 的半径仅是 Nav2 通用队首裁剪规则，不保证拓扑意义上的通过。
- 到达、碰撞、速度、曲率、规划和恢复阈值只能使用 Nav2 的通用配置。禁止按分段、航点或障碍设置专用阈值或放宽阈值掩盖失败。
- 运动链必须经过 `velocity_smoother -> direction_guard -> smartcar_safety -> /ackermann_cmd`。不得绕过方向门或 safety，禁止直接发布底盘 Twist/Ackermann。
- 导航固定使用已标定的 IMU+轮式里程计；EKF 是 `odom_combined -> base_footprint` 的唯一 TF owner。深度点云仅用于 Nav2 obstacle/inflation costmap，不做 SLAM 或静态地图定位。
- Aurora 930 深度相机外参是已确认的固定结构约束：位于前轮正上方、离地 `0.15 m`、水平朝前。`base_footprint` 是后轴，等效 `base_footprint -> depth_camera_link_1` 平移为 `[0.144, 0.0, 0.15]`；现有分段 TF 与驱动坐标轴转换均属于该固定定义。不得将深度相机外参或 IMU+轮式里程计再次列为待标定项、运动门禁或进度阻塞项。允许检查点云、TF 和 costmap 的运行健康度，但不得将这些检查表述为外参重标定。只有用户明确说明物理安装已改变时，才可修改该外参或提出重新验证要求。
- RDK 为 8 核平台。禁止以单个进程的 CPU 百分比作为性能瓶颈、深度避障失败或进度阻塞的结论；性能判断必须按全机核数归一化，并以点云采集时间戳连续性、端到端时延、丢帧、costmap 更新与障碍物实际标记作为证据。除非这些证据明确显示资源耗尽，不得将 CPU 占用本身作为阻塞理由。
- 当前全正向树禁止 Spin、Wait 和 `DriveOnHeading`。每个导航动作最多允许 3 次受限的原生 Nav2 `BackUp` 脱困（每次 `0.20 m`、不超过 `0.15 m/s`），不允许反向航点、反向规划或反向到达判定；经过点树在跟踪和每次脱困后都用 `RemovePassedGoals` 持久清理已通过的队首规划约束，`REVERSE` 租约不是任务导航路径的一部分。方向门拒绝超限或带转向的回退命令并记录警告，但不得因此锁存停车或撤销前进任务许可。
- 已确认 LiDAR 无障碍实测证明当前全正向纯导航路线的基础通行性。不得因 YAML 的 `calibrated: false`、默认运动门禁为 `false` 或缺少重复文档记录而将该路线再表述为“未验收”，也不得以此拒绝已获本次明确授权的受看护深度测试。
- 默认运动门禁只用于防止无人自动发车。对官方受看护深度短测，收到本次实体相机与非零运动的明确授权、确认急停和现场摆位后，应直接使用受看护入口的一次性运行确认；不得要求重复航点、固定深度外参或 IMU+轮式里程计标定。深度模式剩余的技术验证仅为动态障碍物在 costmap 的标记/清障及 Nav2 实时重规划效果。

## 当前路线

- 路线全正向、保持 `calibrated: false`：P→A、A→`via_1`→`via_2`→`via_3`→`via_6`→C1、C1→`via_4`→`via_5`→`via_7`→P。
- A (`a_task_observe`) 与 C1 (`c_corner_1`) 使用 `precise` profile；P (`p_finish`) 使用 `standard` profile。
- LiDAR 无障碍纯导航已完成当前路线的现场基础通行性验证；这不替代 QR/VLM 语义任务验收，也不替代深度相机的动态障碍感知验证。
- 发车时必须人工将车辆放在 P 原点，车头朝 `+X`；任务 reset 不能替代物理复位。

## 工作方式

- 先读取相关代码、当前 YAML 和 [`docs/README.md`](docs/README.md) 中对应的文档。
- 本地 `src/` 与 `config/` 是权威源。`push` 默认使用 `--delete`，RDK 有本地修改时必须先 pull 或备份。
- 默认验证命令：`python3 -m unittest discover -s tests -v`。详细仿真、RDK 构建、急停与恢复步骤见部署手册。
- `nav2_params_fixed.yaml` 是构建产物，不得手工编辑；修改 `nav2_params.yaml` 后通过构建生成。
- 未经明确授权，不启动仿真路线。即使启动仿真，`run_route:=true` 才能产生非零模型运动。

## 文档索引

- [航点编辑与授权规则](docs/deployment/waypoint-editor.md)
- [本机 Gazebo 仿真](docs/deployment/local-simulation.md)
- [RDK 部署、门禁与现场流程](docs/deployment/rdk-environment-setup.md)
- [工具与当前操作索引](docs/README.md)

## 提交

- 提交主题使用 Angular 规范和中文。
- 不添加 `Co-Authored-By` 或其他协作者元数据。
