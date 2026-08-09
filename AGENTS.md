# AGENTS.md

本文件定义本仓库的执行约束。低频操作、历史证据和详细命令见
[`docs/README.md`](docs/README.md)。

## 硬规则

- 禁止未经用户明确授权启动实体相机、发布非零速度、进行实车运动、同步到 RDK 或修改运动门禁。
- 实车语义路线 `default_waypoints.yaml` 与仿真纯导航路线 `nav_only.yaml` 必须使用同一张路线：航点 ID、顺序、坐标、姿态、方向、profile 和 `planning_segments` 必须一致。两者仅可在 A/C1 的任务类型上不同：`qr`/`vlm` 与 `nav`。
- 禁止助手、脚本、预检、仿真结果或任何自动化逻辑擅自新增、删除、移动、重命名航点，或改动姿态、方向、profile、分段和经过约束。不得为绕过规划或测试失败添加连接点。
- 任何航点修改必须先在 RViz 航点编辑器中向用户展示拟议路线和点位，并取得用户对该次具体修改的明确同意。获准后才可同步两份 YAML，并运行航点同步合同测试。
- 仿真与实车的每一段都只能由 Nav2 基于实时 costmap 规划。禁止强制路径、连接器、手工弧线、路径后处理、点位专用绕障、控制器补偿或任何人为路径引导。
- 到达、碰撞、速度、曲率、规划和恢复阈值只能使用 Nav2 的通用配置。禁止按分段、航点或障碍设置专用阈值或放宽阈值掩盖失败。
- 运动链必须经过 `velocity_smoother -> direction_guard -> smartcar_safety -> /ackermann_cmd`。不得绕过方向门或 safety，禁止直接发布底盘 Twist/Ackermann。
- LiDAR 用于连续扫描匹配里程计和 Nav2 obstacle/inflation costmap，不做 SLAM 或静态地图定位；EKF 是 `odom_combined -> base_footprint` 的唯一 TF owner。
- 当前全正向树禁止 Spin、Wait、`BackUp` 和 `DriveOnHeading`。反向基础设施不是当前路线验收依据。

## 当前路线

- 路线全正向、保持 `calibrated: false`：P→A、A→`via_1`→`via_2`→`via_3`→C1、C1→`via_4`→`via_5`→P。
- A (`a_task_observe`) 与 C1 (`c_corner_1`) 使用 `precise` profile；P (`p_finish`) 使用 `standard` profile。
- 当前路线尚未完成 Gazebo、RDK 或实体车辆验收。历史记录不得作为当前路线通过证据。
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
