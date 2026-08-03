# 航点分段编辑与离线几何预检操作手册

本文档说明 `smartcar_tools` 的 `waypoint_editor.launch.py`。它提供一个中文
Matplotlib 图形界面，用于编辑语义航点、规划分段和途经约束，并在保存前做离线
几何预检。

最后更新：2026-07-31。

## 1. 适用范围

编辑器只启动场地参考层和航点编辑界面；它不启动 Nav2、Gazebo、实体底盘、相机或
视觉任务。本机默认不启动 safety 节点或 RViz。RDK 上标定时必须显式开启
`start_safety:=true`，确保软件急停保持锁存。

编辑器显示的红色禁区是离线几何预检约束，不是 SLAM 地图，也不参与实车定位。灰色
航点连线只是任务航点参考约束，不是 Nav2 路径。预检使用与仿真共享的转弯半径和禁区
几何，但不读取实时机器人姿态、代价地图或控制器状态，也不等于 Nav2 在运行时生成的
`/plan` 或 `/local_plan`。实际仿真仍可能因代价地图、起始姿态、控制器或重规划而得到
不同轨迹。RViz 中黄色 C 区双环来自 `Official Field Reference`，同样不是预检或 Nav2
路径；应通过 Display 名称和 Topic 区分显示项。

保存航点会保持 `calibrated: false`，不能解除任何实车运动门禁。

## 2. 相关文件

| 文件 | 用途 |
|---|---|
| `src/smartcar_nav2/config/waypoints/default_waypoints.yaml` | 实车语义任务路线的源文件。 |
| `src/smartcar_nav2/config/waypoints/nav_only.yaml` | Gazebo 纯导航仿真路线，不触发二维码或 VLM。 |
| `src/smartcar_tools/config/routes/field_geometry.yaml` | 官方场地尺寸、区域和走廊参考。 |
| `src/smartcar_tools/config/routes/route_planning.yaml` | 编辑器预检、C 区禁区和仿真 Nav2 的共享调参文件。 |

不要直接编辑构建产物 `nav2_params_fixed.yaml`。它由 `nav2_params.yaml` 在构建时生成，
下一次 `colcon build` 会覆盖它。

## 3. 启动编辑器

### 本机离线编辑

在仓库根目录启动：

```bash
cd /home/zyh/SmartCar_V2026
bash scripts/local_waypoint_editor.sh
```

该入口隔离 Conda/Isaac 环境，并默认打开 Matplotlib 分段编辑器。它不启动 Gazebo、
Nav2、RViz、安全节点或任何实体驱动。

### 本机仿真路线编辑

仿真正在使用的路线应从专用入口打开，而不是沿用实车编辑器的默认文件：

```bash
bash scripts/local_waypoint_editor.sh
```

该入口默认解析与 `smartcar_sim/launch/sim.launch.py` 完全相同的已安装包路径
`smartcar_nav2/config/waypoints/nav_only.yaml`，且默认不启动 safety、Gazebo、Nav2、
RViz 或任何实体驱动。与已运行仿真并行查看时，追加
`use_sim_time:=true use_rviz:=true`。

关闭窗口或在启动终端按 `Ctrl+C` 退出。若旧 launch 残留了 `field_reference_node`，先在
原终端按 `Ctrl+C`，再重新启动；不要同时运行两份编辑器。

### RDK 上查看或标定

RDK 使用已部署的包路径。根据 HDMI 会话设置显示环境后启动，并显式锁存软件急停：

```bash
source ~/source_env.sh
export DISPLAY=:0 XAUTHORITY=/var/run/lightdm/root/:0
ros2 launch smartcar_tools waypoint_editor.launch.py \
  start_safety:=true use_rviz:=false
```

RDK 上编辑前先确认 YAML 的实际位置。日常从本机同步时，`push` 会镜像源文件并
可能覆盖 RDK 的本地修改；应先 pull 或备份。

### Launch 参数

| 参数 | 默认值 | 作用 |
|---|---|---|
| `waypoints_file` | `default_waypoints.yaml` | 要读取和保存的语义航点 YAML。 |
| `geometry_file` | `field_geometry.yaml` | 场地参考与坐标边界。 |
| `route_planning_file` | `route_planning.yaml` | 编辑器预检与禁区显示使用的共享配置。 |
| `use_segment_ui` | `true` | 使用本文档描述的中文分段编辑界面；`false` 为旧 RViz Interactive Marker 编辑器。 |
| `use_rviz` | `true` | 是否额外启动 RViz。专用本机入口默认传入 `false`。 |
| `start_safety` | `false` | 是否启动锁存急停的 safety 节点。本机离线编辑保持 `false`；RDK 标定必须显式设为 `true`。 |
| `use_sim_time` | `false` | 仅在已有仿真时钟的环境中设为 `true`。 |

## 4. 界面与鼠标操作

左侧是场地和航点，右侧是当前分段的编辑面板。灰色虚线为声明的航点参考约束，不是
Nav2 的 `/plan` 或 `/local_plan`。点击“几何预检”后显示的是每段的离线几何候选或
阻塞结果，同样不是 Nav2 路径。红色虚线框是仿真/预检共享的禁区。

| 操作 | 效果 |
|---|---|
| 左键点航点 | 选中航点，黄色环会持续保留，鼠标移到右侧面板不会取消选择。 |
| 左键拖动航点 | 修改位置；P 起点和最终 P 返回点的位置锁定。 |
| 在航点上滚轮 | 每格调整该点朝向 5 度。 |
| 在场地空白处滚轮 | 以光标为中心缩放。 |
| 右键拖动 | 平移视图。 |
| `R` / `Shift+R` | 将当前选中点朝向增减 15 度。 |
| `Ctrl+Z` | 撤销最近一次编辑。 |
| `Esc` | 取消“新增途经点”模式；没有新增模式时取消当前选中。 |
| `Ctrl+S` | 等价于“保存路线”，但仍要求预检通过。 |

比赛约束要求 P、QR 和 VLM 的位置保持不动。界面为调试提供了部分点位编辑能力，
不应把它当作改变这些语义任务点位的授权。

## 5. 推荐编辑流程

### 5.1 选择分段并设置端点

1. 在右上“路径分段”列表选择要编辑的段。
2. 使用“正向（前进）”或“倒向（倒车）”确定该段整体行驶方向。
3. 通过起点、终点文本框输入已有航点 ID，或点击相邻的“点选”后在场地上选择。
4. 在“起点朝向”和“终点朝向”输入角度。P 起点朝向固定为 `+X`。

每段必须与上一段首尾连续。QR、VLM 等语义任务点必须是分段终点；保存前会校验这些
合同。

### 5.2 新增无朝向途经点

新增点用于限制规划路线，不触发二维码或 VLM：

1. 先选择目标分段。
2. 点击“新增途经点”；按钮会变为“点击场地放置”。再次点击可取消。
3. 在场地空白处左键放置。不要覆盖已有航点。
4. 编辑器创建 `via_N`，自动加入当前分段的“按顺序途经点”列表。
5. 用“上移”“下移”调整它在该段内的顺序。

`via_N` 保存为 `task: via`，且不写 `pose.orientation`。它仍继承所属分段的前进或
倒车方向；“无朝向”只表示该点不强制终点航向。预检会按前后路线切线推导它的朝向。

当前 Gazebo `nav_only.yaml` 已在两段倒车中使用标准 `via` 约束。第二段允许
`reverse via ... -> c_corner_1` 组成一个连续动作：`c_corner_1` 的
`reverse_handoff` 必须保持为最后一个、锁定航向的目标，运行时会使用 reverse-locked
ThroughPoses 行为树。不能把 C1 作为中间点，不能把 `precise` 或其他特殊 profile 放在它前面，
也不能把该组合改为前进段。

不要为掩盖 P→A 控制器横向误差、不可达或规划绕行而增加经过点；只有规则或静态安全边界确实
要求通过某一位置时，才应添加一个经过点，并重新完成完整仿真验收。

### 5.3 将已有点作为途经点

1. 在场地上选中要使用的已有点。
2. 选择目标分段，点击“加入已有点”。
3. 如该点不应限制朝向，点击“设为无朝向”；如需恢复，点击“恢复路线朝向”。

分段的起点、终点，以及正在作为其他分段端点的点不能直接加入为途经点。只有当前
分段的途经点可以切换无朝向属性。

### 5.4 拆分、合并和移除

- “在选中途经点拆分”：把选中途经点提升为前一段终点和后一段起点。
- “合并/删除”：只允许合并行驶方向相同的相邻分段。
- “删除途经点”：从航点列表、画布和全部途经引用中删除当前所选点。分段端点不能直接删除；先合并或修改分段边界。

分段和途经点不是独立路线副本。保存时，编辑器会将分段顺序物化回同一份
`waypoints` 列表，并同时写入 `planning_segments`。

### 5.5 预检与保存

每次修改后，状态栏会提示“路线已修改”。完成编辑后：

1. 点击“几何预检”。
2. 查看所有分段：绿色为离线预检通过，红色为静态禁区或最小转弯半径下的预检阻塞。
3. 只有预检通过后，“保存路线”或 `Ctrl+S` 才会写入 YAML。

预检未执行或存在红段时，保存会被阻止。它只验证静态几何，不能作为 Nav2 运行时
可达或轨迹一致的结论。修复点位、朝向、途经顺序或分段边界后重新预检，并通过实际
仿真检查 `/plan` 和 `/local_plan`。

## 6. 共享路线参数与仿真

`src/smartcar_tools/config/routes/route_planning.yaml` 是手动调参入口：

| 字段 | 影响范围 |
|---|---|
| `minimum_turning_radius_m` | 编辑器和实车路线预检使用的保守物理半径，当前为 `0.22 m`（实测极限约 `0.20 m`）；仿真调参不会改写实车 `nav2_params.yaml`。 |
| `simulation_minimum_turning_radius_m` | 仅 Gazebo：仿真 overlay 中的 Smac、正/倒车 controller 与 free-heading 校验，当前授权为 `0.22 m`。 |
| `runtime_footprint` | 编辑器预检和仿真 costmap 的共享车辆足迹。`length_m`/`width_m` 是整车全长/全宽，当前 `0.27 x 0.13 m`；`base_footprint` 在后轴，`center_x_from_base_footprint_m=0.0841 m`。倒车的虚拟航向要求采用包含前后两个方向的对称包络，因此每侧加 `0.03 m` 后的规划足迹为 `0.4982 x 0.19 m`。同步只写 Gazebo overlay，不改变实车 obstacle layer。 |
| `c_zone_keepout` | C 区中央禁区的水平/竖直内缩，保留外圈绕行车道。 |
| `preflight` | 本地预检的网格、采样、终点容差和搜索预算。 |
| `simulation_keepout.map_resolution_m` | 仿真 keepout PGM 的分辨率。 |
| `simulation_keepout.boundary_padding_m` | PGM 在赛场四周延伸的黑色禁行环宽度，防止规划或控制从场外绕行。 |

修改该 YAML 后，在本机仓库中运行以下命令完成验证：

```bash
cd /home/zyh/SmartCar_V2026
bash src/smartcar_sim/scripts/sim_tune.sh --headless --loop 1
```

`sim_tune.sh` 会依次重生成 `field_map.pgm`、同步仿真 Smac、正/倒车 controller、
free-heading、足迹和 KeepoutFilter 参数到 `nav2_keepout_filter.yaml`，再构建并运行
Gazebo 自动路线。它不会修改实车 `nav2_params.yaml`。脚本会发布 Gazebo 的非零速度，
绝不能用于实体底盘。调参、航点和生成物均以当前本机仓库为唯一权威源。

仅检查生成物是否与当前共享配置一致：

```bash
cd /home/zyh/SmartCar_V2026
PYTHONPATH=src/smartcar_tools \
  python3 src/smartcar_sim/scripts/generate_field_map.py --check
python3 src/smartcar_sim/scripts/sync_route_planning.py --check
```

更完整的 Gazebo 环境、结果验证和故障排查说明见
[`local-simulation.md`](local-simulation.md)。

## 7. 常见问题

| 现象 | 处理 |
|---|---|
| 点击“新增途经点”没有新点 | 先选分段，再点击按钮，最后在左侧场地空白处点击；点到已有航点会被拒绝。 |
| 选中环移到右侧后消失 | 使用当前已构建版本。旧版存在右侧 hover 重绘问题；关闭旧编辑器并重启。 |
| `ParameterAlreadyDeclaredException: use_sim_time` | 说明仍在使用旧构建。同步 `smartcar_tools` 后重新执行 `colcon build --packages-select smartcar_tools smartcar_task --symlink-install`。 |
| 保存被阻止 | 先点击“几何预检”，修复所有红色分段，并确认每个航点都恰好属于一条连续分段。 |
| 仿真禁区或转弯半径没有更新 | 修改 `route_planning.yaml` 后运行 `sim_tune.sh`；直接 `ros2 launch smartcar_sim` 不会自动读取一个新的 `route_planning_file`。 |
| 图形窗口崩溃或只剩场地参考节点 | 在原启动终端按 `Ctrl+C` 清理旧 launch，确认没有旧编辑器后重新启动。 |
