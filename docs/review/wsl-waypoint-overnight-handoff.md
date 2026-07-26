# WSL 仿真航点优化夜间交接

更新日期：2026-07-26

## 1. 当前结论

当前剩余主要问题是航点位置及其正方向设计，不是倒车控制器、末端航向闭环、
RViz 路径显示或 WSL 通信。

- `c_corner_1` 的 reverse handoff 已连续两轮成功，方向、速度、曲率和末端航向
  证据均满足合同。
- `c_corner_2`、`c_corner_3` 已连续两轮成功。
- `c_corner_4` 连续两轮走成长环路并在 120 秒超时，是完整路线当前唯一已复现的
  阻塞点。
- 单独把 `c_corner_4` 从 `180 deg` 改为 `-90 deg` 仍然走长环，已撤回，不能再次
  当作新候选测试。
- 下一步应联合优化 `c_corner_3 -> c_corner_4 -> b_corridor_return_enter` 的位置和
  yaw，并考虑普通 goal checker 的 `0.25 m / 0.50 rad` 到达包络。

Gazebo 仿真不能表述为实体倒车验证，也不能据此开启任何实车运动门禁。

## 2. Git 状态

分支：`feat/gazebo-simulation`

已提交基线：

```text
835dbef fix(nav): 增加倒车交接末端航向控制
05efb51 fix(sim): 修复辅助节点关闭异常
006ea14 feat(sim): 完善完整场地与路线验收链
5190e12 fix(nav): 修复 QR 兜圈与仿真路径显示
4c01d0d fix(sim): 缩小 RViz 里程计箭头
48e239d fix(sim): 修复 WSL 仿真通信与启动链路
```

本交接提交包含已经动态验证的 MPPI 参数和验收程序增强。开始工作前先运行：

```powershell
git status --short --branch
git log -5 --oneline
```

不要丢弃用户或其他代理留下的修改，不要使用 `git reset --hard` 或
`git checkout --`。

提交前验证证据：

```text
Windows 根合同                  169/169
WSL colcon tests                35/35
reverse command filter gtest    11/11
插件真实加载与生命周期激活      通过
RelWithDebInfo -Werror 构建      通过
```

## 3. 已验证实现

### 3.1 Reverse handoff 控制

当前采用 virtual-forward MPPI：

- 路径和机器人 yaw 在 wrapper 内加 pi。
- odom `linear.x/y` 取反。
- 物理输出 `linear.x` 取负，`angular.z` 保持不变。
- 输出经过负速度、非有限值、角速度和 `0.55 m` 最小转弯半径硬门。
- local/global footprint 使用 pi 对称保守包络。
- `velocity_smoother.scale_velocities=true`。

当前动态验证参数：

```yaml
time_steps: 100
batch_size: 1000
vx_std: 0.035
wz_std: 0.12
temperature: 0.2
PathAlignCritic.cost_weight: 18.0
PathFollowCritic.cost_weight: 8.0
PathAngleCritic.cost_weight: 12.0
```

物理速度上限、角速度上限和曲率硬门没有放宽。

### 3.2 验收程序

`auto_train.py` 和 `validate_sim_results.py` 已增强：

- 同时检查 `/cmd_vel_nav` 和 `/cmd_vel_candidate`。
- 记录进入 XY 容差后的控制器转向和 yaw 收敛。
- 拒绝 velocity smoother 残余造成的假闭环。
- 记录并限制终点区域最大偏离、额外路程和收敛时间。
- 记录实际 controller wrapper、内部正向 MPPI 速度范围、smoother 配置。
- 校验行为树、规划终点 yaw、速度方向和 Ackermann 曲率。
- 允许在安全范围内调整 goal tolerance，并按本次运行快照验收。

## 4. 动态证据

### 4.1 当前参数，原始 `c_corner_4=180 deg`

结果：`/root/ros2_ws/tune_logs/run_20260726_203316_1.json`

```text
a_task_observe       succeeded  24.85 s  travel 3.173 m
b_corridor_enter     succeeded  16.02 s  travel 1.394 m
b_corridor_out       succeeded   7.98 s  travel 0.742 m
c_corner_1           succeeded  31.96 s  travel 1.887 m
c_corner_2           succeeded   8.32 s  travel 1.019 m
c_corner_3           succeeded  18.94 s  travel 2.396 m
c_corner_4           timeout   120.02 s  travel 15.239 m
```

`c_corner_1` 关键指标：

```text
position error              0.118 m
yaw error                   0.124 rad
XY-entry yaw error          0.164 rad
post-XY yaw reduction       0.041 rad
post-XY travel              0.027 m
controller/candidate sign   reverse only
minimum observed radius     0.55 m
kinematic violations        0
```

`c_corner_4` 超时时位置误差 `1.426 m`、yaw 误差 `2.006 rad`，实际行驶
`15.239 m`。这不是“接近终点后只差航向”，而是完整的长环路。

### 4.2 失败候选：仅把 `c_corner_4` 改为 `-90 deg`

结果：`/root/ros2_ws/tune_logs/run_20260726_204937_1.json`

前六个目标再次成功，`c_corner_1` 为 `31.80 s / 1.874 m`。`c_corner_4` 仍然：

```text
outcome       timeout
duration      120.03 s
position err  1.475 m
yaw err       2.180 rad
travel        14.227 m
```

因此单点 `c_corner_4=-90 deg` 已被证伪并撤回。不要重复该试验。

## 5. 航点几何现状

相关名义航点：

```text
c_corner_2                (0.825, 3.900), yaw   38 deg
c_corner_3                (3.175, 3.900), yaw  -30 deg
c_corner_4                (3.175, 2.750), yaw  180 deg
b_corridor_return_enter   (2.132, 2.517), yaw -135 deg
```

第二轮到达 `c_corner_3` 时的实际终态约为：

```text
position (2.95, 3.98)
yaw      -9 deg
```

普通 goal checker 允许 `0.25 m / 0.50 rad`，所以后续规划起姿可能和航点名义 yaw
相差接近 29 度。只按名义端姿计算最短 Dubins 路径不够，候选必须覆盖到达容差内的
实际起姿包络。

场地 B 区两段实体墙是水平矩形：

```text
west wall: x [-0.5, 1.5], y [1.75, 2.25]
east wall: x [ 2.5, 4.5], y [1.75, 2.25]
```

联合优化时必须把 footprint 和 inflation 余量计入，不能只比较自由空间路径长度。

## 6. 夜间任务

目标：找到并验证一组不会产生长环路的
`c_corner_3 -> c_corner_4 -> b_corridor_return_enter` 航点位置和正方向，使完整十目标
路线通过。

推荐顺序：

1. 写一个只读几何扫描脚本或测试，枚举三点的有限候选位置/yaw。
2. 同时计算三段 Dubins 路径长度：`c2->c3`、`c3->c4`、`c4->return_enter`。
3. 对 `c_corner_3` 的实际到达包络进行采样，至少覆盖名义 yaw `+-0.50 rad` 和
   位置 `+-0.25 m`，拒绝任何会跳到长分支的候选。
4. 对路径采样点检查 B 区墙体、场地边界、对称 footprint 和 inflation 余量。
5. 先跑定向短任务验证候选；如果需要新增 runner 参数，只允许增加显式
   `start_goal_id/end_goal_id` 一类仿真专用参数，并补合同测试。
6. 候选通过后，必须再跑一次带 RViz 的完整十目标路线。
7. 成功后更新 `AGENTS.md`、`CLAUDE.md`、部署手册和本交接文档中的证据状态。

不要通过以下方式掩盖几何问题：

- 不要放宽 `minimum_turning_radius: 0.55`。
- 不要改为 `REEDS_SHEPP`、Spin、原地旋转或开环速度。
- 不要显著放宽 goal tolerance 来接受兜圈。
- 不要降低终点兜圈验收阈值或删除两层命令证据。
- 不要修改已经连续两轮通过的 `c_corner_1=60 deg`。

## 7. 运行命令

Windows 合同：

```powershell
python -m unittest discover -s tests -v
git diff --check
```

带 RViz 的完整 WSL 仿真：

```powershell
wsl.exe -d Ubuntu-22.04 -u root -- bash -lc `
  'export DISPLAY=:0 WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir; `
  cd /root/ros2_ws; `
  bash /root/ros2_ws/src/smartcar_sim/scripts/sim_tune.sh --loop 1'
```

`sim_tune.sh` 会从 Windows 权威源码 rsync、构建、清理旧进程、启动 Gazebo/Nav2/RViz
并保存输入快照和结果。不要使用 `--headless`，用户要求下一次运行显示 RViz。

查看最新结果：

```powershell
wsl.exe -d Ubuntu-22.04 -u root -- bash -lc `
  'ls -lt /root/ros2_ws/tune_logs | head; `
  grep -E "auto_train.*(Goal | t=|Result |Results saved)" `
  /root/ros2_ws/tune_logs/<run-id>.log | tail -30'
```

发现长环后应及时保存证据并停止该候选，不要每个明显失败候选都空跑满 120 秒。

## 8. 验收标准

最终结果必须同时满足：

- 结果包含全部 10 个非起点航点，`overall_outcome=completed`。
- 每个目标 `outcome=succeeded`、action status 为 4。
- 每段均有 `/plan` 证据，规划终点 yaw 与完整位姿目标一致。
- reverse 段两层命令均只有负速度；forward 段均只有正速度。
- reverse handoff 两层命令曲率违规数为 0，最小观测半径不小于 `0.55 m`。
- `c_corner_1` 保持末端位置与 yaw 闭环证据，无进入终点后兜圈。
- `c_corner_4` 不再出现十几米长环；建议该段实际路程控制在几何短路的合理余量内。
- 日志无持续 `Control loop missed`、生命周期失败或进程残留。
- RViz 显示完整场地、全局 `/plan` 和局部路径，缩放可读。

## 9. 安全边界

- 不连接 RDK。
- 不启动实体相机。
- 不发布任何实车非零速度。
- 不修改或开启实车运动门禁。
- 所有非零运动仅允许在 Gazebo 仿真链中发生。

## 10. 交付要求

夜间结束时留下：

- 最终航点位置/yaw 和选择依据。
- 每个尝试对应的输入快照、JSON 和日志路径。
- 完整路线结果摘要，包括每段耗时、误差和实际路程。
- 根合同测试和 WSL 测试结果。
- 更新后的 `AGENTS.md` 与 `CLAUDE.md`，两者内容保持同步。
- 一个或多个 Angular 中文提交，不添加 `Co-Authored-By`。
