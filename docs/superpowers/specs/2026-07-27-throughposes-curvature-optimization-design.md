# ThroughPoses 曲率优化设计

> 日期：2026-07-27 | 分支：feat/gazebo-simulation | 状态：待实现

## 问题描述

三段式 NavigateThroughPoses 在多中间航点场景下，Smac Hybrid Dubins 规划器生成大曲率锯齿路径，RPP 控制器跟踪偏离后触发 0.5 Hz 重规划，形成"偏离→重规划→再偏离"的恶性循环。后两段（反向 2 点 ThroughPoses + 正向 6 点 ThroughPoses）均受影响。

## 根因链

```
angle_quantization_bins=72 (5°)
  → Dubins 状态空间粗糙，路径出现锯齿摆动
non_straight_penalty=1.2
  → 几乎不惩罚转向，选择曲折但略短的路径
analytic_expansion_ratio=3.5
  → 过早连接粗糙节点，产生大曲率 Dubins 弧线
curvature_tolerance=1.20 (反向BT)
  → 校验过松，允许 3.02/m 曲率（R=0.33m，车辆无法执行）
RPP lookahead=0.8m
  → 弯道短视，跟踪偏离
RateController hz=0.5
  → 偏离后快速重规划，从错误位置再次生成劣质路径
```

## 优化方案：全链路四层分层优化

### 第一层：规划器 (Smac Hybrid Dubins)

| 参数 | 当前值 | 目标值 | 理由 |
|---|---|---|---|
| `angle_quantization_bins` | 72 (5°) | **144** (2.5°) | 角度分辨率翻倍，消除锯齿摆动 |
| `non_straight_penalty` | 1.2 | **2.0** | 显著偏向直线段，仅在转弯必要时使用弧线 |
| `analytic_expansion_ratio` | 3.5 | **2.0** | 降低过早 Dubins 连接频率，让搜索充分展开 |
| `analytic_expansion_max_length` | 3.0 | **5.0** | 允许更长的规划前向探索 |
| `tolerance` | 0.0 | **0.05** | 5cm 目标容差，避免末期复杂调整 |
| `max_planning_time` | 3.0 | **5.0** | 给足时间让 144-bin 搜索收敛 |
| `cache_obstacle_heuristic` | false | **true** | 滚动窗口下加速重规划 |
| `retrospective_penalty` | 0.015 | **0.02** | 轻微增加对回退路径的惩罚 |
| `smooth_path` | true | true | 保持开启（当前值正确） |

### 第二层：路径校验 (BT XML)

**正向 ThroughPoses BT** (`navigate_through_poses_w_replanning_and_recovery.xml`)：
- Smac 内置 `smooth_path: true` + 144-bin 高分辨率已提供足够平滑，不额外插入 `SmoothPath` BT 节点（`nav2_smoother_selector_bt_node` 已注册但本分支未配置 `smoother_server`，加入会导致运行时错误）

**反向 ThroughPoses BT** (`navigate_through_poses_reverse_w_replanning_and_recovery.xml`)：

| 参数 | 当前值 | 目标值 | 理由 |
|---|---|---|---|
| `curvature_tolerance` | 1.20 | **0.15** | 允许曲率 = 1/0.55 + 0.15 = 1.97/m (R=0.51m)，贴近物理极限 |
| `maximum_direction_error` | 1.50 | **0.60** | 约 34° 方向偏差上限 |

### 第三层：控制器 (RPP)

| 参数 | 当前值 | 目标值 | 理由 |
|---|---|---|---|
| `lookahead_dist` | 0.8 | **1.0** | 更长前视距离减少弯道短视 |
| `max_lookahead_dist` | 1.2 | **1.5** | 配合速度缩放有更多前视余量 |
| `min_lookahead_dist` | 0.5 | **0.5** | 保持（低速足够） |
| `regulated_linear_scaling_min_radius` | 0.9 | **0.55** | 与最小转弯半径一致，避免过早降速 |

### 第四层：重规划策略 (BT)

| 参数 | 当前值 | 目标值 | 理由 |
|---|---|---|---|
| `RateController hz` | 0.5 (2s) | **0.3** (3.3s) | 降低重规划频率，给控制器更多时间收敛 |
| `RemovePassedGoals radius` | 0.7m | **0.35m** | 避免跳过间距 <0.7m 的相邻航点 |

## 涉及文件

1. `src/smartcar_nav2/config/nav2_params.yaml` — 规划器 + 控制器参数
2. `src/smartcar_nav2/config/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml` — 正向 BT
3. `src/smartcar_nav2/config/behavior_trees/navigate_through_poses_reverse_w_replanning_and_recovery.xml` — 反向 BT

## 验证方法

1. Gazebo 仿真运行三段式 ThroughPoses，观察 `/plan` 话题的路径曲率分布
2. 检查路径是否呈现清晰的"直线→弧线→直线"结构
3. 验证 `auto_train.py` 三阶段 `overall_outcome=completed`
4. 确认重规划次数显著减少

## 回退策略

所有变更为纯配置/XML 文件。若仿真验证不通过，逐步回退：
1. 先回退校验层（保留宽 tolerance）
2. 再回退规划器到 72 bins + 1.2 penalty
3. 控制器和重规划参数保持（低风险）
