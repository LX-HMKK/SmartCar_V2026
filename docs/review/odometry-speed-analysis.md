# 高速里程计丢帧问题分析

**日期**: 2026-07-22
**状态**: 已诊断，待 RDK 实测验证
**相关**: `ekf.yaml`, `base_serial.launch.py`, `origincar_base.cpp`

## 问题现象

0.15 m/s 导航稳定；0.30 m/s 触发 `robot_localization` EKF "Failed to meet update rate" + BT tick rate 超限，导航失控。

## 数据链路总览

```
STM32 (固件) ──串口 115200──▶ origincar_base ──/odom (vel only)──▶ EKF ──/odom_combined──▶ Nav2
                              (Control 循环,         (30Hz,                  (20Hz 控制器)
                               阻塞式读取)             sensor_timeout=0.25)
```

## 瓶颈分析

### 1. 串口层（低概率主因）

| 参数 | 值 | 影响 |
|------|-----|------|
| 波特率 | 115200 | 24B/帧 → 理论 ~480 Hz，远非瓶颈 |
| 读取超时 | 100 ms | Control() 每次阻塞最多 100ms 等待帧 |
| 帧长 | 24 B (接收) | 每帧含 3 轴速度 + 3 轴 IMU 原始值 |

**结论**：串口本身不是瓶颈。但 `Control()` 是阻塞循环——若 STM32 发送间隙变大（高负载下可能），ROS 侧同步阻塞，`/odom` 出现 >250ms 间隙则 EKF 超时。

### 2. EKF 层（中等概率主因）

```yaml
# ekf.yaml 关键参数
frequency: 30.0           # 33ms 周期
sensor_timeout: 0.25      # 传感器 250ms 无数据则超时
odom0_queue_size: 10      # 轮式里程计队列
```

- EKF 只融合 X/Y 线速度 + yaw 角速度，**不融合位置**
- 位置完全靠速度积分 + IMU yaw rate 预报
- 若 `/odom` 或 `/imu/data_raw` 任一超时 250ms，EKF 预报步累积误差
- **高速下速度噪声增大，EKF 预测-更新循环可能超 33ms 无法完成**

### 3. CPU 争抢（高概率主因）

RDK X5 8G 在导航时同时运行：
- `origincar_base`：串口读取 + IMU + 里程计发布
- `ekf_filter_node`：30Hz 预报-更新
- `controller_server`：20Hz RPP 控制
- `planner_server`：5Hz Smac Hybrid 规划
- `local_costmap`：10Hz 刷新
- `global_costmap`：5Hz 刷新
- `bt_navigator`：10Hz 行为树
- `velocity_smoother`：20Hz
- `smartcar_safety`：20Hz
- `ydlidar_ros2_driver`：激光驱动

**高速时 LiDAR 数据变化更剧烈 → costmap 更新更频繁 → planner 重规划更多 → CPU 争抢加剧 → EKF 丢 deadline。**

### 4. STM32 固件层（待验证）

未知 STM32 实际帧率。若为 50Hz（每 20ms 一帧），理论上 OK。若更高速度导致 STM32 内部处理变慢而降帧，则可能触发连锁反应。

### 5. IntegrationClock 静默丢帧（新发现，高概率共因）

`origincar_base` 的 `IntegrationClock`（`sensor_calibration.hpp`）在帧间隔超过 `max_integration_dt_sec`（默认 **0.25s**）时**跳过位置积分**。速度和 IMU 仍然发布，但 `Robot_Pos` 不更新。

这意味着：
- 若 STM32 帧因任何原因出现 >250ms 间隙（CPU 繁忙、串口阻塞、固件延迟），`/odom` 的 pose 字段冻结在该位置
- EKF 收到带旧 pose + 新速度的 odom 消息 → 更新步出现矛盾 → 可能触发异常
- safety 节点的 `raw_odom_timeout_sec=0.25s` **与此完全对齐**：积分暂停和 safety 锁存在同一时间窗口触发

**这是 0.30 m/s 时最可能的主因之一**：高速下 CPU 争抢 → Control() 循环偶尔被延迟 >250ms → IntegrationClock 跳帧 → EKF 观测矛盾 → "Failed to meet update rate"。

### 6. EKF 诊断静默（辅助因素）

`ekf.yaml` 中 `print_diagnostics: false` — EKF 内部诊断信息被抑制，无法在 `/diagnostics` 中看到详细的内部状态（预测步耗时、更新步延迟等）。建议临时开启以便调优。

## 验证方案

在 RDK 上运行诊断工具，分别在 **静止**、**0.15 m/s 导航**、**0.30 m/s 导航** 三种状态下收集数据：

```bash
# 1. 基础速率检查（不跑车时）
source ~/source_env.sh
ros2 run smartcar_tools odom_diag --ros-args -p duration_sec:=30.0

# 2. 跑车时记录（另一终端）
ros2 topic hz /odom /odom_combined /scan &
ros2 bag record /odom /odom_combined /scan /diagnostics -o speed_test

# 3. 检查 EKF 诊断
ros2 topic echo /diagnostics --once | grep -A5 ekf

# 4. CPU 占用
top -b -n 1 | head -20
```

诊断工具会输出：
- 每个 topic 的到达速率、最大/最小/平均间隔、标准差
- EKF 诊断警告
- 综合判断

## 候选修复方案

### 短期（软件参数调优，低风险）

| 措施 | 文件 | 说明 |
|------|------|------|
| EKF 降频 30→20 Hz | `ekf.yaml` | 50ms 周期更容易满足 |
| 放宽 sensor_timeout 0.25→0.35 | `ekf.yaml` | 容忍更大串口间隙 |
| 增大 odom0_queue_size 10→20 | `ekf.yaml` | 减少队列溢出丢帧 |
| 开启 EKF 诊断 | `ekf.yaml` | `print_diagnostics: true` |
| BT 降频 10→5 Hz | `field_test_nav2_params.yaml` | `bt_loop_duration: 200` |
| controller 降频 20→15 Hz | `field_test_nav2_params.yaml` | 减少 CPU 需求 |
| **若启用 RF2O**: 降低 rejection_threshold | `ekf.yaml` | `odom1_pose_rejection_threshold: 5.0` |

### 中期（需验证）

- 调查 STM32 实际帧率 → 若 <50Hz 需固件调整
- 尝试 `use_control: true` 引入 cmd_vel 作为 EKF 控制输入辅助预报
- 考虑将 EKF 从单线程 executor 改为专用线程

### 长期（架构优化）

- 将 `origincar_base` 串口读取与 ROS 回调解耦（独立读线程）
- 考虑硬件升级：串口波特率 115200 → 921600（需 STM32 固件配合）
- 为 Nav2 costmap 降低分辨率和频率以减少 CPU 争抢

## 建议实施顺序

1. **先在 RDK 上跑 `odom_diag`**（静止时）确认基线数据
2. **跑一次 0.30 m/s 导航测试**（短距离），同时运行 `odom_diag` 和 `ros2 topic hz`
3. **根据诊断结果选择对应的参数调整**
4. **逐步提速验证**：0.15 → 0.20 → 0.25 → 0.30 m/s
