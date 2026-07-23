# 里程计更新率告警取证与修复

**日期**: 2026-07-22

**状态**: 已完成日志取证和首批低风险修复，待 RDK 静止复测与授权后的分级运动验证

**相关文件**: `ekf.yaml`、`nav2_params.yaml`、`CLaserOdometry2D.cpp`、`origincar_base.cpp`

## 1. 结论

现有证据不能支持“车速提高到 0.30 m/s 导致 EKF 过载”这一因果关系，也不能支持“`IntegrationClock` 跳过 pose 积分后向 EKF 注入 pose/速度矛盾”。

能够确认的是：

1. 历史 EKF 更新率告警发生过，但最关键的一次发生在路线尚未启动、车辆尚未发车时。
2. `bt_loop_duration: 10` 的单位是毫秒，对应 100 Hz，而不是此前文档所写的 10 Hz。
3. RF2O 在 10 Hz 扫描热路径中每帧输出 3 条 INFO，并持续写入数十 MB 日志。
4. 现场 RDK 同时运行本机 RViz 时，RViz、RF2O 和 safety 的 CPU 占用远高于 EKF，存在明显资源争用。
5. 底盘串口读取、ROS 回调、命令 watchdog 和传感器发布共享一个同步循环，存在积压帧突发发布和旧命令被延后处理的结构性风险，但尚无数据证明它就是本次运动异常的直接原因。

因此，本轮只实施不改变运动学和安全门语义的修复。EKF 频率、传感器超时、输入队列、controller 和 costmap 更新频率保持不变。

## 2. 原始证据

### 2.1 EKF 告警时序

历史 launch 日志中可见：

```text
Failed to meet update rate! Took 0.054241358 seconds
```

已删除的旧纯导航入口所留 `/tmp/nav_test6.log` 中，更严重的一次为：

```text
Failed to meet update rate! Took 0.249854711 seconds
```

该行出现在旧 `navigation_runner`（现已删除）报告“loaded an unstarted route”之前。它只能证明启动期的一次 `periodicUpdate()` 超过 30 Hz 的 33.3 ms 周期，不能由尚未发生的 0.30 m/s 运动解释。

`robot_localization` 的该 warning 计量 `periodicUpdate()` 回调体耗时。输入积压、CPU 抢占和一次处理较多测量都可能造成超期；日志本身不能区分原因。

### 2.2 现场资源快照

车辆保持软件急停、未执行导航时，进程快照约为：

| 进程 | CPU |
|---|---:|
| RDK 本机 `rviz2` | 322% |
| RF2O | 43% |
| `smartcar_safety` | 31% |
| EKF | 6% |

百分比可跨多个 CPU 核，且只是单次快照，但足以说明不能只盯着 EKF 参数。性能采样不得同时在 RDK 本机运行 RViz；需要可视化时应单独采样并明确记录条件。

### 2.3 RF2O 热日志与构建类型

RF2O 每次扫描计算约 36 到 44 ms，随后逐帧输出 execution time、laser pose 和 base pose。现场 `/tmp/rf2o_test4.log` 已超过 33 MB。逐帧 INFO 会增加格式化、终端和磁盘 I/O，且没有必要作为默认运行日志。

RDK 当前 RF2O 的 `CMAKE_BUILD_TYPE` 为空，编译参数没有优化级别。RF2O 是 Eigen 密集算法，因此必须先用 `RelWithDebInfo` 重建再比较执行时间，不能把 `-O0` 下的 40 ms 当作算法固有成本。

### 2.4 BT 参数单位

Nav2 Humble 将 `bt_loop_duration` 直接解释为毫秒。原值 `10` 是约 100 Hz，历史日志也出现过 `tick rate 100.00` 告警。controller 使用独立的 20 Hz 循环，因此适度降低高层 BT tick 不等于降低底盘控制频率。

## 3. 已排除的旧推断

### 3.1 `/odom` pose 不参与当前 EKF

`odom0_config` 的前 6 项全部为 `false`，当前只融合 `/odom` 的 `vx`、`vy`。因此 `IntegrationClock` 在大间隔时跳过原始 pose 积分，不会把“冻结 pose + 新速度”的矛盾观测送入 EKF。

`IntegrationClock` 仍会影响 `/odom` 自带 pose 及其他潜在消费者，后续应增加诊断，但它不是现有证据下的 EKF 根因。

### 3.2 `sensor_timeout` 不是计算超时

`sensor_timeout` 决定无新测量时多久进行一次纯预测，不是串口读取超时、TF 超时或 `periodicUpdate()` 的执行期限。将 `0.25` 放宽到 `0.35` 不会减少 EKF 计算量。

### 3.3 扩大队列不一定减负

输入队列只在滤波频率明显低于传感器频率、且确认 DDS 丢样时才应增大。EKF 会在一次更新中处理队列内符合时间条件的测量；积压状态下扩大队列可能增加单次工作量。

### 3.4 车速不会自动提高 costmap 更新频率

controller、costmap、planner 和 BT 均由配置频率或事件控制。车辆运动会改变数据内容，但没有证据证明 0.30 m/s 会让 costmap 固定循环自动提频。

## 4. 首批修复

| 修改 | 新值 | 理由 |
|---|---:|---|
| EKF `transform_timeout` | `0.0` | 静态外参应常驻；缺 TF 时不允许一次等待跨越多个 30 Hz 周期 |
| EKF `print_diagnostics` | `true` | 暴露传感器和滤波状态；更新率 warning 仍需采集 stderr |
| Nav2 `bt_loop_duration` | `20 ms` | 将 BT 从 100 Hz 降到 50 Hz，controller 仍保持 20 Hz |
| RF2O 逐帧 telemetry | `DEBUG` | 默认停止逐帧 INFO 和日志膨胀 |
| safety 参数文件 | 显式独立传入 | 防止 RF2O 的 `config_file` 覆盖 safety 配置 |
| safety 输入 QoS | `KeepLast(1)` | 过载时丢弃旧命令和旧传感器样本，只评估最新值 |
| `odom_diag` | 可靠退出并新增 IMU | 同时统计 `/odom`、IMU、EKF、scan 和 RF2O，到时不在回调内 shutdown |

以下值刻意保持不变：

- EKF `frequency: 30.0`
- EKF `sensor_timeout: 0.25`
- `odom0_queue_size: 10`、`imu0_queue_size: 10`、`odom1_queue_size: 5`
- controller `20 Hz`
- local/global costmap update frequency
- safety `raw_odom_timeout_sec: 0.25`

## 5. 静止验证

全过程保持 `/smartcar/safety/status = emergency_stop`，不得发布非零速度。

```bash
source ~/source_env.sh

ros2 topic echo --once /smartcar/safety/status
ros2 topic echo --once /cmd_vel_safe

ros2 run smartcar_tools odom_diag --ros-args -p duration_sec:=30.0

ps -eo pid,pcpu,pmem,comm,args --sort=-pcpu | head -25
```

RDK 构建需显式启用编译优化：

```bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

诊断需记录：

- `/odom`、`/imu/data_raw`、`/odom_combined`、`/scan`、`/odom_laser` 的实际频率和最大间隔。
- `/diagnostics` 中的 EKF warning/error。
- launch stderr 中是否仍出现 `Failed to meet update rate`。
- RF2O INFO 日志是否停止逐帧增长。
- 不开本机 RViz、开启本机 RViz、关闭 RF2O 三种条件的 CPU 差异。

## 6. 串口架构风险

`origincar_base::Control()` 当前每轮执行：

```text
spin_some -> watchdog -> blocking serial read (up to 100 ms) -> parse one frame -> publish
```

CPU 停顿或串口积压时，历史传感器帧可能被快速读出，并在处理时统一赋予接近的当前时间戳，形成 `/odom` 和 IMU 突发。命令回调与 watchdog 也共享该循环，无法给出严格的响应延迟上界。

后续结构性修复应包括：

1. 串口接收与 ROS executor 解耦，使用独立 I/O 线程或异步接收。
2. 使用有界 latest-sample 策略并统计被合并的帧，避免积压后突发灌入 EKF。
3. watchdog 使用单调时钟，Ackermann 命令校验消息年龄。
4. 检查完整写入长度并覆盖所有串口异常；STM32 侧实现独立的无命令自动归零。
5. 发布有效帧、坏帧、读取超时、积压高水位、合并帧、短写和 watchdog stop 诊断。

这类修改会改变底盘 I/O 和停机时序，不能在无人看护、车轮着地时直接部署验证。本轮不实施该重构。

## 7. 后续运动验证门禁

只有现场负责人回来并明确授权后，才能按以下顺序验证：

```text
车轮离地 -> 0.15 m/s 最短段 -> 0.20 -> 0.25 -> 0.30 m/s
```

每档只改变线速度一个变量，不同时扩大角速度上限；关闭 RDK 本机 RViz，记录 rosbag、CPU、stderr 和 `odom_diag`。任何 safety fault、里程计间隔超限、EKF 更新率告警或轨迹异常都应立即锁存急停并停止升级。

在完成上述对照实验前，只能表述为“已纠正错误归因并实施低风险调度修复”，不能表述为“0.30 m/s 问题已解决”或“可高速上场”。

## 8. 上游依据

- robot_localization Humble 参数说明：<https://github.com/cra-ros-pkg/robot_localization/blob/8696ee5a9e4f959fcaae37835dcf2ed12ead581b/doc/state_estimation_nodes.rst>
- robot_localization Humble 更新循环：<https://github.com/cra-ros-pkg/robot_localization/blob/8696ee5a9e4f959fcaae37835dcf2ed12ead581b/src/ros_filter.cpp>
- Nav2 Humble BT 参数与执行循环：<https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp>
