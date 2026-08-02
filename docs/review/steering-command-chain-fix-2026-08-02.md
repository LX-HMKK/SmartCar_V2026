# 2026-08-02 转向指令链路修复：0.7 rad 命令被固件折算的问题

## 结论

`0.7 rad` 转向指令在下位机 OLED/调试器中显示 `Move_Z=0.322`（旧固件）或
`0.453`（调低 `MINI_AKM_MIN_TURN_RADIUS` 后）的原因，**不是** ROS 发送端丢失，
而是 STM32 固件把协议帧 `tx[7:8]` 当作**角速度 ω**（rad/s）、再经自行车模型
`Vz_to_Akm_Angle` 反算为前轮转角，并被 `MINI_AKM_MIN_TURN_RADIUS=0.35` 钳制。
将固件 `Vz_to_Akm_Angle` 改为**直接透传**后，`0.7 rad` 命令在下位机正确显示
`Move_Z=0.700`，用户现场确认"正常了"。

该修复只改一个函数体，4 条接收路径（USART1/USART3/UART5/CAN）同时生效。
固件补丁本身不在本仓库；随后本仓库将 ROS 运行默认值同步为直通的
`steering_command_scale=1.0`、`steering_command_offset_rad=0.0`、`0.70 rad`
上限，并将 Nav2/路线预检的保守最小半径设为 `0.22 m`。

## 问题现象

- 通过 `/ackermann_cmd` 发布 `steering_angle=0.7, speed=0.15`（20 Hz 持续）。
- 下位机（STM32）收到后 `Move_Z` 显示 `0.322`（`MINI_AKM_MIN_TURN_RADIUS=0.35`）。
- 用户将 `MINI_AKM_MIN_TURN_RADIUS` 改为 `0.20/0.09/0.05` 后，`Move_Z` 变为
  `0.453`，三种取值结果相同，说明仍有一个"看似限制"的转换存在。
- 用户一度归因于 ROS/传输链路丢值；经字节级核对，`tx[7:8]=700` 逐字节正确发送。

## 根因（固件 `Vz_to_Akm_Angle`，`HARDWARE/usartx.c`）

固件接收路径（`USART1_IRQHandler` / `USART3_IRQHandler` / `UART5_IRQHandler` /
`CAN`）对阿克曼车型统一执行：

```c
Vz     = XYZ_Target_Speed_transition(rxbuf[7], rxbuf[8]);  // 0.7 ——被当作"目标角速度"ω
if(Car_Mode==Akm_Car)
    Move_Z = Vz_to_Akm_Angle(Move_X, Vz);                  // 反算前轮转角
```

`Vz_to_Akm_Angle` 原始逻辑：

```c
Min_Turn_Radius = MINI_AKM_MIN_TURN_RADIUS;   // = 0.350
if(float_abs(Vx/Vz) <= Min_Turn_Radius) {     // |0.15/0.7| = 0.214 <= 0.35 → 触发
    Vz = float_abs(Vx)/Min_Turn_Radius;       // Vz 被改为 0.15/0.35 = 0.4286
}
R = Vx/Vz;                                    // R = 0.15/0.4286 = 0.35
AngleR = atan(Axle_spacing/(R + 0.5f*Wheel_spacing));
//        atan(0.144/(0.35 + 0.081)) = atan(0.334) = 0.3224 rad
```

常量（`BALANCE/robot_select_init.h`）：`Akm_axlespacing=0.144`、
`Akm_wheelspacing=0.162`、`MINI_AKM_MIN_TURN_RADIUS=0.350`。

因此：

| 场景 | `Vz_to_Akm_Angle` 行为 | `Move_Z` |
| --- | --- | --- |
| `Rmin=0.35`，发 0.7 | 钳制触发，强制 R=0.35 | `0.3224 rad`（18.5°） |
| `Rmin∈{0.20,0.09,0.05}`，发 0.7 | `|0.15/0.7|=0.214 > Rmin`，不钳制 | `atan(0.144/(0.2143+0.081)) = 0.4537 rad`（26°） |
| **透传（本次修复）**，发 0.7 | 直接返回 Vz | **`0.700 rad`（40°）** |

**关键点**：固件把 `tx[7:8]` 当成 ω 而不是转向角。即使完全不钳制，发 0.7（当作
ω=0.7）也只能得到 `Move_Z=0.4537`——因为固件先按 `R=Vx/ω` 求半径、再按
`atan(L/(R+0.5W))` 反算转角。要让 `Move_Z=0.7`，要么改固件透传（本次方案），
要么把发送值改为 `ω = Vx/(L/tan(δ)−0.5W) = 1.667 rad/s`（发 1667 毫弧度，且
`MINI_AKM_MIN_TURN_RADIUS < 0.09`）。

## 修复内容（仅下位机固件，`HARDWARE/usartx.c`）

函数 `Vz_to_Akm_Angle` 的函数体改为直接透传：

```c
float Vz_to_Akm_Angle(float Vx, float Vz)
{
    // Pass-through: host sends the front-wheel steering angle (rad) directly.
    // No bicycle-model conversion, so a commanded 0.7 rad is used as 0.7 rad.
    return Vz;
}
```

- 全固件对 `Vz_to_Akm_Angle` 的调用点共 4 处（USART1/USART3/UART5 ISR + CAN），
  全部复用同一函数，改一处全部生效；调用点代码**未改动**。
- 字节级校验：替换前后除该函数体外逐字节一致，备份为 `usartx.c.bak`。
- 后续 `Drive_Motor`（`BALANCE/balance.c`）对 `AngleR` 的 `±1.0 rad` 限幅与
  `Servo=target_limit_int(Servo,800,2200)` 在 `Move_Z=0.7` 时均不会触发
  （对应 `Servo PWM ≈ 970`，在范围内）。

## 验证

- **字节完整**：`diff` 仅 789–822 行（函数体）变化；`can.c`、`balance.c`、
  `robot_select_init.h` 未动。
- **全链路模拟**（Vx=0.15）：`0.7 → Move_Z=0.700, R=0.09m, ServoPWM=969`；
  `0.35 → 0.350, R=0.31m, PWM=1187`；`0.15 → 0.150, R=0.87m, PWM=1348`。
- **实车（RDK 22:27）**：透传固件下发布 `steering=0.700 / speed=0.150`，用户确认
  "正常了"，下位机 `Move_Z` 显示 `0.700`。
- **最小转弯半径实测（2026-08-02）**：地面 0.7 rad 最大转向画圆，用户现场实测
  **R_min ≈ 0.2 m**（卷尺量取）。注意与自行车模型理论值 `R = L/tan(δ)−0.5W ≈ 0.09 m`
  明显偏大——差异来源包括实际舵角达不到 40° 的机械上限、轮胎滑移、以及
  `Drive_Motor` 差速公式对内外轮速的分解假设，属于正常工程偏差。

## 注意 / 后续

- ⚠️ 该固件文件在 RDK 之外（用户本地 Keil 工程 `/home/zyh/下载/origincar_controller/`），
  **不属于本仓库**；重新烧录需用户自行在 Keil 编译。
- `MINI_AKM_MIN_TURN_RADIUS` 在新固件中不再参与转向计算，可保持原值。
- `steering_command_scale=1.0 / offset=0.0` 仍为当前实测配置；低角度（0.15→8.6°、
  0.30→17°）下线性区已核对"准了"。
- 0.7 rad 画圆 CSV 未落盘（强杀），但已用卷尺实测 **R_min ≈ 0.2 m**；
  因此运行配置采用 `minimum_turning_radius=0.22 m`，保留 `0.02 m` 裕量。该参数
  同时约束 Smac、free-heading、controller 和路线预检，不能只改其中一处。
- 该半径同步不构成完整路线或实体倒车验收；运动门禁仍默认关闭，后续仍需车轮离地、
  低速地面和完整路线分级验证。
