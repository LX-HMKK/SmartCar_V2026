# 工作空间基础设施设计

- 日期：2026-07-16
- 状态：已通过设计评审，待实现
- 适用阶段：智慧医疗赛项目首个 spec（基础设施），后续模块各自独立 spec

## 1. 背景与目标

项目进入编码阶段，需先建立本机（Windows + VS Code）与 RDK X5 之间的统一工作空间与同步机制，作为后续所有功能包（smartcar_bringup / nav2 / vision / task）的开发基座。

核心目标：

1. 本仓库 `src/` 作为所有功能包（官方、第三方、自研）的统一权威源码目录。
2. RDK 上以单一工作空间 `/root/ros2_ws` 编译运行，其 `src/` 为本机 `src/` 的镜像。
3. 修复后的官方 launch、EKF/IMU/底盘/LiDAR 配置、URDF 全部纳入本仓库版本控制，避免 RDK 重刷丢失。
4. 一键同步脚本，支持正向推送与可选反向回传。

## 2. 范围

**在范围内（本 spec 交付）**

- `src/` 目录结构与分组约定
- 根 `.gitignore`
- 官方包 vendor 回传（一次性，从 RDK 拉入 VCS）
- 第三方包 `obstacle_detector_2` 迁入与 vendor 化
- `scripts/sync_to_rdk.py`（rsync push/pull/init-vendor/setup）
- `scripts/source_env.sh`（RDK 端 source 链）
- 单一工作空间回归验证（TF/odom/IMU/scan）

**在范围外（后续 spec）**

- `smartcar_bringup`：整合官方 launch、重组配置、首个自研 bringup 包（spec #2，顺带完成配置回传的业务层组织）
- `smartcar_nav2` / `smartcar_vision` / `smartcar_task`：各自独立 spec
- VFH 备用避障包部署、端侧 VLM 模型部署、实车运动测试

## 3. 关键决策汇总

| 决策项 | 选择 | 理由 |
|---|---|---|
| 首个 spec 范围 | 仅基础设施 | 模块独立性强，分开 spec 便于收敛 |
| 构建/source 策略 | 单一工作空间 `/root/ros2_ws` 构建全部 | 修复进 VCS 永不丢失、source 简单、单一权威源 |
| `/userdata/dev_ws` 角色 | 退为原始 vendor 备份，不参与运行时 | 保留回退参考，运行时只依赖 `/root/ros2_ws` |
| 同步机制 | rsync over ssh（Python 封装） | 原生增量/--delete/--exclude，不重复造轮子 |
| 配置放置 | 随包 + repo 根 `config/` 放跨包共享数据 | ROS 惯例 + 航点/速度/标定等跨包数据有归处 |
| 自研包命名 | 平铺 `src/smartcar_*` | 匹配原意，无中间父目录 |
| 第三方包管理 | vendor 化（去 `.git`） | 竞赛固定依赖，简单可控 |
| 官方包目录 | 保留树状 `src/origincar/` | 3rdparty/SDK 相对路径依赖，拆平破坏构建 |

## 4. 目录结构

```
SmartCar/
├── CLAUDE.md / README.md
├── .gitignore                       # 新建
├── config/                          # 跨包共享：waypoints.yaml、speed_profile.yaml、calibration/
├── docs/                            # 现有
├── scripts/
│   ├── sync_to_rdk.py               # rsync push/pull/init-vendor/setup
│   └── source_env.sh                # RDK 端 source 链（setup 子命令 scp 到 RDK ~/）
├── src/
│   ├── origincar/                   # 官方包 vendor 副本（树状，从 /userdata/dev_ws/src/origincar 拉回）
│   │   ├── origincar_base/          #   含修复后 origincar_bringup.launch.py + config/{ekf,imu}.yaml
│   │   ├── origincar_bringup/
│   │   ├── origincar_description/   #   urdf/origincar.xacro 等
│   │   ├── origincar_msg/
│   │   ├── utils/
│   │   ├── ydlidar_ros2_driver/     #   params/TminiPro.yaml
│   │   ├── 3rdparty/                #   ackermann_msgs、serial
│   │   └── YDLidar-SDK-master/
│   ├── third_party/
│   │   └── obstacle_detector_2/     # 从 RDK /root/ros2_ws/src 迁入，vendor（去 .git）
│   └── smartcar_bringup/            # 自研包（spec #2 起逐个加入 smartcar_nav2/vision/task）
├── tools/  assets/                  # 占位
```

要点：

- `src/origincar/` 保留官方树状结构，colcon 递归构建。注意 `obstacle_detector_2` 的 `package.xml` 包名是 `obstacle_detector`（目录名与包名不一致），构建/引用时以包名为准。
- 自研包平铺在 `src/` 下（`src/smartcar_bringup/` 等），不套 `src/smartcar/` 父目录。
- 官方包内任何 `.git` 不入 VCS；`obstacle_detector_2` 去 `.git` vendor 化。
- `.remember/`（现有隐藏目录）加入 `.gitignore`。

### `.gitignore` 内容

```gitignore
# colcon 构建产物
build/
install/
log/

# Python
__pycache__/
*.pyc

# 嵌套 git（vendor 包的 .git）
**/.git/

# 编辑器/IDE
.vscode/
.idea/

# 本地工具目录
.remember/

# 备份
*.bak
*.orig
```

## 5. 构建 / Source 策略

- RDK 上单一工作空间 `/root/ros2_ws`，其 `src/` = 本机 `src/` 的 rsync 镜像。
- **source 链**：`/opt/tros/humble/setup.bash` -> `/root/ros2_ws/install/setup.bash`。**不再 source `/userdata/dev_ws`**。
- `/userdata/dev_ws` 保留为原始 vendor 备份；若 `/root/ros2_ws` 构建失败可临时回退参考。
- 构建命令：`cd /root/ros2_ws && colcon build --symlink-install`（symlink 便于改 Python/launch 不重编 C++）。
- 首次重组后需清理陈旧构建产物：`rm -rf build install log` 再 `colcon build`（因 `obstacle_detector_2` 路径由 `/root/ros2_ws/src/obstacle_detector_2` 迁至 `third_party/obstacle_detector_2`，旧缓存失效）。

### `scripts/source_env.sh`

```bash
#!/usr/bin/env bash
# RDK X5 单一工作空间环境入口
source /opt/tros/humble/setup.bash
[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash
```

## 6. 同步脚本 `scripts/sync_to_rdk.py`

Python 3，`subprocess` 调 rsync over ssh。默认目标 `root@192.168.128.10`，远程工作空间 `/root/ros2_ws`。host/path 可通过常量或 CLI 参数覆盖。

### 子命令

| 子命令 | 方向 | 作用 |
|---|---|---|
| `push`（默认） | 本机 `src/` + `config/` -> RDK `/root/ros2_ws/src/` + `/root/ros2_ws/config/` | 主同步，`--delete` 镜像语义 |
| `pull` | RDK `/root/ros2_ws/src/` -> 本机 `src/` | 反向，带回 RDK 端临时改动（慎用，本机为权威源；`config/` 反向同理可选） |
| `init-vendor` | RDK -> 本机 | 一次性回传官方包与第三方（见 §7） |
| `setup` | 本机 -> RDK `~/` | scp `source_env.sh` 到 RDK 家目录 |

> `push` 同步两棵树：`src/`（包源码）与 `config/`（跨包共享数据，如 waypoints.yaml）。`config/` 当前为空占位，首份共享配置（nav2 spec）出现后即随 push 到达 RDK `/root/ros2_ws/config/`，运行时包按该路径或环境变量引用。

### rsync 标志与排除

- 标志：`-avz --delete --itemize-changes`（`-a` 保权限/时间，`-z` 压缩，`--delete` 删冗余，`--itemize-changes` 显示变更明细）。
- 可选标志：`--dry-run`（预览）、`--checksum`（强制校验和，慢但更可靠）。
- 排除（`--exclude`）：`**/.git/`、`build/`、`install/`、`log/`、`__pycache__/`、`*.pyc`、`.vscode/`、`.idea/`、`*.bak`、`*.orig`。

### 安全护栏

- `push --delete` 前检查本机 `src/origincar/` 存在，否则拒绝执行（防误空同步清空 RDK）。
- 提供 `--dry-run` 标志；首次或大改动建议先 dry-run。
- 复用现有免密 SSH key，无需额外认证。
- rsync 非零退出码捕获并打印 stderr；ssh 超时提示检查有线连接。

## 7. Vendor 回传流程（一次性，`init-vendor`）

1. rsync 拉取 `root@192.168.128.10:/userdata/dev_ws/src/origincar/` -> 本机 `src/origincar/`
   - 排除：`**/.git`、`build/`、`install/`、`log/`
   - 内容：修复后 `origincar_bringup.launch.py`、`config/ekf.yaml`、`config/imu.yaml`、URDF、`TminiPro.yaml` 等，全部进 VCS。
2. rsync 拉取 `root@192.168.128.10:/root/ros2_ws/src/obstacle_detector_2/` -> 本机 `src/third_party/obstacle_detector_2/`
   - 排除：`.git`（完成 vendor 化）。
3. `git add src/ && git commit`。
4. 此后 `push` 即把这些镜像到 `/root/ros2_ws/src/`。

> 此步同时完成任务 3"配置回传纳入版本控制"--官方 launch/配置/URDF 全部进 VCS，`/userdata/dev_ws` 重刷也不再丢失修复（static_transform_publisher 的 Humble 兼容修复随之固化）。

## 8. 验证流程（成功判据）

单一工作空间必须复现已知的 `/userdata/dev_ws` 良好行为：

1. 本机 `python scripts/sync_to_rdk.py setup`（部署 source_env.sh）
2. 本机 `python scripts/sync_to_rdk.py push`（镜像 src/ -> /root/ros2_ws/src/）
3. RDK `source ~/source_env.sh`
4. RDK `cd /root/ros2_ws && rm -rf build install log && colcon build --symlink-install`（首次清理重建）
5. RDK `source ~/source_env.sh`（加载新构建的 install）
6. RDK `ros2 launch origincar_base origincar_bringup.launch.py`
7. 核对：`/odom_combined` ~20Hz、`/imu/data` ~20Hz、`/scan` ~10Hz、TF 树完整（odom_combined->base_footprint->base_link/gyro_link->laser/camera）
8. 通过 = 单一工作空间回归验证 OK；失败则临时回退 `source /userdata/dev_ws/install/setup.bash` 排查。

## 9. 错误处理与测试

- 同步脚本：`--dry-run` 预览、push 后 `ssh root@192.168.128.10 'ls /root/ros2_ws/src/'` 核对镜像、idempotency（二次 push 无变更）。
- 构建回归即端到端测试（§8 步骤 7）。
- 已知风险与对策：
  - RDK 无法访问 GitHub：第三方包（如后续 VFH）由本机下载后 scp 上传，不依赖 RDK 联网。
  - RDK 内存 6.9 GiB 无 swap：本 spec 不涉及 VLM；后续 vision spec 需评估 4B 模型 OOM 风险，必要时降级 2B。
  - `obstacle_detector_2` 目录名与包名不一致：构建引用以包名 `obstacle_detector` 为准。

## 10. 后续 spec

- **spec #2：smartcar_bringup** -- 在本基础设施上整合官方 launch、重组配置到 smartcar_bringup 包、提供统一入口 launch，完成配置回传的业务层组织。
- **spec #3+：smartcar_nav2 / smartcar_vision / smartcar_task** -- 各自独立。
