# 场地航点编辑与三个媒体分项入口

更新时间：2026-07-22

本文用于 RDK X5 上的场地航点标定，以及语音、二维码、图生文三个互相隔离的 bench 入口。仓库只保留一份任务路线：

`src/smartcar_nav2/config/waypoints/default_waypoints.yaml`

旧的 68 点纯导航路线、独立纯导航 launch、runner、probe 和专用 Nav2 参数已经删除。完整运动验证统一通过正式任务链完成，不再维护第二套路由。

未经现场负责人明确授权，不得启动实体相机、音频播放器、底盘或任何实车运动。航点编辑入口不启动 Nav2、LiDAR 或底盘，并默认锁存软件急停。

## 1. 坐标与参考层

车辆发车时手动放在 P 区原点 `(0, 0)`，车头朝 `+X`；`+X` 指向规则图右侧，`+Y` 指向 B/C 区。规则图尺寸由以下文件保存：

```text
src/smartcar_tools/config/routes/field_geometry.yaml
src/smartcar_tools/config/reference/competition_field_dimensions.png
src/smartcar_tools/config/reference/competition_field_route_example.png
```

`field_reference_node` 根据官方尺寸发布 `/smartcar/field_reference/markers`，显示 5 m x 5 m 外框、A/B/C 分区、B 区墙体和通道、C 区环道、P 点及规则图任务标志。

该参考层只用于 RViz 看图和量点：

- 固定在 `odom_combined`。
- 不发布 `/map` 或任何 TF。
- 不启动 `map_server`、AMCL、SLAM。
- 不加入 Nav2 costmap，也不会校正 EKF 漂移。

因此它是“规则图先验参考”，不是静态地图定位。

## 2. 唯一的 9 点任务路线

当前未实测基线顺序为：

```text
p_start
  -> a_task_observe       QR 任务点留距位
  -> b_corridor_out       出站通道口
  -> c_corner_1           VLM，车头朝规则图左侧
  -> c_corner_2
  -> c_corner_3
  -> c_corner_4
  -> b_corridor_return    回程通道口
  -> p_finish
```

QR 观察位距离规则图任务标志约 0.89 m，避免驶得过近。C 区四角取自官方环道中线尺寸，角 1 航向为 `-180 deg` 并触发图生文。任务状态机按 QR、VLM、返回 P 分成三个 `FollowWaypoints` action，通道和其余角点作为段内途经点，避免每个点单独创建 action。

这些数值仍是规则图推算值，文件保持 `calibrated: false`。只有实车逐段验证完成后，才能显式开启 `waypoints_calibrated=true` 运动门禁。

## 3. RViz 直接拖拽编辑

先同步、构建并加载环境：

```powershell
python scripts/sync_to_rdk.py push --dry-run
python scripts/sync_to_rdk.py push
```

```bash
ssh root@172.16.25.27
source ~/source_env.sh
cd /root/ros2_ws
# 从仍含旧纯导航入口的版本升级时，仅清理这两个包的生成目录一次，
# 避免 setuptools/ament 在 install 中保留已删除的 launch 和脚本。
rm -rf /root/ros2_ws/build/smartcar_tools \
  /root/ros2_ws/install/smartcar_tools \
  /root/ros2_ws/build/smartcar_nav2 \
  /root/ros2_ws/install/smartcar_nav2
colcon build --symlink-install --packages-select \
  smartcar_task smartcar_tools smartcar_nav2 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

启动编辑器：

```bash
ros2 launch smartcar_tools waypoint_editor.launch.py
```

RViz 默认同时显示官方场地参考层、任务折线和交互航点。使用顶部 `Interact` 工具：

- 拖动平面手柄修改非 P 点的 XY 位置。
- 拖动旋转环修改航向。
- `p_start` 的位置和朝向固定；`p_finish` 的位置固定，只能修改最终航向。
- 右键任一点可选择 `Save all waypoints`、`Undo last drag` 或 `Reload from disk`。

也可使用服务完成同样操作：

```bash
ros2 service call /smartcar/waypoint_editor/save std_srvs/srv/Trigger "{}"
ros2 service call /smartcar/waypoint_editor/undo std_srvs/srv/Trigger "{}"
ros2 service call /smartcar/waypoint_editor/load std_srvs/srv/Trigger "{}"
ros2 topic echo /smartcar/waypoint_editor/status
```

保存使用原子替换并自动写回 `calibrated: false`。编辑器启动期间不解除软件急停，也不会把修改热加载到正在运行的任务节点。

现场修改后只回传这一份文件：

```powershell
python scripts/sync_to_rdk.py pull-waypoints --dry-run
python scripts/sync_to_rdk.py pull-waypoints
python -m unittest discover -s src/smartcar_task/test -p test_waypoints.py -v
```

只读查看时可分别启动参考层和普通 Marker：

```bash
ros2 run smartcar_tools field_reference_node
ros2 run smartcar_tools waypoint_viz --ros-args \
  -p waypoints_file:=/root/ros2_ws/src/smartcar_nav2/config/waypoints/default_waypoints.yaml
```

## 4. 独立语音测试

该入口只启动火山 TTS consumer，不启动底盘、Nav2、视觉或任务：

```bash
export VOLCENGINE_TTS_APP_ID='<应用 ID>'
export VOLCENGINE_TTS_ACCESS_TOKEN='<access token>'
ros2 launch smartcar_tools speech_test.launch.py
```

另开终端发送一次请求：

```bash
source ~/source_env.sh
ros2 run smartcar_tools speech_probe --text '语音分项测试'
```

该测试会访问网络并实际播放音频，不能纳入无硬件 smoke。

## 5. 独立二维码测试

单张图片回放不打开实体相机：

```bash
ros2 launch smartcar_tools qr_test.launch.py \
  input_source:=file image_file:=/root/test-data/qr.png
```

```bash
source ~/source_env.sh
ros2 run smartcar_tools qr_probe --timeout-sec 3
```

获得实体相机启动授权后，才可切换到相机：

```bash
ros2 launch smartcar_tools qr_test.launch.py \
  input_source:=camera camera_driver:=aurora
```

## 6. 独立图生文与 HDMI UI

该入口启动相机或单图回放、VLM 服务和 PyQt5 UI，不启动语音、底盘、Nav2 或任务。先配置 HDMI 环境：

```bash
export DISPLAY=:0
export XAUTHORITY=/var/run/lightdm/root/:0
```

火山 Ark 模式还需设置：

```bash
export ARK_API_KEY='<火山 Ark API key>'
export VOLC_ARK_MODEL='doubao-1-5-vision-pro-32k-250115'
```

先使用单张图片验证：

```bash
ros2 launch smartcar_tools vlm_test.launch.py \
  input_source:=file image_file:=/root/test-data/person.png \
  display:=:0 xauthority:=/var/run/lightdm/root/:0
```

获得实体相机授权后再切换：

```bash
ros2 launch smartcar_tools vlm_test.launch.py \
  input_source:=camera camera_driver:=aurora \
  display:=:0 xauthority:=/var/run/lightdm/root/:0
```

云端 VLM 必须确认比赛规则允许公网，并在赛场网络下完成成功率和 8 秒时限验收；回放或兜底文案不能证明真实后端可用。

## 7. 验证边界

本地无硬件检查：

```powershell
python -m unittest discover -s tests -v
python -m unittest discover -s src/smartcar_task/test -v
python -m unittest discover -s src/smartcar_tools/test -v
```

这些测试只能证明路线合同、编辑器隔离、文件写回和媒体回放接口。官方参考层不能证明现场尺寸、外参、定位漂移或路线可通行；完成外参、转向、物理急停和航点实测前，不得表述为已具备竞赛现场运行条件。
