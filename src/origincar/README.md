# Origincar 使用说明

## 编译

```bash
# 默认工作空间 /userdata/dev_ws/
cd /userdata/dev_ws/
colcon build --symlink-install
```

## 底盘和控制

启动底盘
```bash
ros2 launch origincar_base origincar_bringup.launch.py
```

启动键盘控制
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
以下按键进行控制
```
   u    i    o
   j    k    l
   m    ,    .
```
键盘节点会发布 速度消息 到 /cmd_vel 话题，底盘节点订阅速度消息实现键盘控制

## USB 相机驱动与图像可视化
```bash
ros2 launch origincar_bringup usb_websocket_display.launch.py
```
运行成功后，在同一网络的PC端，打开浏览器，输入 http://IP:8000，选择“web展示端”，即可查看图像和算法效果，IP为OriginBot的IP地址。

## 深度相机驱动与图像可视化
启动相机

```bash
ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py

```
启动rqt_image_view
```
ros2 run rqt_image_view rqt_image_view
```
