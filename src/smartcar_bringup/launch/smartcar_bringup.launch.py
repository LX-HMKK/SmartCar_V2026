#!/usr/bin/env python3
"""smartcar_bringup 顶层 launch 组合器（学 OriginBot originbot.launch.py 模式）。

自身不启任何 Node，仅 DeclareLaunchArgument + IfCondition + IncludeLaunchDescription 组合：
  - origincar_base/origincar_bringup.launch.py  底盘+IMU+EKF+URDF+静态TF（无条件，底盘必需）
  - ydlidar_ros2_driver/ydlidar_launch.py       LiDAR（IfCondition use_lidar，默认 ydlidar.yaml frame_id=laser）
  - smartcar_bringup/obstacle_detector.launch.py 锥桶检测（IfCondition use_obstacle，重写参数）

命名统一到 RDK 实际：/odom_combined 话题 + odom_combined 帧（见 spec §5）。
端口/帧的深度参数化：use_lidar/use_obstacle/use_nav/odom_frame 经本 launch 暴露；
串口/波特率/IMU/EKF 细节参数仍由 vendor 包自身 config（ydlidar.yaml/ekf.yaml/imu.yaml）持有，
因 vendor launch 未将其暴露为 launch 参数，本期不做改写（spec §8 注）。
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_lidar = LaunchConfiguration('use_lidar')
    use_obstacle = LaunchConfiguration('use_obstacle')
    odom_frame = LaunchConfiguration('odom_frame')

    origincar_dir = get_package_share_directory('origincar_base')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    bringup_dir = get_package_share_directory('smartcar_bringup')

    origincar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(origincar_dir, 'launch', 'origincar_bringup.launch.py')),
    )

    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_dir, 'launch', 'ydlidar_launch.py')),
        condition=IfCondition(use_lidar),
    )

    obstacle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'obstacle_detector.launch.py')),
        launch_arguments={'odom_frame': odom_frame}.items(),
        condition=IfCondition(use_obstacle),
    )

    return LaunchDescription([
        # 功能开关
        DeclareLaunchArgument('use_lidar', default_value='true',
                              description='启动 LiDAR 驱动（/scan）'),
        DeclareLaunchArgument('use_obstacle', default_value='true',
                              description='启动 obstacle_detector（锥桶检测）'),
        DeclareLaunchArgument('use_nav', default_value='false',
                              description='预留：启动 Nav2（本期不实现，smartcar_nav2 未就绪）'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='实车不用仿真时间'),
        # 帧名（透传给 obstacle_detector；EKF/URDF 帧由 vendor config 持有）
        DeclareLaunchArgument('odom_frame', default_value='odom_combined',
                              description='全局里程计帧（对齐 EKF）'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint',
                              description='EKF base 帧'),
        DeclareLaunchArgument('laser_frame', default_value='laser',
                              description='LiDAR 帧（对齐 bringup 静态 TF）'),
        # 组合
        origincar_bringup,
        ydlidar,
        obstacle,
    ])
