#!/usr/bin/env python3
"""obstacle_detector launch（Humble 兼容重写版）。

相对原 ROS1 式 launch（nodes.launch/demo.launch，type=+nodelet+rosparam 不可用）的修正：
  - frame_id: map -> odom_combined（系统无 map 帧，对齐 EKF 全局帧）
  - use_scan: true（系统仅 /scan，无 PCL）；use_pcl: false
  - use_sim_time: false（实车）
  - 去除 /scan -> /merged_scan remap（系统无 scans_merger 产生该话题，直接消费 /scan）

节点：obstacle_extractor_node（包名 obstacle_detector，目录 obstacle_detector_2）。
参数名对照 src/obstacle_extractor.cpp 的 declare_parameter。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_frame = LaunchConfiguration('odom_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_frame', default_value='odom_combined',
            description='obstacle 输出帧，对齐 EKF 全局帧'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='使用 ROS 仿真时钟'),
        Node(
            package='obstacle_detector',
            executable='obstacle_extractor_node',
            name='obstacle_extractor',
            parameters=[{
                'active': True,
                'use_scan': True,
                'use_pcl': False,
                'use_sim_time': use_sim_time,
                'use_split_and_merge': True,
                'circles_from_visibles': True,
                'discard_converted_segments': True,
                'transform_coordinates': True,
                'min_group_points': 10,
                'max_group_distance': 0.1,
                'distance_proportion': 0.00628,
                'max_split_distance': 0.2,
                'max_merge_separation': 0.2,
                'max_merge_spread': 0.2,
                'max_circle_radius': 0.6,
                'radius_enlargement': 0.3,
                'frame_id': odom_frame,
            }],
            output='screen',
        ),
    ])
