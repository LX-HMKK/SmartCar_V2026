"""Compose chassis, LiDAR, obstacle extraction, and velocity safety."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    use_lidar = LaunchConfiguration('use_lidar')
    use_obstacle = LaunchConfiguration('use_obstacle')
    use_safety = LaunchConfiguration('use_safety')
    use_sim_time = LaunchConfiguration('use_sim_time')
    safety_require_scan = LaunchConfiguration('safety_require_scan')
    safety_require_odom = LaunchConfiguration('safety_require_odom')
    safety_require_raw_odom = LaunchConfiguration('safety_require_raw_odom')
    odom_frame = LaunchConfiguration('odom_frame')
    laser_frame = LaunchConfiguration('laser_frame')
    chassis_input_topic = PythonExpression([
        "'/cmd_vel_safe' if '", use_safety,
        "'.lower() in ('true', '1') else '/cmd_vel'",
    ])

    extrinsic_names = (
        'base_x', 'base_y', 'base_z',
        'base_roll', 'base_pitch', 'base_yaw',
        'laser_x', 'laser_y', 'laser_z',
        'laser_roll', 'laser_pitch', 'laser_yaw',
    )
    extrinsics = {
        name: LaunchConfiguration(name)
        for name in extrinsic_names
    }

    origincar_dir = get_package_share_directory('origincar_base')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    bringup_dir = get_package_share_directory('smartcar_bringup')
    safety_dir = get_package_share_directory('smartcar_safety')

    origincar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            origincar_dir, 'launch', 'origincar_bringup.launch.py')),
        launch_arguments={
            'input_topic': chassis_input_topic,
            'use_sim_time': use_sim_time,
            'laser_frame': laser_frame,
            **extrinsics,
        }.items(),
    )
    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            ydlidar_dir, 'launch', 'ydlidar_launch.py')),
        launch_arguments={
            'frame_id': laser_frame,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_lidar),
    )
    obstacle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'obstacle_detector.launch.py')),
        launch_arguments={
            'odom_frame': odom_frame,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_obstacle),
    )
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            safety_dir, 'launch', 'smartcar_safety.launch.py')),
        launch_arguments={
            'require_scan': safety_require_scan,
            'require_odom': safety_require_odom,
            'require_raw_odom': safety_require_raw_odom,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_safety),
    )

    declarations = [
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_obstacle', default_value='true'),
        DeclareLaunchArgument('use_safety', default_value='true'),
        DeclareLaunchArgument('safety_require_scan', default_value='true'),
        DeclareLaunchArgument('safety_require_odom', default_value='true'),
        DeclareLaunchArgument(
            'safety_require_raw_odom', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('odom_frame', default_value='odom_combined'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
    ]
    declarations.extend(
        DeclareLaunchArgument(name, default_value='0.0')
        for name in extrinsic_names
    )
    return LaunchDescription(declarations + [
        origincar_bringup,
        ydlidar,
        obstacle,
        safety,
    ])
