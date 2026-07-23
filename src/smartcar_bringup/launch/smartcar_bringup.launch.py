"""Compose chassis, LiDAR, obstacle extraction, and velocity safety."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    use_base = LaunchConfiguration('use_base')
    use_lidar = LaunchConfiguration('use_lidar')
    use_obstacle = LaunchConfiguration('use_obstacle')
    use_laser_odometry = LaunchConfiguration('use_laser_odometry')
    use_imu_filter = LaunchConfiguration('use_imu_filter')
    use_robot_description = LaunchConfiguration('use_robot_description')
    use_safety = LaunchConfiguration('use_safety')
    use_sim_time = LaunchConfiguration('use_sim_time')
    safety_require_scan = LaunchConfiguration('safety_require_scan')
    safety_require_odom = LaunchConfiguration('safety_require_odom')
    safety_require_raw_odom = LaunchConfiguration('safety_require_raw_odom')
    safety_emergency_stop_on_start = LaunchConfiguration(
        'safety_emergency_stop_on_start')
    use_safety_ackermann = LaunchConfiguration('use_safety_ackermann')
    use_safety_cpp = LaunchConfiguration('use_safety_cpp')
    odom_frame = LaunchConfiguration('odom_frame')
    laser_frame = LaunchConfiguration('laser_frame')
    chassis_input_topic = PythonExpression([
        "'/cmd_vel_safe' if '", use_safety,
        "'.lower() in ('true', '1') else '/cmd_vel'",
    ])
    skip_converter = PythonExpression([
        "'true' if '", use_safety,
        "'.lower() in ('true', '1') and '",
        use_safety_ackermann,
        "'.lower() in ('true', '1') else 'false'",
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

    sensor_calib_names = (
        'longitudinal_velocity_scale',
        'lateral_velocity_scale',
        'yaw_velocity_scale',
        'gyro_z_scale',
        'gyro_z_bias',
        'steering_command_scale',
        'steering_command_offset_rad',
        'max_calibrated_steering_command_rad',
    )
    sensor_calib = {
        name: LaunchConfiguration(name)
        for name in sensor_calib_names
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
            'skip_converter': skip_converter,
            'use_sim_time': use_sim_time,
            'use_imu_filter': use_imu_filter,
            'use_robot_description': use_robot_description,
            'laser_frame': laser_frame,
            **extrinsics,
            **sensor_calib,
        }.items(),
        condition=IfCondition(use_base),
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
    laser_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'laser_odometry.launch.py')),
        launch_arguments={
            'config_file': LaunchConfiguration(
                'laser_odometry_config_file'),
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_laser_odometry),
    )
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            safety_dir, 'launch', 'smartcar_safety.launch.py')),
        launch_arguments={
            'config_file': LaunchConfiguration('safety_config_file'),
            'require_scan': safety_require_scan,
            'require_odom': safety_require_odom,
            'require_raw_odom': safety_require_raw_odom,
            'emergency_stop_on_start': safety_emergency_stop_on_start,
            'use_sim_time': use_sim_time,
            'use_cpp': use_safety_cpp,
        }.items(),
        condition=IfCondition(use_safety),
    )

    declarations = [
        DeclareLaunchArgument('use_base', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_obstacle', default_value='false'),
        DeclareLaunchArgument(
            'use_laser_odometry', default_value='false'),
        DeclareLaunchArgument(
            'laser_odometry_config_file',
            default_value=os.path.join(
                bringup_dir, 'config', 'laser_odometry.yaml')),
        DeclareLaunchArgument('use_safety', default_value='true'),
        DeclareLaunchArgument(
            'use_safety_ackermann', default_value='true'),
        DeclareLaunchArgument(
            'use_safety_cpp', default_value='true'),
        DeclareLaunchArgument(
            'safety_config_file',
            default_value=os.path.join(
                safety_dir, 'config', 'safety.yaml')),
        DeclareLaunchArgument('use_imu_filter', default_value='false'),
        DeclareLaunchArgument('use_robot_description', default_value='false'),
        DeclareLaunchArgument('safety_require_scan', default_value='true'),
        DeclareLaunchArgument('safety_require_odom', default_value='true'),
        DeclareLaunchArgument(
            'safety_require_raw_odom', default_value='true'),
        DeclareLaunchArgument(
            'safety_emergency_stop_on_start', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('odom_frame', default_value='odom_combined'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
    ]
    declarations.extend(
        DeclareLaunchArgument(name, default_value='0.0')
        for name in extrinsic_names
    )
    declarations.extend([
        DeclareLaunchArgument(
            'longitudinal_velocity_scale', default_value='1.03'),
        DeclareLaunchArgument(
            'lateral_velocity_scale', default_value='1.125'),
        DeclareLaunchArgument(
            'yaw_velocity_scale', default_value='1.0'),
        DeclareLaunchArgument(
            'gyro_z_scale', default_value='1.0'),
        DeclareLaunchArgument(
            'gyro_z_bias', default_value='0.0'),
        DeclareLaunchArgument(
            'steering_command_scale', default_value='0.5'),
        DeclareLaunchArgument(
            'steering_command_offset_rad', default_value='0.0'),
        DeclareLaunchArgument(
            'max_calibrated_steering_command_rad', default_value='0.225'),
    ])
    return LaunchDescription(declarations + [
        origincar_bringup,
        ydlidar,
        obstacle,
        laser_odometry,
        safety,
    ])
