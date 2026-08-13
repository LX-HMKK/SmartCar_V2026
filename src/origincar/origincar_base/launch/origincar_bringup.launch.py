"""Launch the OriginCar base, localization, description, and owned TF edges."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ('true', '1'):
        return True
    if value in ('false', '0'):
        return False
    raise RuntimeError(f'{name} must be true or false')


def _ekf_actions(context):
    """Select the estimator inputs while retaining one EKF TF owner."""
    if _as_bool(context, 'carto_slam'):
        return []

    profile = LaunchConfiguration('localization_profile').perform(
        context).strip().lower()
    config_dir = Path(get_package_share_directory('origincar_base')) / 'config'
    profile_files = {
        'wheel_imu': str(config_dir / 'ekf.yaml'),
        'wheel_only': str(config_dir / 'ekf_wheel_only.yaml'),
    }
    try:
        parameter_file = profile_files[profile]
    except KeyError:
        raise RuntimeError(
            'localization_profile must be wheel_imu or wheel_only') from None

    return [Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            parameter_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[('odometry/filtered', 'odom_combined')],
    )]


def generate_launch_description():
    bringup_dir = get_package_share_directory('origincar_base')
    launch_dir = os.path.join(bringup_dir, 'launch')
    imu_config = Path(bringup_dir, 'config', 'imu.yaml')

    skip_converter = LaunchConfiguration('skip_converter')
    input_topic = LaunchConfiguration('input_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_imu_filter = LaunchConfiguration('use_imu_filter')
    use_robot_description = LaunchConfiguration('use_robot_description')
    laser_frame = LaunchConfiguration('laser_frame')
    base_x = LaunchConfiguration('base_x')
    base_y = LaunchConfiguration('base_y')
    base_z = LaunchConfiguration('base_z')
    base_roll = LaunchConfiguration('base_roll')
    base_pitch = LaunchConfiguration('base_pitch')
    base_yaw = LaunchConfiguration('base_yaw')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')

    longitudinal_velocity_scale = LaunchConfiguration('longitudinal_velocity_scale')
    lateral_velocity_scale = LaunchConfiguration('lateral_velocity_scale')
    yaw_velocity_scale = LaunchConfiguration('yaw_velocity_scale')
    gyro_z_scale = LaunchConfiguration('gyro_z_scale')
    gyro_z_bias = LaunchConfiguration('gyro_z_bias')
    steering_command_scale = LaunchConfiguration('steering_command_scale')
    steering_command_offset_rad = LaunchConfiguration('steering_command_offset_rad')
    max_calibrated_steering_command_rad = LaunchConfiguration(
        'max_calibrated_steering_command_rad')

    declarations = [
        DeclareLaunchArgument('carto_slam', default_value='false'),
        DeclareLaunchArgument('skip_converter', default_value='false'),
        DeclareLaunchArgument(
            'input_topic', default_value='/cmd_vel_safe',
            description='Twist input topic for chassis conversion'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'localization_profile', default_value='wheel_imu',
            description=(
                'wheel_imu fuses wheel and gyro yaw rates; wheel_only '
                'loads no IMU sensor into the EKF')),
        DeclareLaunchArgument('use_imu_filter', default_value='false'),
        DeclareLaunchArgument('use_robot_description', default_value='false'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        DeclareLaunchArgument('base_x', default_value='0.0'),
        DeclareLaunchArgument('base_y', default_value='0.0'),
        DeclareLaunchArgument('base_z', default_value='0.0'),
        DeclareLaunchArgument('base_roll', default_value='0.0'),
        DeclareLaunchArgument('base_pitch', default_value='0.0'),
        DeclareLaunchArgument('base_yaw', default_value='0.0'),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.0'),
        DeclareLaunchArgument('laser_roll', default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'longitudinal_velocity_scale', default_value='1.03'),
        DeclareLaunchArgument(
            'lateral_velocity_scale', default_value='1.125'),
        DeclareLaunchArgument('yaw_velocity_scale', default_value='1.0'),
        DeclareLaunchArgument('gyro_z_scale', default_value='1.0'),
        DeclareLaunchArgument('gyro_z_bias', default_value='-0.00036369'),
        DeclareLaunchArgument(
            'steering_command_scale', default_value='1.0'),
        DeclareLaunchArgument(
            'steering_command_offset_rad', default_value='0.0'),
        DeclareLaunchArgument(
            'max_calibrated_steering_command_rad', default_value='0.70'),
    ]

    origincar_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'base_serial.launch.py')),
        launch_arguments={
            'akmcar': 'true',
            'skip_converter': skip_converter,
            'input_topic': input_topic,
            'use_sim_time': use_sim_time,
            'longitudinal_velocity_scale': longitudinal_velocity_scale,
            'lateral_velocity_scale': lateral_velocity_scale,
            'yaw_velocity_scale': yaw_velocity_scale,
            'gyro_z_scale': gyro_z_scale,
            'gyro_z_bias': gyro_z_bias,
            'steering_command_scale': steering_command_scale,
            'steering_command_offset_rad': steering_command_offset_rad,
            'max_calibrated_steering_command_rad':
                max_calibrated_steering_command_rad,
        }.items(),
    )
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robot_mode_description.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_robot_description),
    )

    base_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_link',
        arguments=[
            '--x', base_x, '--y', base_y, '--z', base_z,
            '--roll', base_roll, '--pitch', base_pitch, '--yaw', base_yaw,
            '--frame-id', 'base_footprint', '--child-frame-id', 'base_link',
        ],
    )
    base_to_gyro = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gyro',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_footprint', '--child-frame-id', 'gyro_link',
        ],
    )
    link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link_to_laser',
        arguments=[
            '--x', laser_x, '--y', laser_y, '--z', laser_z,
            '--roll', laser_roll, '--pitch', laser_pitch, '--yaw', laser_yaw,
            '--frame-id', 'base_link', '--child-frame-id', laser_frame,
        ],
    )
    imu_filter = Node(
        condition=IfCondition(use_imu_filter),
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_config, {'use_sim_time': use_sim_time}],
    )
    robot_ekf = OpaqueFunction(function=_ekf_actions)
    joint_state_publisher = Node(
        condition=IfCondition(use_robot_description),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(declarations + [
        origincar_base,
        base_to_link,
        base_to_gyro,
        joint_state_publisher,
        description,
        imu_filter,
        robot_ekf,
        link_to_laser,
    ])
