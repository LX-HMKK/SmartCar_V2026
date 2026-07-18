from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import launch_ros.actions


def generate_launch_description():
    akmcar = LaunchConfiguration('akmcar')
    wheelbase = LaunchConfiguration('wheelbase')
    max_steering_angle = LaunchConfiguration('max_steering_angle')
    command_timeout_sec = LaunchConfiguration('command_timeout_sec')
    max_integration_dt_sec = LaunchConfiguration('max_integration_dt_sec')
    serial_read_timeout_ms = LaunchConfiguration('serial_read_timeout_ms')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    frame_id = LaunchConfiguration('frame_id')
    longitudinal_velocity_scale = LaunchConfiguration(
        'longitudinal_velocity_scale')
    lateral_velocity_scale = LaunchConfiguration('lateral_velocity_scale')
    yaw_velocity_scale = LaunchConfiguration('yaw_velocity_scale')
    gyro_z_scale = LaunchConfiguration('gyro_z_scale')
    gyro_z_bias = LaunchConfiguration('gyro_z_bias')
    steering_command_scale = LaunchConfiguration('steering_command_scale')
    steering_command_offset_rad = LaunchConfiguration(
        'steering_command_offset_rad')
    max_calibrated_steering_command_rad = LaunchConfiguration(
        'max_calibrated_steering_command_rad')

    robot_parameters = [
        {'usart_port_name': '/dev/ttyACM0',
         'serial_baud_rate': 115200,
         'serial_read_timeout_ms': serial_read_timeout_ms,
         'command_timeout_sec': command_timeout_sec,
         'max_integration_dt_sec': max_integration_dt_sec,
         'robot_frame_id': 'base_footprint',
         'odom_frame_id': 'odom',
         'longitudinal_velocity_scale': longitudinal_velocity_scale,
         'lateral_velocity_scale': lateral_velocity_scale,
         'yaw_velocity_scale': yaw_velocity_scale,
         'gyro_z_scale': gyro_z_scale,
         'gyro_z_bias': gyro_z_bias,
         'steering_command_scale': steering_command_scale,
         'steering_command_offset_rad': steering_command_offset_rad,
         'max_calibrated_steering_command_rad':
             max_calibrated_steering_command_rad,
         'product_number': 0}
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'akmcar',
            default_value='true',
            description='Use Ackermann command conversion when true'
        ),
        DeclareLaunchArgument('wheelbase', default_value='0.189'),
        DeclareLaunchArgument('max_steering_angle', default_value='0.45'),
        DeclareLaunchArgument('command_timeout_sec', default_value='0.35'),
        DeclareLaunchArgument('max_integration_dt_sec', default_value='0.25'),
        DeclareLaunchArgument('serial_read_timeout_ms', default_value='100'),
        DeclareLaunchArgument('input_topic', default_value='/cmd_vel_safe'),
        DeclareLaunchArgument('output_topic', default_value='/ackermann_cmd'),
        DeclareLaunchArgument('frame_id', default_value='odom_combined'),
        DeclareLaunchArgument(
            'longitudinal_velocity_scale', default_value='1.03'),
        DeclareLaunchArgument(
            'lateral_velocity_scale', default_value='1.125'),
        DeclareLaunchArgument('yaw_velocity_scale', default_value='1.0'),
        DeclareLaunchArgument('gyro_z_scale', default_value='1.0'),
        DeclareLaunchArgument('gyro_z_bias', default_value='0.0'),
        DeclareLaunchArgument('steering_command_scale', default_value='0.5'),
        DeclareLaunchArgument(
            'steering_command_offset_rad', default_value='0.0'),
        DeclareLaunchArgument(
            'max_calibrated_steering_command_rad', default_value='0.225'),

        launch_ros.actions.Node(
            condition=IfCondition(akmcar),
            package='origincar_base',
            executable='origincar_base_node',
            parameters=robot_parameters + [
                {'command_mode': 'ackermann',
                 'akm_cmd_vel': output_topic}
            ],
        ),

        launch_ros.actions.Node(
            condition=IfCondition(akmcar),
            package='origincar_base',
            executable='cmd_vel_to_ackermann_drive.py',
            name='cmd_vel_to_ackermann_drive',
            parameters=[{
                'wheelbase': wheelbase,
                'max_steering_angle': max_steering_angle,
                'input_topic': input_topic,
                'output_topic': output_topic,
                'frame_id': frame_id,
            }],
        ),

        launch_ros.actions.Node(
            condition=UnlessCondition(akmcar),
            package='origincar_base',
            executable='origincar_base_node',
            parameters=robot_parameters + [
                {'command_mode': 'twist',
                 'cmd_vel': input_topic}
            ],
        )
    ])
