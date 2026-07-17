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
    serial_read_timeout_ms = LaunchConfiguration('serial_read_timeout_ms')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    frame_id = LaunchConfiguration('frame_id')

    robot_parameters = [
        {'usart_port_name': '/dev/ttyACM0',
         'serial_baud_rate': 115200,
         'serial_read_timeout_ms': serial_read_timeout_ms,
         'command_timeout_sec': command_timeout_sec,
         'robot_frame_id': 'base_footprint',
         'odom_frame_id': 'odom_combined',
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
        DeclareLaunchArgument('serial_read_timeout_ms', default_value='100'),
        DeclareLaunchArgument('input_topic', default_value='/cmd_vel_safe'),
        DeclareLaunchArgument('output_topic', default_value='/ackermann_cmd'),
        DeclareLaunchArgument('frame_id', default_value='odom_combined'),

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
