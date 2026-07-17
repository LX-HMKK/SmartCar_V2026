import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import ReplaceString


def generate_launch_description():
    pkg_dir = get_package_share_directory('smartcar_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    bt_through_poses_xml_file = LaunchConfiguration(
        'bt_through_poses_xml_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file')

    declare_bt_xml_file = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(
            pkg_dir, 'config', 'behavior_trees',
            'navigate_to_pose_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree XML file')

    declare_bt_through_poses_xml_file = DeclareLaunchArgument(
        'bt_through_poses_xml_file',
        default_value=os.path.join(
            pkg_dir, 'config', 'behavior_trees',
            'navigate_through_poses_w_replanning_and_recovery.xml'),
        description='Full path to the NavigateThroughPoses behavior tree XML file')

    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Whether to use composed bringup')

    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes')

    # 在参数文件中注入实际的行为树 XML 路径
    params_file = ReplaceString(
        source_file=params_file,
        replacements={
            '<bt_xml_file>': bt_xml_file,
            '<bt_through_poses_xml_file>': bt_through_poses_xml_file,
        })

    # 纯惯导、无 SLAM：直接使用 navigation_launch.py，避免加载 map_server/AMCL
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items())

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_bt_xml_file,
        declare_bt_through_poses_xml_file,
        declare_autostart,
        declare_use_composition,
        declare_use_respawn,
        nav2_launch,
    ])
