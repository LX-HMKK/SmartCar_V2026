import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('smartcar_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    bt_through_poses_xml_file = LaunchConfiguration(
        'bt_through_poses_xml_file')
    waypoints_file = LaunchConfiguration('waypoints_file')
    autostart = LaunchConfiguration('autostart')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Nav2 parameters file')

    declare_bt_xml_file = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(
            pkg_dir, 'config', 'behavior_trees',
            'navigate_to_pose_w_replanning_and_recovery.xml'),
        description='Behavior tree XML file')

    declare_bt_through_poses_xml_file = DeclareLaunchArgument(
        'bt_through_poses_xml_file',
        default_value=os.path.join(
            pkg_dir, 'config', 'behavior_trees',
            'navigate_through_poses_w_replanning_and_recovery.xml'),
        description='NavigateThroughPoses behavior tree XML file')

    declare_waypoints_file = DeclareLaunchArgument(
        'waypoints_file',
        default_value=os.path.join(
            pkg_dir, 'config', 'waypoints', 'default_waypoints.yaml'),
        description='Waypoint sequence YAML file')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically activate Nav2 lifecycle nodes')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'nav2_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'bt_xml_file': bt_xml_file,
            'bt_through_poses_xml_file': bt_through_poses_xml_file,
            'autostart': autostart,
        }.items())

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_bt_xml_file,
        declare_bt_through_poses_xml_file,
        declare_waypoints_file,
        declare_autostart,
        nav2_bringup,
    ])
