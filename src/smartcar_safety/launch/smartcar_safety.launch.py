"""Launch the fail-closed SmartCar velocity safety node.

When use_cpp is true (the default) the C++ implementation is used for
substantially lower CPU on ARM64.  Set use_cpp:=false to fall back to the
Python reference implementation.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("smartcar_safety")
    config_file = LaunchConfiguration("config_file")
    direction_guard_config_file = LaunchConfiguration(
        "direction_guard_config_file")
    require_scan = LaunchConfiguration("require_scan")
    require_odom = LaunchConfiguration("require_odom")
    require_raw_odom = LaunchConfiguration("require_raw_odom")
    require_depth_points = LaunchConfiguration("require_depth_points")
    emergency_stop_on_start = LaunchConfiguration("emergency_stop_on_start")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_cpp = LaunchConfiguration("use_cpp")

    common_params = [
        config_file,
        {
            "require_scan": require_scan,
            "require_odom": require_odom,
            "require_raw_odom": require_raw_odom,
            "require_depth_points": require_depth_points,
            "emergency_stop_on_start": emergency_stop_on_start,
            "use_sim_time": use_sim_time,
        },
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=os.path.join(package_dir, "config", "safety.yaml"),
            description="Safety node parameter file",
        ),
        DeclareLaunchArgument(
            "direction_guard_config_file",
            default_value=os.path.join(
                package_dir, "config", "direction_guard.yaml"),
            description="Direction guard parameter file",
        ),
        DeclareLaunchArgument(
            "require_scan",
            default_value="true",
            description="Require fresh /scan before forwarding velocity",
        ),
        DeclareLaunchArgument(
            "require_odom",
            default_value="true",
            description="Require fresh /odom_combined before forwarding velocity",
        ),
        DeclareLaunchArgument(
            "require_raw_odom",
            default_value="true",
            description="Require fresh raw /odom before forwarding velocity",
        ),
        DeclareLaunchArgument(
            "require_depth_points",
            default_value="false",
            description=(
                "Require fresh /smartcar/depth/points before forwarding "
                "velocity"
            ),
        ),
        DeclareLaunchArgument(
            "emergency_stop_on_start",
            default_value="false",
            description="Latch emergency stop before processing commands",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the ROS simulation clock",
        ),
        DeclareLaunchArgument(
            "use_cpp",
            default_value="true",
            description="Use C++ safety_node (false = Python fallback)",
        ),
        Node(
            package="smartcar_safety",
            executable="direction_guard_node",
            name="direction_guard",
            output="screen",
            parameters=[direction_guard_config_file],
        ),
        # C++ implementation (default).
        Node(
            condition=IfCondition(use_cpp),
            package="smartcar_safety",
            executable="safety_node_cpp",
            name="safety_node",
            output="screen",
            parameters=common_params,
        ),
        # Python reference implementation (fallback).
        Node(
            condition=UnlessCondition(use_cpp),
            package="smartcar_safety",
            executable="safety_node",
            name="safety_node",
            output="screen",
            parameters=common_params,
        ),
    ])
