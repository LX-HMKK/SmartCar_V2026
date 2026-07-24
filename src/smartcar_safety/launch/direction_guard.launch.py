"""Launch the fail-closed command-direction guard."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("smartcar_safety")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=os.path.join(
                package_dir, "config", "direction_guard.yaml"),
            description="Direction guard parameter file",
        ),
        Node(
            package="smartcar_safety",
            executable="direction_guard_node",
            name="direction_guard",
            output="screen",
            parameters=[LaunchConfiguration("config_file")],
        ),
    ])
