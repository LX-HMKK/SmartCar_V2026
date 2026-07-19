"""Launch only the optional speech synthesis consumer for bench testing."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    speech = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("smartcar_speech"),
            "launch",
            "smartcar_speech.launch.py",
        ])),
        launch_arguments={
            "config_file": LaunchConfiguration("config_file"),
            "enabled": LaunchConfiguration("enabled"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_speech"),
                "config",
                "speech.yaml",
            ]),
            description="Speech parameter YAML",
        ),
        DeclareLaunchArgument(
            "enabled",
            default_value="true",
            description="Enable network synthesis and local playback",
        ),
        speech,
    ])
