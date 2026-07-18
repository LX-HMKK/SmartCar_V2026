"""Launch the optional SmartCar speech consumer."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_speech"),
                "config",
                "speech.yaml",
            ]),
            description="Speech synthesis parameter YAML file",
        ),
        DeclareLaunchArgument(
            "enabled",
            default_value="false",
            description="Enable network TTS and local audio playback",
        ),
        Node(
            package="smartcar_speech",
            executable="speech_node",
            name="speech_node",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {"enabled": LaunchConfiguration("enabled")},
            ],
        ),
    ])
