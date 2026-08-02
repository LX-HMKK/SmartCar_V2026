"""Launch an isolated QR test with either a camera or a still image."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DRIVER_TOPICS = {
    "aurora": "/aurora/rgb/image_raw",
    "usb": "/image",
    "mipi": "/image_raw",
}


def _as_bool(value, name):
    normalized = str(value).strip().lower()
    if normalized in ("true", "1"):
        return True
    if normalized in ("false", "0"):
        return False
    raise RuntimeError(f"{name} must be true or false")


def _runtime_actions(context):
    input_source = LaunchConfiguration(
        "input_source").perform(context).strip().lower()
    camera_driver = LaunchConfiguration(
        "camera_driver").perform(context).strip().lower()
    configured_topic = LaunchConfiguration(
        "image_topic").perform(context).strip()
    use_sim_time = _as_bool(
        LaunchConfiguration("use_sim_time").perform(context),
        "use_sim_time",
    )
    if input_source not in ("camera", "file"):
        raise RuntimeError("input_source must be camera or file")
    if camera_driver not in DRIVER_TOPICS:
        raise RuntimeError("camera_driver must be aurora, usb, or mipi")

    actions = []
    if input_source == "camera":
        selected_driver = camera_driver
        use_camera = "true"
        source_topic = configured_topic or DRIVER_TOPICS[camera_driver]
    else:
        image_file = LaunchConfiguration("image_file").perform(context).strip()
        if not image_file:
            raise RuntimeError("image_file is required for file input")
        if not Path(image_file).expanduser().is_file():
            raise RuntimeError(f"image_file does not exist: {image_file}")
        selected_driver = "none"
        use_camera = "false"
        source_topic = configured_topic or "/smartcar/test/image"
        actions.append(Node(
            package="smartcar_tools",
            executable="image_replay_node",
            name="qr_image_replay",
            output="screen",
            parameters=[{
                "image_file": image_file,
                "image_topic": source_topic,
                "frame_id": LaunchConfiguration(
                    "image_frame").perform(context),
                "publish_rate_hz": float(LaunchConfiguration(
                    "publish_rate_hz").perform(context)),
                "use_sim_time": use_sim_time,
            }],
        ))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("smartcar_vision"),
            "launch",
            "smartcar_vision.launch.py",
        ])),
        launch_arguments={
            "camera_driver": selected_driver,
            "use_camera": use_camera,
            "use_services": "true",
            "use_zbar": "true",
            "use_sim_time": "true" if use_sim_time else "false",
            "image_topic": source_topic,
            "usb_video_device": LaunchConfiguration(
                "usb_video_device").perform(context),
            "config_file": LaunchConfiguration(
                "config_file").perform(context),
        }.items(),
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "input_source",
            default_value="camera",
            description="Image source: camera or file",
        ),
        DeclareLaunchArgument("camera_driver", default_value="usb"),
        DeclareLaunchArgument("usb_video_device", default_value="/dev/video0"),
        DeclareLaunchArgument("image_topic", default_value=""),
        DeclareLaunchArgument("image_file", default_value=""),
        DeclareLaunchArgument(
            "image_frame", default_value="smartcar_test_camera"),
        DeclareLaunchArgument("publish_rate_hz", default_value="2.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_vision"),
                "config",
                "vision.yaml",
            ]),
        ),
        OpaqueFunction(function=_runtime_actions),
    ])
