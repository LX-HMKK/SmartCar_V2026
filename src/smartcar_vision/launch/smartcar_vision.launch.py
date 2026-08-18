"""Launch the Aurora depth camera, zbar reader, and vision services."""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


AURORA_RGB_TOPIC = "/aurora/rgb/image_raw"


def _as_bool(value, name):
    normalized = str(value).strip().lower()
    if normalized in ("true", "1"):
        return True
    if normalized in ("false", "0"):
        return False
    raise RuntimeError(f"{name} must be true or false")


def _camera_action(
    *,
    aurora_rgb_enable,
    aurora_depth_enable,
    aurora_point_cloud_enable,
    aurora_rgb_fps,
    aurora_ir_fps,
    aurora_resolution_mode_index,
    aurora_heart_enable,
):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("deptrum-ros-driver-aurora930"),
            "launch",
            "aurora930_launch.py",
        ])),
        launch_arguments={
            "rgb_enable": aurora_rgb_enable,
            "rgb_fps": aurora_rgb_fps,
            "ir_fps": aurora_ir_fps,
            "resolution_mode_index": aurora_resolution_mode_index,
            "depth_enable": aurora_depth_enable,
            "ir_enable": "false",
            "point_cloud_enable": aurora_point_cloud_enable,
            "rgbd_enable": "false",
            "align_mode": "false",
            "depth_correction": "false",
            # Aurora 930 firmware 1.7.2 rejects the SDK heartbeat despite
            # maintaining a healthy depth stream. Depth freshness is
            # instead enforced fail-closed by smartcar_safety.
            "heart_enable": aurora_heart_enable,
        }.items(),
    )


def _runtime_actions(context):
    configured_topic = LaunchConfiguration("image_topic").perform(context).strip()
    use_camera = _as_bool(
        LaunchConfiguration("use_camera").perform(context), "use_camera")
    use_services = _as_bool(
        LaunchConfiguration("use_services").perform(context), "use_services")
    use_zbar = _as_bool(
        LaunchConfiguration("use_zbar").perform(context), "use_zbar")
    use_sim_time = _as_bool(
        LaunchConfiguration("use_sim_time").perform(context), "use_sim_time")
    aurora_rgb_enable = _as_bool(
        LaunchConfiguration("aurora_rgb_enable").perform(context),
        "aurora_rgb_enable")
    aurora_depth_enable = _as_bool(
        LaunchConfiguration("aurora_depth_enable").perform(context),
        "aurora_depth_enable")
    aurora_point_cloud_enable = _as_bool(
        LaunchConfiguration("aurora_point_cloud_enable").perform(context),
        "aurora_point_cloud_enable")
    aurora_heart_enable = _as_bool(
        LaunchConfiguration("aurora_heart_enable").perform(context),
        "aurora_heart_enable")
    aurora_rgb_fps = LaunchConfiguration("aurora_rgb_fps").perform(context)
    aurora_ir_fps = LaunchConfiguration("aurora_ir_fps").perform(context)
    aurora_resolution_mode_index = LaunchConfiguration(
        "aurora_resolution_mode_index").perform(context)
    if aurora_point_cloud_enable and not aurora_depth_enable:
        raise RuntimeError("aurora_point_cloud_enable requires aurora_depth_enable")
    # The Aurora driver requires RGB not to exceed its IR/depth rate. Keep the
    # depth pipeline at its validated 5 Hz setting even when RGB is enabled.
    effective_aurora_rgb_fps = (
        aurora_ir_fps if aurora_depth_enable else aurora_rgb_fps)
    source_topic = configured_topic or AURORA_RGB_TOPIC

    actions = []
    if use_camera:
        camera_action = _camera_action(
            aurora_rgb_enable="true" if aurora_rgb_enable else "false",
            aurora_depth_enable="true" if aurora_depth_enable else "false",
            aurora_point_cloud_enable=(
                "true" if aurora_point_cloud_enable else "false"),
            aurora_rgb_fps=effective_aurora_rgb_fps,
            aurora_ir_fps=aurora_ir_fps,
            aurora_resolution_mode_index=aurora_resolution_mode_index,
            aurora_heart_enable="true" if aurora_heart_enable else "false",
        )
        actions.append(camera_action)

    if use_services:
        actions.append(Node(
            package="smartcar_vision",
            executable="vision_node",
            name="vision_node",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "image_topic": source_topic,
                    "use_sim_time": use_sim_time,
                },
            ],
        ))
    if use_zbar:
        actions.append(Node(
            package="zbar_ros",
            executable="barcode_reader",
            name="barcode_reader",
            output="screen",
            parameters=[{
                "throttle_repeated_barcodes": 0.0,
                "use_sim_time": use_sim_time,
            }],
            remappings=[
                ("image", source_topic),
                ("barcode", "/barcode"),
            ],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_camera",
            default_value="true",
            description="Start the Aurora depth camera",
        ),
        DeclareLaunchArgument(
            "use_services",
            default_value="true",
            description="Start SmartCar vision service node",
        ),
        DeclareLaunchArgument(
            "use_zbar",
            default_value="true",
            description="Start zbar barcode reader",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the ROS simulation clock",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="",
            description="Override Aurora RGB for external image replay",
        ),
        DeclareLaunchArgument(
            "aurora_rgb_enable",
            default_value="true",
            description="Enable Aurora RGB frames for QR/VLM",
        ),
        DeclareLaunchArgument(
            "aurora_depth_enable",
            default_value="false",
            description="Enable Aurora depth frames for obstacle sensing",
        ),
        DeclareLaunchArgument(
            "aurora_point_cloud_enable",
            default_value="false",
            description="Publish Aurora PointCloud2 when depth is enabled",
        ),
        DeclareLaunchArgument(
            "aurora_rgb_fps",
            default_value="10",
            description="Aurora RGB frame rate",
        ),
        DeclareLaunchArgument(
            "aurora_ir_fps",
            default_value="10",
            description="Aurora IR/depth frame rate for obstacle sensing",
        ),
        DeclareLaunchArgument(
            "aurora_resolution_mode_index",
            default_value="0",
            description="Aurora depth resolution mode (320x200)",
        ),
        DeclareLaunchArgument(
            "aurora_heart_enable",
            default_value="false",
            description="Enable the Aurora SDK heartbeat",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_vision"),
                "config",
                "vision.yaml",
            ]),
            description="Vision service parameter file",
        ),
        OpaqueFunction(function=_runtime_actions),
    ])
