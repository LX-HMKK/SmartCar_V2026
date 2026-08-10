"""Compose the complete competition stack with explicit motion gates."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


CAMERA_FRAMES = {
    "aurora": "rgb_camera_link",
    "usb": "default_usb_cam",
    "mipi": "default_cam",
}
CAMERA_TOPICS = {
    "aurora": "/aurora/rgb/image_raw",
    "usb": "/image",
    "mipi": "/image_raw",
}
EXTERNAL_IMAGE_TOPIC = "/smartcar/vision/image"
DEPTH_CAMERA_DRIVER = "aurora"
DEPTH_POINT_CLOUD_INPUT_TOPIC = "/aurora/points2"
DEPTH_POINT_CLOUD_TOPIC = "/smartcar/depth/points"
DEPTH_CAMERA_INPUT_FRAME = "depth_camera_link_1"
DEPTH_CAMERA_FRAME = "depth_camera_link_1"
VALID_CAMERA_DRIVERS = tuple(CAMERA_FRAMES)
MOTION_GATES = (
    "waypoints_calibrated",
    "extrinsics_calibrated",
    "steering_calibrated",
    "emergency_stop_ready",
    "operator_approved",
)


def _as_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ("true", "1"):
        return True
    if value in ("false", "0"):
        return False
    raise RuntimeError(f"{name} must be true or false")


def _resolve_camera_source(context):
    use_camera = _as_bool(context, "use_camera")
    configured_topic = LaunchConfiguration("image_topic").perform(context).strip()
    if not use_camera:
        return "none", configured_topic or EXTERNAL_IMAGE_TOPIC

    camera_driver = LaunchConfiguration(
        "camera_driver").perform(context).strip().lower()
    if camera_driver not in VALID_CAMERA_DRIVERS:
        raise RuntimeError("camera_driver must be aurora, usb, or mipi")
    return camera_driver, configured_topic or CAMERA_TOPICS[camera_driver]


def _vision_and_camera_actions(context):
    use_camera = _as_bool(context, "use_camera")
    use_vision = _as_bool(context, "use_vision")
    use_depth_camera = _as_bool(context, "use_depth_camera")
    if not use_camera and not use_vision and not use_depth_camera:
        return []

    selected_driver, selected_topic = _resolve_camera_source(context)
    if use_depth_camera:
        if use_camera and selected_driver != DEPTH_CAMERA_DRIVER:
            raise RuntimeError(
                "use_depth_camera requires camera_driver=aurora when RGB "
                "camera streaming is also enabled")
        selected_driver = DEPTH_CAMERA_DRIVER
        selected_topic = (
            LaunchConfiguration("image_topic").perform(context).strip()
            or CAMERA_TOPICS[selected_driver]
        )

    start_camera_driver = use_camera or use_depth_camera
    aurora_rgb_enable = use_camera or use_vision
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("smartcar_vision"),
            "launch",
            "smartcar_vision.launch.py",
        ])),
        launch_arguments={
            "camera_driver": selected_driver,
            "image_topic": selected_topic,
            "usb_video_device": LaunchConfiguration(
                "usb_video_device").perform(context),
            "use_camera": "true" if start_camera_driver else "false",
            "use_services": "true" if use_vision else "false",
            # QR scanning is task-driven: barcode_reader is NOT launched at
            # system start; task_node starts it on demand at QR waypoints.
            "use_zbar": "false",
            "config_file": LaunchConfiguration(
                "vision_config_file").perform(context),
            "use_sim_time": LaunchConfiguration(
                "use_sim_time").perform(context),
            "aurora_rgb_enable": "true" if aurora_rgb_enable else "false",
            "aurora_depth_enable": "true" if use_depth_camera else "false",
            "aurora_point_cloud_enable": (
                "true" if use_depth_camera else "false"),
            "aurora_rgb_fps": LaunchConfiguration("aurora_rgb_fps"),
            "aurora_ir_fps": LaunchConfiguration("aurora_ir_fps"),
            "aurora_resolution_mode_index": LaunchConfiguration(
                "aurora_resolution_mode_index"),
            "aurora_heart_enable": LaunchConfiguration("aurora_heart_enable"),
        }.items(),
    )]

    if use_camera or (use_vision and use_depth_camera):
        frame_override = LaunchConfiguration(
            "camera_frame").perform(context).strip()
        camera_frame = frame_override or CAMERA_FRAMES[selected_driver]
        actions.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="link_to_camera_sensor",
            arguments=[
                "--x", LaunchConfiguration("camera_x").perform(context),
                "--y", LaunchConfiguration("camera_y").perform(context),
                "--z", LaunchConfiguration("camera_z").perform(context),
                "--roll", LaunchConfiguration("camera_roll").perform(context),
                "--pitch", LaunchConfiguration("camera_pitch").perform(context),
                "--yaw", LaunchConfiguration("camera_yaw").perform(context),
                "--frame-id", "base_link",
                "--child-frame-id", camera_frame,
            ],
        ))
    if use_depth_camera:
        actions.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="link_to_depth_camera_sensor",
            arguments=[
                "--x", LaunchConfiguration("depth_camera_x").perform(context),
                "--y", LaunchConfiguration("depth_camera_y").perform(context),
                "--z", LaunchConfiguration("depth_camera_z").perform(context),
                "--roll", LaunchConfiguration(
                    "depth_camera_roll").perform(context),
                "--pitch", LaunchConfiguration(
                    "depth_camera_pitch").perform(context),
                "--yaw", LaunchConfiguration("depth_camera_yaw").perform(context),
                "--frame-id", "base_link",
                "--child-frame-id", LaunchConfiguration(
                    "depth_camera_frame").perform(context),
            ],
        ))
        actions.append(Node(
            package="smartcar_vision",
            executable="depth_pointcloud_relay",
            name="depth_pointcloud_relay",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration(
                    "depth_point_cloud_input_topic").perform(context),
                "output_topic": LaunchConfiguration(
                    "depth_point_cloud_topic").perform(context),
                "expected_frame": LaunchConfiguration(
                    "depth_camera_input_frame").perform(context),
                "output_frame": LaunchConfiguration(
                    "depth_camera_frame").perform(context),
                "stale_timeout_sec": 0.50,
                # Aurora capture stamps remain authoritative.  RDK can
                # occasionally schedule the serial/costmap chain for a few
                # hundred milliseconds; keep a bounded acceptance window
                # below the safety watchdog instead of dropping that frame.
                "max_capture_age_sec": 0.35,
                "max_future_skew_sec": 0.05,
                # Bound Aurora's organized cloud before Nav2's two obstacle
                # layers process it.  The relay keeps samples spread across
                # the full image, so narrow obstacles remain represented.
                "max_points": 1024,
                # A 5 Hz obstacle update moves the vehicle no more than 6 cm
                # at 0.30 m/s and stays well inside the 0.50 s depth
                # watchdog, while leaving CPU time for Nav2 control renewals.
                "max_publish_rate_hz": 5.0,
                "use_sim_time": _as_bool(context, "use_sim_time"),
            }],
        ))
    return actions


def _navigation_actions(context):
    if not _as_bool(context, "use_nav"):
        return []
    requested_overlay = LaunchConfiguration("nav2_params_overlay_file").perform(
        context).strip()
    if requested_overlay:
        raise RuntimeError(
            "smartcar_system does not accept external Nav2 parameter overlays")
    overlay = ""
    allow_params_overlay = "false"
    if _as_bool(context, "use_depth_camera"):
        overlay = os.path.join(
            get_package_share_directory("smartcar_nav2"),
            "config",
            "depth_camera_obstacle_overlay.yaml",
        )
        allow_params_overlay = "true"
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("smartcar_nav2"),
            "launch",
            "navigation_launch.py",
        ])),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": LaunchConfiguration("nav_autostart"),
            "nav2_params_overlay_file": overlay,
            "allow_params_overlay": allow_params_overlay,
        }.items(),
    )]


def _task_actions(context):
    if not _as_bool(context, "use_task"):
        return []

    _, image_topic = _resolve_camera_source(context)
    if (
        _as_bool(context, "use_depth_camera")
        and _as_bool(context, "use_vision")
    ):
        image_topic = (
            LaunchConfiguration("image_topic").perform(context).strip()
            or CAMERA_TOPICS[DEPTH_CAMERA_DRIVER]
        )
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("smartcar_task"),
            "launch",
            "smartcar_task.launch.py",
        ])),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "waypoints_file": LaunchConfiguration("waypoints_file"),
            "autostart_mission": LaunchConfiguration("autostart_mission"),
            "navigation_test_end_segment_id": LaunchConfiguration(
                "navigation_test_end_segment_id"),
            "supervised_p_to_a_only": LaunchConfiguration(
                "supervised_p_to_a_only"),
            "supervised_p_to_c1_only": LaunchConfiguration(
                "supervised_p_to_c1_only"),
            "qr_handoff_test_mode": LaunchConfiguration(
                "qr_handoff_test_mode"),
            "use_laser_odometry": LaunchConfiguration("use_laser_odometry"),
            "laser_odometry_calibrated": LaunchConfiguration(
                "laser_odometry_calibrated"),
            "use_depth_camera": LaunchConfiguration("use_depth_camera"),
            "depth_camera_calibrated": LaunchConfiguration(
                "depth_camera_calibrated"),
            "barcode_reader_image_topic": image_topic,
            **{
                name: LaunchConfiguration(name)
                for name in MOTION_GATES
            },
        }.items(),
    )]


def _validate_configuration(context):
    if _as_bool(context, "use_base") and not _as_bool(context, "use_safety"):
        raise RuntimeError("use_base requires use_safety")
    use_lidar = _as_bool(context, "use_lidar")
    use_depth_camera = _as_bool(context, "use_depth_camera")
    if _as_bool(context, "use_nav") and not (
        use_lidar or use_depth_camera
    ):
        raise RuntimeError(
            "use_nav requires use_lidar=true or use_depth_camera=true")
    if _as_bool(context, "use_laser_odometry"):
        required = [
            name for name in ("use_base", "use_lidar")
            if not _as_bool(context, name)
        ]
        if required:
            raise RuntimeError(
                "use_laser_odometry requires: " + ",".join(required))
    short_drive_test = _as_bool(context, "short_drive_test")
    supervised_prefixes = tuple(
        (parameter_name, segment_id)
        for parameter_name, segment_id in (
            ("supervised_p_to_a_only", "p_to_qr"),
            ("supervised_p_to_c1_only", "qr_to_vlm"),
        )
        if _as_bool(context, parameter_name)
    )
    if len(supervised_prefixes) > 1:
        raise RuntimeError("only one supervised navigation prefix may be enabled")
    if supervised_prefixes:
        required_true = (
            "use_base",
            "use_safety",
            "use_nav",
            "use_task",
            "safety_emergency_stop_on_start",
        )
        if short_drive_test:
            required_true += ("use_depth_camera",)
        else:
            required_true += ("use_lidar",)
        required_false = (
            "use_camera",
            "use_vision",
            "autostart_mission",
        )
        if not short_drive_test:
            required_false += ("use_depth_camera",)
        invalid = [
            name for name in required_true if not _as_bool(context, name)
        ]
        invalid.extend(
            name for name in required_false if _as_bool(context, name)
        )
        if (
            LaunchConfiguration("navigation_test_end_segment_id").perform(
                context
            ).strip()
            != supervised_prefixes[0][1]
        ):
            invalid.append(
                "navigation_test_end_segment_id=" + supervised_prefixes[0][1]
            )
        if invalid:
            raise RuntimeError(
                "supervised navigation prefix requires: " + ",".join(invalid)
            )
    if short_drive_test:
        required_true = (
            "use_base",
            "use_safety",
            "use_nav",
            "use_task",
            "use_depth_camera",
            "safety_emergency_stop_on_start",
        )
        required_false = (
            "use_lidar",
            "use_camera",
            "use_vision",
            "autostart_mission",
        )
        invalid = [
            name for name in required_true if not _as_bool(context, name)
        ]
        invalid.extend(
            name for name in required_false if _as_bool(context, name)
        )
        if not supervised_prefixes:
            invalid.append(
                "one_of(supervised_p_to_a_only,supervised_p_to_c1_only)"
            )
        if invalid:
            raise RuntimeError(
                "short_drive_test requires: " + ",".join(invalid))
    if _as_bool(context, "qr_handoff_test_mode"):
        required = (
            "use_nav",
            "use_task",
            "use_camera",
            "use_vision",
        )
        missing = [name for name in required if not _as_bool(context, name)]
        if missing:
            raise RuntimeError(
                "qr_handoff_test_mode requires: " + ",".join(missing))
    if not _as_bool(context, "autostart_mission"):
        return []
    required_components = (
        "use_base",
        "use_safety",
        "use_nav",
        "use_task",
        "nav_autostart",
    )
    missing = [
        name for name in required_components
        if not _as_bool(context, name)
    ]
    missing.extend(
        name for name in MOTION_GATES
        if not _as_bool(context, name)
    )
    if (
        _as_bool(context, "use_laser_odometry")
        and not _as_bool(context, "laser_odometry_calibrated")
    ):
        missing.append("laser_odometry_calibrated")
    if (
        _as_bool(context, "use_depth_camera")
        and not _as_bool(context, "depth_camera_calibrated")
    ):
        missing.append("depth_camera_calibrated")
    if missing:
        raise RuntimeError(
            "autostart_mission requires: " + ",".join(missing))
    return []


def generate_launch_description():
    use_base = LaunchConfiguration("use_base")
    use_lidar = LaunchConfiguration("use_lidar")
    use_laser_odometry = LaunchConfiguration("use_laser_odometry")
    use_imu_filter = LaunchConfiguration("use_imu_filter")
    use_robot_description = LaunchConfiguration("use_robot_description")
    use_safety = LaunchConfiguration("use_safety")
    use_nav = LaunchConfiguration("use_nav")
    use_visualization = LaunchConfiguration("use_visualization")
    use_speech = LaunchConfiguration("use_speech")
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav_autostart = LaunchConfiguration("nav_autostart")
    waypoints_file = LaunchConfiguration("waypoints_file")

    extrinsic_names = (
        "base_x", "base_y", "base_z",
        "base_roll", "base_pitch", "base_yaw",
        "laser_x", "laser_y", "laser_z",
        "laser_roll", "laser_pitch", "laser_yaw",
    )
    extrinsic_defaults = {
        "base_x": "0.0841",
        "base_z": "0.03",
        "laser_x": "-0.05",
        "laser_z": "0.23",
        "laser_yaw": "1.5708",
    }
    camera_extrinsic_defaults = {
        "camera_x": "0.1205",
        "camera_z": "0.11",
    }
    # The base_footprint origin is the rear axle. The reported installation is
    # directly above the front axle (wheelbase 0.189 m), 0.15 m above ground.
    depth_camera_extrinsic_defaults = {
        "depth_camera_x": "0.1049",
        "depth_camera_z": "0.1200",
        # Aurora driver frame: x left, y up, z forward. Convert it to
        # base_link (x forward, y left, z up).
        "depth_camera_roll": "1.5708",
        "depth_camera_yaw": "1.5708",
    }
    sensor_calib_names = (
        "longitudinal_velocity_scale",
        "lateral_velocity_scale",
        "yaw_velocity_scale",
        "gyro_z_scale",
        "gyro_z_bias",
        "steering_command_scale",
        "steering_command_offset_rad",
        "max_calibrated_steering_command_rad",
    )

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("smartcar_bringup"),
            "launch",
            "smartcar_bringup.launch.py",
        ])),
        launch_arguments={
            "use_base": use_base,
            "use_lidar": use_lidar,
            "use_laser_odometry": use_laser_odometry,
            "use_imu_filter": use_imu_filter,
            "use_robot_description": use_robot_description,
            "laser_odometry_config_file": LaunchConfiguration(
                "laser_odometry_config_file"),
            "safety_config_file": LaunchConfiguration(
                "safety_config_file"),
            "use_safety": use_safety,
            "use_safety_ackermann": "true",
            "use_sim_time": use_sim_time,
            # A depth-only build has no /scan producer. The active obstacle
            # sensor must be safety's corresponding freshness heartbeat.
            "safety_require_scan": use_lidar,
            "safety_require_odom": LaunchConfiguration(
                "safety_require_odom"),
            "safety_require_raw_odom": LaunchConfiguration(
                "safety_require_raw_odom"),
            "safety_require_depth_points": LaunchConfiguration(
                "use_depth_camera"),
            "safety_emergency_stop_on_start": LaunchConfiguration(
                "safety_emergency_stop_on_start"),
            "use_safety_cpp": LaunchConfiguration("use_safety_cpp"),
            "laser_frame": LaunchConfiguration("laser_frame"),
            **{
                name: LaunchConfiguration(name)
                for name in extrinsic_names
            },
            **{
                name: LaunchConfiguration(name)
                for name in sensor_calib_names
            },
        }.items(),
    )
    field_reference = Node(
        package="smartcar_tools",
        executable="field_reference_node",
        name="field_reference",
        namespace="/smartcar",
        output="screen",
        condition=IfCondition(use_visualization),
    )
    waypoint_markers = Node(
        package="smartcar_tools",
        executable="waypoint_viz",
        name="waypoint_viz",
        parameters=[{"waypoints_file": waypoints_file}],
        output="screen",
        condition=IfCondition(use_visualization),
    )
    navigation = OpaqueFunction(function=_navigation_actions)
    speech = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("smartcar_speech"),
            "launch",
            "smartcar_speech.launch.py",
        ])),
        launch_arguments={
            "config_file": LaunchConfiguration("speech_config_file"),
            "enabled": use_speech,
        }.items(),
        condition=IfCondition(use_speech),
    )

    declarations = [
        DeclareLaunchArgument("use_base", default_value="true"),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument(
            "use_laser_odometry", default_value="false"),
        DeclareLaunchArgument("use_safety", default_value="true"),
        DeclareLaunchArgument("use_imu_filter", default_value="false"),
        DeclareLaunchArgument(
            "use_robot_description", default_value="false"
        ),
        DeclareLaunchArgument("use_nav", default_value="true"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_vision", default_value="true"),
        DeclareLaunchArgument("use_depth_camera", default_value="false"),
        DeclareLaunchArgument("use_task", default_value="true"),
        DeclareLaunchArgument("use_visualization", default_value="false"),
        DeclareLaunchArgument("use_speech", default_value="false"),
        DeclareLaunchArgument("autostart_mission", default_value="false"),
        DeclareLaunchArgument(
            "navigation_test_end_segment_id", default_value=""),
        DeclareLaunchArgument(
            "supervised_p_to_a_only", default_value="false"),
        DeclareLaunchArgument(
            "supervised_p_to_c1_only", default_value="false"),
        DeclareLaunchArgument(
            "qr_handoff_test_mode", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("nav_autostart", default_value="true"),
        DeclareLaunchArgument("safety_require_scan", default_value="true"),
        DeclareLaunchArgument("safety_require_odom", default_value="true"),
        DeclareLaunchArgument(
            "safety_require_raw_odom", default_value="true"),
        DeclareLaunchArgument(
            "safety_emergency_stop_on_start", default_value="false"),
        DeclareLaunchArgument(
            "use_safety_cpp", default_value="true",
            description="Use C++ safety_node (false = Python fallback)"),
        DeclareLaunchArgument(
            "waypoints_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_nav2"),
                "config",
                "waypoints",
                "default_waypoints.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "laser_odometry_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_bringup"),
                "config",
                "laser_odometry.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "safety_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_safety"),
                "config",
                "safety.yaml",
            ]),
        ),
        DeclareLaunchArgument("camera_driver", default_value="usb"),
        DeclareLaunchArgument("aurora_rgb_fps", default_value="10"),
        DeclareLaunchArgument("aurora_ir_fps", default_value="5"),
        DeclareLaunchArgument(
            "aurora_resolution_mode_index", default_value="0"),
        DeclareLaunchArgument("aurora_heart_enable", default_value="false"),
        DeclareLaunchArgument(
            "depth_point_cloud_input_topic",
            default_value=DEPTH_POINT_CLOUD_INPUT_TOPIC,
        ),
        DeclareLaunchArgument(
            "depth_point_cloud_topic",
            default_value=DEPTH_POINT_CLOUD_TOPIC,
        ),
        DeclareLaunchArgument(
            "depth_camera_frame", default_value=DEPTH_CAMERA_FRAME),
        DeclareLaunchArgument(
            "depth_camera_input_frame", default_value=DEPTH_CAMERA_INPUT_FRAME),
        DeclareLaunchArgument("nav2_params_overlay_file", default_value=""),
        DeclareLaunchArgument("image_topic", default_value=""),
        DeclareLaunchArgument(
            "vision_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_vision"),
                "config",
                "vision.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "speech_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_speech"),
                "config",
                "speech.yaml",
            ]),
        ),
        DeclareLaunchArgument("usb_video_device", default_value="/dev/video0"),
        DeclareLaunchArgument("camera_frame", default_value=""),
        DeclareLaunchArgument("laser_frame", default_value="laser"),
        DeclareLaunchArgument(
            "waypoints_calibrated", default_value="false"),
        DeclareLaunchArgument(
            "extrinsics_calibrated", default_value="false"),
        DeclareLaunchArgument(
            "steering_calibrated", default_value="false"),
        DeclareLaunchArgument(
            "emergency_stop_ready", default_value="false"),
        DeclareLaunchArgument(
            "operator_approved", default_value="false"),
        DeclareLaunchArgument(
            "laser_odometry_calibrated", default_value="false"),
        DeclareLaunchArgument(
            "depth_camera_calibrated", default_value="false"),
        DeclareLaunchArgument("short_drive_test", default_value="false"),
        DeclareLaunchArgument(
            "longitudinal_velocity_scale", default_value="1.03"),
        DeclareLaunchArgument(
            "lateral_velocity_scale", default_value="1.125"),
        DeclareLaunchArgument(
            "yaw_velocity_scale", default_value="1.0"),
        DeclareLaunchArgument(
            "gyro_z_scale", default_value="1.0"),
        DeclareLaunchArgument(
            "gyro_z_bias", default_value="-0.00036369"),
        DeclareLaunchArgument(
            "steering_command_scale", default_value="1.0"),
        DeclareLaunchArgument(
            "steering_command_offset_rad", default_value="0.0"),
        DeclareLaunchArgument(
            "max_calibrated_steering_command_rad", default_value="0.70"),
    ]
    declarations.extend(
        DeclareLaunchArgument(
            name, default_value=extrinsic_defaults.get(name, "0.0"))
        for name in extrinsic_names
    )
    declarations.extend(
        DeclareLaunchArgument(
            name, default_value=camera_extrinsic_defaults.get(name, "0.0")
        )
        for name in (
            "camera_x", "camera_y", "camera_z",
            "camera_roll", "camera_pitch", "camera_yaw",
        )
    )
    declarations.extend(
        DeclareLaunchArgument(
            name,
            default_value=depth_camera_extrinsic_defaults.get(name, "0.0"),
        )
        for name in (
            "depth_camera_x", "depth_camera_y", "depth_camera_z",
            "depth_camera_roll", "depth_camera_pitch", "depth_camera_yaw",
        )
    )
    return LaunchDescription(declarations + [
        OpaqueFunction(function=_validate_configuration),
        base,
        navigation,
        field_reference,
        waypoint_markers,
        OpaqueFunction(function=_vision_and_camera_actions),
        speech,
        OpaqueFunction(function=_task_actions),
    ])
