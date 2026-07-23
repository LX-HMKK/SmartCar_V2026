"""Compose the complete competition stack with explicit motion gates."""
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


def _vision_and_camera_actions(context):
    use_camera = _as_bool(context, "use_camera")
    use_vision = _as_bool(context, "use_vision")
    if not use_camera and not use_vision:
        return []

    camera_driver = LaunchConfiguration(
        "camera_driver").perform(context).strip().lower()
    if camera_driver not in VALID_CAMERA_DRIVERS:
        raise RuntimeError("camera_driver must be aurora, usb, or mipi")
    configured_topic = LaunchConfiguration(
        "image_topic").perform(context).strip()
    selected_driver = camera_driver if use_camera else "none"
    selected_topic = (
        configured_topic
        if configured_topic
        else (
            ""
            if use_camera
            else "/smartcar/vision/image"
        )
    )

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
            "use_camera": "true" if use_camera else "false",
            "use_services": "true" if use_vision else "false",
            "use_zbar": "true" if use_vision else "false",
            "config_file": LaunchConfiguration(
                "vision_config_file").perform(context),
            "use_sim_time": LaunchConfiguration(
                "use_sim_time").perform(context),
        }.items(),
    )]

    if use_camera:
        frame_override = LaunchConfiguration(
            "camera_frame").perform(context).strip()
        camera_frame = frame_override or CAMERA_FRAMES[camera_driver]
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
    return actions


def _validate_configuration(context):
    if _as_bool(context, "use_base") and not _as_bool(context, "use_safety"):
        raise RuntimeError("use_base requires use_safety")
    if _as_bool(context, "use_laser_odometry"):
        required = [
            name for name in ("use_base", "use_lidar")
            if not _as_bool(context, name)
        ]
        if required:
            raise RuntimeError(
                "use_laser_odometry requires: " + ",".join(required))
    if not _as_bool(context, "autostart_mission"):
        return []
    required_components = (
        "use_base",
        "use_safety",
        "use_nav",
        "use_vision",
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
    if missing:
        raise RuntimeError(
            "autostart_mission requires: " + ",".join(missing))
    return []


def generate_launch_description():
    use_base = LaunchConfiguration("use_base")
    use_lidar = LaunchConfiguration("use_lidar")
    use_obstacle = LaunchConfiguration("use_obstacle")
    use_laser_odometry = LaunchConfiguration("use_laser_odometry")
    use_imu_filter = LaunchConfiguration("use_imu_filter")
    use_robot_description = LaunchConfiguration("use_robot_description")
    use_safety = LaunchConfiguration("use_safety")
    use_nav = LaunchConfiguration("use_nav")
    use_task = LaunchConfiguration("use_task")
    use_speech = LaunchConfiguration("use_speech")
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav_autostart = LaunchConfiguration("nav_autostart")
    autostart_mission = LaunchConfiguration("autostart_mission")
    waypoints_file = LaunchConfiguration("waypoints_file")

    extrinsic_names = (
        "base_x", "base_y", "base_z",
        "base_roll", "base_pitch", "base_yaw",
        "laser_x", "laser_y", "laser_z",
        "laser_roll", "laser_pitch", "laser_yaw",
    )
    extrinsic_defaults = {
        "laser_yaw": "1.5708",
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
            "use_obstacle": use_obstacle,
            "use_laser_odometry": use_laser_odometry,
            "use_imu_filter": use_imu_filter,
            "use_robot_description": use_robot_description,
            "laser_odometry_config_file": LaunchConfiguration(
                "laser_odometry_config_file"),
            "safety_config_file": LaunchConfiguration(
                "safety_config_file"),
            "use_safety": use_safety,
            "use_sim_time": use_sim_time,
            "safety_require_scan": LaunchConfiguration(
                "safety_require_scan"),
            "safety_require_odom": LaunchConfiguration(
                "safety_require_odom"),
            "safety_require_raw_odom": LaunchConfiguration(
                "safety_require_raw_odom"),
            "safety_emergency_stop_on_start": LaunchConfiguration(
                "safety_emergency_stop_on_start"),
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
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("smartcar_nav2"),
            "launch",
            "smartcar_nav2.launch.py",
        ])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": nav_autostart,
            "use_waypoint_follower": use_task,
            "waypoints_file": waypoints_file,
        }.items(),
        condition=IfCondition(use_nav),
    )
    task = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("smartcar_task"),
            "launch",
            "smartcar_task.launch.py",
        ])),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "waypoints_file": waypoints_file,
            "autostart_mission": autostart_mission,
            "use_laser_odometry": use_laser_odometry,
            "laser_odometry_calibrated": LaunchConfiguration(
                "laser_odometry_calibrated"),
            **{
                name: LaunchConfiguration(name)
                for name in MOTION_GATES
            },
        }.items(),
        condition=IfCondition(use_task),
    )
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
        DeclareLaunchArgument("use_obstacle", default_value="false"),
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
        DeclareLaunchArgument("use_task", default_value="true"),
        DeclareLaunchArgument("use_speech", default_value="false"),
        DeclareLaunchArgument("autostart_mission", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("nav_autostart", default_value="true"),
        DeclareLaunchArgument("safety_require_scan", default_value="true"),
        DeclareLaunchArgument("safety_require_odom", default_value="true"),
        DeclareLaunchArgument(
            "safety_require_raw_odom", default_value="true"),
        DeclareLaunchArgument(
            "safety_emergency_stop_on_start", default_value="false"),
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
        DeclareLaunchArgument("camera_driver", default_value="aurora"),
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
            "longitudinal_velocity_scale", default_value="1.03"),
        DeclareLaunchArgument(
            "lateral_velocity_scale", default_value="1.125"),
        DeclareLaunchArgument(
            "yaw_velocity_scale", default_value="1.0"),
        DeclareLaunchArgument(
            "gyro_z_scale", default_value="1.0"),
        DeclareLaunchArgument(
            "gyro_z_bias", default_value="0.000853"),
        DeclareLaunchArgument(
            "steering_command_scale", default_value="0.5"),
        DeclareLaunchArgument(
            "steering_command_offset_rad", default_value="0.0"),
        DeclareLaunchArgument(
            "max_calibrated_steering_command_rad", default_value="0.225"),
    ]
    declarations.extend(
        DeclareLaunchArgument(
            name, default_value=extrinsic_defaults.get(name, "0.0"))
        for name in extrinsic_names
    )
    declarations.extend(
        DeclareLaunchArgument(name, default_value="0.0")
        for name in (
            "camera_x", "camera_y", "camera_z",
            "camera_roll", "camera_pitch", "camera_yaw",
        )
    )
    return LaunchDescription(declarations + [
        OpaqueFunction(function=_validate_configuration),
        base,
        navigation,
        OpaqueFunction(function=_vision_and_camera_actions),
        speech,
        task,
    ])
