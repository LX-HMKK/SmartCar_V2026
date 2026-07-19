"""Launch the isolated, explicitly armed full-course navigation test."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


MOTION_GATES = (
    "waypoints_calibrated",
    "extrinsics_calibrated",
    "steering_calibrated",
    "emergency_stop_ready",
    "operator_approved",
)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    route_file = LaunchConfiguration("route_file")
    field_bt = PathJoinSubstitution([
        FindPackageShare("smartcar_nav2"),
        "config",
        "behavior_trees",
        "navigate_through_poses_field_test.xml",
    ])

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("smartcar_bringup"),
            "launch",
            "smartcar_bringup.launch.py",
        ])),
        launch_arguments={
            "use_base": LaunchConfiguration("use_base"),
            "use_lidar": LaunchConfiguration("use_lidar"),
            "use_obstacle": LaunchConfiguration("use_obstacle"),
            "use_laser_odometry": LaunchConfiguration(
                "use_laser_odometry"
            ),
            "laser_odometry_config_file": LaunchConfiguration(
                "laser_odometry_config_file"
            ),
            "use_safety": "true",
            "safety_require_scan": "true",
            "safety_require_odom": "true",
            "safety_require_raw_odom": "true",
            "safety_emergency_stop_on_start": "true",
            "use_sim_time": use_sim_time,
            "odom_frame": "odom_combined",
            "laser_frame": LaunchConfiguration("laser_frame"),
            **{
                name: LaunchConfiguration(name)
                for name in (
                    "base_x", "base_y", "base_z",
                    "base_roll", "base_pitch", "base_yaw",
                    "laser_x", "laser_y", "laser_z",
                    "laser_roll", "laser_pitch", "laser_yaw",
                )
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
            "autostart": "true",
            "params_file": PathJoinSubstitution([
                FindPackageShare("smartcar_nav2"),
                "config",
                "field_test_nav2_params.yaml",
            ]),
            "bt_through_poses_xml_file": field_bt,
        }.items(),
    )

    runner = Node(
        package="smartcar_tools",
        executable="navigation_runner",
        name="navigation_test_runner",
        output="screen",
        parameters=[{
            "route_file": route_file,
            "behavior_tree": field_bt,
            "base_frame": "base_footprint",
            "sensor_timeout_sec": LaunchConfiguration(
                "sensor_timeout_sec"
            ),
            "navigation_timeout_sec": LaunchConfiguration(
                "navigation_timeout_sec"
            ),
            "arm_timeout_sec": 30.0,
            "use_laser_odometry": LaunchConfiguration(
                "use_laser_odometry"
            ),
            "laser_odometry_calibrated": LaunchConfiguration(
                "laser_odometry_calibrated"
            ),
            "laser_odom_topic": LaunchConfiguration("laser_odom_topic"),
            "laser_reset_service": LaunchConfiguration(
                "laser_reset_service"
            ),
            "use_sim_time": use_sim_time,
            **{
                gate: LaunchConfiguration(gate)
                for gate in MOTION_GATES
            },
        }],
    )

    declarations = [
        DeclareLaunchArgument("use_base", default_value="true"),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("use_obstacle", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "use_laser_odometry",
            default_value="false",
            description=(
                "Require a separately launched scan-to-scan /odom_laser "
                "source and admit it as a navigation-test gate"
            ),
        ),
        DeclareLaunchArgument(
            "laser_odometry_calibrated",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "laser_odom_topic",
            default_value="/odom_laser",
        ),
        DeclareLaunchArgument(
            "laser_reset_service",
            default_value="/smartcar/localization/reset_laser_odometry",
            description="Optional std_srvs/Trigger reset service for RF2O",
        ),
        DeclareLaunchArgument(
            "laser_odometry_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_bringup"),
                "config",
                "laser_odometry.yaml",
            ]),
        ),
        DeclareLaunchArgument("laser_frame", default_value="laser"),
        DeclareLaunchArgument("sensor_timeout_sec", default_value="0.50"),
        DeclareLaunchArgument(
            "navigation_timeout_sec",
            default_value="180.0",
        ),
        DeclareLaunchArgument(
            "route_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_tools"),
                "config",
                "routes",
                "full_course_route.yaml",
            ]),
        ),
    ]
    declarations.extend(
        DeclareLaunchArgument(gate, default_value="false")
        for gate in MOTION_GATES
    )
    declarations.extend(
        DeclareLaunchArgument(name, default_value="0.0")
        for name in (
            "base_x", "base_y", "base_z",
            "base_roll", "base_pitch", "base_yaw",
            "laser_x", "laser_y", "laser_z",
            "laser_roll", "laser_pitch", "laser_yaw",
        )
    )
    return LaunchDescription(declarations + [base, navigation, runner])
