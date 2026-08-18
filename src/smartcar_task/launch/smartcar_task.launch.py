"""Launch the semantic SmartCar mission orchestrator."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "waypoints_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_nav2"),
                "config",
                "waypoints",
                "default_waypoints.yaml",
            ]),
            description="Semantic waypoint YAML file",
        ),
        DeclareLaunchArgument(
            "c_zone_direction",
            default_value="counterclockwise",
            description=(
                "C-zone route direction: counterclockwise uses the reviewed "
                "YAML; clockwise mirrors its reviewed C-zone constraints in memory"),
        ),
        DeclareLaunchArgument(
            "task_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_task"),
                "config",
                "task.yaml",
            ]),
            description="Mission parameter YAML file",
        ),
        DeclareLaunchArgument(
            "autostart_mission",
            default_value="false",
            description="Start mission without an explicit Trigger request",
        ),
        DeclareLaunchArgument(
            "navigation_test_end_segment_id",
            default_value="",
            description=(
                "Run only the contiguous route prefix through this segment; "
                "empty runs the complete route"),
        ),
        DeclareLaunchArgument(
            "supervised_p_to_a_only",
            default_value="false",
            description=(
                "Permit the uncalibrated pure-navigation P-to-A test only; "
                "all motion gates still require explicit confirmation"),
        ),
        DeclareLaunchArgument(
            "supervised_p_to_c1_only",
            default_value="false",
            description=(
                "Permit the uncalibrated pure-navigation P-to-A-to-C1 test "
                "only; all motion gates still require explicit confirmation"),
        ),
        DeclareLaunchArgument(
            "supervised_full_route",
            default_value="false",
            description=(
                "Permit one explicitly supervised pure-navigation run of the "
                "complete all-forward route"),
        ),
        DeclareLaunchArgument(
            "supervised_competition_mode",
            default_value="false",
            description=(
                "Permit one e-stop-latched, manually triggered semantic "
                "competition route without changing the route YAML"),
        ),
        DeclareLaunchArgument(
            "steering_calibrated",
            default_value="false",
            description="Confirm measured Ackermann steering calibration",
        ),
        DeclareLaunchArgument(
            "emergency_stop_ready",
            default_value="false",
            description="Confirm an operator-accessible emergency stop",
        ),
        DeclareLaunchArgument(
            "operator_approved",
            default_value="false",
            description="Explicitly authorize physical mission motion",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the ROS simulation clock",
        ),
        DeclareLaunchArgument(
            "barcode_reader_image_topic",
            default_value="/aurora/rgb/image_raw",
            description="Image topic remapped into the on-demand zbar reader",
        ),
        DeclareLaunchArgument(
            "qr_reader_preloaded",
            default_value="false",
            description=(
                "Use the launch-managed barcode reader rather than starting "
                "one at the QR task point"),
        ),
        Node(
            package="smartcar_task",
            executable="task_node",
            name="task_node",
            output="screen",
            parameters=[
                LaunchConfiguration("task_config_file"),
                {
                    "waypoints_file": LaunchConfiguration("waypoints_file"),
                    "c_zone_direction": LaunchConfiguration("c_zone_direction"),
                    "steering_calibrated": LaunchConfiguration(
                        "steering_calibrated"),
                    "emergency_stop_ready": LaunchConfiguration(
                        "emergency_stop_ready"),
                    "operator_approved": LaunchConfiguration(
                        "operator_approved"),
                    "autostart_mission": LaunchConfiguration(
                        "autostart_mission"),
                    "navigation_test_end_segment_id": LaunchConfiguration(
                        "navigation_test_end_segment_id"),
                    "supervised_p_to_a_only": LaunchConfiguration(
                        "supervised_p_to_a_only"),
                    "supervised_p_to_c1_only": LaunchConfiguration(
                        "supervised_p_to_c1_only"),
                    "supervised_full_route": LaunchConfiguration(
                        "supervised_full_route"),
                    "supervised_competition_mode": LaunchConfiguration(
                        "supervised_competition_mode"),
                    "barcode_reader_image_topic": LaunchConfiguration(
                        "barcode_reader_image_topic"),
                    "qr_reader_preloaded": LaunchConfiguration(
                        "qr_reader_preloaded"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                },
            ],
        ),
    ])
