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
            "config_file",
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
            "waypoints_calibrated",
            default_value="false",
            description="Allow motion only after replacing placeholder coordinates",
        ),
        DeclareLaunchArgument(
            "extrinsics_calibrated",
            default_value="false",
            description="Confirm measured base, laser, and camera extrinsics",
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
        Node(
            package="smartcar_task",
            executable="task_node",
            name="task_node",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "waypoints_file": LaunchConfiguration("waypoints_file"),
                    "waypoints_calibrated": LaunchConfiguration(
                        "waypoints_calibrated"),
                    "extrinsics_calibrated": LaunchConfiguration(
                        "extrinsics_calibrated"),
                    "steering_calibrated": LaunchConfiguration(
                        "steering_calibrated"),
                    "emergency_stop_ready": LaunchConfiguration(
                        "emergency_stop_ready"),
                    "operator_approved": LaunchConfiguration(
                        "operator_approved"),
                    "autostart_mission": LaunchConfiguration(
                        "autostart_mission"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                },
            ],
        ),
    ])
