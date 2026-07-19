"""Launch a motion-disabled RViz route editor without Nav2 or a chassis driver."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    route_file = LaunchConfiguration("route_file")
    use_rviz = LaunchConfiguration("use_rviz")
    start_safety = LaunchConfiguration("start_safety")
    use_sim_time = LaunchConfiguration("use_sim_time")

    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("smartcar_safety"),
            "launch",
            "smartcar_safety.launch.py",
        ])),
        launch_arguments={
            "require_scan": "true",
            "require_odom": "true",
            "require_raw_odom": "true",
            "emergency_stop_on_start": "true",
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(start_safety),
    )

    editor = Node(
        package="smartcar_tools",
        executable="route_editor_node",
        name="route_editor",
        output="screen",
        parameters=[{
            "route_file": route_file,
            "latch_emergency_stop": True,
            "use_sim_time": use_sim_time,
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="route_editor_rviz",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("smartcar_tools"),
            "rviz",
            "route_editor.rviz",
        ])],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "route_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_tools"),
                "config",
                "routes",
                "full_course_route.yaml",
            ]),
            description="Route YAML to load and atomically update",
        ),
        DeclareLaunchArgument(
            "start_safety",
            default_value="true",
            description=(
                "Start a standalone emergency-stopped safety node; set false "
                "when reusing an already running safety node"
            ),
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        safety,
        editor,
        rviz,
    ])
