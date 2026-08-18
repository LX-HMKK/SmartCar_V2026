"""Launch the motion-disabled semantic waypoint editor and field reference."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    waypoints_file = LaunchConfiguration("waypoints_file")
    geometry_file = LaunchConfiguration("geometry_file")
    route_planning_file = LaunchConfiguration("route_planning_file")
    start_safety = LaunchConfiguration("start_safety")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("smartcar_safety"),
            "launch",
            "smartcar_safety.launch.py",
        ])),
        launch_arguments={
            "require_odom": "true",
            "require_raw_odom": "true",
            "emergency_stop_on_start": "true",
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(start_safety),
    )

    field_reference = Node(
        package="smartcar_tools",
        executable="field_reference_node",
        name="field_reference",
        output="screen",
        parameters=[{
            "geometry_file": geometry_file,
            "use_sim_time": use_sim_time,
        }],
    )
    editor = Node(
        package="smartcar_tools",
        executable="waypoint_drag_editor",
        name="waypoint_drag_editor",
        output="screen",
        parameters=[{
            "waypoints_file": waypoints_file,
            "geometry_file": geometry_file,
            "use_sim_time": use_sim_time,
        }],
        additional_env={
            "SMARTCAR_ROUTE_PLANNING_CONFIG": route_planning_file,
        },
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="waypoint_editor_rviz",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("smartcar_tools"),
            "rviz",
            "waypoint_editor.rviz",
        ])],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
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
            "geometry_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_tools"),
                "config",
                "routes",
                "field_geometry.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "route_planning_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_tools"),
                "config",
                "routes",
                "route_planning.yaml",
            ]),
        ),
        DeclareLaunchArgument("start_safety", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        safety,
        field_reference,
        editor,
        rviz,
    ])
