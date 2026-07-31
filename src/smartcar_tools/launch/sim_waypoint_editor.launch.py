"""Launch the motion-disabled editor against the simulation waypoint file.

This entry only opens the existing waypoint editor and reference layers.  It
does not start Gazebo, Nav2, or any vehicle driver.  Its default waypoint
document deliberately matches ``smartcar_sim/launch/sim.launch.py``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    waypoints_file = LaunchConfiguration("waypoints_file")
    start_safety = LaunchConfiguration("start_safety")
    use_rviz = LaunchConfiguration("use_rviz")
    use_segment_ui = LaunchConfiguration("use_segment_ui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    editor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("smartcar_tools"),
            "launch",
            "waypoint_editor.launch.py",
        ])),
        launch_arguments={
            "waypoints_file": waypoints_file,
            # The simulator bypasses smartcar_safety, so do not start a second
            # safety stack just to edit its route.  This remains overrideable.
            "start_safety": start_safety,
            "use_rviz": use_rviz,
            "use_segment_ui": use_segment_ui,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "waypoints_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("smartcar_nav2"),
                "config",
                "waypoints",
                "nav_only.yaml",
            ]),
        ),
        DeclareLaunchArgument("start_safety", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_segment_ui", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        editor,
    ])
