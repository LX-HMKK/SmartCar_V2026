#!/usr/bin/env python3
"""SmartCar 仿真一键启动：Gazebo + 机器人 + Nav2 + 任务。

用法：
    ros2 launch smartcar_sim sim.launch.py
    ros2 launch smartcar_sim sim.launch.py world:=track   # 使用 track.world
    ros2 launch smartcar_sim sim.launch.py headless:=true  # 无 GUI 模式
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_sim = get_package_share_directory("smartcar_sim")
    pkg_nav2 = get_package_share_directory("smartcar_nav2")
    pkg_safety = get_package_share_directory("smartcar_safety")
    pkg_task = get_package_share_directory("smartcar_task")

    # ── Launch arguments ──
    world = LaunchConfiguration("world", default="track")
    headless = LaunchConfiguration("headless", default="false")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    autostart = LaunchConfiguration("autostart", default="false")

    # ── Gazebo ──
    world_path = PathJoinSubstitution([pkg_sim, "worlds", PythonExpression(["'", world, ".world'"])])

    gz_server = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-v", "3", world_path],
        output="screen",
        condition=UnlessCondition(headless),
        name="gz_server",
        additional_env={"IGN_RELAY": "1"},
    )

    gz_server_headless = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-s", "-v", "3", world_path],
        output="screen",
        condition=IfCondition(headless),
        name="gz_server_headless",
        additional_env={"IGN_RELAY": "1"},
    )

    # Robot is embedded in world file via <include> — no spawn needed

    # ── Robot state publisher (TF: base_footprint → base_link → laser) ──
    # We provide a minimal URDF matching the SDF for the TF tree
    robot_description = """
    <?xml version="1.0"?>
    <robot name="origincar">
      <link name="base_footprint"/>
      <link name="base_link">
        <visual><geometry><box size="0.276 0.164 0.08"/></geometry></visual>
      </link>
      <link name="laser_link"/>
      <joint name="base_footprint_joint" type="fixed">
        <parent link="base_footprint"/>
        <child link="base_link"/>
        <origin xyz="0.0841 0 0.03" rpy="0 0 0"/>
      </joint>
      <joint name="laser_joint" type="fixed">
        <parent link="base_link"/>
        <child link="laser_link"/>
        <origin xyz="-0.05 0 0.23" rpy="0 0 0"/>
      </joint>
    </robot>
    """

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
    )

    # ── Static TF publishers (match real system) ──
    # base_footprint → base_link
    tf_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0841", "0", "0.03", "0", "0", "0", "base_footprint", "base_link"],
        parameters=[{"use_sim_time": True}],
    )

    # base_link → laser
    tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-0.05", "0", "0.23", "0", "0", "0", "base_link", "laser_link"],
        parameters=[{"use_sim_time": True}],
    )

    # Identity TF: Gazebo spawn namespace → nav2 frames
    tf_ns = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "origincar/base_footprint", "base_footprint"],
        parameters=[{"use_sim_time": True}],
    )

    # ── ros_gz_bridge: bridge Gazebo ↔ ROS ──
    bridge_params = PathJoinSubstitution([pkg_sim, "config", "gz_bridge.yaml"])

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "use_sim_time": True,
            "config_file": bridge_params,
        }],
        output="screen",
        additional_env={"IGN_RELAY": "1"},
    )

    # ── EKF: fuse /odom + /imu → /odom_combined ──
    ekf_params = PathJoinSubstitution([pkg_sim, "config", "sim_ekf.yaml"])

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="robot_ekf",
        parameters=[ekf_params, {"use_sim_time": True}],
    )

    # ── Direction guard (enforces forward/reverse per waypoint) ──
    direction_guard = Node(
        package="smartcar_safety",
        executable="direction_guard_node",
        name="direction_guard",
        parameters=[{
            "use_sim_time": True,
            "direction_lease_timeout_sec": 0.50,
            "direction_prepare_timeout_sec": 1.0,
            "raw_odom_timeout_sec": 0.50,
            "stop_settle_sec": 0.15,
        }],
    )

    # ── Safety node (C++, sim mode: skip STM32 heartbeat) ──
    safety_node = Node(
        package="smartcar_safety",
        executable="safety_node_cpp",
        name="safety_node",
        parameters=[PathJoinSubstitution([pkg_safety, "config", "safety.yaml"]), {
            "use_sim_time": True,
            "use_safety_ackermann": False,  # Output Twist, not Ackermann
            "emergency_stop_on_start": False,
            "minimum_voltage": 0.0,  # Skip voltage check in sim
            "odom_timeout_sec": 5.0,  # Generous timeout for sim
        }],
    )

    # ── Nav2 ──
    nav2_params = PathJoinSubstitution([pkg_nav2, "config", "nav2_params.yaml"])
    bt_forward = PathJoinSubstitution([pkg_nav2, "config", "behavior_trees",
        "navigate_to_pose_w_replanning_and_recovery.xml"])
    bt_precise = PathJoinSubstitution([pkg_nav2, "config", "behavior_trees",
        "navigate_to_pose_precise_w_replanning_and_recovery.xml"])
    bt_reverse = PathJoinSubstitution([pkg_nav2, "config", "behavior_trees",
        "navigate_to_pose_reverse_w_replanning_and_recovery.xml"])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2, "launch", "smartcar_nav2.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": nav2_params,
            "bt_xml_file": bt_forward,
            "bt_through_poses_xml_file": bt_forward,  # not used but required
            "autostart": "true",
        }.items(),
    )

    # ── Task node ──
    task_node = Node(
        package="smartcar_task",
        executable="task_node",
        name="task_node",
        parameters=[PathJoinSubstitution([pkg_task, "config", "task.yaml"]), {
            "use_sim_time": True,
            "waypoints_file": PathJoinSubstitution([pkg_nav2, "config", "waypoints", "nav_only.yaml"]),
            "waypoints_calibrated": True,
            "extrinsics_calibrated": True,
            "steering_calibrated": True,
            "emergency_stop_ready": True,
            "operator_approved": True,
        }],
    )

    # ── RViz ──
    rviz_config = PathJoinSubstitution([pkg_sim, "rviz", "sim_nav.rviz"])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="track"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="false"),
        gz_server,
        gz_server_headless,
        robot_state_pub,
        tf_base,
        tf_laser,
        gz_bridge,
        # Start EKF after sim stable
        TimerAction(period=2.0, actions=[ekf_node]),
        # Start navigation stack after EKF
        TimerAction(period=2.0, actions=[nav2_launch]),
        # Direction guard + safety
        TimerAction(period=5.0, actions=[direction_guard]),
        TimerAction(period=5.0, actions=[safety_node]),
        # Task node starts last
        TimerAction(period=8.0, actions=[task_node]),
        # RViz
        rviz,
    ])
