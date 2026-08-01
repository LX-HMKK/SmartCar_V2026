#!/usr/bin/env python3
"""SmartCar 仿真一键启动：Gazebo + 机器人 + Nav2 + 任务。

用法：
    ros2 launch smartcar_sim sim.launch.py
    ros2 launch smartcar_sim sim.launch.py world:=track              # 使用 track.world
    ros2 launch smartcar_sim sim.launch.py headless:=true            # headless 模式（无 Gazebo GUI，有 RViz）
    ros2 launch smartcar_sim sim.launch.py headless:=true use_rviz:=false  # 纯 headless（无 GUI + 无 RViz）

模式说明：
    headless:=false（默认）: Gazebo GUI + ogre 渲染引擎，适合可视化调试
    headless:=true           : Gazebo server-only (-s)，无 GUI 窗口，适合自动化/CI
    use_rviz（默认 false）   : 独立于 headless，控制是否启动 RViz2 导航可视化
    headless:=false + use_rviz:=true : 同时启动 Gazebo GUI + RViz（高 CPU 负载，调试专用）
    headless:=true  + use_rviz:=true : Gazebo 无窗口 + RViz 可视化（推荐开发组合）
"""

import os
from pathlib import Path
import sys
import tempfile

# Launch files are installed as standalone modules. Keep the local-Gazebo
# speed-profile helper beside this file so both source and install workspaces
# resolve the same constrained profiles.
_LAUNCH_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_DIRECTORY not in sys.path:
    sys.path.insert(0, _LAUNCH_DIRECTORY)

from sim_speed_profiles import (
    resolve_sim_speed_profile,
    speed_overlay_path,
    validate_speed_overlay,
    write_merged_nav2_overlay,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_sim = get_package_share_directory("smartcar_sim")
    pkg_nav2 = get_package_share_directory("smartcar_nav2")
    pkg_tools = get_package_share_directory("smartcar_tools")

    # Keep every simulator participant on the local development host.  Do not
    # inject a custom DDS locator profile: it can split multi-process discovery.

    # ── 清理残留 SHM/临时文件（必须在 DDS/进程启动前执行）──
    # 修复原因：PathJoinSubstitution 返回 Substitution 对象，在
    # ExecuteProcess 的 cmd 参数中可能未正确解析。改用 os.path.join
    # 直接构造字符串路径，与 dds_config 的处理方式保持一致。
    cleanup_script = os.path.join(pkg_sim, "scripts", "sim_cleanup.sh")
    sim_cleanup = ExecuteProcess(
        cmd=["bash", cleanup_script],
        name="sim_cleanup",
    )

    # ── Launch arguments ──
    world = LaunchConfiguration("world", default="track")
    headless = LaunchConfiguration("headless", default="false")
    use_rviz = LaunchConfiguration("use_rviz", default="false")
    run_route = LaunchConfiguration("run_route", default="false")
    sim_speed_profile = LaunchConfiguration("sim_speed_profile")
    active_nav2_overlay_file = LaunchConfiguration(
        "sim_active_nav2_overlay_file")
    shutdown_on_route_exit = LaunchConfiguration(
        "shutdown_on_route_exit", default="false")
    waypoints_file = LaunchConfiguration("waypoints_file")
    results_file = LaunchConfiguration(
        "results_file", default="/tmp/auto_train_results.json")
    use_through_poses_lc = LaunchConfiguration("use_through_poses", default="true")
    start_goal_id = LaunchConfiguration("start_goal_id", default="")
    end_goal_id = LaunchConfiguration("end_goal_id", default="")

    # ── Gazebo ──
    # 注意：gz_server 和 gz_server_headless 互斥，只启动其中一个。
    # headless:=false → 带 GUI（ogre 渲染）；headless:=true → server-only (-s)。
    world_path = PathJoinSubstitution([pkg_sim, "worlds", PythonExpression(["'", world, ".world'"])])

    gz_server = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-v", "3", "--render-engine", "ogre", world_path],
        output="screen",
        condition=UnlessCondition(headless),
        name="gz_server",
    )

    gz_server_headless = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-s", "-v", "3", "--render-engine", "ogre", world_path],
        output="screen",
        condition=IfCondition(headless),
        name="gz_server_headless",
    )

    # DDS environment applies to every process launched below.
    set_rmw = SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    set_localhost = SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1")
    model_path = os.path.join(pkg_sim, "models")
    existing_model_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    set_model_path = SetEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH",
        model_path + (f":{existing_model_path}" if existing_model_path else ""),
    )

    # Robot is embedded in world file via <include> — no spawn needed

    # ── Static TF publishers (match real system) ──
    # TF chain: odom_combined → base_footprint (odom_combined_relay, /tf)
    #           base_footprint → base_link (static, /tf_static)
    #           base_link → laser_link (static, /tf_static)
    # Note: robot_state_publisher is intentionally NOT used here — all joints are
    # fixed, and having both RSP (/tf) and static_transform_publisher (/tf_static)
    # publish the same frames causes TF lookup conflicts in ROS2 Humble.
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

    # Identity TF: ROS laser_link → Gazebo LiDAR sensor frame.
    # Gazebo publishes /scan with frame_id "origincar/laser_link/lidar_sensor"
    # but Nav2 uses "laser_link". Keep laser_link attached to base_link and add
    # the Gazebo sensor as its child so the TF tree has one parent per frame.
    tf_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "laser_link", "origincar/laser_link/lidar_sensor"],
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
    )

    # ── Odom NaN 过滤中继：/odom → /odom_clean ──
    # Gazebo AckermannSteering 初始发布 NaN 四元数，本节点过滤后转发
    odom_relay = Node(
        package="smartcar_sim",
        executable="odom_relay.py",
        name="odom_relay",
        parameters=[{"use_sim_time": True}],
    )

    # ── Odom combined relay: /odom_clean → /odom_combined + TF ──
    # 绕过 EKF，直接用 Gazebo 地面真值里程计
    # The relay forwards the incoming Gazebo stamp verbatim. Keep its executor
    # on wall time so a delayed /clock cannot stall its startup callbacks.
    odom_combined_relay = Node(
        package="smartcar_sim",
        executable="odom_combined_relay.py",
        name="odom_combined_relay",
        parameters=[{"use_sim_time": False}],
    )

    # ── Direction guard: SKIP in simulation (no lease needed) ──
    # The real system requires forward/reverse leases per waypoint.
    # In simulation, we bypass this and let Nav2 control velocity directly.

    # ── Prior-map keepout stack ──
    # The PGM is a mask, not a localization map. CostmapFilterInfoServer turns
    # it into a KeepoutFilter contract for both Nav2 costmaps. This keeps static
    # route constraints simulation-only and avoids the static-layer startup race.
    map_file = os.path.join(pkg_sim, "maps", "field_map.yaml")
    keepout_mask_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="keepout_mask_server",
        parameters=[{
            "use_sim_time": True,
            "yaml_filename": map_file,
            "frame_id": "odom_combined",
            "topic_name": "/keepout_filter_mask",
        }],
        output="screen",
    )
    keepout_filter_info_server = LifecycleNode(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        namespace="",
        name="keepout_filter_info_server",
        parameters=[{
            "use_sim_time": True,
            "type": 0,
            "filter_info_topic": "/keepout_filter_info",
            "mask_topic": "/keepout_filter_mask",
            "base": 0.0,
            "multiplier": 1.0,
        }],
        output="screen",
    )
    keepout_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_keepout",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            # Gazebo and RViz can briefly starve this process during startup.
            # Leave enough room for the first bond heartbeat before resetting
            # the filter servers.
            "bond_timeout": 20.0,
            "node_names": [
                "keepout_mask_server",
                "keepout_filter_info_server",
            ],
        }],
        output="screen",
    )
    keepout_lifecycle_after_servers = RegisterEventHandler(
        OnProcessStart(
            target_action=keepout_filter_info_server,
            # The manager does not retry a node that was absent at its first
            # autostart pass. Under Gazebo startup load one launch cycle is
            # occasionally not enough for Fast DDS service discovery, which
            # leaves the filter stack inactive and prevents Nav2 from starting.
            # Give both lifecycle services a deterministic readiness window.
            on_start=[TimerAction(
                period=3.0,
                actions=[keepout_lifecycle_manager],
            )],
        )
    )

    # ── Safety bypass: simulation doesn't need safety_node ──
    # cmd_vel chain: controller → /cmd_vel_nav → velocity_smoother → /cmd_vel_candidate → bridge → Gazebo
    # No direction_guard or safety_node (both block velocity without real hardware)

    # ── Nav2 ──
    nav2_fixed_params = PathJoinSubstitution([
        pkg_nav2, "config", "nav2_params_fixed.yaml"
    ])
    nav2_keepout_overlay = PathJoinSubstitution([
        pkg_sim, "config", "nav2_keepout_filter.yaml"
    ])
    bt_forward = PathJoinSubstitution([pkg_nav2, "config", "behavior_trees",
        "navigate_to_pose_w_replanning_and_recovery.xml"])
    bt_precise = PathJoinSubstitution([pkg_nav2, "config", "behavior_trees",
        "navigate_to_pose_precise_w_replanning_and_recovery.xml"])
    bt_reverse = PathJoinSubstitution([pkg_nav2, "config", "behavior_trees",
        "navigate_to_pose_reverse_w_replanning_and_recovery.xml"])
    bt_reverse_handoff = PathJoinSubstitution([
        pkg_nav2, "config", "behavior_trees",
        "navigate_to_pose_reverse_handoff_w_replanning_and_recovery.xml",
    ])
    bt_reverse_through_poses = PathJoinSubstitution([
        pkg_nav2, "config", "behavior_trees",
        "navigate_through_poses_reverse_w_replanning_and_recovery.xml",
    ])
    bt_reverse_locked_through_poses = PathJoinSubstitution([
        pkg_nav2, "config", "behavior_trees",
        "navigate_through_poses_reverse_locked_w_replanning_and_recovery.xml",
    ])

    active_nav2_overlay = None
    generated_speed_overlay = None

    def prepare_sim_speed_overlay(context):
        """Validate a named local speed tier before Gazebo or Nav2 starts."""
        nonlocal active_nav2_overlay
        nonlocal generated_speed_overlay

        config_dir = Path(pkg_sim) / "config"
        profile = resolve_sim_speed_profile(
            sim_speed_profile.perform(context), config_dir
        )
        keepout_overlay_path = Path(nav2_keepout_overlay.perform(context))
        overlay_path = speed_overlay_path(profile, config_dir)
        if overlay_path is None:
            active_nav2_overlay = keepout_overlay_path
        else:
            validate_speed_overlay(profile, overlay_path)
            generated_speed_overlay = Path(tempfile.gettempdir()) / (
                f"smartcar_sim_nav2_{os.getpid()}_"
                f"{profile.name.replace('.', '_')}.yaml"
            )
            active_nav2_overlay = write_merged_nav2_overlay(
                keepout_overlay_path,
                overlay_path,
                generated_speed_overlay,
            )
        return [
            SetLaunchConfiguration(
                "sim_active_nav2_overlay_file", str(active_nav2_overlay)),
            LogInfo(msg=(
                "[sim] Nav2 local-Gazebo speed profile "
                f"{profile.name} ({profile.linear_speed_mps:.2f} m/s); "
                "real-vehicle Nav2 parameters are unchanged"
            )),
        ]

    def remove_generated_speed_overlay(context):
        if generated_speed_overlay is not None:
            generated_speed_overlay.unlink(missing_ok=True)
        return []

    def make_nav2_launch(overlay_path):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_nav2, "launch", "smartcar_nav2.launch.py"])
            ),
            launch_arguments={
                "use_sim_time": "true",
                "params_file": nav2_fixed_params,
                "params_overlay_file": str(overlay_path),
                "bt_xml_file": bt_forward,
                "bt_through_poses_xml_file": bt_forward,
                "autostart": "true",
                # Fast DDS can discover a lifecycle manager before all Nav2
                # lifecycle services are ready. Keep this local to Gazebo;
                # the production Nav2 launch retains its zero-delay default.
                "lifecycle_manager_delay_sec": "2.0",
            }.items(),
        )
    # A lifecycle manager can reactivate the filter after a transient startup
    # delay. Launch Nav2 exactly once: repeating this include creates duplicate
    # controller/planner nodes that fight over the same lifecycle services.
    nav2_started = False

    def launch_nav2_once(context):
        nonlocal nav2_started
        if nav2_started:
            return []
        if active_nav2_overlay is None:
            raise RuntimeError("sim speed profile was not prepared before Nav2")
        nav2_started = True
        return [make_nav2_launch(active_nav2_overlay)]

    nav2_after_keepout = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=keepout_filter_info_server,
            goal_state="active",
            entities=[OpaqueFunction(function=launch_nav2_once)],
        )
    )

    # ── Waypoint visualizer (publishes MarkerArray for RViz) ──
    waypoint_viz = Node(
        package="smartcar_tools",
        executable="waypoint_viz",
        name="waypoint_viz",
        parameters=[{
            "use_sim_time": True,
            "waypoints_file": waypoints_file,
        }],
    )
    field_reference = Node(
        package="smartcar_tools",
        executable="field_reference_node",
        name="field_reference",
        parameters=[{
            "use_sim_time": True,
            "geometry_file": os.path.join(
                pkg_tools, "config", "routes", "field_geometry.yaml"),
        }],
    )

    # ── RViz ──
    # 独立于 headless 模式，由 use_rviz:=true 控制。
    # headless 只影响 Gazebo GUI，不影响 RViz。
    rviz_config = PathJoinSubstitution([pkg_sim, "rviz", "sim_nav.rviz"])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(use_rviz),
        name="sim_rviz",
    )

    auto_train = Node(
        package="smartcar_sim",
        executable="auto_train.py",
        name="auto_train",
        parameters=[{
            "use_sim_time": True,
            "waypoints_file": waypoints_file,
            "forward_behavior_tree": bt_forward,
            "precise_behavior_tree": bt_precise,
            "reverse_behavior_tree": bt_reverse,
            "reverse_handoff_behavior_tree": bt_reverse_handoff,
            "through_poses_behavior_tree": PathJoinSubstitution([
                pkg_nav2, "config", "behavior_trees",
                "navigate_through_poses_w_replanning_and_recovery.xml",
            ]),
            "through_poses_reverse_behavior_tree": PathJoinSubstitution([
                pkg_nav2, "config", "behavior_trees",
                "navigate_through_poses_reverse_w_replanning_and_recovery.xml",
            ]),
            "through_poses_reverse_locked_behavior_tree": (
                bt_reverse_locked_through_poses),
            "use_through_poses": use_through_poses_lc,
            "nav2_params_file": nav2_fixed_params,
            "nav2_params_overlay_file": active_nav2_overlay_file,
            # Numeric-looking launch arguments are normally YAML-coerced when
            # rendered into a node parameter file.  Keep profile names such as
            # "0.20" as strings so AutoTrain can record the exact selected
            # local-only speed tier.
            "sim_speed_profile": ParameterValue(
                sim_speed_profile, value_type=str),
            "results_file": results_file,
            "start_goal_id": start_goal_id,
            "end_goal_id": end_goal_id,
            "goal_timeout_sec": 180.0,
        }],
        condition=IfCondition(run_route),
        output="screen",
    )

    auto_train_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=auto_train,
            on_exit=[EmitEvent(event=Shutdown(
                reason="complete route runner exited"))],
        ),
        # A route failure is diagnostic evidence, not a request to close
        # Gazebo/RViz. Keep the simulation open by default so its final plan,
        # costmaps, and robot pose remain inspectable. CI callers can retain
        # the former one-shot behavior explicitly.
        condition=IfCondition(PythonExpression([
            "'", run_route, "' == 'true' and '", shutdown_on_route_exit,
            "' == 'true'",
        ])),
    )

    start_after_cleanup = GroupAction(actions=[
        gz_server,
        gz_server_headless,
        tf_base,
        tf_laser,
        tf_lidar,
        gz_bridge,
        odom_relay,
        odom_combined_relay,
        waypoint_viz,
        field_reference,
        auto_train_exit,
        # Start the mask lifecycle stack after Gazebo begins publishing /clock.
        # Nav2 itself is released only after the filter-info node is active.
        nav2_after_keepout,
        keepout_lifecycle_after_servers,
        TimerAction(period=6.0, actions=[
            keepout_mask_server,
            keepout_filter_info_server,
        ]),
        # Complete route runner, opt-in via run_route:=true. This name must not
        # collide with Nav2's lifecycle autostart launch argument.
        TimerAction(period=30.0, actions=[auto_train]),
        # RViz 延迟 8s 启动（等 Gazebo + odom_combined TF frame 就绪）
        # 修复原因：原 5s 在 Gazebo 启动耗时长时不够，/clock 未发布
        # → RViz 一直停在 "No tf data. Fixed frame [odom_combined] does not exist"
        TimerAction(period=8.0, actions=[rviz]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="track"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("run_route", default_value="false"),
        # Only these exact values are accepted by prepare_sim_speed_overlay.
        # baseline uses the unchanged 0.15 m/s smartcar_nav2 defaults.
        DeclareLaunchArgument(
            "sim_speed_profile",
            default_value="baseline",
            description=(
                "Local Gazebo speed trial: baseline, 0.20, or 0.25. "
                "Never used by the real-vehicle launch."
            ),
        ),
        # A trial route can be supplied without modifying the saved editor
        # file. The default remains the one authoritative simulation route.
        DeclareLaunchArgument(
            "waypoints_file",
            default_value=os.path.join(
                pkg_nav2, "config", "waypoints", "nav_only.yaml"),
        ),
        DeclareLaunchArgument(
            "shutdown_on_route_exit", default_value="false"),
        DeclareLaunchArgument(
            "results_file", default_value="/tmp/auto_train_results.json"),
        DeclareLaunchArgument("use_through_poses", default_value="true"),
        DeclareLaunchArgument("start_goal_id", default_value=""),
        DeclareLaunchArgument("end_goal_id", default_value=""),
        set_rmw,
        set_localhost,
        set_model_path,
        # Resolve a trial before starting cleanup, Gazebo, or Nav2. Invalid
        # values fail closed instead of silently changing a motion limit.
        OpaqueFunction(function=prepare_sim_speed_overlay),
        RegisterEventHandler(
            OnShutdown(on_shutdown=[
                OpaqueFunction(function=remove_generated_speed_overlay),
            ])
        ),
        # ExecuteProcess is asynchronous. Register the handler first so no
        # Gazebo or ROS process can start until cleanup has actually exited.
        RegisterEventHandler(
            OnProcessExit(
                target_action=sim_cleanup,
                on_exit=[start_after_cleanup],
            )
        ),
        sim_cleanup,
    ])
