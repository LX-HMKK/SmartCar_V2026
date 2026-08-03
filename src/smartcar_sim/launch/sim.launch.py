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
# Nav2 parameter helper beside this file so source and install workspaces use
# the same materialized configuration.
_LAUNCH_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_DIRECTORY not in sys.path:
    sys.path.insert(0, _LAUNCH_DIRECTORY)

from sim_nav2_parameters import write_merged_nav2_parameters

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
    enable_perception_monitor = LaunchConfiguration(
        "enable_perception_monitor", default="true")
    sensor_preflight_timeout_sec = LaunchConfiguration(
        "sensor_preflight_timeout_sec", default="35.0")
    run_route = LaunchConfiguration("run_route", default="false")
    active_nav2_overlay_file = LaunchConfiguration(
        "sim_active_nav2_overlay_file")
    active_nav2_params_file = LaunchConfiguration(
        "sim_active_nav2_params_file")
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
    # Both paths use Ogre2 because Fortress' GPU lidar only receives valid
    # ranges when the server owns a render scene.  --headless-rendering keeps
    # that scene off-screen for route/CI runs without opening a Gazebo GUI.
    world_path = PathJoinSubstitution([pkg_sim, "worlds", PythonExpression(["'", world, ".world'"])])

    gz_server = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-v", "3", "--render-engine", "ogre2", world_path],
        output="screen",
        condition=UnlessCondition(headless),
        name="gz_server",
    )

    gz_server_headless = ExecuteProcess(
        cmd=[
            "ign", "gazebo", "-r", "-s", "-v", "3",
            "--headless-rendering", "--render-engine", "ogre2", world_path,
        ],
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
    # TF chain: odom_combined → base_footprint (ground-truth relay, /tf)
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
    )

    # base_link → laser
    tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["-0.05", "0", "0.23", "0", "0", "0", "base_link", "laser_link"],
    )

    # Identity TF: ROS laser_link → Gazebo LiDAR sensor frame.
    # Gazebo publishes /scan with frame_id "origincar/laser_link/lidar_sensor"
    # but Nav2 uses "laser_link". Keep laser_link attached to base_link and add
    # the Gazebo sensor as its child so the TF tree has one parent per frame.
    tf_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "laser_link", "origincar/laser_link/lidar_sensor"],
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

    # ── Ground-truth odometry: Gazebo model pose → /odom_combined + TF ──
    # Do not use the Ackermann plugin's integrated /odom as Nav2's TF owner:
    # it can diverge from the DART world pose carrying the lidar sensor. The
    # relay consumes the model-local continuous PosePublisher stream and is
    # the sole publisher of odom_combined → base_footprint in local Gazebo.
    ground_truth_odom_relay = Node(
        package="smartcar_sim",
        executable="gazebo_ground_truth_odom_relay.py",
        name="gazebo_ground_truth_odom_relay",
        parameters=[{
            "use_sim_time": True,
            "physical_pose_topic": "/model/origincar/pose",
        }],
    )

    # Do not configure Nav2 against an empty or stale sensor graph.  In
    # particular, an interrupted headless Ogre2 instance can still advertise
    # /scan while every range is the lidar minimum and /odom_combined never
    # appears.
    # This short-lived process only observes /clock, /odom_combined and /scan;
    # its successful exit is the launch-level prerequisite for the keepout
    # stack and Nav2 below.
    sensor_preflight = Node(
        package="smartcar_sim",
        executable="sim_sensor_preflight.py",
        name="sim_sensor_preflight",
        parameters=[{
            "use_sim_time": False,
            "timeout_sec": sensor_preflight_timeout_sec,
            "min_sensor_stream_span_sec": 1.0,
            "min_odom_samples": 15,
            "min_scan_samples": 5,
        }],
        output="screen",
    )

    # The first preflight intentionally runs before the static sensor frames
    # exist. After it proves the Gazebo streams are live, start this second
    # gate only after publishing those frames. Nav2 must never construct an
    # obstacle layer until a scan can be transformed at its own source stamp.
    sensor_tf_preflight = Node(
        package="smartcar_sim",
        executable="sim_sensor_preflight.py",
        name="sim_sensor_tf_preflight",
        parameters=[{
            "use_sim_time": False,
            "timeout_sec": sensor_preflight_timeout_sec,
            "min_sensor_stream_span_sec": 1.0,
            "min_odom_samples": 15,
            "min_scan_samples": 5,
            "require_scan_time_tf": True,
            "tf_target_frame": "odom_combined",
        }],
        output="screen",
    )

    # Read-only health signal for the scan -> TF -> raw-costmap path.  It never
    # sends motion commands or changes Nav2 state, but catches an invalid
    # headless lidar or a TF message-filter regression before route evidence is
    # accepted.
    perception_monitor = Node(
        package="smartcar_sim",
        executable="sim_perception_monitor.py",
        name="sim_perception_monitor",
        parameters=[{
            "use_sim_time": True,
            # Parse the physical A-zone cone collision cylinders from this exact
            # world so scan/costmap agreement cannot hide a shared bad frame.
            "track_world_file": world_path,
        }],
        output="screen",
        condition=IfCondition(enable_perception_monitor),
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
        "navigate_to_pose_precise_sim_w_replanning_and_recovery.xml"])
    bt_reverse = PathJoinSubstitution([pkg_nav2, "config", "behavior_trees",
        "navigate_to_pose_reverse_sim_w_replanning_and_recovery.xml"])
    bt_reverse_handoff = PathJoinSubstitution([
        pkg_nav2, "config", "behavior_trees",
        "navigate_to_pose_reverse_handoff_sim_w_replanning_and_recovery.xml",
    ])
    bt_reverse_through_poses = PathJoinSubstitution([
        pkg_nav2, "config", "behavior_trees",
        "navigate_through_poses_reverse_sim_w_replanning_and_recovery.xml",
    ])
    bt_reverse_locked_through_poses = PathJoinSubstitution([
        pkg_nav2, "config", "behavior_trees",
        "navigate_through_poses_reverse_locked_sim_w_replanning_and_recovery.xml",
    ])
    bt_reverse_return_through_poses = PathJoinSubstitution([
        pkg_nav2, "config", "behavior_trees",
        "navigate_through_poses_reverse_return_sim_w_replanning_and_recovery.xml",
    ])

    active_nav2_overlay = None
    active_nav2_params = None
    generated_nav2_params = None

    def prepare_sim_nav2_parameters(context):
        """Materialize the single local-Gazebo Nav2 configuration."""
        nonlocal active_nav2_overlay
        nonlocal active_nav2_params
        nonlocal generated_nav2_params

        keepout_overlay_path = Path(nav2_keepout_overlay.perform(context))
        active_nav2_overlay = keepout_overlay_path
        # Use one resolved parameter file for the local simulator. Passing a
        # base file and an overlay as independent ``--params-file`` entries
        # left controller-plugin replacement order implementation-dependent.
        # The materialized file is the exact runtime contract and is removed
        # on launch shutdown.
        generated_nav2_params = (
            Path(tempfile.gettempdir())
            / f"smartcar_sim_nav2_{os.getpid()}_params"
            / "nav2_params_fixed.yaml"
        )
        active_nav2_params = write_merged_nav2_parameters(
            Path(nav2_fixed_params.perform(context)),
            active_nav2_overlay,
            generated_nav2_params,
        )
        return [
            SetLaunchConfiguration(
                "sim_active_nav2_overlay_file", str(active_nav2_overlay)),
            SetLaunchConfiguration(
                "sim_active_nav2_params_file", str(active_nav2_params)),
            LogInfo(msg=(
                "[sim] Nav2 local-Gazebo speed fixed at 0.30 m/s; "
                "real-vehicle Nav2 parameters are unchanged"
            )),
        ]

    def remove_generated_nav2_parameters(context):
        if generated_nav2_params is not None:
            generated_nav2_params.unlink(missing_ok=True)
            try:
                generated_nav2_params.parent.rmdir()
            except OSError:
                # A failed launch may leave an auxiliary diagnostic file in
                # this run-specific directory. It is safer to leave that
                # empty-or-diagnostic directory than fail shutdown cleanup.
                pass
        return []

    def make_nav2_launch(params_path):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_nav2, "launch", "navigation_launch.py"])
            ),
            launch_arguments={
                "use_sim_time": "true",
                "nav2_params_file": str(params_path),
                "nav2_params_overlay_file": "",
                # Let the verified helper below issue STARTUP only after the
                # lifecycle manager has created and warmed its Fast DDS
                # response readers. Autostart can race controller plugin
                # configuration and strand the manager in Gazebo.
                "autostart": "false",
                # Fast DDS can discover lifecycle services before their reply
                # readers are stable. A 2 s delay still let controller_server
                # finish configuring after its manager had abandoned the
                # change_state response. Keep the larger wait local to Gazebo;
                # the production Nav2 launch retains its zero-delay default.
                "lifecycle_manager_delay_sec": "6.0",
            }.items(),
        )

    nav2_lifecycle_startup = Node(
        package="smartcar_sim",
        executable="nav2_lifecycle_startup.py",
        name="nav2_lifecycle_startup",
        parameters=[{
            # Use wall-time limits: this gate is responsible for reporting a
            # dead lifecycle graph even if Gazebo's simulated clock stalls.
            "use_sim_time": False,
            "warmup_sec": 4.0,
            "service_timeout_sec": 30.0,
            "startup_timeout_sec": 90.0,
            "state_timeout_sec": 15.0,
        }],
        output="screen",
    )
    # A lifecycle manager can reactivate the filter after a transient startup
    # delay. Launch Nav2 exactly once: repeating this include creates duplicate
    # controller/planner nodes that fight over the same lifecycle services.
    nav2_started = False

    def launch_nav2_once(context):
        nonlocal nav2_started
        if nav2_started:
            return []
        if active_nav2_params is None:
            raise RuntimeError("sim Nav2 parameters were not prepared before Nav2")
        nav2_started = True
        return [make_nav2_launch(active_nav2_params), nav2_lifecycle_startup]

    nav2_after_keepout = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=keepout_filter_info_server,
            goal_state="active",
            entities=[OpaqueFunction(function=launch_nav2_once)],
        )
    )

    def start_after_nav2_lifecycle(event, context):
        """Release the optional route only after verified Nav2 activation."""
        if event.returncode == 0:
            return [
                LogInfo(msg="[sim] Nav2 lifecycle startup verified"),
                # Activation returns before both rolling costmaps have their
                # first obstacle-layer update.
                TimerAction(period=2.0, actions=[auto_train]),
            ]
        return [
            LogInfo(msg=(
                "[sim] Nav2 lifecycle startup failed; auto_train is not "
                "started")),
            EmitEvent(event=Shutdown(
                reason="Nav2 lifecycle startup failed")),
        ]

    nav2_lifecycle_startup_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=nav2_lifecycle_startup,
            on_exit=start_after_nav2_lifecycle,
        )
    )

    def start_after_sensor_preflight(event, context):
        """Publish static sensor frames after the raw stream preflight."""
        if event.returncode == 0:
            return [
                LogInfo(msg=(
                    "[sim] sensor preflight passed; publishing static TF, then "
                    "checking scan-time TF"
                )),
                # Static transforms published before the Gazebo clock / Fast
                # DDS graph is live can be absent from late Nav2 subscribers
                # despite their transient-local QoS. Publish them after the
                # live scan+odom gate, then allow discovery to settle before
                # either costmap starts its laser message filter.
                tf_base,
                tf_laser,
                tf_lidar,
                TimerAction(period=1.0, actions=[sensor_tf_preflight]),
            ]
        return [
            LogInfo(msg=(
                "[sim] sensor preflight failed; Nav2 and auto_train are "
                "not started"
            )),
            EmitEvent(event=Shutdown(
                reason="Gazebo sensor preflight failed")),
        ]

    sensor_preflight_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=sensor_preflight,
            # OnProcessExit supplies both the ProcessExited event and launch
            # context to a callable.  OpaqueFunction only receives context,
            # which loses the exit code and would leave Nav2 blocked after a
            # healthy preflight.
            on_exit=start_after_sensor_preflight,
        )
    )

    def start_after_sensor_tf_preflight(event, context):
        """Release keepout and Nav2 only after scan-time TF is proven live."""
        if event.returncode == 0:
            return [
                LogInfo(msg=(
                    "[sim] scan-time TF preflight passed; starting keepout "
                    "and Nav2"
                )),
                keepout_mask_server,
                keepout_filter_info_server,
            ]
        return [
            LogInfo(msg=(
                "[sim] scan-time TF preflight failed; Nav2 and auto_train are "
                "not started"
            )),
            EmitEvent(event=Shutdown(
                reason="Gazebo scan-time TF preflight failed")),
        ]

    sensor_tf_preflight_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=sensor_tf_preflight,
            on_exit=start_after_sensor_tf_preflight,
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
            "through_poses_reverse_behavior_tree": bt_reverse_through_poses,
            "through_poses_reverse_locked_behavior_tree": (
                bt_reverse_locked_through_poses),
            "through_poses_reverse_return_behavior_tree": (
                bt_reverse_return_through_poses),
            "use_through_poses": use_through_poses_lc,
            "nav2_params_file": active_nav2_params_file,
            "nav2_params_overlay_file": active_nav2_overlay_file,
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
        gz_bridge,
        ground_truth_odom_relay,
        sensor_preflight_exit,
        sensor_tf_preflight_exit,
        sensor_preflight,
        perception_monitor,
        waypoint_viz,
        field_reference,
        auto_train_exit,
        nav2_lifecycle_startup_exit,
        # Start the mask lifecycle stack after Gazebo begins publishing /clock.
        # Nav2 itself is released only after the sensor preflight and
        # filter-info activation both succeed.
        nav2_after_keepout,
        keepout_lifecycle_after_servers,
        # RViz 延迟 8s 启动（等 Gazebo + odom_combined TF frame 就绪）
        # 修复原因：原 5s 在 Gazebo 启动耗时长时不够，/clock 未发布
        # → RViz 一直停在 "No tf data. Fixed frame [odom_combined] does not exist"
        TimerAction(period=8.0, actions=[rviz]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="track"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "enable_perception_monitor", default_value="true"),
        DeclareLaunchArgument(
            "sensor_preflight_timeout_sec", default_value="35.0"),
        DeclareLaunchArgument("run_route", default_value="false"),
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
        # Resolve the base plus simulation-only overlay before Gazebo or Nav2
        # starts, so controller plugin selection has one source of truth.
        OpaqueFunction(function=prepare_sim_nav2_parameters),
        RegisterEventHandler(
            OnShutdown(on_shutdown=[
                OpaqueFunction(function=remove_generated_nav2_parameters),
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
