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

import math
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
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _as_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ("true", "1"):
        return True
    if value in ("false", "0"):
        return False
    raise RuntimeError(f"{name} must be true or false")


def _nonnegative_float(context, name):
    value_text = LaunchConfiguration(name).perform(context).strip()
    try:
        value = float(value_text)
    except ValueError as error:
        raise RuntimeError(f"{name} must be a finite non-negative number") from error
    if not math.isfinite(value) or value < 0.0:
        raise RuntimeError(f"{name} must be a finite non-negative number")
    return value


def generate_launch_description():
    pkg_sim = get_package_share_directory("smartcar_sim")
    pkg_nav2 = get_package_share_directory("smartcar_nav2")
    pkg_tools = get_package_share_directory("smartcar_tools")

    # Keep every simulator participant on the local development host.  Do not
    # inject a custom DDS locator profile: it can split multi-process discovery.

    # Clean stale simulation files before starting processes.
    cleanup_script = os.path.join(pkg_sim, "scripts", "sim_cleanup.sh")
    skip_cleanup = LaunchConfiguration("skip_cleanup", default="false")
    sim_cleanup = ExecuteProcess(
        cmd=["bash", cleanup_script],
        name="sim_cleanup",
        condition=UnlessCondition(skip_cleanup),
    )
    # An existing local editor or diagnostic session owns local DDS resources.
    # This explicit opt-in path performs no cleanup; callers must isolate their
    # launch with ROS_DOMAIN_ID and IGN_PARTITION.
    sim_cleanup_skip = ExecuteProcess(
        cmd=["bash", cleanup_script, "--skip"],
        name="sim_cleanup_skip",
        condition=IfCondition(skip_cleanup),
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
    route_start_delay_sec = LaunchConfiguration(
        "route_start_delay_sec", default="8.0")
    use_depth_obstacles = LaunchConfiguration(
        "use_depth_obstacles", default="false")
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

    # Keep the simulator local without selecting an RMW implementation.
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
    # its successful exit is the launch-level prerequisite for Nav2 below.
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
            # The optional depth mode additionally proves the PointCloud2
            # fixture is fresh before any navigation goal is accepted.
            "require_depth_points": ParameterValue(
                use_depth_obstacles, value_type=bool),
        }],
        output="screen",
        condition=IfCondition(enable_perception_monitor),
    )

    # ── Direction guard: SKIP in simulation (no lease needed) ──
    # The simulator exercises the same forward-only Nav2 route contract as
    # the vehicle stack; it does not emulate physical motion leases.
    # In simulation, we bypass this and let Nav2 control velocity directly.

    # ── Safety bypass: simulation doesn't need safety_node ──
    # cmd_vel chain: controller → /cmd_vel_nav → velocity_smoother → /cmd_vel_candidate → bridge → Gazebo
    # No direction_guard or safety_node (both block velocity without real hardware)

    # ── Nav2 ──
    nav2_fixed_params = PathJoinSubstitution([
        pkg_nav2, "config", "nav2_params_fixed.yaml"
    ])
    nav2_simulation_overlay = PathJoinSubstitution([
        pkg_sim, "config", "nav2_simulation.yaml"
    ])
    nav2_depth_obstacle_overlay = PathJoinSubstitution([
        pkg_nav2, "config", "depth_camera_obstacle_overlay.yaml"
    ])
    bt_forward = PathJoinSubstitution([pkg_nav2, "config", "behavior_trees",
        "navigate_to_pose_w_replanning_and_recovery.xml"])
    bt_transit = PathJoinSubstitution([pkg_nav2, "config", "behavior_trees",
        "navigate_to_pose_transit_w_replanning_and_recovery.xml"])
    bt_transit_through_poses = PathJoinSubstitution([
        pkg_nav2, "config", "behavior_trees",
        "navigate_through_poses_transit_w_replanning_and_recovery.xml",
    ])
    bt_precise_through_poses = PathJoinSubstitution([
        pkg_nav2, "config", "behavior_trees",
        "navigate_through_poses_precise_w_replanning_and_recovery.xml",
    ])
    bt_forward_return_through_poses = PathJoinSubstitution([
        pkg_nav2, "config", "behavior_trees",
        "navigate_through_poses_return_w_replanning_and_recovery.xml",
    ])
    bt_precise = PathJoinSubstitution([pkg_nav2, "config", "behavior_trees",
        "navigate_to_pose_precise_w_replanning_and_recovery.xml"])

    active_nav2_overlay = None
    active_nav2_params = None
    generated_nav2_params = None

    def prepare_sim_nav2_parameters(context):
        """Materialize the single local-Gazebo Nav2 configuration."""
        nonlocal active_nav2_overlay
        nonlocal active_nav2_params
        nonlocal generated_nav2_params

        simulation_overlay_path = Path(nav2_simulation_overlay.perform(context))
        active_nav2_overlay = simulation_overlay_path
        overlays = [simulation_overlay_path]
        if _as_bool(context, "use_depth_obstacles"):
            # The Gazebo fixture emits PointCloud2 on the same topic as the
            # Aurora relay. The depth overlay replaces /scan so the fixture
            # exercises the same PointCloud2-only obstacle contract as RDK.
            overlays.insert(
                0, Path(nav2_depth_obstacle_overlay.perform(context)))
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
            overlays,
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
                # lifecycle manager has created and warmed its service
                # readers. Autostart can race controller plugin
                # configuration and strand the manager in Gazebo.
                "autostart": "false",
                # Service discovery can complete before reply readers are
                # stable. A 2 s delay still let controller_server
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

    depth_pointcloud_adapter = Node(
        package="smartcar_sim",
        executable="sim_depth_pointcloud_adapter.py",
        name="sim_depth_pointcloud_adapter",
        parameters=[{
            "use_sim_time": True,
            "input_topic": "/scan",
            # The depth fixture uses synchronized world obstacle projections,
            # not a scan-to-cloud conversion that inherits lidar occlusion.
            "source_mode": "world_obstacles",
            "odom_topic": "/odom_combined",
            "world_obstacles": "1.1,0.3;1.5,0.3;2.2,0.5;1.0,0.8;1.8,0.9;2.45,0.15",
            "output_topic": "/smartcar/depth/points",
            "min_range_m": 0.25,
            "max_range_m": 3.5,
            "horizontal_half_fov_rad": 0.75,
            # Convert the elevated laser fixture into obstacle points in the
            # same base-frame height window that the Aurora overlay accepts.
            "output_frame": "base_footprint",
            "sensor_origin_x_m": 0.0341,
            "sensor_origin_y_m": 0.0,
            "point_height_m": 0.15,
        }],
        output="screen",
        condition=IfCondition(use_depth_obstacles),
    )
    # Launch Nav2 exactly once: repeating this include creates duplicate
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

    def start_after_nav2_lifecycle(event, context):
        """Release the optional route only after verified Nav2 activation."""
        if event.returncode == 0:
            route_delay = _nonnegative_float(context, "route_start_delay_sec")
            return [
                LogInfo(msg="[sim] Nav2 lifecycle startup verified"),
                # Activation can return before both rolling costmaps have
                # received several current depth observations. Keep the
                # route gate read-only, then give the obstacle layer a bounded
                # settling window before the first goal is sent.
                TimerAction(period=route_delay, actions=[auto_train]),
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
                # Static transforms published before the Gazebo clock / DDS
                # graph is live can be absent from late Nav2 subscribers
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
        """Release Nav2 only after scan-time TF is proven live."""
        if event.returncode == 0:
            return [
                LogInfo(msg=(
                    "[sim] scan-time TF preflight passed; starting Nav2"
                )),
                OpaqueFunction(function=launch_nav2_once),
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
            "transit_behavior_tree": bt_transit,
            "precise_behavior_tree": bt_precise,
            "through_poses_behavior_tree": PathJoinSubstitution([
                pkg_nav2, "config", "behavior_trees",
                "navigate_through_poses_w_replanning_and_recovery.xml",
            ]),
            "through_poses_transit_behavior_tree": bt_transit_through_poses,
            "through_poses_precise_behavior_tree": bt_precise_through_poses,
            "through_poses_return_behavior_tree": (
                bt_forward_return_through_poses),
            "use_through_poses": use_through_poses_lc,
            "nav2_params_file": active_nav2_params_file,
            "nav2_params_overlay_file": active_nav2_overlay_file,
            "results_file": results_file,
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
        depth_pointcloud_adapter,
        sensor_preflight_exit,
        sensor_tf_preflight_exit,
        sensor_preflight,
        perception_monitor,
        waypoint_viz,
        field_reference,
        auto_train_exit,
        nav2_lifecycle_startup_exit,
        # Nav2 is released only after the sensor and scan-time TF preflights
        # prove live obstacle observations are usable.
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
        DeclareLaunchArgument("use_depth_obstacles", default_value="false"),
        DeclareLaunchArgument("skip_cleanup", default_value="false"),
        DeclareLaunchArgument(
            "route_start_delay_sec", default_value="8.0"),
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
        RegisterEventHandler(
            OnProcessExit(
                target_action=sim_cleanup_skip,
                on_exit=[start_after_cleanup],
            )
        ),
        sim_cleanup,
        sim_cleanup_skip,
    ])
