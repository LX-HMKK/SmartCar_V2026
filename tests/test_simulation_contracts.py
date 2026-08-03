"""Static contracts for the native Ubuntu Gazebo simulation launch path."""

import copy
import importlib.util
import math
import os
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
LOCAL_SIM = ROOT / "scripts" / "local_sim.sh"
SIM = ROOT / "src" / "smartcar_sim"
LAUNCH = SIM / "launch" / "sim.launch.py"
SIM_ENV = SIM / "scripts" / "sim_env.sh"
SIM_START = SIM / "scripts" / "sim_start.sh"
SIM_CLEANUP = SIM / "scripts" / "sim_cleanup.sh"
AUTO_TRAIN = SIM / "scripts" / "auto_train.py"
GAZEBO_GROUND_TRUTH_ODOM_RELAY = (
    SIM / "scripts" / "gazebo_ground_truth_odom_relay.py"
)
ACKERMANN_ARC_PROBE = SIM / "scripts" / "ackermann_arc_probe.py"
SIM_TUNE = SIM / "scripts" / "sim_tune.sh"
TUNE_PARAMS = SIM / "scripts" / "tune_params.py"
RVIZ = SIM / "rviz" / "sim_nav.rviz"
ROBOT_MODEL = SIM / "models" / "origincar" / "model.sdf"
PERCEPTION_MONITOR = SIM / "scripts" / "sim_perception_monitor.py"
SENSOR_PREFLIGHT = SIM / "scripts" / "sim_sensor_preflight.py"
NAV2_LIFECYCLE_STARTUP = SIM / "scripts" / "nav2_lifecycle_startup.py"
WORLD = SIM / "worlds" / "track.world"
FIELD_MODEL_CONFIG = SIM / "config" / "competition_field_model.yaml"
FIELD_MODEL_GENERATOR = SIM / "scripts" / "generate_competition_field.py"
FIELD_MODEL = SIM / "models" / "competition_field" / "model.sdf"
FIELD_MAP_GENERATOR = SIM / "scripts" / "generate_field_map.py"
FIELD_MAP = SIM / "maps" / "field_map.pgm"
FIELD_MAP_YAML = SIM / "maps" / "field_map.yaml"
KEEPOUT_OVERLAY = SIM / "config" / "nav2_keepout_filter.yaml"
ROUTE_PLANNING_SYNC = SIM / "scripts" / "sync_route_planning.py"
ROUTE_PLANNING = (
    ROOT / "src" / "smartcar_tools" / "config" / "routes" / "route_planning.yaml"
)
NAV2_PARAMS = ROOT / "src" / "smartcar_nav2" / "config" / "nav2_params.yaml"
REAL_PRECISE_BT = (
    ROOT
    / "src"
    / "smartcar_nav2"
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_precise_w_replanning_and_recovery.xml"
)
SIM_PRECISE_BT = (
    ROOT
    / "src"
    / "smartcar_nav2"
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_precise_sim_w_replanning_and_recovery.xml"
)
REAL_REVERSE_BT_FILES = (
    ROOT
    / "src"
    / "smartcar_nav2"
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_reverse_w_replanning_and_recovery.xml",
    ROOT
    / "src"
    / "smartcar_nav2"
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_reverse_handoff_w_replanning_and_recovery.xml",
    ROOT
    / "src"
    / "smartcar_nav2"
    / "config"
    / "behavior_trees"
    / "navigate_through_poses_reverse_w_replanning_and_recovery.xml",
    ROOT
    / "src"
    / "smartcar_nav2"
    / "config"
    / "behavior_trees"
    / "navigate_through_poses_reverse_locked_w_replanning_and_recovery.xml",
)
SIM_REVERSE_BT_FILES = (
    ROOT
    / "src"
    / "smartcar_nav2"
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_reverse_sim_w_replanning_and_recovery.xml",
    ROOT
    / "src"
    / "smartcar_nav2"
    / "config"
    / "behavior_trees"
    / "navigate_to_pose_reverse_handoff_sim_w_replanning_and_recovery.xml",
    ROOT
    / "src"
    / "smartcar_nav2"
    / "config"
    / "behavior_trees"
    / "navigate_through_poses_reverse_sim_w_replanning_and_recovery.xml",
    ROOT
    / "src"
    / "smartcar_nav2"
    / "config"
    / "behavior_trees"
    / "navigate_through_poses_reverse_locked_sim_w_replanning_and_recovery.xml",
)
PACKAGE_XML = SIM / "package.xml"


def read_pgm(path: Path) -> tuple[int, int, bytes]:
    raw = path.read_bytes()
    magic, dimensions, max_value, pixels = raw.split(b"\n", 3)
    if magic != b"P5" or max_value != b"255":
        raise ValueError(f"unsupported PGM header in {path}")
    width, height = (int(value) for value in dimensions.split())
    if len(pixels) != width * height:
        raise ValueError(f"unexpected PGM payload size in {path}")
    return width, height, pixels


def pgm_value_at(
    pixels: bytes,
    width: int,
    height: int,
    origin: tuple[float, float],
    resolution: float,
    x: float,
    y: float,
) -> int:
    col = int((x - origin[0]) // resolution)
    row_from_bottom = int((y - origin[1]) // resolution)
    if not 0 <= col < width or not 0 <= row_from_bottom < height:
        raise ValueError(f"point ({x}, {y}) lies outside the PGM")
    row = height - 1 - row_from_bottom
    return pixels[row * width + col]


class SimulationContractTests(unittest.TestCase):
    def test_local_sim_clears_snap_gui_runtime_without_losing_desktop_session(self) -> None:
        runner = LOCAL_SIM.read_text(encoding="utf-8")

        self.assertIn("is_snap_terminal=false", runner)
        self.assertIn("SNAP_REAL_HOME", runner)
        self.assertIn("XDG_DATA_DIRS_VSCODE_SNAP_ORIG", runner)
        self.assertIn("XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG", runner)
        for variable in (
            "LD_PRELOAD",
            "GTK_DATA_PREFIX",
            "GTK_EXE_PREFIX",
            "GTK_PATH",
            "GTK_IM_MODULE_FILE",
            "GDK_PIXBUF_MODULE_FILE",
            "GDK_PIXBUF_MODULEDIR",
            "GIO_EXTRA_MODULES",
            "GIO_MODULE_DIR",
            "GSETTINGS_SCHEMA_DIR",
            "GI_TYPELIB_PATH",
        ):
            with self.subTest(variable=variable):
                self.assertIn(variable, runner)
        for session_variable in (
            "DISPLAY",
            "WAYLAND_DISPLAY",
            "XAUTHORITY",
            "XDG_RUNTIME_DIR",
            "DBUS_SESSION_BUS_ADDRESS",
        ):
            with self.subTest(session_variable=session_variable):
                self.assertNotIn(f"unset {session_variable}", runner)

    def test_simulation_speed_is_fixed_at_030_and_preserves_keepout(self) -> None:
        keepout = yaml.safe_load(KEEPOUT_OVERLAY.read_text(encoding="utf-8"))
        keepout_controllers = keepout["controller_server"]["ros__parameters"]
        smoother = keepout["velocity_smoother"]["ros__parameters"]
        expected_speed = 0.30
        expected_radius = 0.22
        expected_wz = expected_speed / expected_radius

        self.assertAlmostEqual(
            keepout_controllers["ForwardAvoidance"]
            ["forward_path_max_cross_track_error"],
            0.12,
        )
        self.assertNotIn("ForwardHandoff", keepout_controllers)
        self.assertEqual(
            keepout_controllers["ForwardAvoidance"]["desired_linear_vel"],
            expected_speed,
        )
        self.assertAlmostEqual(
            keepout_controllers["ForwardAvoidance"]
            ["forward_max_angular_velocity"],
            expected_wz,
        )
        self.assertEqual(
            keepout_controllers["FollowPath"]["desired_linear_vel"],
            expected_speed,
        )
        self.assertEqual(
            keepout_controllers["ReverseHandoff"]["vx_max"], expected_speed)
        self.assertAlmostEqual(
            keepout_controllers["ReverseHandoff"]["wz_max"], expected_wz)
        self.assertEqual(
            keepout_controllers["ReverseRecovery"]["vx_max"], expected_speed)
        self.assertEqual(smoother["max_velocity"], [expected_speed, 0.0, expected_wz])
        self.assertEqual(smoother["min_velocity"], [-expected_speed, 0.0, -expected_wz])
        self.assertFalse((SIM / "config" / "nav2_speed_020.yaml").exists())
        self.assertFalse((SIM / "config" / "nav2_speed_025.yaml").exists())

    def test_simulation_launch_has_no_speed_profile_and_keeps_real_defaults(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        runner = AUTO_TRAIN.read_text(encoding="utf-8")
        params = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))

        self.assertNotIn("sim_speed_profile", launch)
        self.assertNotIn("sim_speed_profile", runner)
        self.assertIn('"nav2_params_file": active_nav2_params_file', launch)
        self.assertIn('"nav2_params_overlay_file": active_nav2_overlay_file', launch)
        self.assertIn("OnShutdown", launch)
        self.assertIn('self.declare_parameter("nav2_params_overlay_file", "")', runner)
        self.assertEqual(
            params["controller_server"]["ros__parameters"]["ForwardAvoidance"]
            ["desired_linear_vel"],
            0.15,
        )
        self.assertEqual(
            params["velocity_smoother"]["ros__parameters"]["max_velocity"][0],
            0.15,
        )
        self.assertEqual(
            params["velocity_smoother"]["ros__parameters"]["min_velocity"][0],
            -0.15,
        )
        self.assertEqual(
            params["planner_server"]["ros__parameters"]["GridBased"]
            ["minimum_turning_radius"],
            0.22,
        )

    def test_fastdds_uses_default_transport_across_entrypoints(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        sim_env = SIM_ENV.read_text(encoding="utf-8")

        self.assertIn('"RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"', launch)
        self.assertNotIn('SetEnvironmentVariable(\n        "FASTRTPS_DEFAULT_PROFILES_FILE"', launch)
        self.assertIn("RMW_IMPLEMENTATION=rmw_fastrtps_cpp", sim_env)
        self.assertIn("unset FASTRTPS_DEFAULT_PROFILES_FILE", sim_env)
        self.assertIn("sim_env_script_dir=", sim_env)
        self.assertIn("sim_env_workspace=", sim_env)
        self.assertNotIn("unset script_dir", sim_env)
        self.assertNotIn("unset workspace", sim_env)

    def test_cleanup_finishes_before_simulation_nodes_start(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")

        self.assertIn("RegisterEventHandler", launch)
        self.assertIn("OnProcessExit", launch)
        self.assertIn("target_action=sim_cleanup", launch)
        self.assertIn("on_exit=[start_after_cleanup]", launch)

    def test_cleanup_reclaims_stale_keepout_stack(self) -> None:
        cleanup = SIM_CLEANUP.read_text(encoding="utf-8")

        self.assertIn(
            '"/nav2_map_server/map_server.*__node:=keepout_mask_server"',
            cleanup,
        )
        self.assertIn(
            '"/nav2_map_server/costmap_filter_info_server.*'
            '__node:=keepout_filter_info_server"',
            cleanup,
        )
        # Display-only helpers are Python processes from smartcar_tools, not
        # smartcar_sim. They must be reclaimed too, otherwise every interrupted
        # launch leaves duplicate MarkerArray publishers and raises host load.
        self.assertIn(
            '"/smartcar_tools/lib/smartcar_tools/waypoint_viz"', cleanup)
        self.assertIn(
            '"/smartcar_tools/lib/smartcar_tools/field_reference_node"', cleanup)

    def test_lidar_sensor_is_a_child_of_laser_link(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")

        expected = (
            '"laser_link", "origincar/laser_link/lidar_sensor"'
        )
        reversed_frames = (
            '"origincar/laser_link/lidar_sensor", "laser_link"'
        )
        self.assertIn(expected, launch)
        self.assertNotIn(reversed_frames, launch)

    def test_static_sensor_tf_does_not_wait_for_sim_clock(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        static_tf_start = launch.index("    tf_base = Node(")
        static_tf_end = launch.index("    # ── ros_gz_bridge", static_tf_start)
        static_tf = launch[static_tf_start:static_tf_end]

        # These transforms are valid at every simulation time. Giving their
        # publishers use_sim_time makes their initial transient-local sample
        # race the first /clock message, leaving scan frames absent from RViz
        # and the costmap TF buffer.
        self.assertEqual(
            static_tf.count('executable="static_transform_publisher"'), 3)
        self.assertNotIn('parameters=[{"use_sim_time": True}]', static_tf)
        preflight_release = launch[
            launch.index("    def start_after_sensor_preflight"):
            launch.index("    sensor_preflight_exit =", launch.index(
                "    def start_after_sensor_preflight"))
        ]
        self.assertIn("                tf_base,", preflight_release)
        self.assertIn("                tf_laser,", preflight_release)
        self.assertIn("                tf_lidar,", preflight_release)
        self.assertIn("TimerAction(period=1.0", preflight_release)
        self.assertIn("actions=[sensor_tf_preflight]", preflight_release)

    def test_headless_lidar_and_costmap_observation_contract(self) -> None:
        model = ET.parse(ROBOT_MODEL).getroot()
        sensor = model.find(".//link[@name='laser_link']/sensor[@name='lidar_sensor']")
        self.assertIsNotNone(sensor)
        assert sensor is not None
        self.assertEqual(sensor.attrib["type"], "gpu_lidar")
        self.assertIsNotNone(sensor.find("lidar"))
        self.assertIsNone(sensor.find("ray"))
        self.assertEqual(sensor.findtext("topic"), "/scan")
        self.assertEqual(sensor.findtext("update_rate"), "20")
        self.assertEqual(sensor.findtext("always_on"), "true")

        world = WORLD.read_text(encoding="utf-8")
        launch = LAUNCH.read_text(encoding="utf-8")
        self.assertIn("<render_engine>ogre2</render_engine>", world)
        self.assertIn('"--headless-rendering"', launch)
        self.assertIn('"--render-engine", "ogre2"', launch)

        overlay = yaml.safe_load(KEEPOUT_OVERLAY.read_text(encoding="utf-8"))
        base_params = yaml.safe_load(NAV2_PARAMS.read_text(encoding="utf-8"))
        expected_publish_frequencies = {
            "local_costmap": 10.0,
            "global_costmap": 5.0,
        }
        for costmap_name, expected_publish_frequency in (
            expected_publish_frequencies.items()
        ):
            with self.subTest(costmap=costmap_name):
                parameters = overlay[costmap_name][costmap_name]["ros__parameters"]
                self.assertEqual(parameters["resolution"], 0.025)
                self.assertEqual(parameters["transform_tolerance"], 0.10)
                self.assertEqual(
                    parameters["publish_frequency"], expected_publish_frequency)
                obstacle_layer = parameters["obstacle_layer"]
                self.assertEqual(obstacle_layer["observation_persistence"], 0.0)
                self.assertEqual(obstacle_layer["expected_update_rate"], 0.10)
                scan = obstacle_layer["scan"]
                self.assertEqual(scan["min_obstacle_height"], 0.05)
                self.assertEqual(scan["max_obstacle_height"], 0.50)
                self.assertIs(scan["inf_is_valid"], False)
                # The simulation overlay only narrows freshness and height
                # policy. The base Nav2 file remains the single owner of the
                # source list and /scan transport configuration.
                self.assertNotIn("observation_sources", obstacle_layer)
                self.assertNotIn("topic", scan)
                self.assertNotIn("data_type", scan)
                base_layer = base_params[costmap_name][costmap_name][
                    "ros__parameters"]["obstacle_layer"]
                self.assertEqual(base_layer["observation_sources"], "scan")
                self.assertEqual(base_layer["scan"]["topic"], "/scan")
                self.assertEqual(base_layer["scan"]["data_type"], "LaserScan")

    def test_ackermann_model_can_execute_simulation_turning_radius(self) -> None:
        model = ET.parse(ROBOT_MODEL).getroot()
        plugin = model.find(
            ".//plugin[@name='ignition::gazebo::systems::AckermannSteering']")
        self.assertIsNotNone(plugin)
        assert plugin is not None
        wheelbase = float(plugin.findtext("wheel_base"))
        kingpin_width = float(plugin.findtext("kingpin_width"))
        steering_limit = float(plugin.findtext("steering_limit"))
        required_inner_steering = math.atan(
            wheelbase / (0.22 - kingpin_width / 2.0))
        # Fortress turns its virtual steering limit into L / sin(limit), not
        # the inner-wheel Ackermann angle. Require the value that actually
        # admits the configured rear-axle radius as well as the physical joint
        # range needed by the inner wheel.
        self.assertLessEqual(wheelbase / math.sin(steering_limit), 0.22)
        self.assertIsNone(plugin.find("max_steering_angle"))
        # Nav2 velocity_smoother owns acceleration limiting. The Fortress
        # plugin must not independently ramp linear and angular components at
        # the same rate, which would corrupt a requested Ackermann curvature
        # during a turn-in.
        self.assertLessEqual(float(plugin.findtext("min_acceleration")), -10.0)
        self.assertGreaterEqual(float(plugin.findtext("max_acceleration")), 10.0)

        for joint_name in ("up_left_steer_joint", "up_right_steer_joint"):
            with self.subTest(joint=joint_name):
                limit = model.find(
                    f".//joint[@name='{joint_name}']/axis/limit")
                self.assertIsNotNone(limit)
                assert limit is not None
                self.assertEqual(limit.findtext("effort"), "1000000")
                self.assertGreaterEqual(
                    float(limit.findtext("upper")), required_inner_steering)
                self.assertLessEqual(
                    float(limit.findtext("lower")), -required_inner_steering)

    def test_ackermann_arc_probe_uses_the_simulation_command_chain(self) -> None:
        source = ACKERMANN_ARC_PROBE.read_text(encoding="utf-8")
        cmake = (SIM / "CMakeLists.txt").read_text(encoding="utf-8")

        self.assertIn('self.declare_parameter("command_topic", "/cmd_vel_nav")', source)
        self.assertIn('self.declare_parameter("candidate_topic", "/cmd_vel_candidate")', source)
        self.assertIn('self.declare_parameter("odom_topic", "/odom_combined")', source)
        self.assertIn('self.declare_parameter("turning_radius_m", 0.22)', source)
        self.assertIn('"results_file", "/tmp/smartcar_ackermann_arc_probe.json"', source)
        self.assertNotIn('"/ackermann_cmd"', source)
        self.assertIn("scripts/ackermann_arc_probe.py", cmake)

    def test_ackermann_wheels_are_grounded_and_aligned_with_roll_joints(self) -> None:
        model = ET.parse(ROBOT_MODEL).getroot()
        wheel_radius = 0.03
        quarter_turn = math.pi / 2.0

        # A cylinder's native axis is Z, while all four wheel joints rotate
        # about Y. The collision geometry, rather than only the visual, must
        # be rotated or DART has no tyre-shaped ground contact.
        for link_name in (
            "down_left_Link",
            "down_right_Link",
            "up_left_Link",
            "up_right_Link",
        ):
            with self.subTest(link=link_name):
                link = model.find(f".//link[@name='{link_name}']")
                self.assertIsNotNone(link)
                assert link is not None
                collision_pose = link.findtext("collision/pose")
                visual_pose = link.findtext("visual/pose")
                self.assertIsNotNone(collision_pose)
                self.assertEqual(collision_pose, visual_pose)
                assert collision_pose is not None
                roll, pitch, yaw = (
                    float(value) for value in collision_pose.split()[3:])
                self.assertAlmostEqual(abs(roll), quarter_turn)
                self.assertAlmostEqual(pitch, 0.0)
                self.assertAlmostEqual(yaw, 0.0)

        # The model lives on the world ground plane. Its rear wheel centres
        # are one radius high and the body collision has positive clearance.
        world = ET.parse(WORLD).getroot()
        robot_include = world.find(".//include[uri='model://origincar']")
        self.assertIsNotNone(robot_include)
        assert robot_include is not None
        self.assertEqual(robot_include.findtext("pose"), "0 0 0 0 0 0")
        rear_wheel = model.find(".//link[@name='down_left_Link']/pose")
        body = model.find(".//link[@name='base_link']/pose")
        body_collision_pose = model.find(
            ".//link[@name='base_link']/collision[@name='body_collision']/pose")
        body_collision = model.find(
            ".//link[@name='base_link']/collision[@name='body_collision']/geometry/box/size")
        self.assertIsNotNone(rear_wheel)
        self.assertIsNotNone(body)
        self.assertIsNotNone(body_collision_pose)
        self.assertIsNotNone(body_collision)
        assert (
            rear_wheel is not None and body is not None
            and body_collision_pose is not None and body_collision is not None
        )
        self.assertAlmostEqual(float(rear_wheel.text.split()[2]), wheel_radius)
        body_center_height = float(body.text.split()[2])
        body_collision_offset = float(body_collision_pose.text.split()[2])
        body_height = float(body_collision.text.split()[2])
        self.assertAlmostEqual(body_center_height, wheel_radius)
        self.assertGreater(
            body_center_height + body_collision_offset - body_height / 2.0,
            0.0,
        )
        laser_pose = model.findtext(".//link[@name='laser_link']/pose")
        self.assertIsNotNone(laser_pose)
        assert laser_pose is not None
        self.assertAlmostEqual(
            float(laser_pose.split()[2]) - body_center_height, 0.23)

        # Front wheels share their corresponding steering-knuckle centres.
        # Leaving their link poses at zero pins their collision geometry near
        # P rather than at the front axle after Gazebo resolves the joints.
        for wheel_name, steering_name in (
            ("up_left_Link", "up_left_steer_link"),
            ("up_right_Link", "up_right_steer_link"),
        ):
            with self.subTest(wheel=wheel_name):
                wheel_pose = model.findtext(f".//link[@name='{wheel_name}']/pose")
                steering_pose = model.findtext(
                    f".//link[@name='{steering_name}']/pose")
                self.assertEqual(wheel_pose, steering_pose)

        # The Gazebo system needs both front and rear wheel joints on each
        # side. One driven front pair produces a false, asymmetric contact
        # model even though its reported kinematic odometry looks plausible.
        plugin = model.find(
            ".//plugin[@name='ignition::gazebo::systems::AckermannSteering']")
        self.assertIsNotNone(plugin)
        assert plugin is not None
        left_joints = [
            item.text for item in plugin.findall("left_joint")]
        right_joints = [
            item.text for item in plugin.findall("right_joint")]
        self.assertEqual(left_joints, ["up_left_joint", "down_left_joint"])
        self.assertEqual(right_joints, ["up_right_joint", "down_right_joint"])
        # Fortress' velocity bound applies to both v and w. Nav2 owns the
        # coupled Ackermann limits, so a hidden plugin cap cannot widen a
        # requested 0.22 m arc.
        self.assertIsNone(plugin.find("min_velocity"))
        self.assertIsNone(plugin.find("max_velocity"))

    def test_ground_truth_pose_source_is_continuous_model_pose_publisher(self) -> None:
        model = ET.parse(ROBOT_MODEL).getroot()
        plugin = model.find(
            ".//plugin[@name='ignition::gazebo::systems::PosePublisher']")
        self.assertIsNotNone(plugin)
        assert plugin is not None
        self.assertEqual(
            plugin.attrib["filename"], "ignition-gazebo-pose-publisher-system")
        self.assertEqual(plugin.findtext("publish_link_pose"), "false")
        # Fortress only includes the top-level model when both flags are set.
        self.assertEqual(plugin.findtext("publish_nested_model_pose"), "true")
        self.assertEqual(plugin.findtext("publish_model_pose"), "true")
        self.assertEqual(plugin.findtext("use_pose_vector_msg"), "true")
        self.assertEqual(plugin.findtext("update_frequency"), "30")

        bridge = yaml.safe_load(
            (SIM / "config" / "gz_bridge.yaml").read_text(encoding="utf-8"))
        model_pose = next(
            item for item in bridge
            if item["ros_topic_name"] == "/model/origincar/pose")
        self.assertEqual(model_pose["gz_topic_name"], "/model/origincar/pose")
        self.assertEqual(model_pose["ros_type_name"], "tf2_msgs/msg/TFMessage")
        self.assertEqual(model_pose["gz_type_name"], "gz.msgs.Pose_V")
        self.assertEqual(model_pose["direction"], "GZ_TO_ROS")

        launch = LAUNCH.read_text(encoding="utf-8")
        relay = GAZEBO_GROUND_TRUTH_ODOM_RELAY.read_text(encoding="utf-8")
        self.assertIn('"physical_pose_topic": "/model/origincar/pose"', launch)
        self.assertIn(
            '"physical_pose_topic", "/model/origincar/pose"', relay)
        self.assertIn('"model_name", "origincar"', relay)
        self.assertNotIn('"dynamic_pose_topic"', relay)

    def test_rviz_scan_is_single_frame_for_timestamp_diagnosis(self) -> None:
        rviz = RVIZ.read_text(encoding="utf-8")

        self.assertIn("Name: LaserScan", rviz)
        self.assertIn("Value: /scan", rviz)
        self.assertIn("Depth: 1", rviz)
        self.assertIn("Queue Size: 1", rviz)
        self.assertIn("Decay Time: 0.0", rviz)

    def test_perception_monitor_checks_raw_costmaps_without_motion_output(self) -> None:
        source = PERCEPTION_MONITOR.read_text(encoding="utf-8")
        launch = LAUNCH.read_text(encoding="utf-8")
        cmake = (SIM / "CMakeLists.txt").read_text(encoding="utf-8")
        package = PACKAGE_XML.read_text(encoding="utf-8")

        for topic in (
            '"/scan"',
            '"/odom_combined"',
            '"/local_costmap/costmap_raw"',
            '"/global_costmap/costmap_raw"',
            '"/smartcar/sim_perception_ready"',
        ):
            with self.subTest(topic=topic):
                self.assertIn(topic, source)
        self.assertIn("lookup_transform", source)
        self.assertIn("_costmap_has_match", source)
        self.assertIn("a_zone_probe_bounds", source)
        self.assertIn("require_a_zone_probe", source)
        self.assertIn("parse_track_landmarks", source)
        self.assertIn("point_to_landmark_boundary_distance", source)
        self.assertIn("tf_odom_alignment", source)
        self.assertIn("track_world_file", source)
        self.assertIn("from std_msgs.msg import String", source)
        self.assertIn('"schema_version": 2', source)
        self.assertIn("DurabilityPolicy.TRANSIENT_LOCAL", source)
        self.assertIn("json.dumps(", source)
        for status_field in (
            '"ready"',
            '"checks"',
            '"valid_beams"',
            '"scan_stamp_ns"',
            '"odom_stamp_ns"',
            '"tf_odom_position_error_m"',
            '"tf_odom_yaw_error_rad"',
            '"landmark_matched_ids"',
            '"local_costmap_stamp_ns"',
            '"global_costmap_stamp_ns"',
        ):
            with self.subTest(status_field=status_field):
                self.assertIn(status_field, source)
        self.assertNotIn("cmd_vel", source)
        self.assertNotIn("walls_cloud", source)
        self.assertIn("sim_perception_monitor.py", cmake)
        self.assertIn("<depend>nav2_msgs</depend>", package)
        self.assertIn("<depend>std_msgs</depend>", package)
        self.assertIn("enable_perception_monitor", launch)
        self.assertIn("executable=\"sim_perception_monitor.py\"", launch)

    def test_sensor_preflight_blocks_nav2_until_clock_odom_and_scan_are_live(self) -> None:
        source = SENSOR_PREFLIGHT.read_text(encoding="utf-8")
        launch = LAUNCH.read_text(encoding="utf-8")
        cmake = (SIM / "CMakeLists.txt").read_text(encoding="utf-8")
        package = PACKAGE_XML.read_text(encoding="utf-8")

        for topic in ('"/clock"', '"/odom_combined"', '"/scan"'):
            with self.subTest(topic=topic):
                self.assertIn(topic, source)
        for required_check in (
            "clock_not_advancing",
            "odom_missing",
            "odom_stream_short",
            "scan_stream_short",
            "sensor_stream_short",
            "scan_has_no_nonself_returns",
            "scan_odom_skew",
            "scan_stale",
            "odom_stale",
        ):
            with self.subTest(check=required_check):
                self.assertIn(required_check, source)
        self.assertIn("valid_scan_beam_count", source)
        self.assertIn('declare_parameter("min_sensor_stream_span_sec", 1.0)', source)
        self.assertIn('declare_parameter("min_odom_samples", 15)', source)
        self.assertIn('declare_parameter("min_scan_samples", 5)', source)
        self.assertIn('declare_parameter("max_scan_odom_skew_sec", 0.075)', source)
        self.assertIn('declare_parameter("require_scan_time_tf", False)', source)
        self.assertIn('declare_parameter("tf_target_frame", "odom_combined")', source)
        self.assertIn("scan_time_tf_missing", source)
        self.assertIn("lookup_transform(", source)
        self.assertIn("return 0 if node.wait() else 2", source)
        self.assertNotIn("cmd_vel", source)
        self.assertTrue(os.access(SENSOR_PREFLIGHT, os.X_OK))
        self.assertIn("sim_sensor_preflight.py", cmake)
        self.assertIn("<depend>rosgraph_msgs</depend>", package)

        self.assertIn("sensor_preflight_timeout_sec", launch)
        self.assertIn('executable="sim_sensor_preflight.py"', launch)
        self.assertIn("start_after_sensor_preflight", launch)
        self.assertIn("target_action=sensor_preflight", launch)
        self.assertIn("on_exit=start_after_sensor_preflight", launch)
        self.assertIn("sensor preflight failed; Nav2 and auto_train are", launch)
        self.assertIn('name="sim_sensor_tf_preflight"', launch)
        self.assertIn('"require_scan_time_tf": True', launch)
        self.assertIn("start_after_sensor_tf_preflight", launch)
        self.assertIn("target_action=sensor_tf_preflight", launch)
        self.assertIn("on_exit=start_after_sensor_tf_preflight", launch)
        self.assertIn(
            "scan-time TF preflight failed; Nav2 and auto_train are", launch)
        self.assertLess(
            launch.index("sensor_tf_preflight_exit,"),
            launch.index("nav2_after_keepout,"),
        )
        self.assertNotIn("TimerAction(period=6.0", launch)
        self.assertLess(
            launch.index("sensor_preflight_exit,"),
            launch.index("nav2_after_keepout,"),
        )

    def test_perception_monitor_rejects_stale_scan_or_odom(self) -> None:
        source = PERCEPTION_MONITOR.read_text(encoding="utf-8")

        self.assertIn('declare_parameter("max_sensor_age_sec", 0.35)', source)
        self.assertIn('declare_parameter("max_scan_odom_skew_sec", 0.075)', source)
        self.assertIn("scan_fresh", source)
        self.assertIn("odom_fresh", source)
        self.assertIn('"clock_stamp_ns"', source)
        self.assertIn('"scan_age_sec"', source)
        self.assertIn('"odom_age_sec"', source)

    def test_cleanup_waits_for_stale_gazebo_processes_before_relaunch(self) -> None:
        cleanup = SIM_CLEANUP.read_text(encoding="utf-8")

        self.assertIn("remaining_pids=\"$filtered_pids\"", cleanup)
        self.assertIn("for _ in $(seq 1 50)", cleanup)
        self.assertIn("stale PIDs survived kill", cleanup)

    def test_simulation_shows_the_complete_field_at_a_readable_scale(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        rviz = RVIZ.read_text(encoding="utf-8")

        self.assertIn('executable="field_reference_node"', launch)
        self.assertIn("field_geometry.yaml", launch)
        self.assertIn('"IGN_GAZEBO_RESOURCE_PATH"', launch)
        self.assertIn("Name: Official Field Reference", rviz)
        self.assertIn("Value: /smartcar/field_reference/markers", rviz)
        self.assertIn("Scale: 80.0", rviz)
        self.assertIn("X: 2.0", rviz)
        self.assertIn("Y: 2.25", rviz)

    def test_world_includes_only_the_generated_field_model(self) -> None:
        world = WORLD.read_text(encoding="utf-8")

        self.assertEqual(world.count("model://competition_field"), 1)
        for stale_model in (
            'name="p_zone"',
            'name="qr_zone"',
            'name="b_wall_west"',
            'name="b_wall_east"',
            'name="corridor_wall_',
            'name="czone_wall"',
            'name="b_zone_wall_',
            'name="c_zone_',
            'name="wp_corner_',
        ):
            self.assertNotIn(stale_model, world)

    def test_generated_field_model_is_current_and_collision_scoped(self) -> None:
        completed = subprocess.run(
            [sys.executable, str(FIELD_MODEL_GENERATOR), "--check"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(
            completed.returncode,
            0,
            msg=completed.stdout + completed.stderr,
        )

        config = yaml.safe_load(FIELD_MODEL_CONFIG.read_text(encoding="utf-8"))
        collision_policy = config["collision"]
        self.assertIs(collision_policy["b_zone_walls"], True)
        self.assertIs(collision_policy["diagnosis_room_inner_ring"], False)
        self.assertIs(collision_policy["c_ring_outer_boundary"], False)
        self.assertIs(collision_policy["field_outer_boundary"], False)

        root = ET.parse(FIELD_MODEL).getroot()
        model = root.find("model")
        self.assertIsNotNone(model)
        self.assertEqual(model.attrib["name"], "competition_field")
        link = model.find("link")
        visuals = {item.attrib["name"]: item for item in link.findall("visual")}
        collisions = {
            item.attrib["name"]: item for item in link.findall("collision")
        }

        for visual in (
            "zone_a_surface",
            "zone_b_surface",
            "zone_c_surface",
            "b_corridor_surface",
            "c_ring_outer_center",
            "c_ring_outer_west_cap",
            "c_ring_outer_east_cap",
            "diagnosis_room_inner_center",
            "diagnosis_room_inner_west_cap",
            "diagnosis_room_inner_east_cap",
            "b_wall_west",
            "b_wall_east",
        ):
            self.assertIn(visual, visuals)
        self.assertEqual(
            set(collisions),
            {"b_wall_west_collision", "b_wall_east_collision"},
        )

        self.assertEqual(
            [
                float(value)
                for value in collisions["b_wall_west_collision"].findtext(
                    "pose"
                ).split()
            ],
            [0.5, 2.0, 0.15, 0.0, 0.0, 0.0],
        )
        self.assertEqual(
            [
                float(value)
                for value in collisions["b_wall_west_collision"].findtext(
                    "geometry/box/size"
                ).split()
            ],
            [2.0, 0.5, 0.3],
        )
        self.assertEqual(
            [
                float(value)
                for value in collisions["b_wall_east_collision"].findtext(
                    "pose"
                ).split()
            ],
            [3.5, 2.0, 0.15, 0.0, 0.0, 0.0],
        )

        outer_center = visuals["c_ring_outer_center"]
        self.assertEqual(
            [float(value) for value in outer_center.findtext("pose").split()[:2]],
            [2.0, 3.325],
        )
        self.assertEqual(
            [
                float(value)
                for value in outer_center.findtext("geometry/box/size").split()[:2]
            ],
            [2.35, 1.65],
        )
        for side, expected_x in (("west", 0.825), ("east", 3.175)):
            cap = visuals[f"c_ring_outer_{side}_cap"]
            self.assertEqual(float(cap.findtext("pose").split()[0]), expected_x)
            self.assertEqual(
                float(cap.findtext("geometry/cylinder/radius")), 0.825
            )

    def test_field_generator_resolves_isolated_install_layout(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "generate_competition_field", FIELD_MODEL_GENERATOR
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        with tempfile.TemporaryDirectory() as temporary:
            install = Path(temporary) / "install"
            script = (
                install
                / "smartcar_sim"
                / "share"
                / "smartcar_sim"
                / "scripts"
                / "generate_competition_field.py"
            )
            tools_share = (
                install / "smartcar_tools" / "share" / "smartcar_tools"
            )
            geometry = tools_share / "config" / "routes" / "field_geometry.yaml"
            geometry.parent.mkdir(parents=True)
            geometry.touch()

            resolved = module.resolve_tools_share(
                script,
                lambda package: str(tools_share) if package == "smartcar_tools" else "",
            )
            self.assertEqual(resolved, tools_share)

    def test_rviz_launches_directly_on_native_ubuntu(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")

        self.assertIn('package="rviz2"', launch)
        self.assertIn('executable="rviz2"', launch)
        self.assertIn('name="sim_rviz"', launch)
        self.assertNotIn("wait_for_wslg.sh", launch)

    def test_ground_truth_odom_relay_handles_launch_shutdown_idempotently(self) -> None:
        source = GAZEBO_GROUND_TRUTH_ODOM_RELAY.read_text(encoding="utf-8")

        self.assertIn(
            "from rclpy.executors import ExternalShutdownException",
            source,
        )
        self.assertIn(
            "except (KeyboardInterrupt, ExternalShutdownException):",
            source,
        )
        self.assertIn("if rclpy.ok():", source)

    def test_ground_truth_odom_is_the_only_simulation_tf_owner(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        relay = GAZEBO_GROUND_TRUTH_ODOM_RELAY.read_text(encoding="utf-8")
        bridge = yaml.safe_load((SIM / "config" / "gz_bridge.yaml").read_text(
            encoding="utf-8"))
        package = PACKAGE_XML.read_text(encoding="utf-8")

        self.assertIn(
            'executable="gazebo_ground_truth_odom_relay.py"',
            launch,
        )
        self.assertIn('name="gazebo_ground_truth_odom_relay"', launch)
        self.assertIn('"use_sim_time": True', launch)
        self.assertNotIn('executable="odom_combined_relay.py"', launch)
        self.assertIn("from geometry_msgs.msg import TransformStamped", relay)
        self.assertIn("from tf2_msgs.msg import TFMessage", relay)
        self.assertIn(
            "def planar_pose_from_transform(transform: TransformStamped)", relay)
        self.assertIn("TFMessage, physical_pose_topic", relay)
        self.assertIn(
            "stamp_ns = stamp_to_nanoseconds(model_transform.header.stamp)",
            relay,
        )
        self.assertIn("transform.child_frame_id == self._model_name", relay)
        self.assertIn("expected_pose_count", relay)
        self.assertIn("non-monotonic Gazebo physical pose timestamp", relay)
        self.assertNotIn("PoseArray", relay)
        self.assertNotIn("from rosgraph_msgs.msg import Clock", relay)
        self.assertNotIn("self._latest_clock_ns", relay)
        self.assertNotIn("self._maybe_publish_latest", relay)
        self.assertNotIn("get_clock()", relay)
        self.assertNotIn("create_timer(", relay)
        self.assertIn("transform.header.stamp = odom.header.stamp", relay)
        self.assertIn("self._tf_broadcaster.sendTransform", relay)
        self.assertNotIn('executable="odom_relay.py"', launch)
        bridge_source = (SIM / "config" / "gz_bridge.yaml").read_text(
            encoding="utf-8")
        self.assertNotIn('ros_topic_name: "/odom"', bridge_source)
        self.assertIn("<depend>tf2_msgs</depend>", package)

        physical_pose = next(
            item for item in bridge
            if item["ros_topic_name"] == "/model/origincar/pose"
        )
        self.assertEqual(physical_pose["gz_topic_name"], "/model/origincar/pose")
        self.assertEqual(physical_pose["ros_type_name"], "tf2_msgs/msg/TFMessage")
        self.assertEqual(physical_pose["gz_type_name"], "gz.msgs.Pose_V")
        self.assertEqual(physical_pose["direction"], "GZ_TO_ROS")

    def test_start_script_has_no_wsl_network_dependency(self) -> None:
        source = SIM_START.read_text(encoding="utf-8")

        self.assertIn("native Ubuntu", source)
        self.assertIn("sim_cleanup.sh", source)
        self.assertNotIn("loopback0", source)
        self.assertNotIn("networkingMode", source)
        self.assertNotIn("wsl.exe", source)

    def test_runtime_dependencies_cover_simulation_entrypoints(self) -> None:
        package = PACKAGE_XML.read_text(encoding="utf-8")
        cmake = (SIM / "CMakeLists.txt").read_text(encoding="utf-8")

        for dependency in (
            "ament_index_python",
            "python3-yaml",
            "rmw_fastrtps_cpp",
            "rviz2",
            "smartcar_tools",
            "nav2_lifecycle_manager",
            "nav2_map_server",
        ):
            self.assertIn(f"<exec_depend>{dependency}</exec_depend>", package)
        for stale_dependency in (
            "ament_cmake_python",
            "robot_localization",
            "robot_state_publisher",
            "smartcar_safety",
        ):
            with self.subTest(dependency=stale_dependency):
                self.assertNotIn(stale_dependency, package)
        self.assertIn("scripts/sim_env.sh", cmake)
        self.assertNotIn("\n    scripts\n", cmake)
        for runtime_script in (
            "scripts/sim_cleanup.sh",
            "scripts/sim_env.sh",
            "scripts/sim_start.sh",
        ):
            with self.subTest(runtime_script=runtime_script):
                self.assertIn(runtime_script, cmake)
        for source_only_tool in (
            "scripts/generate_competition_field.py",
            "scripts/generate_field_map.py",
            "scripts/sim_tune.sh",
            "scripts/validate_sim_results.py",
        ):
            with self.subTest(source_only_tool=source_only_tool):
                self.assertNotIn(source_only_tool, cmake)

    def test_keepout_mask_is_current_and_uses_authoritative_geometry(self) -> None:
        completed = subprocess.run(
            [sys.executable, str(FIELD_MAP_GENERATOR), "--check"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(
            completed.returncode,
            0,
            msg=completed.stdout + completed.stderr,
        )

        descriptor = yaml.safe_load(FIELD_MAP_YAML.read_text(encoding="utf-8"))
        self.assertEqual(descriptor["mode"], "trinary")
        self.assertEqual(descriptor["resolution"], 0.025)
        self.assertEqual(descriptor["origin"], [-0.75, -0.5, 0.0])
        self.assertEqual(descriptor["negate"], 0)

        width, height, pixels = read_pgm(FIELD_MAP)
        self.assertEqual((width, height), (220, 220))
        origin = tuple(descriptor["origin"][:2])
        resolution = descriptor["resolution"]
        samples = {
            "P origin": ((0.0, 0.0), 254),
            # Heading-dependent vehicle clearance is checked by the runtime
            # full-body sweep, not painted into this 2-D static mask.
            "P-side lower free space": ((1.0, 0.0), 254),
            "P-side upper free space": ((1.0, 0.15), 254),
            "south exterior": ((0.0, -0.30), 0),
            "west exterior": ((-0.60, 0.0), 0),
            "east exterior": ((4.60, 0.0), 0),
            "north exterior": ((0.0, 4.85), 0),
            "B corridor": ((2.0, 2.0), 254),
            "B west wall": ((0.5, 2.0), 0),
            "C inner": ((2.0, 3.3), 0),
            "C inner west lane": ((1.1, 3.3), 254),
            "C inner north lane": ((2.0, 3.6), 254),
            "C north": ((2.0, 4.4), 0),
            "C west": ((-0.25, 3.0), 0),
            "C east": ((4.25, 3.0), 0),
            "C ring": ((0.7, 4.0), 254),
        }
        for label, ((x, y), expected) in samples.items():
            with self.subTest(label=label):
                self.assertEqual(
                    pgm_value_at(
                        pixels,
                        width,
                        height,
                        origin,
                        resolution,
                        x,
                        y,
                    ),
                    expected,
                )

        source = FIELD_MAP_GENERATOR.read_text(encoding="utf-8")
        self.assertIn("load_field_reference", source)
        self.assertIn("keepout_bounds", source)
        self.assertNotIn("WALLS = [", source)

    def test_keepout_filter_is_scoped_to_simulation_and_ready_before_nav2(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        overlay = yaml.safe_load(KEEPOUT_OVERLAY.read_text(encoding="utf-8"))
        planner = overlay["planner_server"]["ros__parameters"]["GridBased"]

        controller = overlay["controller_server"]["ros__parameters"]
        navigator = overlay["bt_navigator"]["ros__parameters"]
        self.assertEqual(planner["minimum_turning_radius"], 0.22)
        # A locked P->A terminal is planned directly by Smac Hybrid-A*.
        self.assertEqual(planner["analytic_expansion_max_length"], 1.5)
        self.assertEqual(
            navigator["free_heading_minimum_turning_radius"],
            planner["minimum_turning_radius"],
        )
        for node_name in (
            "bt_navigator_navigate_to_pose_rclcpp_node",
            "bt_navigator_navigate_through_poses_rclcpp_node",
        ):
            with self.subTest(node=node_name):
                self.assertEqual(
                    overlay[node_name]["ros__parameters"]
                    ["free_heading_minimum_turning_radius"],
                    planner["minimum_turning_radius"],
                )
        self.assertEqual(
            controller["ForwardAvoidance"]["forward_min_turning_radius"],
            planner["minimum_turning_radius"],
        )
        self.assertAlmostEqual(
            controller["ForwardAvoidance"]["forward_max_angular_velocity"],
            0.30 / planner["minimum_turning_radius"],
        )
        self.assertNotIn("ForwardHandoff", controller)
        self.assertEqual(
            controller["controller_plugins"],
            ["FollowPath", "ForwardAvoidance", "ReverseHandoff", "ReverseRecovery"],
        )
        follow_path = controller["FollowPath"]
        self.assertEqual(follow_path["desired_linear_vel"], 0.30)
        self.assertAlmostEqual(follow_path["lookahead_dist"], 0.5)
        self.assertAlmostEqual(follow_path["min_lookahead_dist"], 0.35)
        self.assertAlmostEqual(follow_path["max_lookahead_dist"], 0.8)
        self.assertIs(follow_path["use_velocity_scaled_lookahead_dist"], True)
        self.assertEqual(
            follow_path["regulated_linear_scaling_min_radius"],
            planner["minimum_turning_radius"],
        )
        self.assertIs(follow_path["use_collision_detection"], True)
        self.assertIs(follow_path["allow_reversing"], False)
        self.assertIs(follow_path["use_rotate_to_heading"], False)
        for key in follow_path:
            self.assertNotIn("forward_", key)
        self.assertEqual(
            controller["ReverseHandoff"]["AckermannConstraints"]
            ["min_turning_r"], planner["minimum_turning_radius"])
        self.assertEqual(controller["ReverseHandoff"]["vx_max"], 0.30)
        self.assertAlmostEqual(
            controller["ReverseHandoff"]["wz_max"],
            0.30 / planner["minimum_turning_radius"],
        )
        self.assertEqual(
            controller["ReverseRecovery"]["AckermannConstraints"]
            ["min_turning_r"], planner["minimum_turning_radius"])
        self.assertEqual(controller["ReverseRecovery"]["vx_max"], 0.30)

        # Keep heading quantization finer than the base planner; the separate
        # continuous full-body sweep remains the final safety authority.
        self.assertGreaterEqual(planner["angle_quantization_bins"], 144)
        self.assertGreaterEqual(planner["cost_penalty"], 4.0)
        self.assertEqual(planner["lookup_table_size"], 6.0)

        for costmap_name in ("local_costmap", "global_costmap"):
            parameters = overlay[costmap_name][costmap_name]["ros__parameters"]
            self.assertEqual(parameters["filters"], ["keepout_filter"])
            keepout = parameters["keepout_filter"]
            self.assertEqual(keepout["plugin"], "nav2_costmap_2d::KeepoutFilter")
            self.assertIs(keepout["enabled"], True)
            self.assertEqual(keepout["filter_info_topic"], "/keepout_filter_info")
            self.assertEqual(parameters["inflation_layer"]["inflation_radius"], 0.15)

        self.assertIn('executable="costmap_filter_info_server"', launch)
        self.assertIn('"topic_name": "/keepout_filter_mask"', launch)
        self.assertIn('"mask_topic": "/keepout_filter_mask"', launch)
        self.assertIn("OnStateTransition", launch)
        self.assertIn('goal_state="active"', launch)
        self.assertIn('"bond_timeout": 20.0', launch)
        self.assertIn("launch_nav2_once", launch)
        self.assertIn("OpaqueFunction(function=launch_nav2_once)", launch)

        # Every run starts from the same simulation-only keepout overlay,
        # materialized into one Nav2 parameter file before lifecycle startup.
        self.assertIn(
            "keepout_overlay_path = Path(nav2_keepout_overlay.perform(context))",
            launch,
        )
        self.assertIn("write_merged_nav2_parameters(", launch)
        self.assertIn('"params_file": str(params_path)', launch)
        self.assertIn('"params_overlay_file": ""', launch)
        self.assertIn('"lifecycle_manager_delay_sec": "6.0"', launch)
        self.assertIn('"autostart": "false"', launch)
        self.assertIn('executable="nav2_lifecycle_startup.py"', launch)
        self.assertIn("target_action=nav2_lifecycle_startup", launch)
        self.assertIn("on_exit=start_after_nav2_lifecycle", launch)
        self.assertIn("Nav2 lifecycle startup failed; auto_train is not", launch)
        self.assertIn("nav2_lifecycle_startup_exit,", launch)
        self.assertTrue(os.access(NAV2_LIFECYCLE_STARTUP, os.X_OK))
        startup_source = NAV2_LIFECYCLE_STARTUP.read_text(encoding="utf-8")
        self.assertIn("ManageLifecycleNodes.Request.STARTUP", startup_source)
        self.assertEqual(
            startup_source.count("ManageLifecycleNodes.Request.STARTUP"), 1)
        self.assertIn("State.PRIMARY_STATE_ACTIVE", startup_source)
        self.assertIn("Nav2 lifecycle services discovered", startup_source)
        self.assertIn(
            "def _wait_for_startup_or_active", startup_source)
        self.assertIn(
            "STARTUP response is pending, but ",
            startup_source,
        )
        self.assertIn(
            "STARTUP pending activation",
            startup_source,
        )
        self.assertIn("STARTUP_STATE_PROBE_INTERVAL_SEC", startup_source)
        self.assertNotIn("cmd_vel", startup_source)
        self.assertNotIn("ros2 lifecycle set /map_server", launch)

    def test_route_planning_sync_keeps_native_rpp_radius_in_lockstep(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "sync_route_planning_for_test", ROUTE_PLANNING_SYNC)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        try:
            spec.loader.exec_module(module)
            route_planning = yaml.safe_load(
                ROUTE_PLANNING.read_text(encoding="utf-8"))
            expected_radius = route_planning[
                "simulation_minimum_turning_radius_m"]
            with tempfile.TemporaryDirectory() as temporary:
                overlay_path = Path(temporary) / "nav2_keepout_filter.yaml"
                overlay_path.write_text(
                    KEEPOUT_OVERLAY.read_text(encoding="utf-8"),
                    encoding="utf-8",
                )
                self.assertFalse(
                    module.synchronize(ROUTE_PLANNING, overlay_path, check=True))
                overlay = yaml.safe_load(
                    overlay_path.read_text(encoding="utf-8"))
        finally:
            sys.modules.pop(spec.name, None)

        follow_path = overlay["controller_server"]["ros__parameters"]["FollowPath"]
        self.assertEqual(
            follow_path["regulated_linear_scaling_min_radius"], expected_radius)

    def test_run_route_uses_an_isolated_switch_and_explicit_trees(self) -> None:
        launch = LAUNCH.read_text(encoding="utf-8")
        runner = AUTO_TRAIN.read_text(encoding="utf-8")

        self.assertIn('LaunchConfiguration("run_route"', launch)
        self.assertIn('LaunchConfiguration("waypoints_file")', launch)
        self.assertIn('DeclareLaunchArgument(\n            "waypoints_file"', launch)
        self.assertIn('pkg_nav2, "config", "waypoints", "nav_only.yaml"', launch)
        self.assertIn("condition=IfCondition(run_route)", launch)
        self.assertNotIn('LaunchConfiguration("autostart"', launch)
        self.assertIn("target_action=auto_train", launch)
        self.assertIn("complete route runner exited", launch)
        self.assertIn('LaunchConfiguration(\n        "shutdown_on_route_exit"', launch)
        self.assertIn('DeclareLaunchArgument(\n            "shutdown_on_route_exit"', launch)
        self.assertIn("Gazebo/RViz", launch)
        for parameter in (
            '"waypoints_file"',
            '"forward_behavior_tree"',
            '"precise_behavior_tree"',
            '"reverse_behavior_tree"',
            '"reverse_handoff_behavior_tree"',
            '"nav2_params_file"',
        ):
            self.assertIn(parameter, launch)
            self.assertIn(parameter, runner)
        self.assertNotIn("TRAIN_WAYPOINTS", runner)
        self.assertIn('goal.behavior_tree = str(behavior_tree)', runner)
        self.assertIn("materialize_navigation_segments", runner)
        self.assertIn("materialize_free_yaws", runner)
        self.assertIn("NavigateThroughPoses", runner)
        self.assertIn("NavigateThroughPoses.Goal()", runner)
        self.assertIn("goal.poses.append(ps)", runner)
        self.assertIn("os.replace(temporary, path)", runner)
        self.assertIn("raise SystemExit(exit_code)", runner)
        self.assertIn("self._input_manifest_cache = self._input_manifest()", runner)
        self.assertIn("path_start = len(self._path_messages)", runner)
        self.assertIn("accepted_path_start = len(self._accepted_path_messages)", runner)
        self.assertIn("ACCEPTED_CONTROLLER_PATH_TOPIC", runner)
        self.assertIn("MAX_ACCEPTED_PATH_TRACE_POINTS", runner)
        self.assertIn("MAX_EXECUTION_TRACE_SAMPLES", runner)
        self.assertIn('"accepted_path_trace"', runner)
        self.assertIn('"tracking_trace"', runner)
        self.assertIn('"station_m"', runner)
        self.assertIn('"cross_track_m"', runner)
        self.assertIn('"path_heading_error_rad"', runner)
        self.assertIn('"forward_path_max_cross_track_error_m"', runner)
        self.assertIn(
            'ACCEPTED_CONTROLLER_PATH_TOPIC = "/smartcar/accepted_global_plan"',
            runner,
        )
        accepted_qos = runner[
            runner.index("accepted_path_qos = QoSProfile("):
            runner.index("perception_qos = QoSProfile(")
        ]
        self.assertIn("ReliabilityPolicy.RELIABLE", accepted_qos)
        self.assertIn("DurabilityPolicy.TRANSIENT_LOCAL", accepted_qos)

        self.assertIn("self._accepted_path_cb", runner)
        self.assertIn("rclpy.spin_once(self, timeout_sec=0.1)", runner)
        self.assertNotIn("time.sleep(delay)", runner)
        self.assertIn('"action_endpoint_settle_sec"', runner)
        self.assertIn("wait_for_server() only proves the request path exists", runner)
        self.assertIn("Settling Nav2 action endpoints", runner)
        self.assertIn("self._settle_action_endpoints()", runner)
        self.assertIn("load_planning_segments", runner)
        self.assertIn('stage["direction"]', runner)
        self.assertIn("_run_stage", runner)
        self.assertNotIn("EXPECTED_ROUTE", runner)
        self.assertNotIn("reverse_tp_start_id", runner)

        nav_only = yaml.safe_load(
            (ROOT / "src" / "smartcar_nav2" / "config" / "waypoints" / "nav_only.yaml")
            .read_text(encoding="utf-8")
        )
        self.assertEqual(
            [
                (segment["id"], segment["direction"], segment["start_id"], segment["end_id"])
                for segment in nav_only["planning_segments"]
            ],
            [
                ("p_to_qr", "forward", "p_start", "a_task_observe"),
                ("qr_to_vlm", "reverse", "a_task_observe", "c_corner_1"),
                ("return_to_p", "reverse", "c_corner_1", "p_finish"),
            ],
        )
        self.assertEqual(
            [segment["through_ids"] for segment in nav_only["planning_segments"]],
            [[], ["via_2"], ["via_1", "via_3"]],
        )
        self.assertEqual(
            [waypoint["id"] for waypoint in nav_only["waypoints"]],
            [
                "p_start",
                "a_task_observe",
                "via_2",
                "c_corner_1",
                "via_1",
                "via_3",
                "p_finish",
            ],
        )
        waypoint_ids = {waypoint["id"] for waypoint in nav_only["waypoints"]}
        self.assertFalse(
            {
                "b_corridor_gate", "b_corridor_enter", "c_entry_west",
                "c_corner_2", "c_corner_3", "c_corner_4",
                "b_corridor_return_enter", "b_corridor_return_drop",
                "b_corridor_return",
            }
            & waypoint_ids
        )
        c_corner_1 = next(
            waypoint for waypoint in nav_only["waypoints"]
            if waypoint["id"] == "c_corner_1"
        )
        self.assertEqual(c_corner_1["task"], "nav")
        self.assertEqual(c_corner_1["direction"], "reverse")
        self.assertEqual(c_corner_1["heading_mode"], "locked")
        self.assertEqual(c_corner_1["goal_profile"], "reverse_handoff")
        self.assertIn("orientation", c_corner_1["pose"])
        via_1 = next(
            waypoint for waypoint in nav_only["waypoints"]
            if waypoint["id"] == "via_1"
        )
        self.assertEqual(via_1["task"], "via")
        self.assertEqual(via_1["direction"], "reverse")
        self.assertEqual(via_1["goal_profile"], "standard")
        self.assertNotIn("orientation", via_1["pose"])
        direct_return = next(
            waypoint for waypoint in nav_only["waypoints"]
            if waypoint["id"] == "p_finish"
        )
        self.assertEqual(direct_return["task"], "return")
        self.assertEqual(direct_return["direction"], "reverse")
        self.assertEqual(
            direct_return["pose"]["orientation"],
            {
                "x": 0.0,
                "y": 0.0,
                "z": 0.3826834323650898,
                "w": 0.9238795325112867,
            },
        )
        self.assertIn("orientation", direct_return["pose"])

    def test_sim_precise_tree_uses_native_smac_and_rpp(self) -> None:
        real_compute = ET.parse(REAL_PRECISE_BT).find(
            ".//ComputeFreeHeadingPathToPose")
        root = ET.parse(SIM_PRECISE_BT).getroot()
        tags = [element.tag for element in root.iter()]
        sim_compute = root.find(".//ComputePathToPose")
        sim_follow = root.find(".//FollowPath")

        self.assertIsNotNone(real_compute)
        self.assertIsNotNone(sim_compute)
        self.assertIsNotNone(sim_follow)
        self.assertEqual(sim_compute.attrib["planner_id"], "GridBased")
        self.assertEqual(sim_follow.attrib["controller_id"], "FollowPath")
        self.assertEqual(sim_follow.attrib["goal_checker_id"], "precise_goal_checker")
        for custom_tag in (
            "ComputeFreeHeadingPathToPose",
            "RecordFollowPath",
            "OdomDistanceBudget",
            "LatchSuccess",
        ):
            self.assertNotIn(custom_tag, tags)
        for forbidden_tag in ("Spin", "BackUp", "Wait", "DriveOnHeading"):
            self.assertNotIn(forbidden_tag, tags)

        keepout_overlay = yaml.safe_load(KEEPOUT_OVERLAY.read_text(encoding="utf-8"))
        controller = keepout_overlay["controller_server"]["ros__parameters"]
        self.assertNotIn("ForwardHandoff", controller)
        # The simulation inherits the base reverse arrival envelope rather
        # than adding a simulation-only terminal constraint.
        self.assertNotIn("reverse_goal_checker", controller)
        self.assertEqual(
            controller["controller_plugins"],
            ["FollowPath", "ForwardAvoidance", "ReverseHandoff", "ReverseRecovery"],
        )
        follow_path = controller["FollowPath"]
        self.assertIs(follow_path["allow_reversing"], False)
        self.assertIs(follow_path["use_rotate_to_heading"], False)
        self.assertEqual(follow_path["regulated_linear_scaling_min_radius"], 0.22)
        self.assertFalse(any(key.startswith("forward_") for key in follow_path))

        launch = LAUNCH.read_text(encoding="utf-8")
        self.assertIn(
            '"navigate_to_pose_precise_sim_w_replanning_and_recovery.xml"',
            launch,
        )

    def test_sim_reverse_retreat_sweeps_keepout_without_changing_real_trees(self) -> None:
        def signature(element):
            return (
                element.tag,
                tuple(sorted(element.attrib.items())),
                tuple(signature(child) for child in element),
            )

        for real_tree, sim_tree in zip(
            REAL_REVERSE_BT_FILES, SIM_REVERSE_BT_FILES
        ):
            with self.subTest(behavior_tree=sim_tree.name):
                self.assertTrue(sim_tree.is_file())
                real_root = ET.parse(real_tree).getroot()
                sim_root = ET.parse(sim_tree).getroot()
                retreat_nodes = sim_root.findall(
                    ".//AckermannReverseRetreat")
                self.assertEqual(len(retreat_nodes), 1)
                retreat = retreat_nodes[0]
                self.assertEqual(
                    retreat.attrib["static_keepout_mask_topic"],
                    "/keepout_filter_mask",
                )
                self.assertEqual(retreat.attrib["retreat_distance_m"], "0.15")
                retreat.attrib.pop("static_keepout_mask_topic")

                if sim_tree.name == (
                    "navigate_to_pose_reverse_handoff_sim_w_replanning_and_recovery.xml"
                ):
                    # C1 uses the configured reverse arrival envelope. Once
                    # Nav2 reports a valid arrival, do not add a second,
                    # stricter terminal-pose check followed by a recovery.
                    follow = sim_root.find(".//RecordFollowPath")
                    self.assertIsNotNone(follow)
                    self.assertNotIn("verify_physical_terminal_pose", follow.attrib)
                    self.assertNotIn("terminal_goal", follow.attrib)
                    self.assertNotIn("terminal_position_tolerance_m", follow.attrib)
                    self.assertNotIn("terminal_yaw_tolerance_rad", follow.attrib)
                    self.assertNotIn("terminal_verification_delay_ms", follow.attrib)
                    self.assertNotIn("terminal_recovery_eligible", follow.attrib)
                    compute = sim_root.find(
                        ".//ComputeReverseFreeHeadingPathToPose")
                    self.assertIsNotNone(compute)
                    self.assertEqual(
                        compute.attrib["recovery_eligible"],
                        "{reverse_handoff_recovery_eligible}")
                    self.assertEqual(
                        retreat.attrib["allow_retreat"],
                        "{reverse_handoff_recovery_eligible}")
                    self.assertEqual(retreat.attrib["controller_id"], "FollowPath")
                    self.assertEqual(retreat.attrib["retreat_direction"], "forward")
                    self.assertEqual(
                        retreat.attrib["allow_static_scan_only_evidence"], "true")
                    recovery = sim_root.find(
                        ".//Fallback[@name='RecoverReverseHandoffTerminalPoseOrClear']")
                    self.assertIsNotNone(recovery)
                    retreat_branch = recovery.find(
                        "./Sequence[@name='ReverseHandoffRetreatBeforeReplan']")
                    self.assertIsNotNone(retreat_branch)
                    self.assertEqual(
                        [child.tag for child in retreat_branch][:4],
                        [
                            "ReverseRecoveryEligible",
                            "ClearEntireCostmap",
                            "ClearEntireCostmap",
                            "AckermannReverseRetreat",
                        ],
                    )
                    clear_branch = recovery.find(
                        "./Sequence[@name='ClearReverseHandoffCostmapsRecovery']")
                    self.assertIsNotNone(clear_branch)
                    self.assertEqual(clear_branch[0].tag, "Inverter")
                    self.assertEqual(
                        clear_branch[0][0].attrib["eligible"],
                        "{reverse_handoff_recovery_eligible}")

                    record_follow_source = (
                        ROOT / "src" / "smartcar_nav2" / "src"
                        / "record_follow_path_action.cpp"
                    ).read_text(encoding="utf-8")
                    verifier = record_follow_source.split(
                        "RecordFollowPathAction::verifyPhysicalTerminalPose()",
                        1,
                    )[1].split("void RecordFollowPathAction::halt()", 1)[0]
                    self.assertIn('getInput("terminal_goal", target)', verifier)
                    self.assertIn('getInput("terminal_verification_delay_ms",', verifier)
                    self.assertIn("terminal_verification_deadline_", verifier)
                    self.assertIn("TerminalVerificationResult::kWaiting", verifier)
                    position_failure = verifier.index(
                        "if (position_error > position_tolerance_m)")
                    retreat_authorization = verifier.index(
                        'setOutput("terminal_recovery_eligible", true)')
                    self.assertLess(position_failure, retreat_authorization)
                    position_failure_block = verifier.split(
                        "if (position_error > position_tolerance_m)", 1
                    )[1].split(
                        "// The vehicle is still at the intended terminal position", 1
                    )[0]
                    self.assertNotIn(
                        'setOutput("terminal_recovery_eligible", true)',
                        position_failure_block,
                    )
                    continue

                self.assertEqual(retreat.attrib["controller_id"], "ReverseRecovery")
                self.assertNotIn("retreat_direction", retreat.attrib)

                real_retreat_nodes = real_root.findall(
                    ".//AckermannReverseRetreat")
                self.assertEqual(len(real_retreat_nodes), 1)
                self.assertNotIn(
                    "static_keepout_mask_topic", real_retreat_nodes[0].attrib)
                self.assertEqual(signature(sim_root), signature(real_root))

        launch = LAUNCH.read_text(encoding="utf-8")
        for behavior_tree in SIM_REVERSE_BT_FILES:
            self.assertIn(f'"{behavior_tree.name}"', launch)
        self.assertIn('"reverse_behavior_tree": bt_reverse', launch)
        self.assertIn('"reverse_handoff_behavior_tree": bt_reverse_handoff', launch)
        self.assertIn(
            '"through_poses_reverse_behavior_tree": bt_reverse_through_poses',
            launch,
        )
        self.assertIn(
            '"through_poses_reverse_locked_behavior_tree": (\n'
            '                bt_reverse_locked_through_poses)',
            launch,
        )

        cmake = (ROOT / "src" / "smartcar_nav2" / "CMakeLists.txt").read_text(
            encoding="utf-8")
        for behavior_tree in SIM_REVERSE_BT_FILES:
            self.assertIn(behavior_tree.name, cmake)

    def test_route_results_preserve_planning_and_execution_yaw_evidence(self) -> None:
        runner = AUTO_TRAIN.read_text(encoding="utf-8")

        for field in (
            '"heading_mode"',
            '"target_yaw_rad"',
            '"final_yaw_rad"',
            '"signed_goal_yaw_error_rad"',
            '"plan_final_yaw_rad"',
            '"signed_plan_goal_yaw_error_rad"',
            '"cmd_positive_sample_count"',
            '"cmd_negative_sample_count"',
            '"goal_profile"',
        ):
            self.assertIn(field, runner)
        self.assertIn("orientation = endpoint_pose.orientation", runner)
        self.assertIn("matching_paths[-1][2]", runner)
        self.assertIn("plan_execution_final_yaw = plan_final_yaw", runner)
        self.assertNotIn(
            'plan_final_yaw + (math.pi if direction == "reverse" else 0.0)',
            runner,
        )
        self.assertIn('contract_errors.append("reverse_velocity_sign")', runner)
        self.assertIn("if positive_command_samples > 0:", runner)
        self.assertIn('if heading_mode == "locked":', runner)
        self.assertIn("free-heading route goals must use the zero quaternion", runner)

    def test_route_runner_requires_perception_and_executed_travel_evidence(self) -> None:
        runner = AUTO_TRAIN.read_text(encoding="utf-8")
        validator = (
            SIM / "scripts" / "validate_sim_results.py"
        ).read_text(encoding="utf-8")

        self.assertIn("from std_msgs.msg import String", runner)
        self.assertIn('"perception_ready_topic"', runner)
        self.assertIn("DurabilityPolicy.TRANSIENT_LOCAL", runner)
        self.assertIn("def _wait_for_perception", runner)
        self.assertIn('self._save_results("failed", "sim_perception_not_ready")', runner)
        self.assertLess(
            runner.index("if not self._wait_for_perception():"),
            runner.index("for index, stage in enumerate(stages, start=1):"),
        )
        for field in (
            '"executed_travel_m"',
            '"executed_travel_baseline_m"',
            '"executed_travel_detour_ratio"',
            '"executed_travel_limit_m"',
            '"executed_travel_detour_violation"',
        ):
            with self.subTest(field=field):
                self.assertIn(field, runner)
                self.assertIn(field, validator)
        self.assertIn("_validate_perception(data, errors)", validator)
        self.assertIn("_validate_executed_travel_detour", validator)

    def test_rviz_path_qos_distinguishes_accepted_global_path(self) -> None:
        rviz = RVIZ.read_text(encoding="utf-8")

        for topic in (
            "/plan",
            "/transformed_global_plan",
        ):
            topic_offset = rviz.index(f"Value: {topic}")
            qos_block = rviz[max(0, topic_offset - 180):topic_offset]
            self.assertIn("Durability Policy: Volatile", qos_block)
        accepted_topic = "/smartcar/accepted_global_plan"
        topic_offset = rviz.index(f"Value: {accepted_topic}")
        accepted_qos = rviz[max(0, topic_offset - 220):topic_offset]
        self.assertIn("Durability Policy: Transient Local", accepted_qos)
        self.assertIn("Reliability Policy: Reliable", accepted_qos)
        self.assertIn(
            "Name: Accepted Global Path (/smartcar/accepted_global_plan)", rviz)
        self.assertIn("Name: Planner Candidate Path (/plan)", rviz)

    def test_tuning_rebuilds_fixed_params_and_targets_qr_checker(self) -> None:
        runner = SIM_TUNE.read_text(encoding="utf-8")
        tuner = TUNE_PARAMS.read_text(encoding="utf-8")

        self.assertIn("--packages-up-to smartcar_sim", runner)
        self.assertIn("run_route:=true", runner)
        self.assertIn('for run_index in $(seq 1 "$loop_count")', runner)
        self.assertIn("precise_goal_checker.yaw_goal_tolerance", tuner)
        self.assertIn("precise_goal_checker.xy_goal_tolerance", tuner)
        self.assertNotIn("GridBased.curvature_tolerance", tuner)
        self.assertIn("render_params", tuner)
        self.assertIn("flock -n 9", runner)
        self.assertIn("workspace_lock_id=$(printf", runner)
        self.assertIn('/tmp/smartcar_sim_tune_${workspace_lock_id}.lock', runner)
        self.assertIn("9>&- >\"$log_file\" 2>&1 &", runner)
        self.assertIn("WORKSPACE_LOCK_ID = hashlib.sha256", tuner)
        self.assertIn("/tmp/smartcar_sim_tune_{WORKSPACE_LOCK_ID}.lock", tuner)
        self.assertNotIn('WS / ".sim_tune.lock"', tuner)
        self.assertIn('source_root="${workspace}/src"', runner)
        self.assertIn('workspace=${SMARTCAR_WS:-$(cd "${script_dir}/../../.." && pwd)}', runner)
        self.assertNotIn("--sync-from-windows", runner)
        self.assertNotIn("SMARTCAR_WINDOWS_SRC", runner)
        self.assertNotIn("backup_wsl_file_before_sync", runner)
        self.assertIn(
            'waypoints_file="${source_root}/smartcar_nav2/config/waypoints/nav_only.yaml"',
            runner,
        )
        self.assertIn(
            '"${source_root}/smartcar_nav2/config/nav2_params.yaml"',
            runner,
        )
        self.assertIn('sha256sum "$1"', runner)
        self.assertIn('waypoints_file:="$snapshot_dir/nav_only.yaml"', runner)
        self.assertIn('--waypoints-file "$snapshot_dir/nav_only.yaml"', runner)
        self.assertIn('SOURCE_ROOT = WS / "src"', tuner)
        self.assertIn('SCRIPT = Path(__file__).resolve()', tuner)
        self.assertNotIn("require_workspace_source", tuner)
        self.assertNotIn("SMARTCAR_NAV2_PARAMS", tuner)
        self.assertIn("fcntl.flock", tuner)

    def test_tunable_defaults_match_yaml_and_render_preserves_comments(self) -> None:
        spec = importlib.util.spec_from_file_location("tune_params", TUNE_PARAMS)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        params_file = ROOT / "src" / "smartcar_nav2" / "config" / "nav2_params.yaml"
        data = module.load_params(params_file)

        for info in module.TUNABLE_PARAMS.values():
            self.assertEqual(
                module.get_nested(data, info["path"]), info["default"])

        changed = copy.deepcopy(data)
        precise = module.TUNABLE_PARAMS["1"]
        module.set_nested(changed, precise["path"], 0.20)
        source = params_file.read_text(encoding="utf-8")
        rendered = module.render_params(changed, source)
        self.assertIn("# The QR pose seeds a direction change.", rendered)
        self.assertEqual(len(rendered.splitlines()), len(source.splitlines()))
        self.assertIn("yaw_goal_tolerance: 0.2", rendered)
        self.assertNotIn(
            "minimum_turning_radius",
            {info["name"] for info in module.TUNABLE_PARAMS.values()},
        )
        self.assertNotIn(
            "GridBased.minimum_turning_radius",
            {info["path"] for info in module.TUNABLE_PARAMS.values()},
        )

    def test_tuning_uses_explicit_workspace_when_requested(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary) / "workspace"
            with mock.patch.dict(
                os.environ,
                {
                    "SMARTCAR_WS": str(workspace),
                },
                clear=False,
            ):
                spec = importlib.util.spec_from_file_location(
                    "tune_params_workspace_source", TUNE_PARAMS)
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)

                self.assertEqual(module.SOURCE_ROOT, workspace / "src")

    def test_restore_only_changes_tunable_parameters(self) -> None:
        spec = importlib.util.spec_from_file_location("tune_params_restore", TUNE_PARAMS)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        source = ROOT / "src" / "smartcar_nav2" / "config" / "nav2_params.yaml"

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            current_file = root / "nav2_params.yaml"
            backup_dir = root / "backups"
            backup_dir.mkdir()

            current = module.load_params(source)
            backup = copy.deepcopy(current)
            tunable_path = module.TUNABLE_PARAMS["1"]["path"]
            non_tunable_path = "controller_server.ros__parameters.controller_frequency"
            module.set_nested(current, tunable_path, 0.15)
            module.set_nested(current, non_tunable_path, 17.0)
            module.set_nested(backup, tunable_path, 0.20)
            module.set_nested(backup, non_tunable_path, 99.0)
            current_file.write_text(
                yaml.safe_dump(current, sort_keys=False),
                encoding="utf-8",
            )
            (backup_dir / "sample.yaml").write_text(
                yaml.safe_dump(backup, sort_keys=False),
                encoding="utf-8",
            )

            module.PARAMS_FILE = current_file
            module.BACKUP_DIR = backup_dir
            self.assertTrue(module.restore_params("sample"))
            restored = module.load_params(current_file)
            self.assertEqual(module.get_nested(restored, tunable_path), 0.20)
            self.assertEqual(module.get_nested(restored, non_tunable_path), 17.0)
            self.assertFalse(module.restore_params("../sample"))


if __name__ == "__main__":
    unittest.main()
