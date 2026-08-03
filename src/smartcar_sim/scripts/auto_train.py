#!/usr/bin/env python3
"""Run the complete navigation-only route inside the Gazebo simulation."""

import json
import hashlib
import math
import os
import time
import traceback
from collections.abc import Mapping
import copy
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from smartcar_task.planning_segments import (
    PlanningSegmentError,
    allows_reverse_handoff_through_poses,
    load_planning_segments,
    materialize_mission_route,
    materialize_navigation_segments,
)
from smartcar_task.route_geometry import materialize_free_yaws
from smartcar_task.waypoints import is_heading_locked, load_waypoint_document
from std_msgs.msg import String


VELOCITY_EPSILON = 1.0e-3
CONFIG_TOLERANCE_EPSILON = 2.0e-3
POSITION_OBSERVER_MARGIN_M = 2.0e-2
YAW_OBSERVER_MARGIN_RAD = 1.0e-2
YAW_IMPROVEMENT_EPSILON = 1.0e-2
HANDOFF_POST_XY_MAX_POSITION_ERROR_M = 0.75
HANDOFF_POST_XY_MAX_TRAVEL_M = 1.00
HANDOFF_POST_XY_MAX_DURATION_SEC = 25.0
REVERSE_HANDOFF_CONTROLLER = "smartcar_nav2::ReverseOnlyMPPIController"
FORWARD_AVOIDANCE_CONTROLLER = "smartcar_nav2::ForwardOnlyRPPController"
NATIVE_RPP_CONTROLLER = (
    "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
)
# Runtime removes ordinary free through-poses goals inside 0.50 m. Keep only
# the odometry observer margin above that implementation contract.
THROUGH_POSE_PASS_DISTANCE_TOLERANCE_M = 0.52
# A controller may track a valid Ackermann arc slightly longer than a global
# plan, but it must not turn a series of short replans into a large circle.
# This is deliberately looser than the per-plan contract above while still
# bounding the complete motion observed for one action or ThroughPoses stage.
MAX_EXECUTED_TRAVEL_DETOUR_RATIO = 2.00
MAX_EXECUTED_TRAVEL_DETOUR_ALLOWANCE_M = 0.80
PATH_ENDPOINT_MATCH_TOLERANCE_M = 0.35
# Keep route evidence inspectable without allowing a high-resolution Nav2 path
# to make every simulation result unbounded in size.
MAX_ACCEPTED_PATH_TRACE_POINTS = 64
# Result JSON must remain inspectable for a 120 s timeout, including failed
# actions that spend their whole budget retrying. Keep all three time series
# uniformly downsampled to this fixed maximum.
MAX_EXECUTION_TRACE_SAMPLES = 128
EXECUTION_TRACE_SCHEMA_VERSION = 1
PERCEPTION_READY_TOPIC = "/smartcar/sim_perception_ready"
# Custom route trees publish an acknowledged controller path. The native P->A
# tree has no custom FollowPath wrapper, so its route evidence is Nav2's
# native /plan publication.
ACCEPTED_CONTROLLER_PATH_TOPIC = "/smartcar/accepted_global_plan"
PERCEPTION_REQUIRED_CHECKS = (
    "scan",
    "odom",
    "tf",
    "tf_odom_alignment",
    "local",
    "global",
    "a_zone_probe",
    "landmarks",
)
PERCEPTION_STATUS_SCHEMA_VERSION = 2
HEADING_MODES = frozenset({"free", "locked"})


class AutoTrain(Node):
    def __init__(self):
        super().__init__("auto_train")

        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("forward_behavior_tree", "")
        self.declare_parameter("precise_behavior_tree", "")
        self.declare_parameter("reverse_behavior_tree", "")
        self.declare_parameter("reverse_handoff_behavior_tree", "")
        self.declare_parameter("nav2_params_file", "")
        self.declare_parameter("nav2_params_overlay_file", "")
        self.declare_parameter("through_poses_behavior_tree", "")
        self.declare_parameter("through_poses_reverse_behavior_tree", "")
        self.declare_parameter(
            "through_poses_reverse_locked_behavior_tree", "")
        self.declare_parameter(
            "through_poses_reverse_return_behavior_tree", "")
        self.declare_parameter("use_through_poses", True)
        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("inter_goal_delay_sec", 1.0)
        self.declare_parameter("action_endpoint_settle_sec", 2.0)
        self.declare_parameter("start_goal_id", "")
        self.declare_parameter("end_goal_id", "")
        self.declare_parameter(
            "results_file", "/tmp/auto_train_results.json")
        self.declare_parameter(
            "perception_ready_topic", PERCEPTION_READY_TOPIC)
        self.declare_parameter("perception_ready_timeout_sec", 60.0)
        self.declare_parameter("perception_status_max_age_sec", 3.0)

        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # GazeboGroundTruthOdomRelay publishes the Nav2 TF-owner odometry
        # reliably.  Subscribe with the matching policy so route execution
        # cannot miss the only odometry stream after a lifecycle restart.
        odom_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        planner_path_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        accepted_path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        perception_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Odometry, "/odom_combined", self._odom_cb, odom_qos)
        self.create_subscription(
            Twist, "/cmd_vel_nav", self._controller_cmd_cb, qos)
        self.create_subscription(
            Twist, "/cmd_vel_candidate", self._cmd_cb, qos)
        self.create_subscription(
            NavPath, "/plan", self._path_cb, planner_path_qos)
        self.create_subscription(
            NavPath,
            ACCEPTED_CONTROLLER_PATH_TOPIC,
            self._accepted_path_cb,
            accepted_path_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("perception_ready_topic").value),
            self._perception_cb,
            perception_qos,
        )
        self._action_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose")
        self._through_client = ActionClient(
            self, NavigateThroughPoses, "/navigate_through_poses")

        self._latest_odom = None
        self._odom_samples = []
        self._controller_cmd_samples = []
        self._cmd_samples = []
        self._path_messages = []
        self._path_metrics = []
        self._path_traces = []
        self._accepted_path_messages = []
        self._accepted_path_metrics = []
        self._accepted_path_traces = []
        self._results = []
        self._expected_goal_count = 0
        self._input_manifest_cache = None
        self._route_manifest = None
        self._perception_status = None
        self._perception_received_at = None
        self._execution_manifest = {
            "use_through_poses": bool(
                self.get_parameter("use_through_poses").value),
        }

    def _odom_cb(self, msg):
        pose = msg.pose.pose
        yaw = math.atan2(
            2.0 * (
                pose.orientation.w * pose.orientation.z
                + pose.orientation.x * pose.orientation.y
            ),
            1.0 - 2.0 * (
                pose.orientation.y * pose.orientation.y
                + pose.orientation.z * pose.orientation.z
            ),
        )
        sample = (
            time.monotonic(),
            pose.position.x,
            pose.position.y,
            yaw,
        )
        self._latest_odom = sample
        self._odom_samples.append(sample)

    def _cmd_cb(self, msg):
        self._cmd_samples.append(
            (time.monotonic(), msg.linear.x, msg.angular.z))

    def _controller_cmd_cb(self, msg):
        self._controller_cmd_samples.append(
            (time.monotonic(), msg.linear.x, msg.angular.z))

    def _perception_cb(self, msg):
        """Keep only structured evidence from the read-only simulator monitor."""
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().debug("Ignoring malformed simulation perception status")
            return
        if not isinstance(status, dict):
            self.get_logger().debug("Ignoring non-object simulation perception status")
            return
        self._perception_status = status
        self._perception_received_at = time.monotonic()

    @staticmethod
    def _compact_path_trace(msg):
        """Serialize an evenly sampled XY trace for one accepted path."""
        pose_count = len(msg.poses)
        if pose_count <= MAX_ACCEPTED_PATH_TRACE_POINTS:
            indices = range(pose_count)
        else:
            indices = sorted({
                round(index * (pose_count - 1) / (MAX_ACCEPTED_PATH_TRACE_POINTS - 1))
                for index in range(MAX_ACCEPTED_PATH_TRACE_POINTS)
            })
        return [
            {
                "x": round(msg.poses[index].pose.position.x, 3),
                "y": round(msg.poses[index].pose.position.y, 3),
            }
            for index in indices
        ]

    @staticmethod
    def _bounded_sample_indices(sample_count, maximum_count):
        """Return an ordered, endpoint-preserving bounded sample selection."""
        if sample_count <= 0:
            return ()
        if sample_count <= maximum_count:
            return range(sample_count)
        return tuple(sorted({
            round(index * (sample_count - 1) / (maximum_count - 1))
            for index in range(maximum_count)
        }))

    @staticmethod
    def _project_odom_to_accepted_path(odom_sample, path_trace):
        """Project one odom sample onto a compact acknowledged path trace."""
        if not isinstance(path_trace, list) or len(path_trace) < 2:
            return None
        try:
            robot_x = float(odom_sample[1])
            robot_y = float(odom_sample[2])
            robot_yaw = float(odom_sample[3])
        except (IndexError, TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in (robot_x, robot_y, robot_yaw)):
            return None

        points = []
        for pose in path_trace:
            if not isinstance(pose, Mapping):
                return None
            try:
                point = (float(pose["x"]), float(pose["y"]))
            except (KeyError, TypeError, ValueError):
                return None
            if not all(math.isfinite(value) for value in point):
                return None
            points.append(point)

        station_m = 0.0
        best_distance_squared = math.inf
        best_projection = None
        for index, (first, second) in enumerate(zip(points, points[1:])):
            dx = second[0] - first[0]
            dy = second[1] - first[1]
            segment_length_squared = dx * dx + dy * dy
            if not math.isfinite(segment_length_squared):
                return None
            segment_length = math.sqrt(segment_length_squared)
            if segment_length <= 1.0e-6:
                continue
            factor = min(max(
                ((robot_x - first[0]) * dx + (robot_y - first[1]) * dy)
                / segment_length_squared,
                0.0,
            ), 1.0)
            projected_x = first[0] + factor * dx
            projected_y = first[1] + factor * dy
            distance_squared = (
                (robot_x - projected_x) ** 2 + (robot_y - projected_y) ** 2)
            if not math.isfinite(distance_squared):
                return None
            if distance_squared < best_distance_squared:
                best_distance_squared = distance_squared
                best_projection = {
                    "station_m": station_m + factor * segment_length,
                    "cross_track_m": math.sqrt(distance_squared),
                    "path_heading_error_rad": math.remainder(
                        robot_yaw - math.atan2(dy, dx), 2.0 * math.pi),
                    "segment_index": index,
                }
            station_m += segment_length
        if best_projection is None or not all(
            math.isfinite(value)
            for key, value in best_projection.items()
            if key != "segment_index"
        ):
            return None
        return best_projection

    @staticmethod
    def _serialize_command_trace(samples, start_time):
        return [
            {
                "t_sec": round(sample[0] - start_time, 3),
                "linear_x": round(sample[1], 4),
                "angular_z": round(sample[2], 4),
            }
            for index in AutoTrain._bounded_sample_indices(
                len(samples), MAX_EXECUTION_TRACE_SAMPLES)
            for sample in (samples[index],)
        ]

    def _execution_trace(
        self,
        start_time,
        odom_samples,
        controller_command_samples,
        candidate_command_samples,
        path_traces,
        path_start,
    ):
        """Serialize bounded command/odom evidence against selected Nav2 plans."""
        accepted_paths = sorted(
            path_traces[path_start:],
            key=lambda sample: sample[0],
        )
        active_path_index = -1
        next_path_index = 0
        odom_trace = []
        for index in self._bounded_sample_indices(
            len(odom_samples), MAX_EXECUTION_TRACE_SAMPLES):
            sample = odom_samples[index]
            while (
                next_path_index < len(accepted_paths)
                and accepted_paths[next_path_index][0] <= sample[0]
            ):
                active_path_index = next_path_index
                next_path_index += 1
            projection = None
            if active_path_index >= 0:
                projection = self._project_odom_to_accepted_path(
                    sample, accepted_paths[active_path_index][3])
            odom_trace.append({
                "t_sec": round(sample[0] - start_time, 3),
                "x": round(sample[1], 4),
                "y": round(sample[2], 4),
                "yaw_rad": round(sample[3], 4),
                "accepted_path_sequence": (
                    active_path_index + 1 if active_path_index >= 0 else None),
                "station_m": (
                    round(projection["station_m"], 4)
                    if projection is not None else None),
                "cross_track_m": (
                    round(projection["cross_track_m"], 4)
                    if projection is not None else None),
                "path_heading_error_rad": (
                    round(projection["path_heading_error_rad"], 4)
                    if projection is not None else None),
                "path_segment_index": (
                    projection["segment_index"] if projection is not None else None),
            })
        return {
            "schema_version": EXECUTION_TRACE_SCHEMA_VERSION,
            "sample_limit": MAX_EXECUTION_TRACE_SAMPLES,
            "accepted_path_count": len(accepted_paths),
            "odom_combined": odom_trace,
            "cmd_vel_nav": self._serialize_command_trace(
                controller_command_samples, start_time),
            "cmd_vel_candidate": self._serialize_command_trace(
                candidate_command_samples, start_time),
        }

    def _record_path(self, msg, messages, metrics, traces=None):
        """Record endpoint and geometry from one Nav2 path topic."""
        if not msg.poses:
            return
        path_length = sum(
            math.hypot(
                current.pose.position.x - previous.pose.position.x,
                current.pose.position.y - previous.pose.position.y,
            )
            for previous, current in zip(msg.poses, msg.poses[1:])
        )
        first = msg.poses[0].pose.position
        endpoint_pose = msg.poses[-1].pose
        endpoint = endpoint_pose.position
        orientation = endpoint_pose.orientation
        endpoint_yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        received_at = time.monotonic()
        messages.append((received_at, endpoint.x, endpoint.y, endpoint_yaw))
        chord = math.hypot(endpoint.x - first.x, endpoint.y - first.y)
        metrics.append((received_at, path_length, chord, endpoint.x, endpoint.y))
        if traces is not None:
            traces.append((
                received_at,
                endpoint.x,
                endpoint.y,
                self._compact_path_trace(msg),
            ))

    def _path_cb(self, msg):
        """Record Nav2's native planner output for precise-route evidence."""
        self._record_path(
            msg,
            self._path_messages,
            self._path_metrics,
            self._path_traces,
        )

    def _accepted_path_cb(self, msg):
        """Record only an acknowledged raw FollowPath goal."""
        self._record_path(
            msg,
            self._accepted_path_messages,
            self._accepted_path_metrics,
            self._accepted_path_traces,
        )

    def _accepted_path_summary(self, path_start, target_x, target_y, metrics):
        """Return one correlated longest accepted Nav2 path sample for one endpoint."""
        samples = [
            sample
            for sample in metrics[path_start:]
            if math.hypot(sample[3] - target_x, sample[4] - target_y)
            <= PATH_ENDPOINT_MATCH_TOLERANCE_M
        ]
        if not samples:
            return {
                "planned_path_max_length_m": None,
                "planned_path_max_chord_m": None,
                "planned_path_max_detour_ratio": None,
            }

        candidates = []
        for sample in samples:
            _, length, chord, _, _ = sample
            if not math.isfinite(length) or not math.isfinite(chord):
                continue
            ratio = length / max(chord, 1.0e-3)
            candidates.append((length, chord, ratio))
        if not candidates:
            return {
                "planned_path_max_length_m": None,
                "planned_path_max_chord_m": None,
                "planned_path_max_detour_ratio": None,
            }

        # Keep all reported path metrics from one observed Nav2 path.
        length, chord, ratio = max(candidates, key=lambda value: (value[2], value[0]))
        return {
            "planned_path_max_length_m": length,
            "planned_path_max_chord_m": chord,
            "planned_path_max_detour_ratio": ratio,
        }

    def _required_path(self, parameter_name):
        value = str(self.get_parameter(parameter_name).value).strip()
        path = Path(value)
        if not value or not path.is_file():
            raise ValueError(f"{parameter_name} is not a file: {value}")
        return path.resolve()

    @staticmethod
    def _merge_nav2_documents(base, overlay):
        """Recursively apply a launch-time Nav2 overlay without mutating YAML input."""
        merged = copy.deepcopy(dict(base))
        for key, value in overlay.items():
            current = merged.get(key)
            if isinstance(current, Mapping) and isinstance(value, Mapping):
                merged[key] = AutoTrain._merge_nav2_documents(current, value)
            else:
                merged[key] = copy.deepcopy(value)
        return merged

    @staticmethod
    def _load_nav2_document(path, parameter_name):
        try:
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as error:
            raise ValueError(
                f"cannot read {parameter_name}: {path}: {error}") from error
        if not isinstance(document, Mapping):
            raise ValueError(f"{parameter_name} must contain a YAML mapping: {path}")
        return document

    def _runtime_nav2_params(self):
        """Return exactly the base-plus-overlay configuration Nav2 receives."""
        base_path = self._required_path("nav2_params_file")
        document = self._load_nav2_document(base_path, "nav2_params_file")
        overlay_value = str(
            self.get_parameter("nav2_params_overlay_file").value).strip()
        if not overlay_value:
            return document
        overlay_path = Path(overlay_value)
        if not overlay_path.is_file():
            raise ValueError(
                "nav2_params_overlay_file is not a file: " + overlay_value)
        overlay = self._load_nav2_document(
            overlay_path.resolve(), "nav2_params_overlay_file")
        return self._merge_nav2_documents(document, overlay)

    @staticmethod
    def _heading_mode(goal):
        mode = goal.get("heading_mode")
        if mode not in HEADING_MODES:
            raise ValueError("route goal heading_mode must be free or locked")
        return mode

    @classmethod
    def _uses_reverse_return_tree(cls, waypoints):
        """Return whether a ThroughPoses segment ends at the locked P return."""
        if not waypoints:
            return False
        terminal = waypoints[-1]
        return (
            terminal.get("direction") == "reverse"
            and terminal.get("task") == "return"
            and cls._heading_mode(terminal) == "locked"
        )

    @classmethod
    def _orientation_mapping(cls, pose, heading_mode):
        """Return a locked unit quaternion or the free-heading sentinel."""
        orientation = pose.get("orientation")
        if not isinstance(orientation, dict):
            raise ValueError("route goal is missing its heading mode")
        components = tuple(float(orientation[name]) for name in ("x", "y", "z", "w"))
        norm = math.sqrt(sum(component * component for component in components))
        if not math.isfinite(norm):
            raise ValueError("route goal orientation must be finite")
        if heading_mode == "free" and norm > 1.0e-3:
            raise ValueError(
                "free-heading route goals must use the zero quaternion"
            )
        if heading_mode == "locked" and abs(norm - 1.0) > 1.0e-3:
            raise ValueError(
                "heading-locked route goals must use a unit quaternion"
            )
        return orientation

    @classmethod
    def _route_goal_manifest(cls, goal):
        """Serialize the exact physical target passed to Nav2 for a stage."""
        pose = goal["pose"]
        position = pose["position"]
        heading_mode = cls._heading_mode(goal)
        orientation = cls._orientation_mapping(pose, heading_mode)
        return {
            "id": goal["id"],
            "task": goal["task"],
            "direction": goal["direction"],
            "goal_profile": goal.get("goal_profile", "standard"),
            "heading_mode": heading_mode,
            "frame_id": str(goal.get("frame_id", "odom_combined")),
            "pose": {
                "position": {
                    "x": float(position["x"]),
                    "y": float(position["y"]),
                    "z": float(position.get("z", 0.0)),
                },
                "orientation": {
                    "x": float(orientation["x"]),
                    "y": float(orientation["y"]),
                    "z": float(orientation["z"]),
                    "w": float(orientation["w"]),
                },
            },
        }

    def _load_route(self):
        path = self._required_path("waypoints_file")
        try:
            document, authored_waypoints = load_waypoint_document(path)
            segments = load_planning_segments(document, authored_waypoints)
            ordered_waypoints = materialize_mission_route(
                authored_waypoints,
                segments,
            )
            materialized_route = materialize_free_yaws(ordered_waypoints)
            action_segments = materialize_navigation_segments(
                authored_waypoints,
                segments,
            )
        except PlanningSegmentError as error:
            raise ValueError(f"planning_segments invalid: {error}") from error
        except ValueError as error:
            raise ValueError(f"waypoints_file has no executable route: {error}") from error

        materialized_by_id = {
            waypoint.id: waypoint
            for waypoint in materialized_route
        }
        stages = []
        for index, action_segment in enumerate(action_segments, start=1):
            goals = []
            for source in action_segment:
                materialized = materialized_by_id[source.id]
                x, y, z = materialized.position
                qx, qy, qz, qw = materialized.orientation
                goals.append({
                    "id": materialized.id,
                    "frame_id": materialized.frame_id,
                    "task": materialized.task,
                    "direction": materialized.direction,
                    "goal_profile": materialized.goal_profile,
                    "heading_mode": (
                        "locked"
                        if is_heading_locked(materialized)
                        else "free"
                    ),
                    "pose": {
                        "position": {
                            "x": float(x),
                            "y": float(y),
                            "z": float(z),
                        },
                        "orientation": {
                            "x": qx,
                            "y": qy,
                            "z": qz,
                            "w": qw,
                        },
                    },
                })
            stages.append({
                "id": f"action_{index}_{goals[0]['id']}_to_{goals[-1]['id']}",
                "direction": goals[0]["direction"],
                "goals": goals,
            })
        for stage in stages:
            nonstandard_goals = [
                goal["id"] for goal in stage["goals"]
                if goal.get("goal_profile", "standard") != "standard"
            ]
            if (
                len(stage["goals"]) > 1
                and nonstandard_goals
                and not allows_reverse_handoff_through_poses(stage["goals"])
            ):
                raise ValueError(
                    f"Segment {stage['id']!r} combines nonstandard goals "
                    f"({', '.join(nonstandard_goals)}) in NavigateThroughPoses; "
                    "only a terminal locked reverse_handoff in a reverse "
                    "stage is supported"
                )
        route = [goal for stage in stages for goal in stage["goals"]]

        start_id = str(self.get_parameter("start_goal_id").value).strip()
        end_id = str(self.get_parameter("end_goal_id").value).strip()
        if start_id or end_id:
            filtered_stages = []
            in_range = not start_id  # if no start, include from beginning
            reached_end = False
            for stage in stages:
                filtered_goals = []
                for item in stage["goals"]:
                    waypoint_id = item.get("id")
                    if start_id and waypoint_id == start_id:
                        in_range = True
                    if in_range and not reached_end:
                        filtered_goals.append(item)
                    if end_id and waypoint_id == end_id and in_range:
                        reached_end = True
                if filtered_goals:
                    filtered_stages.append({
                        **stage,
                        "goals": filtered_goals,
                    })
            if not filtered_stages:
                raise ValueError(
                    f"Goal filter {start_id!r}..{end_id!r} matched nothing")
            self.get_logger().info(
                f"Goal filter {start_id!r}..{end_id!r}: "
                f"{len(route)} -> "
                f"{sum(len(stage['goals']) for stage in filtered_stages)} waypoints")
            stages = filtered_stages
            route = [goal for stage in stages for goal in stage["goals"]]
        self.get_logger().info(
            "Route definition: %s" % ", ".join(
                f"{stage['id']}[{stage['direction']}]:"
                + ",".join(goal["id"] for goal in stage["goals"])
                for stage in stages
            )
        )
        return route, stages

    def _behavior_tree_for(self, waypoint):
        if waypoint.get("goal_profile") == "reverse_handoff":
            return self._required_path("reverse_handoff_behavior_tree")
        if waypoint.get("direction") == "reverse":
            return self._required_path("reverse_behavior_tree")
        if waypoint.get("goal_profile") == "precise":
            return self._required_path("precise_behavior_tree")
        return self._required_path("forward_behavior_tree")

    @classmethod
    def _goal_checker_for(cls, waypoint, reverse_return=False):
        if reverse_return:
            return "return_goal_checker"
        if waypoint.get("goal_profile") == "precise":
            return "precise_goal_checker"
        if cls._heading_mode(waypoint) == "free":
            return "transit_goal_checker"
        if waypoint.get("direction") == "reverse":
            return "reverse_goal_checker"
        return "goal_checker"

    def _goal_tolerances(self, waypoint, reverse_return=False):
        document = self._runtime_nav2_params()
        controller = document["controller_server"]["ros__parameters"]
        checker_name = self._goal_checker_for(waypoint, reverse_return)
        checker = controller[checker_name]
        return (
            checker_name,
            float(checker["xy_goal_tolerance"]),
            (
                float(checker["yaw_goal_tolerance"])
                if "yaw_goal_tolerance" in checker
                else None
            ),
        )

    def _reverse_handoff_contract(self):
        document = self._runtime_nav2_params()
        controller = document["controller_server"]["ros__parameters"]
        handoff = controller["ReverseHandoff"]
        smoother = document["velocity_smoother"]["ros__parameters"]
        return {
            "plugin": str(handoff["plugin"]),
            "vx_min": float(handoff["vx_min"]),
            "vx_max": float(handoff["vx_max"]),
            "wz_max": abs(float(handoff["wz_max"])),
            "min_turning_radius": float(
                handoff["AckermannConstraints"]["min_turning_r"]),
            "scale_velocities": bool(smoother["scale_velocities"]),
        }

    def _forward_controller_contract(self, waypoint):
        """Read the final-command envelope selected by this forward goal."""
        document = self._runtime_nav2_params()
        controller = document["controller_server"]["ros__parameters"]
        smoother = document["velocity_smoother"]["ros__parameters"]
        if waypoint.get("goal_profile") == "precise":
            forward = controller["FollowPath"]
            planner = document["planner_server"]["ros__parameters"]["GridBased"]
            return {
                "plugin": str(forward["plugin"]),
                "vx_max": float(forward["desired_linear_vel"]),
                "wz_max": abs(float(smoother["max_velocity"][2])),
                "min_turning_radius": float(planner["minimum_turning_radius"]),
                "path_max_cross_track_error": None,
                "scale_velocities": bool(smoother["scale_velocities"]),
            }

        forward = controller["ForwardAvoidance"]
        plugin = str(forward["plugin"])
        if plugin == FORWARD_AVOIDANCE_CONTROLLER:
            return {
                "plugin": plugin,
                "vx_max": float(forward["desired_linear_vel"]),
                "wz_max": abs(float(forward["forward_max_angular_velocity"])),
                "min_turning_radius": float(
                    forward["forward_min_turning_radius"]),
                "path_max_cross_track_error": float(
                    forward["forward_path_max_cross_track_error"]),
                "scale_velocities": bool(smoother["scale_velocities"]),
            }
        raise ValueError(
            "forward controller must be the configured Ackermann-safe RPP wrapper: "
            + plugin)

    def _wait_for_odom(self, timeout_sec=10.0):
        deadline = time.monotonic() + timeout_sec
        while self._latest_odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._latest_odom is not None

    def _settle_action_endpoints(self):
        # Fast DDS discovers an action's request and response endpoints
        # separately. wait_for_server() only proves the request path exists;
        # a short spin prevents the first goal response from racing the
        # newly-created client's response reader.
        settle_sec = float(
            self.get_parameter("action_endpoint_settle_sec").value)
        if not math.isfinite(settle_sec) or settle_sec < 0.0:
            raise ValueError(
                "action_endpoint_settle_sec must be a nonnegative finite number")
        if settle_sec <= 0.0:
            return

        self.get_logger().info(
            f"Settling Nav2 action endpoints for {settle_sec:.1f}s")
        deadline = time.monotonic() + settle_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(
                self,
                timeout_sec=min(0.1, deadline - time.monotonic()),
            )

    @staticmethod
    def _travel_distance(samples):
        return sum(
            math.hypot(second[1] - first[1], second[2] - first[2])
            for first, second in zip(samples, samples[1:])
        )

    @staticmethod
    def _round_optional(value):
        return round(value, 3) if value is not None else None

    def _executed_travel_metrics(self, start_pose, odom_samples, waypoints):
        """Bound real odometry travel against the stage's intended geometry."""
        unavailable = {
            "executed_travel_m": None,
            "executed_travel_baseline_m": None,
            "executed_travel_detour_ratio": None,
            "executed_travel_limit_m": None,
            "executed_travel_detour_violation": True,
        }
        if start_pose is None or not odom_samples or not waypoints:
            return unavailable

        points = [(float(start_pose[1]), float(start_pose[2]))]
        try:
            points.extend(
                (
                    float(waypoint["pose"]["position"]["x"]),
                    float(waypoint["pose"]["position"]["y"]),
                )
                for waypoint in waypoints
            )
        except (KeyError, TypeError, ValueError):
            return unavailable
        if not all(math.isfinite(value) for point in points for value in point):
            return unavailable

        baseline = sum(
            math.hypot(second[0] - first[0], second[1] - first[1])
            for first, second in zip(points, points[1:])
        )
        observed = [start_pose, *odom_samples]
        travel = self._travel_distance(observed)
        if not math.isfinite(baseline) or not math.isfinite(travel):
            return unavailable
        limit = max(
            baseline * MAX_EXECUTED_TRAVEL_DETOUR_RATIO,
            baseline + MAX_EXECUTED_TRAVEL_DETOUR_ALLOWANCE_M,
        )
        if baseline > 1.0e-3:
            ratio = travel / baseline
        elif travel > MAX_EXECUTED_TRAVEL_DETOUR_ALLOWANCE_M:
            ratio = MAX_EXECUTED_TRAVEL_DETOUR_RATIO + 1.0
        else:
            ratio = 0.0
        return {
            "executed_travel_m": travel,
            "executed_travel_baseline_m": baseline,
            "executed_travel_detour_ratio": ratio,
            "executed_travel_limit_m": limit,
            "executed_travel_detour_violation": (
                travel > limit + CONFIG_TOLERANCE_EPSILON),
        }

    def _serialized_travel_metrics(self, metrics):
        return {
            field: self._round_optional(value)
            if field != "executed_travel_detour_violation" else value
            for field, value in metrics.items()
        }

    def _perception_manifest(self):
        """Snapshot the monitor evidence consumed before sending route goals."""
        status = self._perception_status
        received_at = self._perception_received_at
        now = time.monotonic()
        age = now - received_at if received_at is not None else None
        status = status if isinstance(status, dict) else {}
        checks = status.get("checks")
        return {
            "schema_version": status.get("schema_version"),
            "topic": str(self.get_parameter("perception_ready_topic").value),
            "ready": status.get("ready") is True,
            "checks": checks if isinstance(checks, dict) else {},
            "valid_beams": status.get("valid_beams"),
            "scan_stamp_ns": status.get("scan_stamp_ns"),
            "odom_stamp_ns": status.get("odom_stamp_ns"),
            "tf_odom_position_error_m": status.get(
                "tf_odom_position_error_m"),
            "tf_odom_yaw_error_rad": status.get("tf_odom_yaw_error_rad"),
            "tf_odom_bracket_span_sec": status.get(
                "tf_odom_bracket_span_sec"),
            "landmark_required_ids": status.get("landmark_required_ids"),
            "landmark_matched_ids": status.get("landmark_matched_ids"),
            "landmark_matched_points": status.get(
                "landmark_matched_points"),
            "landmark_max_residual_m": status.get(
                "landmark_max_residual_m"),
            "landmark_current_expected_ids": status.get(
                "landmark_current_expected_ids"),
            "landmark_current_matched_ids": status.get(
                "landmark_current_matched_ids"),
            "landmark_current_matched_points": status.get(
                "landmark_current_matched_points"),
            "landmark_current_max_residual_m": status.get(
                "landmark_current_max_residual_m"),
            "landmark_current_valid": status.get("landmark_current_valid"),
            "local_costmap_stamp_ns": status.get("local_costmap_stamp_ns"),
            "global_costmap_stamp_ns": status.get("global_costmap_stamp_ns"),
            "received_monotonic_sec": received_at,
            "status_age_sec": age,
        }

    def _perception_ready(self):
        evidence = self._perception_manifest()
        failures = []
        if evidence["schema_version"] != PERCEPTION_STATUS_SCHEMA_VERSION:
            failures.append("schema")
        if evidence["ready"] is not True:
            failures.append("ready")
        checks = evidence["checks"]
        for name in PERCEPTION_REQUIRED_CHECKS:
            if checks.get(name) is not True:
                failures.append(name)
        valid_beams = evidence["valid_beams"]
        if (
            isinstance(valid_beams, bool)
            or not isinstance(valid_beams, int)
            or valid_beams <= 0
        ):
            failures.append("valid_beams")
        for name in (
            "scan_stamp_ns",
            "odom_stamp_ns",
            "local_costmap_stamp_ns",
            "global_costmap_stamp_ns",
        ):
            value = evidence[name]
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                failures.append(name)
        max_age = float(self.get_parameter("perception_status_max_age_sec").value)
        age = evidence["status_age_sec"]
        if (
            not math.isfinite(max_age)
            or max_age <= 0.0
            or age is None
            or not math.isfinite(age)
            or age < 0.0
            or age > max_age
        ):
            failures.append("status_age")
        return not failures, failures

    def _wait_for_perception(self):
        timeout = float(self.get_parameter("perception_ready_timeout_sec").value)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError(
                "perception_ready_timeout_sec must be a positive finite number")
        deadline = time.monotonic() + timeout
        last_failures = []
        while time.monotonic() < deadline:
            ready, failures = self._perception_ready()
            if ready:
                evidence = self._perception_manifest()
                self.get_logger().info(
                    "Simulator perception gate passed "
                    f"(valid_beams={evidence['valid_beams']})")
                return True
            last_failures = failures
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().error(
            "Simulator perception gate timed out: "
            + ", ".join(last_failures or ["no status"]))
        return False

    @staticmethod
    def _command_metrics(samples, minimum_turning_radius=None):
        linear_commands = [
            linear
            for _, linear, _ in samples
            if abs(linear) > VELOCITY_EPSILON
        ]
        angular_commands = [
            angular
            for _, _, angular in samples
            if abs(angular) > VELOCITY_EPSILON
        ]
        radius_samples = [
            abs(linear) / abs(angular)
            for _, linear, angular in samples
            if (
                abs(linear) > VELOCITY_EPSILON
                and abs(angular) > VELOCITY_EPSILON
            )
        ]
        kinematic_violations = 0
        if minimum_turning_radius is not None:
            for _, linear, angular in samples:
                if abs(linear) <= VELOCITY_EPSILON:
                    if abs(angular) > VELOCITY_EPSILON:
                        kinematic_violations += 1
                    continue
                angular_limit = (
                    abs(linear) / float(minimum_turning_radius))
                if abs(angular) > angular_limit + VELOCITY_EPSILON:
                    kinematic_violations += 1
        return {
            "linear_min": min(linear_commands) if linear_commands else None,
            "linear_max": max(linear_commands) if linear_commands else None,
            "linear_abs_max": (
                max(abs(value) for value in linear_commands)
                if linear_commands else None
            ),
            "angular_abs_max": (
                max(abs(value) for value in angular_commands)
                if angular_commands else 0.0
            ),
            "positive_count": sum(
                value > VELOCITY_EPSILON for value in linear_commands),
            "negative_count": sum(
                value < -VELOCITY_EPSILON for value in linear_commands),
            "minimum_turning_radius": (
                min(radius_samples) if radius_samples else None
            ),
            "kinematic_violation_count": kinematic_violations,
        }

    def _send_goal(self, waypoint):
        behavior_tree = self._behavior_tree_for(waypoint)
        timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        start_time = time.monotonic()
        odom_start = len(self._odom_samples)
        controller_cmd_start = len(self._controller_cmd_samples)
        cmd_start = len(self._cmd_samples)
        path_start = len(self._path_messages)
        accepted_path_start = len(self._accepted_path_messages)
        start_pose = self._latest_odom

        pose = waypoint["pose"]
        position = pose["position"]
        heading_mode = self._heading_mode(waypoint)
        orientation = self._orientation_mapping(pose, heading_mode)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = waypoint.get(
            "frame_id", "odom_combined")
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(position["x"])
        goal.pose.pose.position.y = float(position["y"])
        goal.pose.pose.position.z = float(position.get("z", 0.0))
        goal.pose.pose.orientation.x = float(orientation["x"])
        goal.pose.pose.orientation.y = float(orientation["y"])
        goal.pose.pose.orientation.z = float(orientation["z"])
        goal.pose.pose.orientation.w = float(orientation["w"])
        goal.behavior_tree = str(behavior_tree)

        self.get_logger().info(
            "Goal %s direction=%s bt=%s"
            % (
                waypoint["id"],
                waypoint.get("direction", "forward"),
                behavior_tree.name,
            )
        )
        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            return self._result(
                waypoint,
                behavior_tree,
                "goal_response_timeout",
                None,
                start_time,
                start_pose,
                odom_start,
                controller_cmd_start,
                cmd_start,
                path_start,
                accepted_path_start,
            )
        try:
            goal_handle = send_future.result()
        except Exception:
            return self._result(
                waypoint,
                behavior_tree,
                "goal_response_error",
                None,
                start_time,
                start_pose,
                odom_start,
                controller_cmd_start,
                cmd_start,
                path_start,
                accepted_path_start,
            )
        if goal_handle is None or not goal_handle.accepted:
            return self._result(
                waypoint,
                behavior_tree,
                "rejected",
                None,
                start_time,
                start_pose,
                odom_start,
                controller_cmd_start,
                cmd_start,
                path_start,
                accepted_path_start,
            )

        result_future = goal_handle.get_result_async()
        next_log = 10.0
        while (
            not result_future.done()
            and time.monotonic() - start_time < timeout_sec
        ):
            rclpy.spin_once(self, timeout_sec=0.25)
            elapsed = time.monotonic() - start_time
            if elapsed >= next_log:
                current = self._latest_odom
                if current is not None:
                    self.get_logger().info(
                        "  %s t=%.0fs pos=(%.2f, %.2f)"
                        % (waypoint["id"], elapsed, current[1], current[2])
                    )
                next_log += 10.0

        if not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(
                self, cancel_future, timeout_sec=5.0)
            outcome = "timeout"
            status = None
        else:
            status = result_future.result().status
            outcome = (
                "succeeded"
                if status == GoalStatus.STATUS_SUCCEEDED
                else "failed"
            )

        return self._result(
            waypoint,
            behavior_tree,
            outcome,
            status,
            start_time,
            start_pose,
            odom_start,
            controller_cmd_start,
            cmd_start,
            path_start,
            accepted_path_start,
        )

    def _result(
        self,
        waypoint,
        behavior_tree,
        outcome,
        status,
        start_time,
        start_pose,
        odom_start,
        controller_cmd_start,
        cmd_start,
        path_start,
        accepted_path_start,
    ):
        current = self._latest_odom or start_pose
        goal_checker, xy_tolerance, yaw_tolerance = self._goal_tolerances(
            waypoint)
        position = waypoint["pose"]["position"]
        heading_mode = self._heading_mode(waypoint)
        orientation = self._orientation_mapping(
            waypoint["pose"], heading_mode)
        target_yaw = None
        if heading_mode == "locked":
            target_yaw = math.atan2(
                2.0 * (
                    float(orientation["w"]) * float(orientation["z"])
                    + float(orientation["x"]) * float(orientation["y"])
                ),
                1.0 - 2.0 * (
                    float(orientation["y"]) ** 2
                    + float(orientation["z"]) ** 2
                ),
            )
        goal_error = None
        goal_yaw_error = None
        signed_goal_yaw_error = None
        final_yaw = None
        if current is not None:
            goal_error = math.hypot(
                current[1] - float(position["x"]),
                current[2] - float(position["y"]),
            )
            final_yaw = current[3]
            if target_yaw is not None:
                signed_goal_yaw_error = math.remainder(
                    final_yaw - target_yaw,
                    2.0 * math.pi,
                )
                goal_yaw_error = abs(signed_goal_yaw_error)
        goal_odom_samples = self._odom_samples[odom_start:]
        xy_tolerance_entry = next((
            sample
            for sample in goal_odom_samples
            if math.hypot(
                sample[1] - float(position["x"]),
                sample[2] - float(position["y"]),
            ) <= xy_tolerance + POSITION_OBSERVER_MARGIN_M
        ), None)
        xy_tolerance_entry_yaw_error = None
        xy_tolerance_entry_elapsed = None
        post_xy_elapsed = None
        post_xy_max_goal_error = None
        post_xy_travel = None
        post_xy_controller_commands = []
        post_xy_commands = []
        if xy_tolerance_entry is not None:
            xy_tolerance_entry_elapsed = xy_tolerance_entry[0] - start_time
            if target_yaw is not None:
                xy_tolerance_entry_yaw_error = abs(math.remainder(
                    xy_tolerance_entry[3] - target_yaw,
                    2.0 * math.pi,
                ))
            post_xy_odom = [
                sample
                for sample in goal_odom_samples
                if sample[0] >= xy_tolerance_entry[0]
            ]
            if post_xy_odom:
                post_xy_elapsed = (
                    post_xy_odom[-1][0] - xy_tolerance_entry[0])
                post_xy_max_goal_error = max(
                    math.hypot(
                        sample[1] - float(position["x"]),
                        sample[2] - float(position["y"]),
                    )
                    for sample in post_xy_odom
                )
                post_xy_travel = self._travel_distance(post_xy_odom)
            post_xy_controller_commands = [
                sample
                for sample
                in self._controller_cmd_samples[controller_cmd_start:]
                if sample[0] >= xy_tolerance_entry[0]
                and abs(sample[1]) > VELOCITY_EPSILON
            ]
            post_xy_commands = [
                sample
                for sample in self._cmd_samples[cmd_start:]
                if sample[0] >= xy_tolerance_entry[0]
                and abs(sample[1]) > VELOCITY_EPSILON
            ]
        post_xy_controller_angular_samples = sum(
            abs(angular) > VELOCITY_EPSILON
            for _, _, angular in post_xy_controller_commands
        )
        post_xy_angular_samples = sum(
            abs(angular) > VELOCITY_EPSILON
            for _, _, angular in post_xy_commands
        )
        post_xy_yaw_error_reduction = (
            xy_tolerance_entry_yaw_error - goal_yaw_error
            if (
                xy_tolerance_entry_yaw_error is not None
                and goal_yaw_error is not None
            ) else None
        )
        direction = waypoint.get("direction", "forward")
        handoff_contract = None
        if waypoint.get("goal_profile") == "reverse_handoff":
            handoff_contract = self._reverse_handoff_contract()
        forward_contract = (
            self._forward_controller_contract(waypoint)
            if direction != "reverse" else None)
        minimum_turning_radius = (
            handoff_contract["min_turning_radius"]
            if handoff_contract is not None
            else (
                forward_contract["min_turning_radius"]
                if forward_contract is not None else None))
        controller_metrics = self._command_metrics(
            self._controller_cmd_samples[controller_cmd_start:],
            minimum_turning_radius,
        )
        candidate_metrics = self._command_metrics(
            self._cmd_samples[cmd_start:],
            minimum_turning_radius,
        )
        positive_command_samples = candidate_metrics["positive_count"]
        negative_command_samples = candidate_metrics["negative_count"]
        target_x = float(position["x"])
        target_y = float(position["y"])
        executed_travel = self._executed_travel_metrics(
            start_pose, goal_odom_samples, [waypoint])
        uses_native_precise_path = waypoint.get("goal_profile") == "precise"
        if uses_native_precise_path:
            evidence_messages = self._path_messages
            evidence_metrics = self._path_metrics
            evidence_traces = self._path_traces
            evidence_start = path_start
        else:
            evidence_messages = self._accepted_path_messages
            evidence_metrics = self._accepted_path_metrics
            evidence_traces = self._accepted_path_traces
            evidence_start = accepted_path_start
        matching_paths = [
            (path_x, path_y, path_yaw)
            for _, path_x, path_y, path_yaw
            in evidence_messages[evidence_start:]
            if math.hypot(path_x - target_x, path_y - target_y)
            <= PATH_ENDPOINT_MATCH_TOLERANCE_M
        ]
        matching_path_traces = [
            trace
            for _, path_x, path_y, trace
            in evidence_traces[evidence_start:]
            if math.hypot(path_x - target_x, path_y - target_y)
            <= PATH_ENDPOINT_MATCH_TOLERANCE_M
        ]
        tracking_trace = self._execution_trace(
            start_time,
            goal_odom_samples,
            self._controller_cmd_samples[controller_cmd_start:],
            self._cmd_samples[cmd_start:],
            evidence_traces,
            evidence_start,
        )
        path_messages = len(matching_paths)
        path_metrics = self._accepted_path_summary(
            evidence_start,
            target_x,
            target_y,
            evidence_metrics,
        )
        planner_candidate_path_messages = sum(
            1
            for _, path_x, path_y, _ in self._path_messages[path_start:]
            if math.hypot(path_x - target_x, path_y - target_y)
            <= PATH_ENDPOINT_MATCH_TOLERANCE_M
        )
        plan_final_yaw = (
            matching_paths[-1][2] if matching_paths else None)
        plan_execution_final_yaw = None
        if plan_final_yaw is not None:
            # ComputeReverseFreeHeadingPathToPose restores the physical yaw
            # before publishing its controller-accepted path. Do not shift
            # that already-restored reverse endpoint a second time.
            plan_execution_final_yaw = plan_final_yaw
        signed_plan_goal_yaw_error = (
            math.remainder(
                plan_execution_final_yaw - target_yaw,
                2.0 * math.pi,
            )
            if plan_execution_final_yaw is not None and target_yaw is not None
            else None
        )
        contract_errors = []
        if outcome == "succeeded" and path_messages <= 0:
            contract_errors.append("path_missing")
        if outcome == "succeeded" and executed_travel[
            "executed_travel_detour_violation"
        ]:
            contract_errors.append("executed_travel_detour")
        if direction == "reverse":
            if outcome == "succeeded" and negative_command_samples <= 0:
                contract_errors.append("reverse_velocity_missing")
            if positive_command_samples > 0:
                contract_errors.append("reverse_velocity_sign")
            if controller_metrics["positive_count"] > 0:
                contract_errors.append("reverse_controller_velocity_sign")
        if outcome == "succeeded" and direction != "reverse":
            if positive_command_samples <= 0:
                contract_errors.append("forward_velocity_missing")
            if negative_command_samples > 0:
                contract_errors.append("forward_velocity_sign")
            if controller_metrics["negative_count"] > 0:
                contract_errors.append("forward_controller_velocity_sign")
            expected_forward_controller = (
                NATIVE_RPP_CONTROLLER
                if uses_native_precise_path else FORWARD_AVOIDANCE_CONTROLLER
            )
            if forward_contract["plugin"] != expected_forward_controller:
                contract_errors.append("forward_controller_plugin")
            if not forward_contract["scale_velocities"]:
                contract_errors.append("forward_smoother_scaling")
            if not uses_native_precise_path and (
                not math.isfinite(forward_contract["path_max_cross_track_error"])
                or forward_contract["path_max_cross_track_error"] <= 0.0
            ):
                contract_errors.append("forward_path_tracking_guard_disabled")
            for prefix, metrics in (
                ("forward_controller", controller_metrics),
                ("forward_candidate", candidate_metrics),
            ):
                if metrics["kinematic_violation_count"] > 0:
                    contract_errors.append(f"{prefix}_curvature")
        if outcome == "succeeded":
            if (
                goal_error is None
                or goal_error > xy_tolerance + POSITION_OBSERVER_MARGIN_M
            ):
                contract_errors.append("goal_position_tolerance")
            if heading_mode == "locked":
                if (
                    goal_yaw_error is None
                    or goal_yaw_error > yaw_tolerance + YAW_OBSERVER_MARGIN_RAD
                ):
                    contract_errors.append("goal_yaw_tolerance")
                if (
                    signed_plan_goal_yaw_error is None
                    or abs(signed_plan_goal_yaw_error)
                    > 0.15 + CONFIG_TOLERANCE_EPSILON
                ):
                    contract_errors.append("plan_goal_yaw")
        if (
            outcome == "succeeded"
            and waypoint.get("goal_profile") == "reverse_handoff"
        ):
            speed_cap = handoff_contract["vx_max"]
            angular_cap = handoff_contract["wz_max"]
            if handoff_contract["plugin"] != REVERSE_HANDOFF_CONTROLLER:
                contract_errors.append("handoff_controller_plugin")
            if (
                handoff_contract["vx_min"] <= 0.0
                or handoff_contract["vx_max"] < handoff_contract["vx_min"]
            ):
                contract_errors.append("handoff_virtual_forward_velocity")
            if not handoff_contract["scale_velocities"]:
                contract_errors.append("handoff_smoother_scaling")
            for prefix, metrics in (
                ("handoff_controller", controller_metrics),
                ("handoff_candidate", candidate_metrics),
            ):
                if (
                    metrics["linear_abs_max"] is None
                    or metrics["linear_abs_max"]
                    > speed_cap + VELOCITY_EPSILON
                ):
                    contract_errors.append(f"{prefix}_speed_limit")
                if metrics["angular_abs_max"] > angular_cap + VELOCITY_EPSILON:
                    contract_errors.append(f"{prefix}_angular_limit")
                if metrics["kinematic_violation_count"] > 0:
                    contract_errors.append(f"{prefix}_curvature")
            if xy_tolerance_entry_yaw_error is None:
                contract_errors.append("handoff_xy_entry_missing")
            elif (
                xy_tolerance_entry_yaw_error
                > yaw_tolerance + YAW_OBSERVER_MARGIN_RAD
            ):
                if not post_xy_controller_commands:
                    contract_errors.append(
                        "handoff_terminal_controller_control_missing")
                if post_xy_controller_angular_samples <= 0:
                    contract_errors.append(
                        "handoff_terminal_controller_steering_missing")
                if not post_xy_commands:
                    contract_errors.append("handoff_terminal_control_missing")
                if post_xy_angular_samples <= 0:
                    contract_errors.append("handoff_terminal_steering_missing")
                if (
                    post_xy_yaw_error_reduction is None
                    or post_xy_yaw_error_reduction
                    <= YAW_IMPROVEMENT_EPSILON
                ):
                    contract_errors.append("handoff_yaw_not_converged")
            if (
                post_xy_max_goal_error is None
                or post_xy_max_goal_error
                > HANDOFF_POST_XY_MAX_POSITION_ERROR_M
            ):
                contract_errors.append("handoff_post_xy_position_excursion")
            if (
                post_xy_travel is None
                or post_xy_travel > HANDOFF_POST_XY_MAX_TRAVEL_M
            ):
                contract_errors.append("handoff_post_xy_travel")
            if (
                post_xy_elapsed is None
                or post_xy_elapsed > HANDOFF_POST_XY_MAX_DURATION_SEC
            ):
                contract_errors.append("handoff_post_xy_duration")
        if contract_errors and outcome == "succeeded":
            outcome = "contract_failed"

        result = {
            "id": waypoint["id"],
            "task": waypoint.get("task", "nav"),
            "direction": direction,
            "goal_profile": waypoint.get("goal_profile", "standard"),
            "heading_mode": heading_mode,
            "goal_checker": goal_checker,
            "behavior_tree": behavior_tree.name,
            "outcome": outcome,
            "status": status,
            "duration_sec": round(time.monotonic() - start_time, 2),
            "goal_error_m": (
                round(goal_error, 3) if goal_error is not None else None
            ),
            "goal_yaw_error_rad": (
                round(goal_yaw_error, 3)
                if goal_yaw_error is not None else None
            ),
            "xy_goal_tolerance_m": round(xy_tolerance, 3),
            "yaw_goal_tolerance_rad": (
                round(yaw_tolerance, 3)
                if yaw_tolerance is not None else None),
            "position_observer_margin_m": POSITION_OBSERVER_MARGIN_M,
            "yaw_observer_margin_rad": YAW_OBSERVER_MARGIN_RAD,
            "xy_tolerance_entry_elapsed_sec": (
                round(xy_tolerance_entry_elapsed, 3)
                if xy_tolerance_entry_elapsed is not None else None
            ),
            "xy_tolerance_entry_yaw_error_rad": (
                round(xy_tolerance_entry_yaw_error, 3)
                if xy_tolerance_entry_yaw_error is not None else None
            ),
            "post_xy_elapsed_sec": (
                round(post_xy_elapsed, 3)
                if post_xy_elapsed is not None else None
            ),
            "post_xy_max_goal_error_m": (
                round(post_xy_max_goal_error, 3)
                if post_xy_max_goal_error is not None else None
            ),
            "post_xy_travel_m": (
                round(post_xy_travel, 3)
                if post_xy_travel is not None else None
            ),
            "post_xy_controller_cmd_sample_count": len(
                post_xy_controller_commands),
            "post_xy_controller_angular_sample_count": (
                post_xy_controller_angular_samples),
            "post_xy_cmd_sample_count": len(post_xy_commands),
            "post_xy_angular_sample_count": post_xy_angular_samples,
            "post_xy_yaw_error_reduction_rad": (
                round(post_xy_yaw_error_reduction, 3)
                if post_xy_yaw_error_reduction is not None else None
            ),
            "target_yaw_rad": (
                round(target_yaw, 3) if target_yaw is not None else None
            ),
            "final_yaw_rad": (
                round(final_yaw, 3) if final_yaw is not None else None
            ),
            "signed_goal_yaw_error_rad": (
                round(signed_goal_yaw_error, 3)
                if signed_goal_yaw_error is not None else None
            ),
            "plan_final_yaw_rad": (
                round(plan_final_yaw, 3)
                if plan_final_yaw is not None else None
            ),
            "plan_execution_final_yaw_rad": (
                round(plan_execution_final_yaw, 3)
                if plan_execution_final_yaw is not None else None
            ),
            "signed_plan_goal_yaw_error_rad": (
                round(signed_plan_goal_yaw_error, 3)
                if signed_plan_goal_yaw_error is not None else None
            ),
            # Keep the legacy travel_m key as a convenient summary, but the
            # contract below records its geometric baseline and a hard limit.
            "travel_m": self._round_optional(
                executed_travel["executed_travel_m"]),
            **self._serialized_travel_metrics(executed_travel),
            # Native P->A records /plan; custom route trees record only their
            # controller-accepted path through the same legacy field.
            "path_messages": path_messages,
            "planner_candidate_path_messages": planner_candidate_path_messages,
            "accepted_path_trace": (
                matching_path_traces[-1] if matching_path_traces else []
            ),
            "tracking_trace": tracking_trace,
            **path_metrics,
            "handoff_speed_cap_mps": (
                round(handoff_contract["vx_max"], 3)
                if handoff_contract is not None else None
            ),
            "handoff_wz_cap_radps": (
                round(handoff_contract["wz_max"], 3)
                if handoff_contract is not None else None
            ),
            "handoff_min_turning_radius_m": (
                round(handoff_contract["min_turning_radius"], 3)
                if handoff_contract is not None else None
            ),
            "handoff_controller_plugin": (
                handoff_contract["plugin"]
                if handoff_contract is not None else None
            ),
            "handoff_internal_vx_min_mps": (
                round(handoff_contract["vx_min"], 3)
                if handoff_contract is not None else None
            ),
            "handoff_internal_vx_max_mps": (
                round(handoff_contract["vx_max"], 3)
                if handoff_contract is not None else None
            ),
            "velocity_smoother_scale_velocities": (
                handoff_contract["scale_velocities"]
                if handoff_contract is not None else None
            ),
            "forward_speed_cap_mps": (
                round(forward_contract["vx_max"], 3)
                if forward_contract is not None else None
            ),
            "forward_wz_cap_radps": (
                round(forward_contract["wz_max"], 3)
                if forward_contract is not None else None
            ),
            "forward_min_turning_radius_m": (
                round(forward_contract["min_turning_radius"], 3)
                if forward_contract is not None else None
            ),
            "forward_path_max_cross_track_error_m": (
                self._round_optional(forward_contract["path_max_cross_track_error"])
                if forward_contract is not None else None
            ),
            "forward_controller_plugin": (
                forward_contract["plugin"]
                if forward_contract is not None else None
            ),
            "forward_velocity_smoother_scale_velocities": (
                forward_contract["scale_velocities"]
                if forward_contract is not None else None
            ),
            "controller_cmd_linear_min": (
                round(controller_metrics["linear_min"], 3)
                if controller_metrics["linear_min"] is not None else None
            ),
            "controller_cmd_linear_max": (
                round(controller_metrics["linear_max"], 3)
                if controller_metrics["linear_max"] is not None else None
            ),
            "controller_cmd_angular_abs_max": round(
                controller_metrics["angular_abs_max"], 3),
            "controller_cmd_min_turning_radius_m": (
                round(controller_metrics["minimum_turning_radius"], 3)
                if controller_metrics["minimum_turning_radius"] is not None
                else None
            ),
            "controller_cmd_kinematic_violation_count": (
                controller_metrics["kinematic_violation_count"]),
            "controller_cmd_positive_sample_count": (
                controller_metrics["positive_count"]),
            "controller_cmd_negative_sample_count": (
                controller_metrics["negative_count"]),
            "cmd_linear_min": (
                round(candidate_metrics["linear_min"], 3)
                if candidate_metrics["linear_min"] is not None else None
            ),
            "cmd_linear_max": (
                round(candidate_metrics["linear_max"], 3)
                if candidate_metrics["linear_max"] is not None else None
            ),
            "cmd_angular_abs_max": round(
                candidate_metrics["angular_abs_max"], 3),
            "cmd_min_turning_radius_m": (
                round(candidate_metrics["minimum_turning_radius"], 3)
                if candidate_metrics["minimum_turning_radius"] is not None
                else None
            ),
            "cmd_kinematic_violation_count": (
                candidate_metrics["kinematic_violation_count"]),
            "cmd_positive_sample_count": positive_command_samples,
            "cmd_negative_sample_count": negative_command_samples,
            "contract_errors": contract_errors,
        }
        self.get_logger().info(
            "Result %s" % json.dumps(result, ensure_ascii=False))
        return result

    @staticmethod
    def _file_manifest(path):
        resolved = path.resolve()
        data = resolved.read_bytes()
        return {
            "path": str(path),
            "realpath": str(resolved),
            "sha256": hashlib.sha256(data).hexdigest(),
        }

    def _input_manifest(self, best_effort=False):
        parameters = (
            "waypoints_file",
            "forward_behavior_tree",
            "precise_behavior_tree",
            "reverse_behavior_tree",
            "reverse_handoff_behavior_tree",
            "nav2_params_file",
            "nav2_params_overlay_file",
            "through_poses_behavior_tree",
            "through_poses_reverse_behavior_tree",
            "through_poses_reverse_locked_behavior_tree",
            "through_poses_reverse_return_behavior_tree",
        )
        manifest = {}
        for name in parameters:
            try:
                manifest[name] = self._file_manifest(
                    self._required_path(name))
            except Exception as error:
                if not best_effort:
                    raise
                manifest[name] = {
                    "path": str(self.get_parameter(name).value),
                    "error": f"{type(error).__name__}: {error}",
                }
        return manifest

    def _route_manifest_for(self, stages):
        """Record the exact edited segment plan that this run executes."""
        return {
            "segments": [
                {
                    "id": stage["id"],
                    "direction": stage["direction"],
                    "goals": [
                        self._route_goal_manifest(goal)
                        for goal in stage["goals"]
                    ],
                }
                for stage in stages
            ]
        }

    def _save_results(self, overall_outcome, error=None):
        path = Path(str(self.get_parameter("results_file").value))
        data = {
            "overall_outcome": overall_outcome,
            "expected_goal_count": self._expected_goal_count,
            "results": self._results,
            "route": self._route_manifest,
            "execution": self._execution_manifest,
            "perception": self._perception_manifest(),
            "inputs": (
                self._input_manifest_cache
                if self._input_manifest_cache is not None
                else self._input_manifest(best_effort=True)
            ),
            "timestamp": time.time(),
        }
        if error:
            data["error"] = error
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
        try:
            temporary.write_text(
                json.dumps(data, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)
        self.get_logger().info(f"Results saved: {path}")

    def _send_through_poses(
        self,
        waypoints,
        stage_id,
        direction,
        behavior_tree_param="through_poses_behavior_tree",
    ):
        """Send multiple waypoints as a single NavigateThroughPoses goal."""
        bt_path = self.get_parameter(behavior_tree_param).value
        if not bt_path or not Path(str(bt_path)).is_file():
            self.get_logger().warn(
                f"{behavior_tree_param} not set, falling back to single-pose")
            return None
        bt = Path(str(bt_path))
        timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        start_time = time.monotonic()
        start_pose = self._latest_odom
        odom_start = len(self._odom_samples)
        controller_cmd_start = len(self._controller_cmd_samples)
        cmd_start = len(self._cmd_samples)
        path_start = len(self._path_messages)
        accepted_path_start = len(self._accepted_path_messages)

        goal = NavigateThroughPoses.Goal()
        for w in waypoints:
            pose = w["pose"]
            ps = PoseStamped()
            ps.header.frame_id = w.get("frame_id", "odom_combined")
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = float(pose["position"]["x"])
            ps.pose.position.y = float(pose["position"]["y"])
            ps.pose.position.z = float(pose["position"].get("z", 0.0))
            orientation = self._orientation_mapping(
                pose, self._heading_mode(w))
            ps.pose.orientation.x = float(orientation["x"])
            ps.pose.orientation.y = float(orientation["y"])
            ps.pose.orientation.z = float(orientation["z"])
            ps.pose.orientation.w = float(orientation["w"])
            goal.poses.append(ps)
        goal.behavior_tree = str(bt)

        ids = ", ".join(w["id"] for w in waypoints)
        self.get_logger().info(
            f"[through_poses] {len(waypoints)} wps: {ids}  bt={bt.name}")

        send_future = self._through_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            self.get_logger().error("[through_poses] goal response timeout")
            return None
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("[through_poses] goal rejected")
            return None

        result_future = goal_handle.get_result_async()
        next_log = start_time + 15.0
        while (not result_future.done()
               and time.monotonic() - start_time < timeout_sec):
            rclpy.spin_once(self, timeout_sec=0.25)
            now = time.monotonic()
            if now >= next_log:
                cur = self._latest_odom
                if cur is not None:
                    elapsed = now - start_time
                    self.get_logger().info(
                        f"  [through_poses] t={elapsed:.0f}s "
                        f"pos=({cur[1]:.2f},{cur[2]:.2f})")
                next_log += 15.0

        if not result_future.done():
            goal_handle.cancel_goal_async()
            self.get_logger().error("[through_poses] timeout")
            return None

        status = result_future.result().status
        success = status == GoalStatus.STATUS_SUCCEEDED
        duration = time.monotonic() - start_time
        cur = self._latest_odom or start_pose

        # Record the same physical evidence as single-pose results.  A
        # NavigateThroughPoses success alone does not prove direction or final
        # pose behavior in a custom reverse tree.
        odom_slice = self._odom_samples[odom_start:]
        terminal = waypoints[-1]
        terminal_reverse_return = self._uses_reverse_return_tree(waypoints)
        terminal_reverse_handoff = allows_reverse_handoff_through_poses(
            waypoints)
        handoff_contract = (
            self._reverse_handoff_contract()
            if terminal_reverse_handoff else None
        )
        forward_contract = (
            self._forward_controller_contract(waypoints[-1])
            if direction != "reverse" else None)
        minimum_turning_radius = (
            handoff_contract["min_turning_radius"]
            if handoff_contract is not None else (
                forward_contract["min_turning_radius"]
                if forward_contract is not None else None))
        controller_metrics = self._command_metrics(
            self._controller_cmd_samples[controller_cmd_start:],
            minimum_turning_radius)
        candidate_metrics = self._command_metrics(
            self._cmd_samples[cmd_start:], minimum_turning_radius)
        terminal_position = terminal["pose"]["position"]
        terminal_x = float(terminal_position["x"])
        terminal_y = float(terminal_position["y"])
        executed_travel = self._executed_travel_metrics(
            start_pose, odom_slice, waypoints)
        terminal_heading_mode = self._heading_mode(terminal)
        terminal_orientation = self._orientation_mapping(
            terminal["pose"], terminal_heading_mode)
        target_yaw = None
        if terminal_heading_mode == "locked":
            target_yaw = math.atan2(
                2.0 * (
                    float(terminal_orientation["w"])
                    * float(terminal_orientation["z"])
                    + float(terminal_orientation["x"])
                    * float(terminal_orientation["y"])
                ),
                1.0 - 2.0 * (
                    float(terminal_orientation["y"]) ** 2
                    + float(terminal_orientation["z"]) ** 2
                ),
            )
        goal_checker, xy_tolerance, yaw_tolerance = self._goal_tolerances(
            terminal, reverse_return=terminal_reverse_return)
        goal_error = None
        goal_yaw_error = None
        signed_goal_yaw_error = None
        final_yaw = None
        if cur is not None:
            goal_error = math.hypot(
                cur[1] - float(terminal_position["x"]),
                cur[2] - float(terminal_position["y"]),
            )
            final_yaw = cur[3]
            if target_yaw is not None:
                signed_goal_yaw_error = math.remainder(
                    final_yaw - target_yaw, 2.0 * math.pi)
                goal_yaw_error = abs(signed_goal_yaw_error)

        # Per-waypoint min-distance check. The free ThroughPoses BT removes
        # a normal guide inside 0.50 m; retain a 0.02 m observer margin.
        passed = []
        for w in waypoints:
            wx = float(w["pose"]["position"]["x"])
            wy = float(w["pose"]["position"]["y"])
            min_d = min(
                (math.hypot(s[1] - wx, s[2] - wy) for s in odom_slice),
                default=None)
            passed.append({
                "id": w["id"],
                "min_distance_m": round(min_d, 3) if min_d is not None else None,
            })

        matching_paths = [
            sample
            for sample in self._accepted_path_messages[accepted_path_start:]
            if math.hypot(sample[1] - terminal_x, sample[2] - terminal_y)
            <= PATH_ENDPOINT_MATCH_TOLERANCE_M
        ]
        matching_path_traces = [
            trace
            for _, path_x, path_y, trace
            in self._accepted_path_traces[accepted_path_start:]
            if math.hypot(path_x - terminal_x, path_y - terminal_y)
            <= PATH_ENDPOINT_MATCH_TOLERANCE_M
        ]
        tracking_trace = self._execution_trace(
            start_time,
            odom_slice,
            self._controller_cmd_samples[controller_cmd_start:],
            self._cmd_samples[cmd_start:],
            self._accepted_path_traces,
            accepted_path_start,
        )
        path_messages = len(matching_paths)
        path_metrics = self._accepted_path_summary(
            accepted_path_start,
            terminal_x,
            terminal_y,
            self._accepted_path_metrics,
        )
        planner_candidate_path_messages = sum(
            1
            for sample in self._path_messages[path_start:]
            if math.hypot(sample[1] - terminal_x, sample[2] - terminal_y)
            <= PATH_ENDPOINT_MATCH_TOLERANCE_M
        )
        contract_errors = []
        if success and path_messages <= 0:
            contract_errors.append("path_missing")
        if success and executed_travel["executed_travel_detour_violation"]:
            contract_errors.append("executed_travel_detour")
        if success:
            if (
                goal_error is None
                or goal_error > xy_tolerance + POSITION_OBSERVER_MARGIN_M
            ):
                contract_errors.append("goal_position_tolerance")
            if terminal_heading_mode == "locked" and (
                goal_yaw_error is None
                or goal_yaw_error > yaw_tolerance + YAW_OBSERVER_MARGIN_RAD
            ):
                contract_errors.append("goal_yaw_tolerance")
            for passed_goal in passed:
                distance = passed_goal["min_distance_m"]
                if (
                    distance is None
                    or distance > THROUGH_POSE_PASS_DISTANCE_TOLERANCE_M
                ):
                    contract_errors.append(
                        f"through_pose_not_passed:{passed_goal['id']}")
            if direction == "reverse":
                for prefix, metrics in (
                    ("reverse_controller", controller_metrics),
                    ("reverse_candidate", candidate_metrics),
                ):
                    if metrics["negative_count"] <= 0:
                        contract_errors.append(f"{prefix}_velocity_missing")
                    if metrics["positive_count"] > 0:
                        contract_errors.append(f"{prefix}_velocity_sign")
                if terminal_reverse_handoff:
                    if handoff_contract["plugin"] != REVERSE_HANDOFF_CONTROLLER:
                        contract_errors.append("handoff_controller_plugin")
                    if (
                        handoff_contract["vx_min"] <= 0.0
                        or handoff_contract["vx_max"] < handoff_contract["vx_min"]
                    ):
                        contract_errors.append("handoff_virtual_forward_velocity")
                    if not handoff_contract["scale_velocities"]:
                        contract_errors.append("handoff_smoother_scaling")
                    for prefix, metrics in (
                        ("handoff_controller", controller_metrics),
                        ("handoff_candidate", candidate_metrics),
                    ):
                        if (
                            metrics["linear_abs_max"] is None
                            or metrics["linear_abs_max"]
                            > handoff_contract["vx_max"] + VELOCITY_EPSILON
                        ):
                            contract_errors.append(f"{prefix}_speed_limit")
                        if (
                            metrics["angular_abs_max"]
                            > handoff_contract["wz_max"] + VELOCITY_EPSILON
                        ):
                            contract_errors.append(f"{prefix}_angular_limit")
                        if metrics["kinematic_violation_count"] > 0:
                            contract_errors.append(f"{prefix}_curvature")
            else:
                for prefix, metrics in (
                    ("forward_controller", controller_metrics),
                    ("forward_candidate", candidate_metrics),
                ):
                    if metrics["positive_count"] <= 0:
                        contract_errors.append(f"{prefix}_velocity_missing")
                    if metrics["negative_count"] > 0:
                        contract_errors.append(f"{prefix}_velocity_sign")
                if forward_contract["plugin"] != FORWARD_AVOIDANCE_CONTROLLER:
                    contract_errors.append("forward_controller_plugin")
                if not forward_contract["scale_velocities"]:
                    contract_errors.append("forward_smoother_scaling")
                if (
                    not math.isfinite(forward_contract["path_max_cross_track_error"])
                    or forward_contract["path_max_cross_track_error"] <= 0.0
                ):
                    contract_errors.append("forward_path_tracking_guard_disabled")
                for prefix, metrics in (
                    ("forward_controller", controller_metrics),
                    ("forward_candidate", candidate_metrics),
                ):
                    if metrics["kinematic_violation_count"] > 0:
                        contract_errors.append(f"{prefix}_curvature")
        outcome = "succeeded" if success else "failed"
        if contract_errors and success:
            outcome = "contract_failed"

        result = {
            "id": f"through_poses[{ids}]",
            "mode": "through_poses",
            "segment_id": stage_id,
            "direction": direction,
            "heading_mode": terminal_heading_mode,
            "goal_ids": [waypoint["id"] for waypoint in waypoints],
            "goal_profiles": [
                waypoint.get("goal_profile", "standard")
                for waypoint in waypoints
            ],
            "behavior_tree": bt.name,
            "waypoint_count": len(waypoints),
            "outcome": outcome,
            "status": int(status),
            "duration_sec": round(duration, 2),
            "travel_m": self._round_optional(
                executed_travel["executed_travel_m"]),
            **self._serialized_travel_metrics(executed_travel),
            "path_messages": path_messages,
            "planner_candidate_path_messages": planner_candidate_path_messages,
            "accepted_path_trace": (
                matching_path_traces[-1] if matching_path_traces else []
            ),
            "tracking_trace": tracking_trace,
            **path_metrics,
            "handoff_speed_cap_mps": (
                round(handoff_contract["vx_max"], 3)
                if handoff_contract is not None else None
            ),
            "handoff_wz_cap_radps": (
                round(handoff_contract["wz_max"], 3)
                if handoff_contract is not None else None
            ),
            "handoff_min_turning_radius_m": (
                round(handoff_contract["min_turning_radius"], 3)
                if handoff_contract is not None else None
            ),
            "handoff_controller_plugin": (
                handoff_contract["plugin"]
                if handoff_contract is not None else None
            ),
            "handoff_internal_vx_min_mps": (
                round(handoff_contract["vx_min"], 3)
                if handoff_contract is not None else None
            ),
            "handoff_internal_vx_max_mps": (
                round(handoff_contract["vx_max"], 3)
                if handoff_contract is not None else None
            ),
            "velocity_smoother_scale_velocities": (
                handoff_contract["scale_velocities"]
                if handoff_contract is not None else None
            ),
            "final_pos": (round(cur[1], 3), round(cur[2], 3)),
            "goal_checker": goal_checker,
            "xy_goal_tolerance_m": round(xy_tolerance, 3),
            "yaw_goal_tolerance_rad": (
                round(yaw_tolerance, 3)
                if yaw_tolerance is not None else None),
            "position_observer_margin_m": POSITION_OBSERVER_MARGIN_M,
            "yaw_observer_margin_rad": YAW_OBSERVER_MARGIN_RAD,
            "target_yaw_rad": (
                round(target_yaw, 3) if target_yaw is not None else None
            ),
            "final_yaw_rad": (
                round(final_yaw, 3) if final_yaw is not None else None),
            "goal_error_m": (
                round(goal_error, 3) if goal_error is not None else None),
            "goal_yaw_error_rad": (
                round(goal_yaw_error, 3) if goal_yaw_error is not None else None),
            "signed_goal_yaw_error_rad": (
                round(signed_goal_yaw_error, 3)
                if signed_goal_yaw_error is not None else None),
            "controller_cmd_linear_min": (
                round(controller_metrics["linear_min"], 3)
                if controller_metrics["linear_min"] is not None else None),
            "controller_cmd_linear_max": (
                round(controller_metrics["linear_max"], 3)
                if controller_metrics["linear_max"] is not None else None),
            "controller_cmd_angular_abs_max": round(
                controller_metrics["angular_abs_max"], 3),
            "controller_cmd_min_turning_radius_m": (
                round(controller_metrics["minimum_turning_radius"], 3)
                if controller_metrics["minimum_turning_radius"] is not None
                else None),
            "controller_cmd_kinematic_violation_count": (
                controller_metrics["kinematic_violation_count"]),
            "controller_cmd_positive_sample_count": (
                controller_metrics["positive_count"]),
            "controller_cmd_negative_sample_count": (
                controller_metrics["negative_count"]),
            "cmd_linear_min": (
                round(candidate_metrics["linear_min"], 3)
                if candidate_metrics["linear_min"] is not None else None),
            "cmd_linear_max": (
                round(candidate_metrics["linear_max"], 3)
                if candidate_metrics["linear_max"] is not None else None),
            "cmd_angular_abs_max": round(
                candidate_metrics["angular_abs_max"], 3),
            "cmd_min_turning_radius_m": (
                round(candidate_metrics["minimum_turning_radius"], 3)
                if candidate_metrics["minimum_turning_radius"] is not None
                else None),
            "cmd_kinematic_violation_count": (
                candidate_metrics["kinematic_violation_count"]),
            "cmd_positive_sample_count": candidate_metrics["positive_count"],
            "cmd_negative_sample_count": candidate_metrics["negative_count"],
            "forward_speed_cap_mps": (
                round(forward_contract["vx_max"], 3)
                if forward_contract is not None else None
            ),
            "forward_wz_cap_radps": (
                round(forward_contract["wz_max"], 3)
                if forward_contract is not None else None
            ),
            "forward_min_turning_radius_m": (
                round(forward_contract["min_turning_radius"], 3)
                if forward_contract is not None else None
            ),
            "forward_path_max_cross_track_error_m": (
                round(forward_contract["path_max_cross_track_error"], 3)
                if forward_contract is not None else None
            ),
            "forward_controller_plugin": (
                forward_contract["plugin"]
                if forward_contract is not None else None
            ),
            "forward_velocity_smoother_scale_velocities": (
                forward_contract["scale_velocities"]
                if forward_contract is not None else None
            ),
            "waypoints_passed": passed,
            "contract_errors": contract_errors,
        }
        self.get_logger().info(
            f"[through_poses] {'OK' if success else 'FAIL'} "
            f"dur={duration:.1f}s")
        for wp in passed:
            self.get_logger().info(f"  {wp['id']}: min_dist={wp['min_distance_m']}m")
        return result

    def _run_stage(self, stage, use_through_poses):
        """Execute one user-defined segment, preserving its direction and goals."""
        goals = stage["goals"]
        if not goals:
            return True
        direction = stage["direction"]
        if use_through_poses and len(goals) > 1:
            if direction == "reverse":
                behavior_tree_param = (
                    "through_poses_reverse_return_behavior_tree"
                    if self._uses_reverse_return_tree(goals)
                    else (
                        "through_poses_reverse_locked_behavior_tree"
                        if self._heading_mode(goals[-1]) == "locked"
                        else "through_poses_reverse_behavior_tree"
                    )
                )
            else:
                behavior_tree_param = "through_poses_behavior_tree"
            if self._through_client.wait_for_server(timeout_sec=10.0):
                result = self._send_through_poses(
                    goals,
                    stage["id"],
                    direction,
                    behavior_tree_param,
                )
                if result is None:
                    self._save_results(
                        "failed", f"{stage['id']}_through_poses_failed"
                    )
                    return False
                self._results.append(result)
                return result["outcome"] == "succeeded"
            self.get_logger().error(
                "/navigate_through_poses unavailable; refusing to replace "
                "a configured multi-goal segment with single-pose navigation"
            )
            self._save_results(
                "failed", f"{stage['id']}_through_poses_unavailable"
            )
            return False

        delay = float(self.get_parameter("inter_goal_delay_sec").value)
        for waypoint in goals:
            result = self._send_goal(waypoint)
            self._results.append(result)
            if result["outcome"] != "succeeded":
                return False
            if delay > 0.0:
                deadline = time.monotonic() + delay
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def run(self):
        self._input_manifest_cache = self._input_manifest()
        route, stages = self._load_route()
        self._expected_goal_count = len(route)
        self._route_manifest = self._route_manifest_for(stages)
        if not self._action_client.wait_for_server(timeout_sec=90.0):
            raise RuntimeError("navigate_to_pose action server unavailable")
        self._settle_action_endpoints()
        if not self._wait_for_odom():
            raise RuntimeError("odom_combined unavailable")

        use_through_poses = bool(self.get_parameter("use_through_poses").value)
        self._execution_manifest = {
            "use_through_poses": use_through_poses,
        }
        # Do not send even the first Nav2 goal until a live scan is proven to
        # transform at its own stamp and mark both raw costmaps, including the
        # movable A-zone cone probe. This makes a Gazebo/RViz display mismatch
        # a route failure instead of an invisible planning condition.
        if not self._wait_for_perception():
            self._save_results("failed", "sim_perception_not_ready")
            return False
        self.get_logger().info(
            "Executing %d configured segments (%d goals, through_poses=%s)"
            % (len(stages), len(route), use_through_poses)
        )
        for index, stage in enumerate(stages, start=1):
            self.get_logger().info(
                "Segment %d/%d %s [%s]: %s"
                % (
                    index,
                    len(stages),
                    stage["id"],
                    stage["direction"],
                    ", ".join(goal["id"] for goal in stage["goals"]),
                )
            )
            if not self._run_stage(stage, use_through_poses):
                self._save_results("failed")
                return False

        self._save_results("completed")
        return True


def main():
    rclpy.init()
    node = AutoTrain()
    exit_code = 0
    try:
        if not node.run():
            exit_code = 1
    except Exception as error:
        message = f"{type(error).__name__}: {error}"
        node.get_logger().error(traceback.format_exc())
        node._save_results("error", message)
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
