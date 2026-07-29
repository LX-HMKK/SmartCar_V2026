#!/usr/bin/env python3
"""Run the complete navigation-only route inside the Gazebo simulation."""

import json
import hashlib
import math
import os
import time
import traceback
from pathlib import Path
from types import SimpleNamespace

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
from smartcar_tools.planning_segments import PlanningSegmentError, load_planning_segments


VELOCITY_EPSILON = 1.0e-3
CONFIG_TOLERANCE_EPSILON = 2.0e-3
POSITION_OBSERVER_MARGIN_M = 2.0e-2
YAW_OBSERVER_MARGIN_RAD = 1.0e-2
YAW_IMPROVEMENT_EPSILON = 1.0e-2
HANDOFF_POST_XY_MAX_POSITION_ERROR_M = 0.75
HANDOFF_POST_XY_MAX_TRAVEL_M = 1.00
HANDOFF_POST_XY_MAX_DURATION_SEC = 25.0
REVERSE_HANDOFF_CONTROLLER = "smartcar_nav2::ReverseOnlyMPPIController"


class AutoTrain(Node):
    def __init__(self):
        super().__init__("auto_train")

        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("forward_behavior_tree", "")
        self.declare_parameter("precise_behavior_tree", "")
        self.declare_parameter("reverse_behavior_tree", "")
        self.declare_parameter("reverse_handoff_behavior_tree", "")
        self.declare_parameter("nav2_params_file", "")
        self.declare_parameter("through_poses_behavior_tree", "")
        self.declare_parameter("through_poses_reverse_behavior_tree", "")
        self.declare_parameter("use_through_poses", True)
        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("inter_goal_delay_sec", 1.0)
        self.declare_parameter("action_endpoint_settle_sec", 2.0)
        self.declare_parameter("start_goal_id", "")
        self.declare_parameter("end_goal_id", "")
        self.declare_parameter(
            "results_file", "/tmp/auto_train_results.json")

        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        path_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Odometry, "/odom_combined", self._odom_cb, qos)
        self.create_subscription(
            Twist, "/cmd_vel_nav", self._controller_cmd_cb, qos)
        self.create_subscription(
            Twist, "/cmd_vel_candidate", self._cmd_cb, qos)
        self.create_subscription(NavPath, "/plan", self._path_cb, path_qos)
        self._action_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose")
        self._through_client = ActionClient(
            self, NavigateThroughPoses, "/navigate_through_poses")

        self._latest_odom = None
        self._odom_samples = []
        self._controller_cmd_samples = []
        self._cmd_samples = []
        self._path_messages = []
        self._results = []
        self._expected_goal_count = 0
        self._input_manifest_cache = None
        self._route_manifest = None

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

    def _path_cb(self, msg):
        if msg.poses:
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
            self._path_messages.append(
                (time.monotonic(), endpoint.x, endpoint.y, endpoint_yaw))

    def _required_path(self, parameter_name):
        value = str(self.get_parameter(parameter_name).value).strip()
        path = Path(value)
        if not value or not path.is_file():
            raise ValueError(f"{parameter_name} is not a file: {value}")
        return path

    @staticmethod
    def _orientation_mapping(pose):
        """Honor omitted orientation for pass-through points as Nav2 zero yaw."""
        orientation = pose.get("orientation")
        if not isinstance(orientation, dict):
            return {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}
        return orientation

    def _load_route(self):
        path = self._required_path("waypoints_file")
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        waypoints = document.get("waypoints", [])
        if not isinstance(waypoints, list) or len(waypoints) < 2:
            raise ValueError("waypoints_file has no navigation route")
        if not all(isinstance(item, dict) for item in waypoints):
            raise ValueError("waypoints_file entries must be mappings")
        proxy_waypoints = [
            SimpleNamespace(
                id=item.get("id"),
                task=item.get("task"),
                direction=item.get("direction", "forward"),
            )
            for item in waypoints
        ]
        try:
            segments = load_planning_segments(document, proxy_waypoints)
        except PlanningSegmentError as error:
            raise ValueError(f"planning_segments invalid: {error}") from error

        waypoint_by_id = {item["id"]: item for item in waypoints}
        stages = []
        for segment in segments:
            goals = []
            for waypoint_id in (*segment.through_ids, segment.end_id):
                source = waypoint_by_id[waypoint_id]
                goal = dict(source)
                goal["direction"] = segment.direction
                goals.append(goal)
            stages.append({
                "id": segment.id,
                "direction": segment.direction,
                "goals": goals,
            })
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

    @staticmethod
    def _goal_checker_for(waypoint):
        if waypoint.get("goal_profile") == "precise":
            return "precise_goal_checker"
        if waypoint.get("direction") == "reverse":
            return "reverse_goal_checker"
        return "goal_checker"

    def _goal_tolerances(self, waypoint):
        params_path = self._required_path("nav2_params_file")
        document = yaml.safe_load(params_path.read_text(encoding="utf-8"))
        controller = document["controller_server"]["ros__parameters"]
        checker_name = self._goal_checker_for(waypoint)
        checker = controller[checker_name]
        return (
            checker_name,
            float(checker["xy_goal_tolerance"]),
            float(checker["yaw_goal_tolerance"]),
        )

    def _reverse_handoff_contract(self):
        params_path = self._required_path("nav2_params_file")
        document = yaml.safe_load(params_path.read_text(encoding="utf-8"))
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
        start_pose = self._latest_odom

        pose = waypoint["pose"]
        position = pose["position"]
        orientation = self._orientation_mapping(pose)
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
    ):
        current = self._latest_odom or start_pose
        goal_checker, xy_tolerance, yaw_tolerance = self._goal_tolerances(
            waypoint)
        position = waypoint["pose"]["position"]
        orientation = self._orientation_mapping(waypoint["pose"])
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
        handoff_contract = None
        if waypoint.get("goal_profile") == "reverse_handoff":
            handoff_contract = self._reverse_handoff_contract()
        minimum_turning_radius = (
            handoff_contract["min_turning_radius"]
            if handoff_contract is not None else None)
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
        direction = waypoint.get("direction", "forward")
        matching_paths = [
            (path_x, path_y, path_yaw)
            for _, path_x, path_y, path_yaw
            in self._path_messages[path_start:]
            if math.hypot(path_x - target_x, path_y - target_y) <= 0.35
        ]
        path_messages = len(matching_paths)
        plan_final_yaw = (
            matching_paths[-1][2] if matching_paths else None)
        plan_execution_final_yaw = None
        if plan_final_yaw is not None:
            plan_execution_final_yaw = math.remainder(
                plan_final_yaw + (math.pi if direction == "reverse" else 0.0),
                2.0 * math.pi,
            )
        signed_plan_goal_yaw_error = (
            math.remainder(
                plan_execution_final_yaw - target_yaw,
                2.0 * math.pi,
            )
            if plan_execution_final_yaw is not None else None
        )
        contract_errors = []
        if outcome == "succeeded" and path_messages <= 0:
            contract_errors.append("path_missing")
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
        if outcome == "succeeded":
            if (
                goal_error is None
                or goal_error > xy_tolerance + POSITION_OBSERVER_MARGIN_M
            ):
                contract_errors.append("goal_position_tolerance")
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
            "yaw_goal_tolerance_rad": round(yaw_tolerance, 3),
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
            "target_yaw_rad": round(target_yaw, 3),
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
            "travel_m": round(
                self._travel_distance(self._odom_samples[odom_start:]), 3
            ),
            "path_messages": path_messages,
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
        data = path.read_bytes()
        return {
            "path": str(path),
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
            "through_poses_behavior_tree",
            "through_poses_reverse_behavior_tree",
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

    @staticmethod
    def _route_manifest_for(stages):
        """Record the exact edited segment plan that this run executes."""
        return {
            "segments": [
                {
                    "id": stage["id"],
                    "direction": stage["direction"],
                    "goals": [
                        {
                            "id": goal["id"],
                            "direction": goal["direction"],
                            "goal_profile": goal.get("goal_profile", "standard"),
                        }
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
        path_start = len(self._path_messages)

        goal = NavigateThroughPoses.Goal()
        for w in waypoints:
            pose = w["pose"]
            ps = PoseStamped()
            ps.header.frame_id = w.get("frame_id", "odom_combined")
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = float(pose["position"]["x"])
            ps.pose.position.y = float(pose["position"]["y"])
            ps.pose.position.z = float(pose["position"].get("z", 0.0))
            orientation = self._orientation_mapping(pose)
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

        # Per-waypoint min-dist check
        passed = []
        odom_slice = self._odom_samples[
            next(i for i, s in enumerate(self._odom_samples)
                 if s[0] >= start_time):]
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

        result = {
            "id": f"through_poses[{ids}]",
            "mode": "through_poses",
            "segment_id": stage_id,
            "direction": direction,
            "goal_ids": [waypoint["id"] for waypoint in waypoints],
            "goal_profiles": [
                waypoint.get("goal_profile", "standard")
                for waypoint in waypoints
            ],
            "behavior_tree": bt.name,
            "waypoint_count": len(waypoints),
            "outcome": "succeeded" if success else "failed",
            "status": int(status),
            "duration_sec": round(duration, 2),
            "travel_m": round(self._travel_distance(odom_slice), 3),
            "path_messages": len(self._path_messages[path_start:]),
            "final_pos": (round(cur[1], 3), round(cur[2], 3)),
            "waypoints_passed": passed,
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
            behavior_tree_param = (
                "through_poses_reverse_behavior_tree"
                if direction == "reverse"
                else "through_poses_behavior_tree"
            )
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
            self.get_logger().warning(
                "/navigate_through_poses unavailable; falling back to single goals"
            )

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
