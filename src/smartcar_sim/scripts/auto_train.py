#!/usr/bin/env python3
"""Run the complete navigation-only route inside the Gazebo simulation."""

import json
import hashlib
import math
import os
import time
import traceback
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


EXPECTED_ROUTE = (
    ("a_task_observe", "forward", "precise"),
    ("b_corridor_enter", "reverse", "standard"),
    ("b_corridor_out", "reverse", "standard"),
    ("c_corner_1", "reverse", "reverse_handoff"),
    ("c_corner_2", "forward", "standard"),
    ("c_corner_3", "forward", "standard"),
    ("c_corner_4", "forward", "standard"),
    ("b_corridor_return_enter", "forward", "standard"),
    ("b_corridor_return", "forward", "standard"),
    ("p_finish", "forward", "standard"),
)


class AutoTrain(Node):
    def __init__(self):
        super().__init__("auto_train")

        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("forward_behavior_tree", "")
        self.declare_parameter("precise_behavior_tree", "")
        self.declare_parameter("reverse_behavior_tree", "")
        self.declare_parameter("reverse_handoff_behavior_tree", "")
        self.declare_parameter("nav2_params_file", "")
        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("inter_goal_delay_sec", 1.0)
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
            Twist, "/cmd_vel_candidate", self._cmd_cb, qos)
        self.create_subscription(NavPath, "/plan", self._path_cb, path_qos)
        self._action_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose")

        self._latest_odom = None
        self._odom_samples = []
        self._cmd_samples = []
        self._path_messages = []
        self._results = []
        self._expected_goal_count = 0
        self._input_manifest_cache = None

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

    def _load_route(self):
        path = self._required_path("waypoints_file")
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        waypoints = document.get("waypoints", [])
        if not isinstance(waypoints, list) or len(waypoints) < 2:
            raise ValueError("waypoints_file has no navigation route")
        route = [item for item in waypoints if item.get("task") != "start"]
        actual = []
        for item in route:
            profile = item.get("goal_profile", "standard")
            actual.append((
                item.get("id"), item.get("direction", "forward"), profile))
        if tuple(actual) != EXPECTED_ROUTE:
            raise ValueError(
                f"waypoints_file route contract mismatch: {actual!r}")
        return route

    def _behavior_tree_for(self, waypoint):
        if waypoint.get("goal_profile") == "reverse_handoff":
            return self._required_path("reverse_handoff_behavior_tree")
        if waypoint.get("direction") == "reverse":
            return self._required_path("reverse_behavior_tree")
        if waypoint.get("goal_profile") == "precise":
            return self._required_path("precise_behavior_tree")
        return self._required_path("forward_behavior_tree")

    def _wait_for_odom(self, timeout_sec=10.0):
        deadline = time.monotonic() + timeout_sec
        while self._latest_odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._latest_odom is not None

    @staticmethod
    def _travel_distance(samples):
        return sum(
            math.hypot(second[1] - first[1], second[2] - first[2])
            for first, second in zip(samples, samples[1:])
        )

    def _send_goal(self, waypoint):
        behavior_tree = self._behavior_tree_for(waypoint)
        timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        start_time = time.monotonic()
        odom_start = len(self._odom_samples)
        cmd_start = len(self._cmd_samples)
        path_start = len(self._path_messages)
        start_pose = self._latest_odom

        pose = waypoint["pose"]
        position = pose["position"]
        orientation = pose["orientation"]
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
        cmd_start,
        path_start,
    ):
        current = self._latest_odom or start_pose
        position = waypoint["pose"]["position"]
        orientation = waypoint["pose"]["orientation"]
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
        commands = [
            linear
            for _, linear, _ in self._cmd_samples[cmd_start:]
            if abs(linear) > 1.0e-3
        ]
        positive_command_samples = sum(
            linear > 1.0e-3 for linear in commands)
        negative_command_samples = sum(
            linear < -1.0e-3 for linear in commands)
        target_x = float(position["x"])
        target_y = float(position["y"])
        matching_paths = [
            (path_x, path_y, path_yaw)
            for _, path_x, path_y, path_yaw
            in self._path_messages[path_start:]
            if math.hypot(path_x - target_x, path_y - target_y) <= 0.35
        ]
        path_messages = len(matching_paths)
        plan_final_yaw = (
            matching_paths[-1][2] if matching_paths else None)
        signed_plan_goal_yaw_error = (
            math.remainder(plan_final_yaw - target_yaw, 2.0 * math.pi)
            if plan_final_yaw is not None else None
        )
        contract_errors = []
        direction = waypoint.get("direction", "forward")
        if outcome == "succeeded" and path_messages <= 0:
            contract_errors.append("path_missing")
        if direction == "reverse":
            if outcome == "succeeded" and negative_command_samples <= 0:
                contract_errors.append("reverse_velocity_missing")
            if positive_command_samples > 0:
                contract_errors.append("reverse_velocity_sign")
        if outcome == "succeeded" and direction != "reverse":
            if positive_command_samples <= 0:
                contract_errors.append("forward_velocity_missing")
            if negative_command_samples > 0:
                contract_errors.append("forward_velocity_sign")
        if contract_errors and outcome == "succeeded":
            outcome = "contract_failed"

        result = {
            "id": waypoint["id"],
            "task": waypoint.get("task", "nav"),
            "direction": direction,
            "goal_profile": waypoint.get("goal_profile", "standard"),
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
            "signed_plan_goal_yaw_error_rad": (
                round(signed_plan_goal_yaw_error, 3)
                if signed_plan_goal_yaw_error is not None else None
            ),
            "travel_m": round(
                self._travel_distance(self._odom_samples[odom_start:]), 3
            ),
            "path_messages": path_messages,
            "cmd_linear_min": round(min(commands), 3) if commands else None,
            "cmd_linear_max": round(max(commands), 3) if commands else None,
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

    def _save_results(self, overall_outcome, error=None):
        path = Path(str(self.get_parameter("results_file").value))
        data = {
            "overall_outcome": overall_outcome,
            "expected_goal_count": self._expected_goal_count,
            "results": self._results,
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

    def run(self):
        self._input_manifest_cache = self._input_manifest()
        route = self._load_route()
        self._expected_goal_count = len(route)
        if not self._action_client.wait_for_server(timeout_sec=90.0):
            raise RuntimeError("navigate_to_pose action server unavailable")
        if not self._wait_for_odom():
            raise RuntimeError("odom_combined unavailable")

        self.get_logger().info(
            f"Starting complete navigation route ({len(route)} goals)")
        delay = float(self.get_parameter("inter_goal_delay_sec").value)
        for waypoint in route:
            result = self._send_goal(waypoint)
            self._results.append(result)
            if result["outcome"] != "succeeded":
                self._save_results("failed")
                return False
            if delay > 0.0:
                deadline = time.monotonic() + delay
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.1)

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
