#!/usr/bin/env python3
"""Execute the current forward-only Nav2 route in local Gazebo.

This runner sends only native ``NavigateToPose`` and
``NavigateThroughPoses`` actions.  It records the trees selected for the
configured semantic route and the observed command range, but never builds or
modifies a path itself.
"""

import hashlib
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from smartcar_task.planning_segments import (
    PlanningSegmentError,
    allows_precise_terminal_through_poses,
    load_planning_segments,
    materialize_mission_route,
    materialize_navigation_segments,
)
from smartcar_task.route_geometry import materialize_free_yaws
from smartcar_task.waypoints import is_heading_locked, load_waypoint_document
from std_msgs.msg import String


PERCEPTION_READY_TOPIC = "/smartcar/sim_perception_ready"
PERCEPTION_STATUS_SCHEMA_VERSION = 2
RESULT_SCHEMA_VERSION = 2
VELOCITY_EPSILON = 1.0e-3

TREE_PARAMETERS = (
    "forward_behavior_tree",
    "transit_behavior_tree",
    "precise_behavior_tree",
    "through_poses_behavior_tree",
    "through_poses_transit_behavior_tree",
    "through_poses_precise_behavior_tree",
    "through_poses_return_behavior_tree",
)


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _yaw_from_quaternion(orientation):
    return math.atan2(
        2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


class AutoTrain(Node):
    def __init__(self):
        super().__init__("auto_train")
        self.declare_parameter("waypoints_file", "")
        for name in TREE_PARAMETERS:
            self.declare_parameter(name, "")
        self.declare_parameter("use_through_poses", True)
        self.declare_parameter("goal_timeout_sec", 180.0)
        self.declare_parameter("results_file", "/tmp/auto_train_results.json")
        self.declare_parameter("perception_ready_topic", PERCEPTION_READY_TOPIC)
        self.declare_parameter("perception_ready_timeout_sec", 60.0)

        command_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        odom_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        perception_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Odometry, "/odom_combined", self._on_odom, odom_qos)
        self.create_subscription(Twist, "/cmd_vel_nav", self._on_controller_command, command_qos)
        self.create_subscription(Twist, "/cmd_vel_candidate", self._on_candidate_command, command_qos)
        self.create_subscription(NavPath, "/plan", self._on_plan, 10)
        self.create_subscription(
            String,
            str(self.get_parameter("perception_ready_topic").value),
            self._on_perception,
            perception_qos,
        )
        self._pose = None
        self._controller_commands = []
        self._candidate_commands = []
        self._plan_count = 0
        self._perception = None
        self._perception_received_at = None
        self._results = []
        self._route_manifest = None
        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._through_client = ActionClient(
            self, NavigateThroughPoses, "/navigate_through_poses")

    def _on_odom(self, message):
        pose = message.pose.pose
        self._pose = {
            "x": round(float(pose.position.x), 4),
            "y": round(float(pose.position.y), 4),
            "yaw_rad": round(_yaw_from_quaternion(pose.orientation), 4),
        }

    def _on_controller_command(self, message):
        self._controller_commands.append((
            time.monotonic(), float(message.linear.x), float(message.angular.z)))

    def _on_candidate_command(self, message):
        self._candidate_commands.append((
            time.monotonic(), float(message.linear.x), float(message.angular.z)))

    def _on_plan(self, _message):
        self._plan_count += 1

    def _on_perception(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if isinstance(status, dict):
            self._perception = status
            self._perception_received_at = time.monotonic()

    def _required_path(self, parameter_name):
        value = str(self.get_parameter(parameter_name).value).strip()
        path = Path(value)
        if not value or not path.is_file():
            raise ValueError(f"{parameter_name} is not a file: {value}")
        return path.resolve()

    @staticmethod
    def _goal_manifest(waypoint):
        x, y, z = waypoint.position
        qx, qy, qz, qw = waypoint.orientation
        return {
            "id": waypoint.id,
            "task": waypoint.task,
            "direction": waypoint.direction,
            "goal_profile": waypoint.goal_profile,
            "heading_mode": "locked" if is_heading_locked(waypoint) else "free",
            "frame_id": waypoint.frame_id,
            "pose": {
                "position": {"x": x, "y": y, "z": z},
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
            },
        }

    @staticmethod
    def _pose_stamped(node, waypoint):
        pose = PoseStamped()
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.header.frame_id = waypoint.frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = waypoint.position
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = waypoint.orientation
        return pose

    def _load_route(self):
        waypoint_path = self._required_path("waypoints_file")
        try:
            document, authored = load_waypoint_document(waypoint_path)
            segments = load_planning_segments(document, authored)
            materialized = materialize_free_yaws(
                materialize_mission_route(authored, segments))
            action_segments = materialize_navigation_segments(authored, segments)
        except PlanningSegmentError as error:
            raise ValueError(f"planning_segments invalid: {error}") from error
        except ValueError as error:
            raise ValueError(f"waypoints_file has no executable route: {error}") from error

        by_id = {waypoint.id: waypoint for waypoint in materialized}
        stages = []
        for segment, action_segment in zip(segments, action_segments):
            goals = tuple(by_id[waypoint.id] for waypoint in action_segment)
            if not goals or segment.direction != "forward" or any(
                waypoint.direction != "forward" for waypoint in goals
            ):
                raise ValueError(f"planning segment {segment.id!r} is not forward-only")
            if len(goals) > 1 and any(
                waypoint.goal_profile != "standard" for waypoint in goals
            ) and not allows_precise_terminal_through_poses(goals):
                raise ValueError(
                    f"planning segment {segment.id!r} has an invalid "
                    "nonstandard ThroughPoses endpoint")
            stages.append({"id": segment.id, "goals": goals})
        return waypoint_path, tuple(stages)

    def _tree_for_pose(self, waypoint):
        if waypoint.goal_profile == "precise":
            return self._required_path("precise_behavior_tree")
        if not is_heading_locked(waypoint):
            return self._required_path("transit_behavior_tree")
        return self._required_path("forward_behavior_tree")

    def _tree_for_through(self, goals):
        terminal = goals[-1]
        if terminal.task == "return":
            name = "through_poses_return_behavior_tree"
        elif terminal.goal_profile == "precise":
            name = "through_poses_precise_behavior_tree"
        elif not is_heading_locked(terminal):
            name = "through_poses_transit_behavior_tree"
        else:
            name = "through_poses_behavior_tree"
        return self._required_path(name)

    def _wait_for_odom(self, timeout_sec=20.0):
        deadline = time.monotonic() + timeout_sec
        while self._pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._pose is not None

    def _wait_for_perception(self):
        timeout = float(self.get_parameter("perception_ready_timeout_sec").value)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("perception_ready_timeout_sec must be finite and positive")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self._perception
            if (
                isinstance(status, dict)
                and status.get("schema_version") == PERCEPTION_STATUS_SCHEMA_VERSION
                and status.get("ready") is True
            ):
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def _wait_for_result(self, client, goal):
        timeout = float(self.get_parameter("goal_timeout_sec").value)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("goal_timeout_sec must be finite and positive")
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=min(10.0, timeout))
        handle = sent.result()
        if handle is None or not handle.accepted:
            return None, "goal_rejected"
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        response = result_future.result()
        if response is None:
            handle.cancel_goal_async()
            return None, "result_timeout"
        return int(response.status), "ok"

    @staticmethod
    def _command_metrics(samples):
        values = [sample[1] for sample in samples]
        angular = [sample[2] for sample in samples]
        return {
            "linear_min_mps": round(min(values), 4) if values else 0.0,
            "linear_max_mps": round(max(values), 4) if values else 0.0,
            "angular_abs_max_radps": round(max(map(abs, angular)), 4) if angular else 0.0,
            "positive_count": sum(value > VELOCITY_EPSILON for value in values),
            "negative_count": sum(value < -VELOCITY_EPSILON for value in values),
        }

    def _run_stage(self, stage):
        goals = stage["goals"]
        through = len(goals) > 1 and bool(self.get_parameter("use_through_poses").value)
        if len(goals) > 1 and not through:
            raise ValueError("use_through_poses must remain enabled for multi-goal segments")
        if through:
            action = NavigateThroughPoses.Goal()
            action.poses = [self._pose_stamped(self, waypoint) for waypoint in goals]
            tree = self._tree_for_through(goals)
            client = self._through_client
            action_name = "navigate_through_poses"
        else:
            action = NavigateToPose.Goal()
            action.pose = self._pose_stamped(self, goals[0])
            tree = self._tree_for_pose(goals[0])
            client = self._action_client
            action_name = "navigate_to_pose"
        action.behavior_tree = str(tree)

        controller_start = len(self._controller_commands)
        candidate_start = len(self._candidate_commands)
        plans_start = self._plan_count
        started = time.monotonic()
        status, transport = self._wait_for_result(client, action)
        controller = self._command_metrics(self._controller_commands[controller_start:])
        candidate = self._command_metrics(self._candidate_commands[candidate_start:])
        return {
            "segment_id": stage["id"],
            "direction": "forward",
            "action": action_name,
            "behavior_tree": tree.name,
            "goal_ids": [waypoint.id for waypoint in goals],
            "outcome": (
                "succeeded"
                if status == GoalStatus.STATUS_SUCCEEDED else "failed"
            ),
            "status": status,
            "transport": transport,
            "duration_sec": round(time.monotonic() - started, 3),
            "planner_path_count": self._plan_count - plans_start,
            "controller_commands": controller,
            "candidate_commands": candidate,
            "final_pose": self._pose,
        }

    def _input_manifest(self, waypoint_path):
        trees = {}
        for parameter in TREE_PARAMETERS:
            path = self._required_path(parameter)
            trees[parameter] = {"name": path.name, "sha256": _sha256(path)}
        return {
            "waypoints_file": str(waypoint_path),
            "waypoints_sha256": _sha256(waypoint_path),
            "behavior_trees": trees,
        }

    def _save(self, status, reason=""):
        destination = Path(str(self.get_parameter("results_file").value)).expanduser()
        payload = {
            "schema_version": RESULT_SCHEMA_VERSION,
            "status": status,
            "reason": reason,
            "created_at_epoch_sec": time.time(),
            "input_manifest": self._input_manifest_cache,
            "route": self._route_manifest,
            "results": self._results,
            "perception": self._perception,
        }
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_suffix(destination.suffix + ".tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(destination)

    def run(self):
        waypoint_path, stages = self._load_route()
        self._input_manifest_cache = self._input_manifest(waypoint_path)
        self._route_manifest = {
            "segments": [
                {
                    "id": stage["id"],
                    "direction": "forward",
                    "goals": [self._goal_manifest(goal) for goal in stage["goals"]],
                }
                for stage in stages
            ]
        }
        if not self._action_client.wait_for_server(timeout_sec=90.0):
            raise RuntimeError("navigate_to_pose action server unavailable")
        if any(len(stage["goals"]) > 1 for stage in stages) and not self._through_client.wait_for_server(timeout_sec=90.0):
            raise RuntimeError("navigate_through_poses action server unavailable")
        if not self._wait_for_odom():
            raise RuntimeError("odom_combined unavailable")
        if not self._wait_for_perception():
            self._save("failed", "sim_perception_not_ready")
            return False
        for stage in stages:
            result = self._run_stage(stage)
            self._results.append(result)
            if result["outcome"] != "succeeded":
                self._save("failed", f"{stage['id']}_failed")
                return False
        self._save("completed")
        return True


def main():
    rclpy.init()
    node = AutoTrain()
    try:
        node.run()
    except Exception as error:
        node.get_logger().error(f"simulation route failed: {type(error).__name__}: {error}")
        if node._input_manifest_cache is not None:
            node._save("failed", f"runner_exception:{type(error).__name__}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
