#!/usr/bin/env python3
"""Validate NavigateThroughPoses for the forward segment in simulation.

This is a standalone test script — it does NOT modify any production code.
It replaces the per-waypoint NavigateToPose loop for the forward segment
(c_corner_2 .. p_finish, 6 waypoints) with a single NavigateThroughPoses
call, while keeping the reverse segment per-waypoint.

Usage (inside WSL2 simulation):
    ros2 run smartcar_sim test_through_poses.py --ros-args \
      -p waypoints_file:=<path_to_nav_only.yaml> \
      -p through_poses_bt:=<path_to_through_poses_bt.xml> \
      -p forward_behavior_tree:=<path_to_forward_bt.xml> \
      -p precise_behavior_tree:=<path_to_precise_bt.xml> \
      -p reverse_behavior_tree:=<path_to_reverse_bt.xml> \
      -p reverse_handoff_behavior_tree:=<path_to_reverse_handoff_bt.xml> \
      -p nav2_params_file:=<path_to_nav2_params_fixed.yaml> \
      -p results_file:=/tmp/through_poses_test_results.json
"""

import json
import math
import os
import time
import traceback
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# ── Route structure (must match nav_only.yaml) ──
# Waypoints before c_corner_2 use per-waypoint NavigateToPose (reverse segment).
# Waypoints from c_corner_2 onward use a single NavigateThroughPoses.
THROUGH_POSES_START = "c_corner_2"

VELOCITY_EPSILON = 1.0e-3
POSITION_OBSERVER_MARGIN_M = 2.0e-2
YAW_OBSERVER_MARGIN_RAD = 1.0e-2
CONFIG_TOLERANCE_EPSILON = 2.0e-3


class ThroughPosesTester(Node):
    """Hybrid test: per-waypoint for reverse, ThroughPoses for forward."""

    def __init__(self):
        super().__init__("test_through_poses")

        # use_sim_time is a built-in ROS parameter; do not declare it here.
        # Pass --ros-args -p use_sim_time:=true on the command line instead.
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("through_poses_bt", "")
        self.declare_parameter("forward_behavior_tree", "")
        self.declare_parameter("precise_behavior_tree", "")
        self.declare_parameter("reverse_behavior_tree", "")
        self.declare_parameter("reverse_handoff_behavior_tree", "")
        self.declare_parameter("nav2_params_file", "")
        self.declare_parameter("goal_timeout_sec", 180.0)
        self.declare_parameter("settle_delay_sec", 8.0)
        self.declare_parameter("results_file", "/tmp/through_poses_test_results.json")

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
        self.create_subscription(Odometry, "/odom_combined", self._odom_cb, qos)
        self.create_subscription(Twist, "/cmd_vel_nav", self._controller_cmd_cb, qos)
        self.create_subscription(Twist, "/cmd_vel_candidate", self._cmd_cb, qos)
        self.create_subscription(NavPath, "/plan", self._path_cb, path_qos)

        self._single_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._through_client = ActionClient(
            self, NavigateThroughPoses, "/navigate_through_poses"
        )

        self._latest_odom = None
        self._odom_samples = []
        self._controller_cmd_samples = []
        self._cmd_samples = []
        self._path_messages = []
        self._results = []

    # ── callbacks ──

    def _odom_cb(self, msg):
        pose = msg.pose.pose
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z
                   + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2),
        )
        self._latest_odom = (
            time.monotonic(), pose.position.x, pose.position.y, yaw)
        self._odom_samples.append(self._latest_odom)

    def _cmd_cb(self, msg):
        self._cmd_samples.append(
            (time.monotonic(), msg.linear.x, msg.angular.z))

    def _controller_cmd_cb(self, msg):
        self._controller_cmd_samples.append(
            (time.monotonic(), msg.linear.x, msg.angular.z))

    def _path_cb(self, msg):
        if msg.poses:
            ep = msg.poses[-1].pose
            yaw = math.atan2(
                2.0 * (ep.orientation.w * ep.orientation.z
                       + ep.orientation.x * ep.orientation.y),
                1.0 - 2.0 * (ep.orientation.y ** 2 + ep.orientation.z ** 2),
            )
            self._path_messages.append(
                (time.monotonic(), ep.position.x, ep.position.y, yaw))

    # ── helpers ──

    def _required_path(self, name):
        value = str(self.get_parameter(name).value).strip()
        path = Path(value)
        if not value or not path.is_file():
            raise ValueError(f"{name} is not a file: {value}")
        return path

    def _wait_for_odom(self, timeout_sec=10.0):
        deadline = time.monotonic() + timeout_sec
        while self._latest_odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._latest_odom is not None

    @staticmethod
    def _travel_distance(samples):
        return sum(
            math.hypot(b[1] - a[1], b[2] - a[2])
            for a, b in zip(samples, samples[1:])
        )

    def _load_waypoints(self):
        path = self._required_path("waypoints_file")
        doc = yaml.safe_load(path.read_text(encoding="utf-8"))
        waypoints = doc.get("waypoints", [])
        route = [w for w in waypoints if w.get("task") != "start"]
        # Split at THROUGH_POSES_START
        single = []
        through = []
        collecting_through = False
        for w in route:
            if w["id"] == THROUGH_POSES_START:
                collecting_through = True
            if collecting_through:
                through.append(w)
            else:
                single.append(w)
        if not through:
            raise ValueError(
                f"ThroughPoses start waypoint '{THROUGH_POSES_START}' not found")
        self.get_logger().info(
            f"Split: {len(single)} single-pose + {len(through)} through-poses")
        for w in single:
            self.get_logger().info(f"  single: {w['id']} dir={w.get('direction')}")
        for w in through:
            self.get_logger().info(f"  through: {w['id']} dir={w.get('direction')}")
        return single, through

    def _single_bt(self, waypoint):
        if waypoint.get("goal_profile") == "reverse_handoff":
            return self._required_path("reverse_handoff_behavior_tree")
        if waypoint.get("direction") == "reverse":
            return self._required_path("reverse_behavior_tree")
        if waypoint.get("goal_profile") == "precise":
            return self._required_path("precise_behavior_tree")
        return self._required_path("forward_behavior_tree")

    def _goal_tolerances(self, waypoint):
        params_path = self._required_path("nav2_params_file")
        doc = yaml.safe_load(params_path.read_text(encoding="utf-8"))
        ctrl = doc["controller_server"]["ros__parameters"]
        if waypoint.get("goal_profile") == "reverse_handoff":
            checker = ctrl.get("reverse_handoff_goal_checker",
                                ctrl["reverse_goal_checker"])
        elif waypoint.get("goal_profile") == "precise":
            checker = ctrl["precise_goal_checker"]
        elif waypoint.get("direction") == "reverse":
            checker = ctrl["reverse_goal_checker"]
        else:
            checker = ctrl["goal_checker"]
        return (
            float(checker["xy_goal_tolerance"]),
            float(checker["yaw_goal_tolerance"]),
        )

    # ── single-pose navigation (for reverse segment) ──

    def _send_single_goal(self, waypoint):
        bt = self._single_bt(waypoint)
        timeout = float(self.get_parameter("goal_timeout_sec").value)
        start_time = time.monotonic()
        odom_start = len(self._odom_samples)
        start_pose = self._latest_odom

        pose = waypoint["pose"]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = waypoint.get("frame_id", "odom_combined")
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose["position"]["x"])
        goal.pose.pose.position.y = float(pose["position"]["y"])
        goal.pose.pose.position.z = float(pose["position"].get("z", 0.0))
        goal.pose.pose.orientation.x = float(pose["orientation"]["x"])
        goal.pose.pose.orientation.y = float(pose["orientation"]["y"])
        goal.pose.pose.orientation.z = float(pose["orientation"]["z"])
        goal.pose.pose.orientation.w = float(pose["orientation"]["w"])
        goal.behavior_tree = str(bt)

        self.get_logger().info(
            f"[single] {waypoint['id']} dir={waypoint.get('direction')} bt={bt.name}")

        send_future = self._single_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            return self._make_result(waypoint, bt, "goal_response_timeout",
                                     start_time, start_pose, odom_start, None)
        try:
            goal_handle = send_future.result()
        except Exception as exc:
            return self._make_result(waypoint, bt, f"send_error:{exc}",
                                     start_time, start_pose, odom_start, None)
        if goal_handle is None or not goal_handle.accepted:
            return self._make_result(waypoint, bt, "rejected",
                                     start_time, start_pose, odom_start, None)

        result_future = goal_handle.get_result_async()
        while not result_future.done() and time.monotonic() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.25)

        if not result_future.done():
            goal_handle.cancel_goal_async()
            outcome, status = "timeout", None
        else:
            try:
                status = result_future.result().status
                outcome = "succeeded" if status == GoalStatus.STATUS_SUCCEEDED else "failed"
            except Exception as exc:
                outcome, status = f"result_error:{exc}", None

        return self._make_result(waypoint, bt, outcome, start_time, start_pose,
                                 odom_start, status)

    # ── ThroughPoses navigation (for forward segment) ──

    def _send_through_poses(self, waypoints):
        """Send all forward waypoints as a single NavigateThroughPoses goal."""
        bt = self._required_path("through_poses_bt")
        timeout = float(self.get_parameter("goal_timeout_sec").value)
        start_time = time.monotonic()
        odom_start = len(self._odom_samples)
        path_start = len(self._path_messages)
        start_pose = self._latest_odom

        goal = NavigateThroughPoses.Goal()
        for w in waypoints:
            pose = w["pose"]
            ps = PoseStamped()
            ps.header.frame_id = w.get("frame_id", "odom_combined")
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = float(pose["position"]["x"])
            ps.pose.position.y = float(pose["position"]["y"])
            ps.pose.position.z = float(pose["position"].get("z", 0.0))
            ps.pose.orientation.x = float(pose["orientation"]["x"])
            ps.pose.orientation.y = float(pose["orientation"]["y"])
            ps.pose.orientation.z = float(pose["orientation"]["z"])
            ps.pose.orientation.w = float(pose["orientation"]["w"])
            goal.poses.append(ps)
        goal.behavior_tree = str(bt)

        ids = ", ".join(w["id"] for w in waypoints)
        self.get_logger().info(
            f"[through] {len(waypoints)} waypoints: {ids}\n  bt={bt.name}")

        send_future = self._through_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            return self._make_through_result(
                waypoints, bt, "goal_response_timeout",
                start_time, start_pose, odom_start, path_start, None)

        try:
            goal_handle = send_future.result()
        except Exception as exc:
            return self._make_through_result(
                waypoints, bt, f"send_error:{exc}",
                start_time, start_pose, odom_start, path_start, None)
        if goal_handle is None or not goal_handle.accepted:
            return self._make_through_result(
                waypoints, bt, "rejected",
                start_time, start_pose, odom_start, path_start, None)

        result_future = goal_handle.get_result_async()
        next_log = start_time + 15.0
        while not result_future.done() and time.monotonic() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.25)
            now = time.monotonic()
            if now >= next_log:
                cur = self._latest_odom
                if cur is not None:
                    elapsed = now - start_time
                    self.get_logger().info(
                        f"  [through] t={elapsed:.0f}s pos=({cur[1]:.2f},{cur[2]:.2f})")
                next_log += 15.0

        if not result_future.done():
            goal_handle.cancel_goal_async()
            outcome, status = "timeout", None
        else:
            try:
                status = result_future.result().status
                outcome = "succeeded" if status == GoalStatus.STATUS_SUCCEEDED else "failed"
            except Exception as exc:
                outcome, status = f"result_error:{exc}", None
            outcome = "succeeded" if status == GoalStatus.STATUS_SUCCEEDED else "failed"

        return self._make_through_result(
            waypoints, bt, outcome, start_time, start_pose, odom_start, path_start,
            status)

    # ── result builders ──

    def _make_result(self, waypoint, bt, outcome, start_time, start_pose,
                     odom_start, status):
        cur = self._latest_odom or start_pose
        pose = waypoint["pose"]
        tx, ty = float(pose["position"]["x"]), float(pose["position"]["y"])
        ori = pose["orientation"]
        target_yaw = math.atan2(
            2.0 * (float(ori["w"]) * float(ori["z"])
                   + float(ori["x"]) * float(ori["y"])),
            1.0 - 2.0 * (float(ori["y"]) ** 2 + float(ori["z"]) ** 2))
        goal_err = math.hypot(cur[1] - tx, cur[2] - ty) if cur else None
        yaw_err = abs(math.remainder(
            cur[3] - target_yaw, 2.0 * math.pi)) if cur else None
        xy_tol, yaw_tol = self._goal_tolerances(waypoint)
        travel = self._travel_distance(self._odom_samples[odom_start:])
        duration = round(time.monotonic() - start_time, 2)

        result = {
            "id": waypoint["id"],
            "mode": "single",
            "direction": waypoint.get("direction", "forward"),
            "goal_profile": waypoint.get("goal_profile", "standard"),
            "behavior_tree": bt.name,
            "outcome": outcome,
            "status": status,
            "duration_sec": duration,
            "goal_error_m": round(goal_err, 3) if goal_err is not None else None,
            "goal_yaw_error_rad": round(yaw_err, 3) if yaw_err is not None else None,
            "xy_goal_tolerance_m": round(xy_tol, 3),
            "yaw_goal_tolerance_rad": round(yaw_tol, 3),
            "travel_m": round(travel, 3),
        }
        self.get_logger().info(
            f"Result [single] {waypoint['id']}: {outcome} "
            f"err={goal_err}m yaw={yaw_err}rad dur={duration}s")
        return result

    def _make_through_result(self, waypoints, bt, outcome, start_time, start_pose,
                             odom_start, path_start, status):
        cur = self._latest_odom or start_pose
        # Use the LAST waypoint for final position error
        last = waypoints[-1]
        pose = last["pose"]
        tx, ty = float(pose["position"]["x"]), float(pose["position"]["y"])
        ori = pose["orientation"]
        target_yaw = math.atan2(
            2.0 * (float(ori["w"]) * float(ori["z"])
                   + float(ori["x"]) * float(ori["y"])),
            1.0 - 2.0 * (float(ori["y"]) ** 2 + float(ori["z"]) ** 2))
        goal_err = math.hypot(cur[1] - tx, cur[2] - ty) if cur else None
        yaw_err = abs(math.remainder(
            cur[3] - target_yaw, 2.0 * math.pi)) if cur else None

        travel = self._travel_distance(self._odom_samples[odom_start:])
        duration = round(time.monotonic() - start_time, 2)

        # Per-waypoint pass-through check: did the vehicle come within
        # reasonable range of each intermediate waypoint?
        passed = []
        for w in waypoints:
            wp_x = float(w["pose"]["position"]["x"])
            wp_y = float(w["pose"]["position"]["y"])
            min_dist = min(
                (math.hypot(s[1] - wp_x, s[2] - wp_y)
                 for s in self._odom_samples[odom_start:]),
                default=None,
            )
            passed.append({
                "id": w["id"],
                "min_distance_m": round(min_dist, 3) if min_dist is not None else None,
            })

        # Path endpoints from /plan topic
        path_endpoints = [
            {"x": round(px, 3), "y": round(py, 3), "yaw": round(pyaw, 3)}
            for _, px, py, pyaw in self._path_messages[path_start:]
        ]

        ids = ", ".join(w["id"] for w in waypoints)
        result = {
            "id": f"through_poses[{ids}]",
            "mode": "through_poses",
            "waypoint_count": len(waypoints),
            "behavior_tree": bt.name,
            "outcome": outcome,
            "status": status,
            "duration_sec": duration,
            "final_goal_error_m": round(goal_err, 3) if goal_err is not None else None,
            "final_yaw_error_rad": round(yaw_err, 3) if yaw_err is not None else None,
            "target_yaw_rad": round(target_yaw, 3),
            "travel_m": round(travel, 3),
            "waypoints_passed": passed,
            "path_endpoints": path_endpoints,
        }
        self.get_logger().info(
            f"Result [through] {len(waypoints)}wps: {outcome} "
            f"err={goal_err}m yaw={yaw_err}rad dur={duration}s travel={travel}m")
        for wp in passed:
            self.get_logger().info(f"  {wp['id']}: min_dist={wp['min_distance_m']}m")
        return result

    # ── main ──

    def run(self):
        single_wps, through_wps = self._load_waypoints()

        # Wait for action servers
        if not self._single_client.wait_for_server(timeout_sec=90.0):
            raise RuntimeError("/navigate_to_pose action server unavailable")
        if not self._through_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().warn(
                "/navigate_through_poses action server unavailable — "
                "will only test single-pose navigation")
            through_wps = []  # skip through-poses test
        if not self._wait_for_odom():
            raise RuntimeError("odom_combined unavailable")

        # Settle delay: let costmaps stabilise after lifecycle activation
        settle = float(self.get_parameter("settle_delay_sec").value)
        if settle > 0.0:
            self.get_logger().info(f"Settling for {settle:.1f}s...")
            deadline = time.monotonic() + settle
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.2)

        # Phase 1: Per-waypoint single-pose navigation (reverse segment)
        for w in single_wps:
            result = self._send_single_goal(w)
            self._results.append(result)
            if result["outcome"] != "succeeded":
                self._save_results("failed", "single_pose_failure")
                return False
            # Brief pause between goals
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)

        # Phase 2: Single NavigateThroughPoses (forward segment)
        if through_wps:
            result = self._send_through_poses(through_wps)
            self._results.append(result)
            if result["outcome"] != "succeeded":
                self._save_results("failed", "through_poses_failure")
                return False

        self._save_results("completed")
        return True

    def _save_results(self, outcome, error=None):
        path = Path(str(self.get_parameter("results_file").value))
        data = {
            "overall_outcome": outcome,
            "mode": "hybrid_single_through_poses",
            "through_poses_start": THROUGH_POSES_START,
            "results": self._results,
        }
        if error:
            data["error"] = error
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
        try:
            tmp.write_text(
                json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
            os.replace(tmp, path)
        finally:
            tmp.unlink(missing_ok=True)
        self.get_logger().info(f"Results saved: {path}")


def main():
    rclpy.init()
    node = None
    exit_code = 0
    try:
        node = ThroughPosesTester()
        if not node.run():
            exit_code = 1
    except Exception as exc:
        if node is not None:
            node.get_logger().error(traceback.format_exc())
            node._save_results("error", f"{type(exc).__name__}: {exc}")
        else:
            print(traceback.format_exc(), flush=True)
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
