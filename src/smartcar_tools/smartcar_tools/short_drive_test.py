"""Bounded, supervised forward drive through the production Nav2 chain."""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


MAX_TEST_SPEED_MPS = 0.30
# Each profile is a fixed, contiguous route prefix. These limits retain a
# fail-closed travel and time bound for a watched physical test.
ROUTE_PROFILES = {
    "p_to_a": {
        "task_parameters": {
            "supervised_p_to_a_only": True,
            "supervised_p_to_c1_only": False,
            "navigation_test_end_segment_id": "p_to_qr",
        },
        "max_distance_m": 3.5,
        "max_timeout_sec": 120.0,
        "terminal_pose": (2.898403475881205, 0.9659945286391132, 0.20943951023931956),
        "position_tolerance_m": 0.12,
        "yaw_tolerance_rad": 0.15,
    },
    "p_to_c1": {
        "task_parameters": {
            "supervised_p_to_a_only": False,
            "supervised_p_to_c1_only": True,
            "navigation_test_end_segment_id": "qr_to_vlm",
        },
        "max_distance_m": 10.5,
        "max_timeout_sec": 240.0,
        "terminal_pose": (0.9330276705276708, 3.8337068653474913, 1.171056970790554),
        "position_tolerance_m": 0.12,
        "yaw_tolerance_rad": 0.15,
    },
}
MAX_TEST_DISTANCE_M = max(profile["max_distance_m"] for profile in ROUTE_PROFILES.values())
MAX_TEST_TIMEOUT_SEC = max(profile["max_timeout_sec"] for profile in ROUTE_PROFILES.values())
DEFAULT_RESULTS_FILE = "/tmp/smartcar_short_drive_result.json"


def completed_distance_reason(reason):
    """Return whether a bounded run met its only successful completion condition."""
    return str(reason).startswith("distance_limit:")


def outcome_passed(reason, require_mission_complete):
    """Apply the requested completion contract to a terminal test reason."""
    if require_mission_complete:
        return str(reason) == "mission_completed"
    return completed_distance_reason(reason)


def route_profile_spec(route_profile):
    try:
        return ROUTE_PROFILES[str(route_profile)]
    except KeyError as error:
        raise ValueError(
            "route_profile must be one of: " + ", ".join(ROUTE_PROFILES)
        ) from error


def runtime_mode_errors(safety_params, task_params, speed_mps, route_profile="p_to_a"):
    """Reject a test stack that is not the dedicated depth-only short-drive mode."""
    profile = route_profile_spec(route_profile)
    errors = []
    speed_cap = safety_params.get("max_linear_speed_mps")
    if (
        not isinstance(speed_cap, (int, float))
        or isinstance(speed_cap, bool)
        or not math.isfinite(speed_cap)
        or speed_cap <= 0.0
        or speed_cap > speed_mps + 1.0e-6
    ):
        errors.append(
            "safety.max_linear_speed_mps must be positive and no greater than "
            f"the requested test speed ({speed_mps:.3f})"
        )

    expected_safety = {
        "require_depth_points": True,
        "require_scan": False,
        "emergency_stop_on_start": True,
    }
    for name, expected in expected_safety.items():
        if safety_params.get(name) is not expected:
            errors.append(f"safety.{name} must be {expected}")

    expected_task = {
        "use_depth_camera": True,
        "depth_camera_calibrated": True,
        "autostart_mission": False,
        "waypoints_calibrated": True,
        "extrinsics_calibrated": True,
        "steering_calibrated": True,
        "emergency_stop_ready": True,
        "operator_approved": True,
    }
    expected_task.update(profile["task_parameters"])
    for name, expected in expected_task.items():
        if task_params.get(name) != expected:
            errors.append(f"task.{name} must be {expected!r}")
    return errors


def validate_test_limits(distance_m, speed_mps, timeout_sec, route_profile="p_to_a"):
    """Reject unsafe test bounds before any command can be sent."""
    profile = route_profile_spec(route_profile)
    values = {
        "distance_m": float(distance_m),
        "speed_mps": float(speed_mps),
        "timeout_sec": float(timeout_sec),
    }
    for name, value in values.items():
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    if values["speed_mps"] > MAX_TEST_SPEED_MPS:
        raise ValueError(
            f"speed_mps exceeds bounded test cap {MAX_TEST_SPEED_MPS:.2f}")
    if values["distance_m"] > profile["max_distance_m"]:
        raise ValueError(
            "distance_m exceeds bounded test cap "
            f"{profile['max_distance_m']:.1f} for {route_profile}")
    if values["timeout_sec"] > profile["max_timeout_sec"]:
        raise ValueError(
            "timeout_sec exceeds bounded test cap "
            f"{profile['max_timeout_sec']:.0f} for {route_profile}")
    return values


class ShortDrive(Node):
    def __init__(
        self,
        distance_m=0.25,
        speed_mps=0.05,
        timeout_sec=10.0,
        require_mission_complete=False,
        route_profile="p_to_a",
        results_file=DEFAULT_RESULTS_FILE,
    ):
        super().__init__("short_drive_test")
        self.route_profile = str(route_profile)
        self.profile = route_profile_spec(self.route_profile)
        limits = validate_test_limits(
            distance_m, speed_mps, timeout_sec, self.route_profile)
        self.distance_m = limits["distance_m"]
        self.speed_mps = limits["speed_mps"]
        self.timeout_sec = limits["timeout_sec"]
        self.require_mission_complete = bool(require_mission_complete)
        self.results_file = Path(results_file)
        self.invoked_at = time.monotonic()
        self.start_pose = None
        self.last_nav_position = None
        self.fused_travel_distance_m = 0.0
        self.latest_odom = None
        self.last_odom_at = None
        self.latest_nav_odom = None
        self.last_nav_odom_at = None
        self.last_depth_at = None
        self.last_local_costmap_at = None
        self.last_global_costmap_at = None
        self.local_costmap_occupied_cells = None
        self.global_costmap_occupied_cells = None
        self.last_ackermann = None
        self.max_ackermann_speed = 0.0
        self.last_safety = ""
        self.last_state = ""
        self.saw_navigating = False
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            Odometry, "/odom_combined", self._on_nav_odom, 10)
        self.create_subscription(
            PointCloud2,
            "/smartcar/depth/points",
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            AckermannDriveStamped, "/ackermann_cmd", self._on_ackermann, 10)
        self.create_subscription(
            Costmap,
            "/local_costmap/costmap_raw",
            self._on_local_costmap,
            10,
        )
        self.create_subscription(
            Costmap,
            "/global_costmap/costmap_raw",
            self._on_global_costmap,
            10,
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/smartcar/safety/status", self._on_safety, status_qos)
        self.create_subscription(
            String, "/smartcar/task/state", self._on_state, 10)
        self.start_client = self.create_client(Trigger, "/smartcar/task/start")
        self.stop_client = self.create_client(Trigger, "/smartcar/task/stop")
        self.estop_client = self.create_client(
            SetBool, "/smartcar/safety/emergency_stop")
        self.controller_params = self.create_client(
            SetParameters, "/controller_server/set_parameters")
        self.smoother_params = self.create_client(
            SetParameters, "/velocity_smoother/set_parameters")
        self.safety_params = self.create_client(
            GetParameters, "/safety_node/get_parameters")
        self.task_params = self.create_client(
            GetParameters, "/task_node/get_parameters")

    def _on_odom(self, message):
        self.latest_odom = message
        self.last_odom_at = time.monotonic()

    def _on_nav_odom(self, message):
        self.latest_nav_odom = message
        self.last_nav_odom_at = time.monotonic()
        if self.start_pose is not None:
            position = message.pose.pose.position
            if self.last_nav_position is not None:
                self.fused_travel_distance_m += math.hypot(
                    position.x - self.last_nav_position[0],
                    position.y - self.last_nav_position[1],
                )
            self.last_nav_position = (position.x, position.y)

    def _on_depth(self, _message):
        self.last_depth_at = time.monotonic()

    def _on_ackermann(self, message):
        self.last_ackermann = message.drive.speed
        self.max_ackermann_speed = max(
            self.max_ackermann_speed, abs(float(message.drive.speed)))

    @staticmethod
    def _occupied_cells(message):
        return sum(int(value) >= 253 for value in message.data)

    def _on_local_costmap(self, message):
        self.last_local_costmap_at = time.monotonic()
        self.local_costmap_occupied_cells = self._occupied_cells(message)

    def _on_global_costmap(self, message):
        self.last_global_costmap_at = time.monotonic()
        self.global_costmap_occupied_cells = self._occupied_cells(message)

    def _on_safety(self, message):
        self.last_safety = str(message.data)

    def _on_state(self, message):
        self.last_state = str(message.data)

    def spin_until(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.10)
            if predicate():
                return True
        return False

    def call(self, client, request, timeout_sec=3.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        if not self.spin_until(future.done, timeout_sec):
            raise RuntimeError(f"service timeout: {client.srv_name}")
        response = future.result()
        if response is None:
            raise RuntimeError(f"empty response: {client.srv_name}")
        return response

    def read_parameters(self, client, names):
        request = GetParameters.Request()
        request.names = list(names)
        response = self.call(client, request)
        if len(response.values) != len(names):
            raise RuntimeError(f"parameter response incomplete: {client.srv_name}")
        return dict(zip(names, response.values))

    @staticmethod
    def _parameter_value(name, value, parameter_type):
        if value.type != parameter_type:
            raise RuntimeError(f"parameter {name} has unexpected type {value.type}")
        if parameter_type == ParameterType.PARAMETER_BOOL:
            return bool(value.bool_value)
        if parameter_type == ParameterType.PARAMETER_DOUBLE:
            return float(value.double_value)
        if parameter_type == ParameterType.PARAMETER_STRING:
            return str(value.string_value)
        raise RuntimeError(f"unsupported expected type for parameter {name}")

    def verify_runtime_mode(self):
        safety_types = {
            "max_linear_speed_mps": ParameterType.PARAMETER_DOUBLE,
            "require_depth_points": ParameterType.PARAMETER_BOOL,
            "require_scan": ParameterType.PARAMETER_BOOL,
            "emergency_stop_on_start": ParameterType.PARAMETER_BOOL,
        }
        task_types = {
            "use_depth_camera": ParameterType.PARAMETER_BOOL,
            "depth_camera_calibrated": ParameterType.PARAMETER_BOOL,
            "supervised_p_to_a_only": ParameterType.PARAMETER_BOOL,
            "supervised_p_to_c1_only": ParameterType.PARAMETER_BOOL,
            "navigation_test_end_segment_id": ParameterType.PARAMETER_STRING,
            "autostart_mission": ParameterType.PARAMETER_BOOL,
            "waypoints_calibrated": ParameterType.PARAMETER_BOOL,
            "extrinsics_calibrated": ParameterType.PARAMETER_BOOL,
            "steering_calibrated": ParameterType.PARAMETER_BOOL,
            "emergency_stop_ready": ParameterType.PARAMETER_BOOL,
            "operator_approved": ParameterType.PARAMETER_BOOL,
        }
        raw_safety = self.read_parameters(self.safety_params, safety_types)
        raw_task = self.read_parameters(self.task_params, task_types)
        safety = {
            name: self._parameter_value(name, raw_safety[name], expected_type)
            for name, expected_type in safety_types.items()
        }
        task = {
            name: self._parameter_value(name, raw_task[name], expected_type)
            for name, expected_type in task_types.items()
        }
        errors = runtime_mode_errors(
            safety, task, self.speed_mps, self.route_profile)
        if errors:
            raise RuntimeError("runtime mode rejected: " + "; ".join(errors))
        return safety, task

    @staticmethod
    def _double(name, value):
        return Parameter(name, Parameter.Type.DOUBLE, float(value)).to_parameter_msg()

    @staticmethod
    def _double_array(name, values):
        return Parameter(
            name, Parameter.Type.DOUBLE_ARRAY, [float(v) for v in values]
        ).to_parameter_msg()

    def set_speed_limits(self):
        controller_request = SetParameters.Request()
        controller_request.parameters = [
            self._double("FollowPath.desired_linear_vel", self.speed_mps),
            self._double("ForwardAvoidance.desired_linear_vel", self.speed_mps),
        ]
        smoother_request = SetParameters.Request()
        smoother_request.parameters = [
            self._double_array("max_velocity", [self.speed_mps, 0.0, 1.363636]),
            self._double_array("min_velocity", [-self.speed_mps, 0.0, -1.363636]),
        ]
        controller = self.call(self.controller_params, controller_request)
        smoother = self.call(self.smoother_params, smoother_request)
        if not all(result.successful for result in controller.results):
            raise RuntimeError("controller speed parameter rejected")
        if not all(result.successful for result in smoother.results):
            raise RuntimeError("velocity smoother speed parameter rejected")

    def clear_estop(self):
        request = SetBool.Request()
        request.data = False
        response = self.call(self.estop_client, request)
        if not response.success:
            raise RuntimeError(f"failed to clear software e-stop: {response.message}")

    def stop_and_latch(self, reason):
        self.get_logger().warning(f"Stopping bounded drive: {reason}")
        try:
            response = self.call(self.stop_client, Trigger.Request(), timeout_sec=4.0)
            self.get_logger().info(f"task stop: {response.message}")
        except Exception as error:
            self.get_logger().error(f"task stop failed: {error}")
        try:
            request = SetBool.Request()
            request.data = True
            response = self.call(self.estop_client, request)
            self.get_logger().info(
                f"software e-stop latched: {response.message}")
        except Exception as error:
            self.get_logger().error(f"e-stop latch failed: {error}")

    def _result(self, passed, reason, started_at):
        terminal_error = self.terminal_error()
        terminal_x, terminal_y, terminal_yaw = self.profile["terminal_pose"]
        return {
            "schema_version": 1,
            "test": "supervised_depth_short_drive",
            "passed": bool(passed),
            "reason": str(reason),
            "duration_sec": round(max(0.0, time.monotonic() - started_at), 3),
            "limits": {
                "distance_m": self.distance_m,
                "speed_mps": self.speed_mps,
                "timeout_sec": self.timeout_sec,
                "require_mission_complete": self.require_mission_complete,
                "route_profile": self.route_profile,
            },
            "measurements": {
                "fused_travel_distance_m": round(self.fused_travel_distance_m, 4),
                "fused_forward_displacement_m": round(
                    self.forward_displacement(), 4),
                "terminal_position_error_m": terminal_error[0],
                "terminal_yaw_error_rad": terminal_error[1],
                "max_ackermann_speed_mps": round(self.max_ackermann_speed, 4),
                "last_ackermann_speed_mps": self.last_ackermann,
                "local_raw_occupied_cells": self.local_costmap_occupied_cells,
                "global_raw_occupied_cells": self.global_costmap_occupied_cells,
            },
            "final_state": {
                "safety": self.last_safety,
                "task": self.last_state,
            },
            "expected_terminal_pose": {
                "x": terminal_x,
                "y": terminal_y,
                "yaw": terminal_yaw,
                "position_tolerance_m": self.profile["position_tolerance_m"],
                "yaw_tolerance_rad": self.profile["yaw_tolerance_rad"],
            },
        }

    def write_result(self, result):
        self.results_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.results_file.with_suffix(self.results_file.suffix + ".tmp")
        temporary.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.results_file)
        self.get_logger().info(f"Bounded test result: {self.results_file}")

    def forward_displacement(self):
        if self.start_pose is None or self.latest_nav_odom is None:
            return 0.0
        p0 = self.start_pose.pose.pose.position
        p1 = self.latest_nav_odom.pose.pose.position
        q = self.start_pose.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (p1.x - p0.x) * math.cos(yaw) + (p1.y - p0.y) * math.sin(yaw)

    @staticmethod
    def _yaw(orientation):
        return math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )

    def terminal_error(self):
        if self.latest_nav_odom is None:
            return None, None
        target_x, target_y, target_yaw = self.profile["terminal_pose"]
        pose = self.latest_nav_odom.pose.pose
        position_error = math.hypot(
            pose.position.x - target_x, pose.position.y - target_y)
        yaw_error = math.atan2(
            math.sin(self._yaw(pose.orientation) - target_yaw),
            math.cos(self._yaw(pose.orientation) - target_yaw),
        )
        return round(position_error, 4), round(abs(yaw_error), 4)

    def terminal_pose_passed(self):
        position_error, yaw_error = self.terminal_error()
        return (
            position_error is not None
            and yaw_error is not None
            and position_error <= self.profile["position_tolerance_m"]
            and yaw_error <= self.profile["yaw_tolerance_rad"]
        )

    def begin_measurement(self):
        self.start_pose = self.latest_nav_odom
        position = self.start_pose.pose.pose.position
        self.last_nav_position = (position.x, position.y)
        self.fused_travel_distance_m = 0.0

    def run(self):
        if not self.spin_until(
            lambda: (
                self.latest_odom is not None
                and self.latest_nav_odom is not None
                and self.last_depth_at is not None
                and self.last_ackermann is not None
                and self.last_local_costmap_at is not None
                and self.last_global_costmap_at is not None
                and self.last_safety == "emergency_stop"
            ),
            8.0,
        ):
            raise RuntimeError(
                "odom/depth/ackermann/raw-costmap/e-stop preflight unavailable")
        if abs(float(self.last_ackermann)) > 1.0e-4:
            raise RuntimeError("ackermann output was nonzero while e-stop was latched")
        self.verify_runtime_mode()
        self.begin_measurement()
        self.set_speed_limits()
        self.get_logger().info(
            f"Preflight passed: speed cap {self.speed_mps:.3f} m/s, "
            f"travel cap {self.distance_m:.3f} m ({self.route_profile})")
        self.clear_estop()
        response = self.call(self.start_client, Trigger.Request(), timeout_sec=4.0)
        if not response.success:
            raise RuntimeError(f"task start rejected: {response.message}")
        started_at = time.monotonic()
        self.get_logger().info(f"Mission started: {response.message}")
        reason = "timeout"
        while time.monotonic() - started_at < self.timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            displacement = self.forward_displacement()
            now = time.monotonic()
            if self.last_state == "NAVIGATING":
                self.saw_navigating = True
            if self.last_state == "COMPLETED":
                if (
                    self.saw_navigating
                    and self.max_ackermann_speed > 1.0e-4
                    and self.terminal_pose_passed()
                ):
                    reason = "mission_completed"
                else:
                    position_error, yaw_error = self.terminal_error()
                    reason = (
                        "mission_completed_without_terminal_proof:"
                        f"position={position_error},yaw={yaw_error}"
                    )
                break
            if self.last_state in ("FAILED", "STOPPED"):
                reason = f"task_terminal:{self.last_state}"
                break
            if self.fused_travel_distance_m >= self.distance_m:
                reason = f"distance_limit:{self.fused_travel_distance_m:.3f}m"
                break
            if displacement < -0.03:
                reason = f"unexpected_reverse:{displacement:.3f}m"
                break
            if self.last_odom_at is None or now - self.last_odom_at > 0.45:
                reason = "raw_odom_stale"
                break
            if self.last_nav_odom_at is None or now - self.last_nav_odom_at > 0.45:
                reason = "fused_odom_stale"
                break
            if self.last_depth_at is None or now - self.last_depth_at > 0.65:
                reason = "depth_points_stale"
                break
            if (
                self.last_local_costmap_at is None
                or now - self.last_local_costmap_at > 2.0
            ):
                reason = "local_raw_costmap_stale"
                break
            if (
                self.last_global_costmap_at is None
                or now - self.last_global_costmap_at > 2.0
            ):
                reason = "global_raw_costmap_stale"
                break
            if self.last_safety and self.last_safety != "ok":
                reason = f"safety_blocked:{self.last_safety}"
                break
            if self.last_ackermann is not None and abs(self.last_ackermann) > self.speed_mps + 0.005:
                reason = f"ackermann_speed_limit:{self.last_ackermann:.3f}m/s"
                break
        self.stop_and_latch(reason)
        zero_deadline = time.monotonic() + 3.0
        nonzero = []
        while time.monotonic() < zero_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_ackermann is not None and abs(self.last_ackermann) > 1.0e-4:
                nonzero.append(self.last_ackermann)
        if nonzero:
            raise RuntimeError(f"final ackermann not zero: {nonzero[-1]:.3f}")
        result = self._result(
            outcome_passed(reason, self.require_mission_complete),
            reason,
            started_at,
        )
        self.get_logger().info(
            "Bounded test %s: reason=%s travel=%.3fm state=%s safety=%s"
            % (
                "passed" if result["passed"] else "failed",
                reason,
                self.fused_travel_distance_m,
                self.last_state,
                self.last_safety,
            )
        )
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Run a bounded supervised forward Nav2 drive test.")
    parser.add_argument(
        "--distance-m", type=float, default=0.25,
        help="maximum cumulative fused-odometry travel (default: 0.25)")
    parser.add_argument(
        "--speed-mps", type=float, default=0.05,
        help="forward speed cap, at most 0.30 m/s (default: 0.05)")
    parser.add_argument(
        "--timeout-sec", type=float, default=10.0,
        help="maximum run time within the selected route-profile cap (default: 10)")
    parser.add_argument(
        "--route-profile", choices=tuple(ROUTE_PROFILES), default="p_to_a",
        help="fixed supervised route prefix (default: p_to_a)")
    parser.add_argument(
        "--require-mission-complete", action="store_true",
        help="pass only after the fixed supervised mission reaches COMPLETED at its terminal pose")
    parser.add_argument(
        "--results-file", default=DEFAULT_RESULTS_FILE,
        help=f"JSON evidence path (default: {DEFAULT_RESULTS_FILE})")
    args = parser.parse_args(argv)
    rclpy.init(args=[])
    node = ShortDrive(
        distance_m=args.distance_m,
        speed_mps=args.speed_mps,
        timeout_sec=args.timeout_sec,
        require_mission_complete=args.require_mission_complete,
        route_profile=args.route_profile,
        results_file=args.results_file,
    )
    try:
        result = node.run()
        node.write_result(result)
        return 0 if result["passed"] else 1
    except Exception as error:
        node.stop_and_latch(f"preflight/test failure: {error}")
        node.write_result(node._result(False, f"error:{error}", node.invoked_at))
        node.get_logger().error(f"Bounded test failed: {error}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
