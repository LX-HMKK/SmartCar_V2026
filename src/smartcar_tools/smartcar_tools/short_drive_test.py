"""Bounded, supervised forward drive through the production Nav2 chain."""

import argparse
import math
import sys
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


MAX_TEST_SPEED_MPS = 0.30
MAX_TEST_DISTANCE_M = 3.0
MAX_TEST_TIMEOUT_SEC = 120.0


def validate_test_limits(distance_m, speed_mps, timeout_sec):
    """Reject unsafe test bounds before any command can be sent."""
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
    if values["distance_m"] > MAX_TEST_DISTANCE_M:
        raise ValueError(
            f"distance_m exceeds bounded test cap {MAX_TEST_DISTANCE_M:.1f}")
    if values["timeout_sec"] > MAX_TEST_TIMEOUT_SEC:
        raise ValueError(
            f"timeout_sec exceeds bounded test cap {MAX_TEST_TIMEOUT_SEC:.0f}")
    return values


class ShortDrive(Node):
    def __init__(self, distance_m=0.25, speed_mps=0.05, timeout_sec=10.0):
        super().__init__("short_drive_test")
        limits = validate_test_limits(distance_m, speed_mps, timeout_sec)
        self.distance_m = limits["distance_m"]
        self.speed_mps = limits["speed_mps"]
        self.timeout_sec = limits["timeout_sec"]
        self.start_pose = None
        self.latest_odom = None
        self.last_odom_at = None
        self.last_depth_at = None
        self.last_ackermann = None
        self.last_safety = ""
        self.last_state = ""
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            PointCloud2,
            "/smartcar/depth/points",
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            AckermannDriveStamped, "/ackermann_cmd", self._on_ackermann, 10)
        self.create_subscription(
            String, "/smartcar/safety/status", self._on_safety, 10)
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

    def _on_odom(self, message):
        self.latest_odom = message
        self.last_odom_at = time.monotonic()

    def _on_depth(self, _message):
        self.last_depth_at = time.monotonic()

    def _on_ackermann(self, message):
        self.last_ackermann = message.drive.speed

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

    def forward_displacement(self):
        if self.start_pose is None or self.latest_odom is None:
            return 0.0
        p0 = self.start_pose.pose.pose.position
        p1 = self.latest_odom.pose.pose.position
        q = self.start_pose.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (p1.x - p0.x) * math.cos(yaw) + (p1.y - p0.y) * math.sin(yaw)

    def run(self):
        if not self.spin_until(
            lambda: (
                self.latest_odom is not None
                and self.last_depth_at is not None
                and self.last_ackermann is not None
            ),
            8.0,
        ):
            raise RuntimeError("odom/depth/ackermann preflight unavailable")
        self.start_pose = self.latest_odom
        self.set_speed_limits()
        self.get_logger().info(
            f"Preflight passed: speed cap {self.speed_mps:.3f} m/s, "
            f"distance cap {self.distance_m:.3f} m")
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
            if displacement >= self.distance_m:
                reason = f"distance_limit:{displacement:.3f}m"
                break
            if displacement < -0.03:
                reason = f"unexpected_reverse:{displacement:.3f}m"
                break
            if self.last_odom_at is None or now - self.last_odom_at > 0.45:
                reason = "raw_odom_stale"
                break
            if self.last_depth_at is None or now - self.last_depth_at > 0.65:
                reason = "depth_points_stale"
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
        self.get_logger().info(
            f"Bounded test complete: reason={reason} "
            f"displacement={self.forward_displacement():.3f}m "
            f"state={self.last_state} safety={self.last_safety}")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Run a bounded supervised forward Nav2 drive test.")
    parser.add_argument(
        "--distance-m", type=float, default=0.25,
        help="maximum forward odometry distance (default: 0.25)")
    parser.add_argument(
        "--speed-mps", type=float, default=0.05,
        help="forward speed cap, at most 0.30 m/s (default: 0.05)")
    parser.add_argument(
        "--timeout-sec", type=float, default=10.0,
        help="maximum run time, at most 120 s (default: 10)")
    args = parser.parse_args(argv)
    rclpy.init(args=[])
    node = ShortDrive(
        distance_m=args.distance_m,
        speed_mps=args.speed_mps,
        timeout_sec=args.timeout_sec,
    )
    try:
        node.run()
    except Exception as error:
        node.stop_and_latch(f"preflight/test failure: {error}")
        node.get_logger().error(f"Bounded test failed: {error}")
        rclpy.shutdown()
        return 1
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
