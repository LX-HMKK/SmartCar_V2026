#!/usr/bin/env python3
"""Measure the local Gazebo Ackermann response through the Nav2 speed chain.

This diagnostic is simulation-only.  It publishes a bounded low-speed Twist to
``/cmd_vel_nav``, so the message still travels through velocity_smoother and
the /cmd_vel_candidate Gazebo bridge.  It never talks to the RDK interfaces or
to /ackermann_cmd.
"""

import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


STEERING_JOINTS = ("up_left_steer_joint", "up_right_steer_joint")


def normalize_angle(angle: float) -> float:
    """Return an angle in [-pi, pi] without relying on a ROS helper."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


class AckermannArcProbe(Node):
    """Issue one bounded left/right arc and report physical-model measurements."""

    def __init__(self) -> None:
        super().__init__("ackermann_arc_probe")
        self.declare_parameter("command_topic", "/cmd_vel_nav")
        self.declare_parameter("candidate_topic", "/cmd_vel_candidate")
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("linear_speed_mps", 0.05)
        self.declare_parameter("turning_radius_m", 0.22)
        self.declare_parameter("arc_angle_rad", 0.65)
        self.declare_parameter("turn_direction", "left")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("startup_timeout_sec", 10.0)
        self.declare_parameter("settle_sec", 0.5)
        self.declare_parameter(
            "results_file", "/tmp/smartcar_ackermann_arc_probe.json"
        )

        self._command_topic = str(self.get_parameter("command_topic").value)
        self._candidate_topic = str(self.get_parameter("candidate_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._joint_states_topic = str(
            self.get_parameter("joint_states_topic").value
        )
        self._publisher = self.create_publisher(Twist, self._command_topic, 20)
        self.create_subscription(Odometry, self._odom_topic, self._odom_callback, 100)
        self.create_subscription(
            Twist, self._candidate_topic, self._candidate_callback, 100
        )
        self.create_subscription(
            JointState, self._joint_states_topic, self._joint_callback, 100
        )

        self._latest_odom = None
        self._odom_samples = []
        self._candidate_samples = []
        self._steering_extrema = {
            name: {"minimum": None, "maximum": None} for name in STEERING_JOINTS
        }

    def _odom_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        sample = {
            "monotonic_sec": time.monotonic(),
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": yaw_from_quaternion(pose.orientation),
        }
        self._latest_odom = sample
        self._odom_samples.append(sample)

    def _candidate_callback(self, message: Twist) -> None:
        self._candidate_samples.append(
            {
                "monotonic_sec": time.monotonic(),
                "linear_x": message.linear.x,
                "angular_z": message.angular.z,
            }
        )

    def _joint_callback(self, message: JointState) -> None:
        for name, position in zip(message.name, message.position):
            if name not in self._steering_extrema or not math.isfinite(position):
                continue
            extrema = self._steering_extrema[name]
            if extrema["minimum"] is None or position < extrema["minimum"]:
                extrema["minimum"] = position
            if extrema["maximum"] is None or position > extrema["maximum"]:
                extrema["maximum"] = position

    def _spin_until_odom(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while self._latest_odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._latest_odom is not None

    def _publish(self, linear_x: float, angular_z: float) -> None:
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        self._publisher.publish(message)

    def _stop_and_settle(self, settle_sec: float) -> None:
        # Multiple zero messages cover discovery delays and make this command
        # safe to interrupt between speed-smoother update ticks.
        for _ in range(5):
            self._publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.03)
        deadline = time.monotonic() + settle_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    @staticmethod
    def _path_length(samples: list[dict]) -> float:
        return sum(
            math.hypot(second["x"] - first["x"], second["y"] - first["y"])
            for first, second in zip(samples, samples[1:])
        )

    def run_probe(self) -> dict:
        speed = float(self.get_parameter("linear_speed_mps").value)
        radius = float(self.get_parameter("turning_radius_m").value)
        arc_angle = float(self.get_parameter("arc_angle_rad").value)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        startup_timeout = float(self.get_parameter("startup_timeout_sec").value)
        settle_sec = float(self.get_parameter("settle_sec").value)
        direction = str(self.get_parameter("turn_direction").value).strip().lower()
        if (
            not all(math.isfinite(value) for value in (speed, radius, arc_angle, rate_hz))
            or speed <= 0.0
            or radius <= 0.0
            or arc_angle <= 0.0
            or rate_hz <= 0.0
            or startup_timeout <= 0.0
            or settle_sec < 0.0
            or direction not in ("left", "right")
        ):
            raise ValueError("invalid arc probe parameters")
        if not self._spin_until_odom(startup_timeout):
            raise RuntimeError(f"no odometry received on {self._odom_topic}")

        sign = 1.0 if direction == "left" else -1.0
        angular_speed = sign * speed / radius
        target_duration = arc_angle / abs(angular_speed)
        initial = dict(self._latest_odom)
        begin = time.monotonic()
        deadline = begin + target_duration
        interval = 1.0 / rate_hz
        while time.monotonic() < deadline:
            self._publish(speed, angular_speed)
            rclpy.spin_once(self, timeout_sec=interval)
        command_end = time.monotonic()
        self._stop_and_settle(settle_sec)

        samples = [
            sample
            for sample in self._odom_samples
            if begin <= sample["monotonic_sec"] <= command_end
        ]
        if not samples:
            samples = [initial]
        elif samples[0]["monotonic_sec"] - begin > 0.15:
            samples.insert(0, initial)
        final = dict(self._latest_odom)
        if samples[-1]["monotonic_sec"] < command_end - 0.15:
            samples.append(final)

        travelled_m = self._path_length(samples)
        yaw_change = normalize_angle(final["yaw"] - initial["yaw"])
        measured_radius = (
            travelled_m / abs(yaw_change) if abs(yaw_change) >= 0.05 else None
        )
        candidate_samples = [
            sample
            for sample in self._candidate_samples
            if begin <= sample["monotonic_sec"] <= command_end
        ]
        candidate_matches_command = any(
            sample["linear_x"] > speed * 0.5
            and sample["angular_z"] * sign > abs(angular_speed) * 0.5
            for sample in candidate_samples
        )
        expected_travel_m = radius * arc_angle
        direction_ok = yaw_change * sign > 0.05
        translation_ok = travelled_m >= expected_travel_m * 0.50
        radius_ok = (
            measured_radius is not None
            and abs(measured_radius - radius) <= max(0.06, radius * 0.35)
        )
        result = {
            "passed": candidate_matches_command and direction_ok and translation_ok and radius_ok,
            "command": {
                "topic": self._command_topic,
                "candidate_topic": self._candidate_topic,
                "linear_speed_mps": speed,
                "angular_speed_radps": angular_speed,
                "turning_radius_m": radius,
                "arc_angle_rad": arc_angle,
                "direction": direction,
                "target_duration_sec": target_duration,
                "expected_travel_m": expected_travel_m,
            },
            "measurements": {
                "initial_pose": initial,
                "final_pose": final,
                "travelled_m": travelled_m,
                "yaw_change_rad": yaw_change,
                "measured_radius_m": measured_radius,
                "odom_samples": len(samples),
                "candidate_samples": len(candidate_samples),
                "candidate_matches_command": candidate_matches_command,
                "steering_extrema_rad": self._steering_extrema,
            },
            "checks": {
                "direction_ok": direction_ok,
                "translation_ok": translation_ok,
                "radius_ok": radius_ok,
            },
        }
        return result


def main() -> int:
    rclpy.init()
    node = AckermannArcProbe()
    try:
        result = node.run_probe()
        output_path = Path(str(node.get_parameter("results_file").value))
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        node.get_logger().info(
            "Ackermann arc probe %s: radius=%s m, yaw=%s rad, travel=%s m"
            % (
                "passed" if result["passed"] else "failed",
                result["measurements"]["measured_radius_m"],
                result["measurements"]["yaw_change_rad"],
                result["measurements"]["travelled_m"],
            )
        )
        return 0 if result["passed"] else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().warning("Ackermann arc probe interrupted")
        return 130
    except Exception as error:
        node.get_logger().error(f"Ackermann arc probe failed: {error}")
        return 1
    finally:
        node._stop_and_settle(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
