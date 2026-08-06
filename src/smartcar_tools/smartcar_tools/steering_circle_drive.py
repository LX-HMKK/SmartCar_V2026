#!/usr/bin/env python3
"""Run a bounded steering circle through the normal SmartCar motion chain."""

import argparse
import csv
import math
from pathlib import Path
import time
import uuid

from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile
from sensor_msgs.msg import Imu
from smartcar_interfaces.srv import (
    ActivateMotion,
    PrepareMotion,
    RenewMotion,
    StopMotion,
)
from unique_identifier_msgs.msg import UUID

from smartcar_tools.steering_calibration import (
    angular_velocity_for_steering,
    validate_circle_request,
)


MOTION_FORWARD = 1
MOTION_REVERSE = 2
RENEW_PERIOD_SEC = 0.10
SERVICE_TIMEOUT_SEC = 1.0
CONTROLLER_STATE_SERVICE = "/controller_server/get_state"


def quaternion_yaw(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


class SteeringCircleDrive(Node):
    """Record a ground-circle run while holding a direction lease."""

    def __init__(self, angle, speed, duration, out_path, rate, wheelbase,
                 direction):
        super().__init__("steering_circle_drive")
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10)
        signed_speed = speed if direction == MOTION_FORWARD else -speed
        self._command = Twist()
        self._command.linear.x = signed_speed
        self._command.angular.z = angular_velocity_for_steering(
            signed_speed, angle, wheelbase)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav", qos)
        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._on_odom, qos)
        self._ekf_sub = self.create_subscription(
            Odometry, "/odom_combined", self._on_ekf, qos)
        self._imu_sub = self.create_subscription(
            Imu, "/imu/data_raw", self._on_imu, qos)
        self._prepare_client = self.create_client(
            PrepareMotion, "/smartcar/direction_guard/prepare")
        self._activate_client = self.create_client(
            ActivateMotion, "/smartcar/direction_guard/activate")
        self._renew_client = self.create_client(
            RenewMotion, "/smartcar/direction_guard/renew")
        self._stop_client = self.create_client(
            StopMotion, "/smartcar/direction_guard/stop")
        self._controller_state_client = self.create_client(
            GetState, CONTROLLER_STATE_SERVICE)

        self._angle = angle
        self._speed = signed_speed
        self._duration = duration
        self._out_path = Path(out_path)
        self._rate = rate
        self._rows = []
        self._last_odom = None
        self._last_ekf = None
        self._last_imu = None
        self._identity = None
        self._direction = direction

    def _on_odom(self, message):
        self._last_odom = message

    def _on_ekf(self, message):
        self._last_ekf = message

    def _on_imu(self, message):
        self._last_imu = message

    def _publish(self, command):
        self._cmd_pub.publish(command)

    def _publish_zero(self):
        self._publish(Twist())

    def _call(self, client, request, name):
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
            raise RuntimeError(f"{name} service is unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=SERVICE_TIMEOUT_SEC)
        if not future.done():
            raise RuntimeError(f"{name} service timed out")
        response = future.result()
        if response is None:
            raise RuntimeError(f"{name} service returned no response")
        return response

    def _require_controller_inactive(self):
        """Reject shared command input before a calibration drive begins."""

        if not self._controller_state_client.wait_for_service(
                timeout_sec=SERVICE_TIMEOUT_SEC):
            raise RuntimeError("controller lifecycle service is unavailable")
        future = self._controller_state_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=SERVICE_TIMEOUT_SEC)
        if not future.done() or future.result() is None:
            raise RuntimeError("controller lifecycle state query timed out")
        state = future.result().current_state
        if state.id != State.PRIMARY_STATE_INACTIVE:
            raise RuntimeError(
                "controller_server must be inactive before calibration "
                f"(current state: {state.label})")

    @staticmethod
    def _new_action_uuid():
        message = UUID()
        message.uuid = list(uuid.uuid4().bytes)
        return message

    def _fill_identity(self, request):
        request.boot_epoch = self._identity["boot_epoch"]
        request.lease_id = self._identity["lease_id"]
        request.generation = self._identity["generation"]
        request.action_uuid = self._identity["action_uuid"]

    def _prepare_motion(self):
        request = PrepareMotion.Request()
        request.direction = self._direction
        request.generation = max(1, time.monotonic_ns())
        request.action_uuid = self._new_action_uuid()
        response = self._call(self._prepare_client, request, "prepare")
        if not response.success:
            raise RuntimeError(
                f"direction lease prepare refused: {response.status}")
        self._identity = {
            "boot_epoch": response.boot_epoch,
            "lease_id": response.lease_id,
            "generation": request.generation,
            "action_uuid": request.action_uuid,
        }

        activate = ActivateMotion.Request()
        self._fill_identity(activate)
        response = self._call(self._activate_client, activate, "activate")
        if not response.success:
            raise RuntimeError(
                f"direction lease activate refused: {response.status}")

    def _renew_motion(self):
        request = RenewMotion.Request()
        self._fill_identity(request)
        response = self._call(self._renew_client, request, "renew")
        if not response.success:
            raise RuntimeError(
                f"direction lease renewal refused: {response.status}")

    def _stop_motion(self):
        self._publish_zero()
        if self._identity is None:
            return
        try:
            request = StopMotion.Request()
            self._fill_identity(request)
            response = self._call(self._stop_client, request, "stop")
            if not response.success:
                self.get_logger().error(
                    f"direction lease stop refused: {response.status}")
        except RuntimeError as error:
            self.get_logger().error(f"could not stop direction lease: {error}")
        finally:
            self._identity = None

    def _snapshot(self, elapsed):
        row = {
            "t": elapsed,
            "cmd_angle": self._angle,
            "cmd_speed": self._speed,
        }
        if self._last_odom is not None:
            pose = self._last_odom.pose.pose
            twist = self._last_odom.twist.twist
            row.update({
                "x": pose.position.x,
                "y": pose.position.y,
                "yaw": quaternion_yaw(pose.orientation),
                "vx": twist.linear.x,
                "odom_wz": twist.angular.z,
            })
        if self._last_ekf is not None:
            pose = self._last_ekf.pose.pose
            row.update({
                "ekf_x": pose.position.x,
                "ekf_y": pose.position.y,
                "ekf_yaw": quaternion_yaw(pose.orientation),
            })
        if self._last_imu is not None:
            row["gyro_wz"] = self._last_imu.angular_velocity.z
        self._rows.append(row)

    def _write_csv(self):
        if not self._rows:
            return
        self._out_path.parent.mkdir(parents=True, exist_ok=True)
        fields = sorted(set().union(*(row.keys() for row in self._rows)))
        with self._out_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self._rows)
        print(
            f"wrote {len(self._rows)} samples to {self._out_path}",
            flush=True,
        )

    def run(self):
        degrees = math.degrees(self._angle)
        print(
            "CIRCLE PLAN: "
            f"speed={self._speed:+.3f} m/s steering={self._angle:+.3f} rad "
            f"({degrees:+.1f} deg) duration={self._duration:.1f} s",
            flush=True,
        )
        print("Starting in 5 s. Keep the physical e-stop in hand.", flush=True)
        time.sleep(5.0)

        try:
            self._require_controller_inactive()
            self._prepare_motion()
            started = time.monotonic()
            next_renew = started + RENEW_PERIOD_SEC
            period = 1.0 / self._rate
            while True:
                now = time.monotonic()
                elapsed = now - started
                if elapsed >= self._duration:
                    break
                rclpy.spin_once(self, timeout_sec=0.0)
                self._publish(self._command)
                self._snapshot(elapsed)
                if now >= next_renew:
                    self._renew_motion()
                    next_renew = now + RENEW_PERIOD_SEC
                time.sleep(period)
        finally:
            self._stop_motion()
            self._write_csv()


def parse_arguments(args=None):
    parser = argparse.ArgumentParser(
        description=(
            "Drive a bounded arc for steering-radius calibration."))
    parser.add_argument("--angle", type=float, required=True,
                        help="signed steering angle in radians")
    parser.add_argument("--speed", type=float, default=0.15,
                        help="speed magnitude in m/s (maximum 0.15)")
    parser.add_argument("--direction", choices=("forward", "reverse"),
                        default="forward",
                        help="motion direction for the direction lease")
    parser.add_argument("--duration", type=float, default=20.0,
                        help="run duration in seconds (maximum 60)")
    parser.add_argument("--out", default="/tmp/steering_circle.csv",
                        help="CSV output path")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="command/record rate in Hz (maximum 50)")
    parser.add_argument("--wheelbase", type=float, default=0.189,
                        help="wheelbase used for Twist curvature conversion")
    parser.add_argument("--yes", action="store_true",
                        help="required confirmation for real vehicle motion")
    parsed = parser.parse_args(args)
    if not parsed.yes:
        parser.error("--yes is required for real vehicle motion")
    try:
        (
            parsed.angle,
            parsed.speed,
            parsed.duration,
            parsed.rate,
            parsed.wheelbase,
        ) = validate_circle_request(
            parsed.angle,
            parsed.speed,
            parsed.duration,
            parsed.rate,
            parsed.wheelbase,
        )
    except ValueError as error:
        parser.error(str(error))
    return parsed


def main(args=None):
    parsed = parse_arguments(args)
    rclpy.init(args=[])
    node = SteeringCircleDrive(
        parsed.angle, parsed.speed, parsed.duration, parsed.out, parsed.rate,
        parsed.wheelbase,
        MOTION_FORWARD if parsed.direction == "forward" else MOTION_REVERSE)
    try:
        node.run()
    except KeyboardInterrupt:
        print(
            "circle run interrupted; stopping through direction guard",
            flush=True,
        )
        return 130
    except RuntimeError as error:
        print(f"circle run refused: {error}", flush=True)
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
