#!/usr/bin/env python3
"""Ask smartcar_safety to hold a bounded zero-speed steering angle."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from smartcar_interfaces.srv import HoldSteeringCalibration

from smartcar_tools.steering_calibration import validate_hold_request


SERVICE_NAME = "/smartcar/safety/steering_calibration_hold"
SERVICE_TIMEOUT_SEC = 2.0


class SteeringHold(Node):
    """Client for the safety-owned, finite steering-only calibration hold."""

    def __init__(self):
        super().__init__("steering_hold")
        self._client = self.create_client(
            HoldSteeringCalibration, SERVICE_NAME)
        self._active = False

    def _call(self, angle, duration):
        if not self._client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
            raise RuntimeError(
                "safety steering-calibration service is unavailable")
        request = HoldSteeringCalibration.Request()
        request.steering_angle = float(angle)
        request.duration_sec = float(duration)
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=SERVICE_TIMEOUT_SEC)
        if not future.done():
            raise RuntimeError("safety steering-calibration service timed out")
        response = future.result()
        if response is None:
            raise RuntimeError(
                "safety steering-calibration service returned no response")
        if not response.success:
            raise RuntimeError(response.status)
        return response.status

    def hold(self, angle, duration):
        status = self._call(angle, duration)
        self._active = status == "steering_calibration_active"

    def cancel(self):
        if not self._active:
            return
        try:
            self._call(0.0, 0.0)
        except RuntimeError as error:
            self.get_logger().error(f"could not cancel steering hold: {error}")
        finally:
            self._active = False


def parse_arguments(args=None):
    parser = argparse.ArgumentParser(
        description="Hold a zero-speed steering angle for wheel measurement.")
    parser.add_argument("--angle", type=float, required=True,
                        help="signed steering angle in radians")
    parser.add_argument("--hold", type=float, default=12.0,
                        help="hold duration in seconds (maximum 15)")
    parser.add_argument("--yes", action="store_true",
                        help=(
                            "required confirmation that front wheels are "
                            "off ground"))
    parsed = parser.parse_args(args)
    if not parsed.yes:
        parser.error("--yes is required before actuating steering")
    try:
        parsed.angle, parsed.hold = validate_hold_request(
            parsed.angle, parsed.hold)
    except ValueError as error:
        parser.error(str(error))
    return parsed


def main(args=None):
    parsed = parse_arguments(args)
    rclpy.init(args=[])
    node = SteeringHold()
    try:
        node.hold(parsed.angle, parsed.hold)
        degrees = math.degrees(parsed.angle)
        print(
            f"HOLDING: {parsed.angle:+.3f} rad ({degrees:+.1f} deg) for "
            f"{parsed.hold:.1f} s",
            flush=True,
        )
        deadline = time.monotonic() + parsed.hold
        while time.monotonic() < deadline:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("steering hold interrupted; recentring", flush=True)
        return 130
    except RuntimeError as error:
        print(f"steering hold refused: {error}", flush=True)
        return 2
    finally:
        node.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
