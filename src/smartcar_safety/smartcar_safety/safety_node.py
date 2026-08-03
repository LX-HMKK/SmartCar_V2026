"""ROS 2 wrapper for the fail-closed velocity guard."""
import math
import time

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool

from smartcar_interfaces.srv import HoldSteeringCalibration
from smartcar_safety.guard import SafetyGuard, validate_publish_frequency
from smartcar_safety.velocity import (
    ZERO_TWIST_COMPONENTS,
    sanitize_twist_components,
)


LATEST_RELIABLE_QOS = QoSProfile(depth=1)
LATEST_SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
STATUS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def twist_from_components(components):
    """Return a fresh Twist populated from six sanitized components."""
    result = Twist()
    result.linear.x = components[0]
    result.linear.y = components[1]
    result.linear.z = components[2]
    result.angular.x = components[3]
    result.angular.y = components[4]
    result.angular.z = components[5]
    return result


def components_are_quiescent(components):
    return all(abs(float(component)) <= 1.0e-6 for component in components)


class SafetyNode(Node):
    """Gate /cmd_vel to /cmd_vel_safe and continuously publish a safe command."""

    def __init__(self):
        super().__init__("safety_node")
        self.declare_parameter("command_timeout_sec", 0.30)
        self.declare_parameter("scan_timeout_sec", 0.35)
        self.declare_parameter("odom_timeout_sec", 0.35)
        self.declare_parameter("minimum_voltage", 0.0)
        self.declare_parameter("voltage_timeout_sec", 1.0)
        self.declare_parameter("max_linear_speed_mps", 0.30)
        self.declare_parameter("publish_frequency_hz", 20.0)
        self.declare_parameter("require_scan", True)
        self.declare_parameter("require_odom", True)
        self.declare_parameter("require_raw_odom", True)
        self.declare_parameter("raw_odom_timeout_sec", 0.25)
        self.declare_parameter("odom_throttle_interval_sec", 0.05)
        self.declare_parameter("wheelbase", 0.189)
        self.declare_parameter("max_steering_angle", 0.70)
        self.declare_parameter("ackermann_frame_id", "odom_combined")
        self.declare_parameter("allow_steering_calibration", False)
        self.declare_parameter("steering_calibration_max_hold_sec", 15.0)
        self.declare_parameter("emergency_stop_on_start", False)

        self.guard = SafetyGuard(
            command_timeout_sec=self.get_parameter("command_timeout_sec").value,
            scan_timeout_sec=self.get_parameter("scan_timeout_sec").value,
            odom_timeout_sec=self.get_parameter("odom_timeout_sec").value,
            raw_odom_timeout_sec=self.get_parameter("raw_odom_timeout_sec").value,
            minimum_voltage=self.get_parameter("minimum_voltage").value,
            voltage_timeout_sec=self.get_parameter("voltage_timeout_sec").value,
            max_linear_speed_mps=self.get_parameter("max_linear_speed_mps").value,
            require_scan=self.get_parameter("require_scan").value,
            require_odom=self.get_parameter("require_odom").value,
            require_raw_odom=self.get_parameter("require_raw_odom").value,
        )
        if bool(self.get_parameter("emergency_stop_on_start").value):
            self.guard.set_emergency_stop(True)
        frequency_hz = validate_publish_frequency(
            self.get_parameter("publish_frequency_hz").value)

        self._zero_command = twist_from_components(ZERO_TWIST_COMPONENTS)
        self._last_command_components = None
        self._last_command_message = None
        self._last_status_reason = None
        self._last_odom_processed_at = None
        self._last_raw_odom_processed_at = None
        self._steering_calibration = None
        self._odom_throttle_interval = max(
            0.0, float(self.get_parameter("odom_throttle_interval_sec").value))

        self._safe_publisher = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self._ackermann_publisher = self.create_publisher(
            AckermannDriveStamped, "/ackermann_cmd", 10)
        self._wheelbase = float(self.get_parameter("wheelbase").value)
        self._max_steering_angle = float(
            self.get_parameter("max_steering_angle").value)
        self._ackermann_frame_id = str(
            self.get_parameter("ackermann_frame_id").value)
        self._status_publisher = self.create_publisher(
            String, "/smartcar/safety/status", STATUS_QOS)
        self.create_subscription(
            Twist, "/cmd_vel", self._on_command, LATEST_RELIABLE_QOS)
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, LATEST_SENSOR_QOS, raw=True)
        self.create_subscription(
            Odometry, "/odom_combined", self._on_odom, LATEST_RELIABLE_QOS,
            raw=True)
        self.create_subscription(
            Odometry, "/odom", self._on_raw_odom, LATEST_RELIABLE_QOS,
            raw=True)
        self.create_subscription(
            Float32, "/PowerVoltage", self._on_voltage, LATEST_RELIABLE_QOS)
        self.create_service(
            SetBool, "/smartcar/safety/emergency_stop", self._on_emergency_stop)
        self.create_service(
            HoldSteeringCalibration,
            "/smartcar/safety/steering_calibration_hold",
            self._on_steering_calibration_hold,
        )
        self.create_timer(1.0 / frequency_hz, self._on_timer)

        now_sec = self._now_sec()
        self._publish_status_if_changed(self.guard.evaluate(now_sec))

    def _now_sec(self):
        return time.monotonic()

    def _on_command(self, message):
        now_sec = self._now_sec()
        valid, components = sanitize_twist_components((
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        ))
        if not valid:
            self._steering_calibration = None
            self._last_command_components = None
            self._last_command_message = None
            self.guard.mark_command_invalid()
            self._publish_zero_command()
        elif not self.guard.mark_command(now_sec, components[0]):
            self._steering_calibration = None
            self._last_command_components = None
            self._last_command_message = None
            self._publish_zero_command()
        else:
            if (self._steering_calibration is not None
                    and not components_are_quiescent(components)):
                self._steering_calibration = None
            self._last_command_components = components
            self._last_command_message = twist_from_components(components)
        self._publish_status_if_changed(self.guard.evaluate(now_sec))

    def _on_scan(self, _message):
        now_sec = self._now_sec()
        self.guard.mark_scan(now_sec)

    def _on_odom(self, _serialized_message):
        # Throttle: effective watchdog window = odom_timeout + throttle_interval + timer_period.
        # At the configured 0.30 m/s limit, worst-case watchdog travel is
        # ~135 mm, within the configured safety margins.
        now = self._now_sec()
        if (self._last_odom_processed_at is not None
                and now - self._last_odom_processed_at < self._odom_throttle_interval):
            return
        self._last_odom_processed_at = now
        self.guard.mark_odom(now)

    def _on_raw_odom(self, _serialized_message):
        now = self._now_sec()
        if (self._last_raw_odom_processed_at is not None
                and now - self._last_raw_odom_processed_at < self._odom_throttle_interval):
            return
        self._last_raw_odom_processed_at = now
        self.guard.mark_raw_odom(now)

    def _on_voltage(self, message):
        now_sec = self._now_sec()
        self.guard.mark_voltage(message.data, now_sec)
        self._publish_status_if_changed(self.guard.evaluate(now_sec))

    def _on_emergency_stop(self, request, response):
        if request.data:
            self._steering_calibration = None
        else:
            self.guard.clear_command_speed_limit_fault()
        self.guard.set_emergency_stop(request.data)
        now_sec = self._now_sec()
        result = self.guard.evaluate(now_sec)
        self._publish_status_if_changed(result)
        response.success = True
        response.message = (
            "emergency stop latched"
            if request.data
            else "emergency stop and speed-limit latch cleared"
        )
        return response

    def _on_steering_calibration_hold(self, request, response):
        now_sec = self._now_sec()
        angle = float(request.steering_angle)
        duration = float(request.duration_sec)

        if duration == 0.0:
            self._steering_calibration = None
            response.success = True
            response.status = "steering_calibration_cancelled"
            return response

        if not bool(self.get_parameter("allow_steering_calibration").value):
            response.success = False
            response.status = "steering_calibration_disabled"
            return response

        max_hold = float(
            self.get_parameter("steering_calibration_max_hold_sec").value)
        if (
            not math.isfinite(angle)
            or not math.isfinite(duration)
            or not math.isfinite(max_hold)
            or max_hold <= 0.0
            or duration < 0.0
            or duration > max_hold
            or abs(angle) > self._max_steering_angle
        ):
            response.success = False
            response.status = "steering_calibration_request_invalid"
            return response

        verdict = self.guard.evaluate(now_sec)
        if not verdict["allowed"]:
            response.success = False
            response.status = (
                "steering_calibration_safety_blocked:" + verdict["reason"])
            return response
        if (
            self._last_command_components is None
            or not components_are_quiescent(self._last_command_components)
        ):
            response.success = False
            response.status = "steering_calibration_command_not_quiescent"
            return response

        self._steering_calibration = (angle, now_sec + duration)
        response.success = True
        response.status = "steering_calibration_active"
        return response

    def _publish_zero_command(self):
        self._safe_publisher.publish(self._zero_command)
        self._publish_ackermann(self._zero_command)

    def _publish_ackermann(self, twist, steering_override=None):
        """Convert Twist to AckermannDriveStamped and publish to /ackermann_cmd."""
        speed = float(twist.linear.x)
        angular = float(twist.angular.z)
        if steering_override is not None:
            steering = float(steering_override)
        elif abs(speed) < 0.001:
            steering = 0.0
        else:
            steering = math.atan(
                self._wheelbase * angular / speed)
            steering = max(-self._max_steering_angle,
                           min(self._max_steering_angle, steering))
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._ackermann_frame_id
        msg.drive.steering_angle = steering
        msg.drive.speed = speed
        self._ackermann_publisher.publish(msg)

    def _on_timer(self):
        now_sec = self._now_sec()
        result = self.guard.evaluate(now_sec)
        if result["allowed"] and self._last_command_message is not None:
            command = self._last_command_message
        else:
            command = self._zero_command
        steering_override = None
        if self._steering_calibration is not None:
            angle, expires_at = self._steering_calibration
            command_quiescent = (
                self._last_command_components is not None
                and components_are_quiescent(self._last_command_components)
            )
            if (not result["allowed"] or not command_quiescent
                    or now_sec >= expires_at):
                self._steering_calibration = None
            else:
                steering_override = angle
        self._safe_publisher.publish(command)
        self._publish_ackermann(command, steering_override)
        self._publish_status_if_changed(result)

    def _publish_status_if_changed(self, result):
        reason = result["reason"]
        if reason == self._last_status_reason:
            return
        self._status_publisher.publish(String(data=reason))
        self._last_status_reason = reason


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
