"""ROS-independent fail-closed velocity safety decisions."""
import math


def _finite_float(name, value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def require_positive_finite(name, value):
    value = _finite_float(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be greater than zero")
    return value


def require_nonnegative_finite(name, value):
    value = _finite_float(name, value)
    if value < 0.0:
        raise ValueError(f"{name} must be nonnegative")
    return value


def validate_publish_frequency(value):
    return require_positive_finite("publish_frequency_hz", value)


class SafetyGuard:
    """Track received inputs and decide whether a velocity command is safe."""

    def __init__(
        self,
        command_timeout_sec=0.30,
        scan_timeout_sec=0.35,
        odom_timeout_sec=0.35,
        minimum_voltage=0.0,
        require_scan=True,
        require_odom=True,
        raw_odom_timeout_sec=0.25,
        require_raw_odom=True,
    ):
        self.command_timeout_sec = require_positive_finite(
            "command_timeout_sec", command_timeout_sec)
        self.scan_timeout_sec = require_positive_finite(
            "scan_timeout_sec", scan_timeout_sec)
        self.odom_timeout_sec = require_positive_finite(
            "odom_timeout_sec", odom_timeout_sec)
        self.raw_odom_timeout_sec = require_positive_finite(
            "raw_odom_timeout_sec", raw_odom_timeout_sec)
        self.minimum_voltage = require_nonnegative_finite(
            "minimum_voltage", minimum_voltage)
        self.require_scan = bool(require_scan)
        self.require_odom = bool(require_odom)
        self.require_raw_odom = bool(require_raw_odom)

        self.command_received_at = None
        self.command_invalid = False
        self.scan_received_at = None
        self.odom_received_at = None
        self.raw_odom_received_at = None
        self.voltage_received_at = None
        self.voltage = None
        self.emergency_stop = False

    def mark_command(self, receipt_time_sec):
        self.command_received_at = float(receipt_time_sec)
        self.command_invalid = False

    def mark_command_invalid(self):
        self.command_invalid = True

    def mark_scan(self, receipt_time_sec):
        self.scan_received_at = float(receipt_time_sec)

    def mark_odom(self, receipt_time_sec):
        self.odom_received_at = float(receipt_time_sec)

    def mark_raw_odom(self, receipt_time_sec):
        self.raw_odom_received_at = float(receipt_time_sec)

    def mark_voltage(self, voltage, receipt_time_sec):
        self.voltage = float(voltage)
        self.voltage_received_at = float(receipt_time_sec)

    def set_emergency_stop(self, enabled):
        """Latch a stop until a caller explicitly clears it with False."""
        self.emergency_stop = bool(enabled)

    @staticmethod
    def _fresh(receipt_time_sec, now_sec, timeout_sec):
        if receipt_time_sec is None:
            return False
        age_sec = float(now_sec) - receipt_time_sec
        return 0.0 <= age_sec <= timeout_sec

    @staticmethod
    def _result(allowed, reason):
        return {"allowed": allowed, "reason": reason}

    def command_is_fresh(self, now_sec):
        return self._fresh(
            self.command_received_at, now_sec, self.command_timeout_sec)

    def evaluate(self, now_sec):
        """Return a dict containing the fail-closed decision and its reason."""
        if self.emergency_stop:
            return self._result(False, "emergency_stop")

        if self.command_invalid:
            return self._result(False, "command_invalid")

        if self.command_received_at is None:
            return self._result(False, "command_missing")
        if not self._fresh(self.command_received_at, now_sec, self.command_timeout_sec):
            return self._result(False, "command_stale")

        if self.require_scan:
            if self.scan_received_at is None:
                return self._result(False, "scan_missing")
            if not self._fresh(self.scan_received_at, now_sec, self.scan_timeout_sec):
                return self._result(False, "scan_stale")

        if self.require_odom:
            if self.odom_received_at is None:
                return self._result(False, "odom_missing")
            if not self._fresh(self.odom_received_at, now_sec, self.odom_timeout_sec):
                return self._result(False, "odom_stale")

        if self.require_raw_odom:
            if self.raw_odom_received_at is None:
                return self._result(False, "raw_odom_missing")
            if not self._fresh(
                self.raw_odom_received_at, now_sec, self.raw_odom_timeout_sec
            ):
                return self._result(False, "raw_odom_stale")

        if self.minimum_voltage > 0.0:
            if self.voltage is None:
                return self._result(False, "voltage_missing")
            if not math.isfinite(self.voltage):
                return self._result(False, "voltage_invalid")
            if self.voltage < self.minimum_voltage:
                return self._result(False, "voltage_low")

        return self._result(True, "ok")
