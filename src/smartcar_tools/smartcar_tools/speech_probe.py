"""Publish one speech request and follow its request-scoped status stream."""
import argparse
import json
import math
import sys
import time


SUCCESS = 0
REQUEST_FAILED = 1
TIMED_OUT = 2
CONSUMER_UNAVAILABLE = 3

_TRACKED_STATES = {
    "queued",
    "synthesizing",
    "playing",
    "completed",
    "failed",
    "dropped",
    "disabled",
    "unconfigured",
    "cancelled",
    "ignored",
}
_TERMINAL_STATES = {
    "completed",
    "failed",
    "dropped",
    "disabled",
    "unconfigured",
    "cancelled",
    "ignored",
}
_VALID_INITIAL_STATES = {
    "queued",
    "failed",
    "dropped",
    "disabled",
    "unconfigured",
    "ignored",
}


def parse_status(payload):
    """Return a normalized speech status dictionary or ``None``."""
    try:
        value = json.loads(str(payload))
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    if not isinstance(value, dict):
        return None
    state = value.get("state")
    request_id = value.get("request_id")
    detail = value.get("detail", "")
    if not isinstance(state, str) or not isinstance(request_id, str):
        return None
    state = state.strip()
    request_id = request_id.strip()
    if state not in _TRACKED_STATES or not request_id:
        return None
    return {
        "state": state,
        "request_id": request_id,
        "detail": str(detail),
    }


class SpeechStatusTracker:
    """Bind to the first post-publication request and ignore all others."""

    def __init__(self):
        self.request_id = None
        self.history = []
        self.terminal = None
        self._armed = False

    def arm(self):
        self._armed = True

    def consume(self, payload):
        status = parse_status(payload)
        if not self._armed or status is None:
            return None
        if self.request_id is None:
            # A transient-local topic may deliver the previous request's
            # completed/playing sample just after publication. A new request
            # starts at queued or at an immediate rejection state.
            if status["state"] not in _VALID_INITIAL_STATES:
                return None
            self.request_id = status["request_id"]
        if status["request_id"] != self.request_id:
            return None
        state = status["state"]
        if not self.history or self.history[-1] != state:
            self.history.append(state)
        if state in _TERMINAL_STATES:
            self.terminal = status
        return status


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Publish text and track the matching SmartCar TTS request."
        ),
    )
    parser.add_argument("--text", required=True, help="Text to synthesize")
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=45.0,
        help="Overall status deadline",
    )
    parser.add_argument(
        "--consumer-wait-sec",
        type=float,
        default=5.0,
        help="How long to wait for the speech subscriber",
    )
    parser.add_argument(
        "--input-topic",
        default="/smartcar/output/speech",
    )
    parser.add_argument(
        "--status-topic",
        default="/smartcar/speech/status",
    )
    return parser


def _positive(name, value):
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _run(options, ros_args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import String

    timeout_sec = _positive("timeout-sec", options.timeout_sec)
    wait_sec = _positive("consumer-wait-sec", options.consumer_wait_sec)
    text = str(options.text).strip()
    if not text:
        raise ValueError("text must be nonempty")

    rclpy.init(args=ros_args)
    node = Node("speech_probe")
    tracker = SpeechStatusTracker()
    status_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    def on_status(message):
        status = tracker.consume(message.data)
        if status is not None:
            print(json.dumps(status, ensure_ascii=False), flush=True)

    subscription = node.create_subscription(
        String,
        options.status_topic,
        on_status,
        status_qos,
    )
    publisher = node.create_publisher(String, options.input_topic, 10)
    del subscription
    try:
        discovery_deadline = time.monotonic() + wait_sec
        while (
            publisher.get_subscription_count() < 1
            and time.monotonic() < discovery_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() < 1:
            print("speech consumer unavailable", file=sys.stderr)
            return CONSUMER_UNAVAILABLE

        tracker.arm()
        publisher.publish(String(data=text))
        deadline = time.monotonic() + timeout_sec
        while tracker.terminal is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if tracker.terminal is None:
            print("speech request timed out", file=sys.stderr)
            return TIMED_OUT
        if tracker.terminal["state"] == "completed":
            return SUCCESS
        return REQUEST_FAILED
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    import rclpy
    from rclpy.utilities import remove_ros_args

    raw_args = list(sys.argv if args is None else [sys.argv[0], *args])
    options = build_parser().parse_args(remove_ros_args(args=raw_args)[1:])
    ros_args = None if args is None else list(args)
    try:
        return _run(options, ros_args)
    except ValueError as error:
        print(str(error), file=sys.stderr)
        return REQUEST_FAILED
    except KeyboardInterrupt:
        return 130
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
